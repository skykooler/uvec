//! Extract a sub-arrangement into a new scene, and merge one scene into another.
//! Used by hosts for grouping / clipboard / movie-clip conversion.
//!
//! A subset of a planar arrangement is still planar (deleting curves cannot
//! create crossings), so neither operation needs re-integration to stay valid —
//! *except* `merge`, which brings independent geometry together and must
//! re-planarize the seam. Curve tags and fill payloads are preserved throughout.

use std::collections::{HashMap, HashSet};

use crate::scene::FillData;
use crate::{CurveId, FillId, Point, Scene, VertexId};

impl Scene {
    /// Move `curves` and `fills` out of this scene into a new one, leaving the
    /// remainder here. The selection is completed automatically:
    ///
    /// - every boundary curve of an extracted fill is pulled in too (so the fill
    ///   isn't dangling), and
    /// - any extracted curve that *also* bounds a fill we are keeping is
    ///   **duplicated** — copied into the new scene but kept here — so the kept
    ///   fill still has its wall. Purely disjoint selections move cleanly with no
    ///   duplication.
    ///
    /// Shared vertices are duplicated by position. Curve tags and fill payloads
    /// are preserved. Returns the new scene and old→new id maps.
    pub fn extract(
        &mut self,
        curves: &HashSet<CurveId>,
        fills: &HashSet<FillId>,
    ) -> (Scene, HashMap<CurveId, CurveId>, HashMap<FillId, FillId>) {
        // 1. Augment the curve set with the boundary curves of extracted fills.
        let mut inside = curves.clone();
        for &fid in fills {
            if let Some(Some(f)) = self.fills.get(fid.0 as usize) {
                for lp in &f.loops {
                    for &(cid, _) in lp {
                        inside.insert(cid);
                    }
                }
            }
        }

        // 2. Boundary curves = inside curves still referenced by a kept fill.
        let mut boundary: HashSet<CurveId> = HashSet::new();
        for (i, slot) in self.fills.iter().enumerate() {
            if fills.contains(&FillId(i as u32)) {
                continue;
            }
            if let Some(f) = slot {
                for lp in &f.loops {
                    for &(cid, _) in lp {
                        if inside.contains(&cid) {
                            boundary.insert(cid);
                        }
                    }
                }
            }
        }

        // 3. Build the new scene.
        let mut new = Scene::new();
        new.snap = self.snap;
        new.tol = self.tol;
        let mut vmap: HashMap<VertexId, VertexId> = HashMap::new();
        let mut cmap: HashMap<CurveId, CurveId> = HashMap::new();

        for &cid in &inside {
            let (v0, v1, c0, c1, tag) = match self.curves.get(cid.0 as usize).and_then(|o| o.as_ref())
            {
                Some(c) => (c.v0, c.v1, c.c0, c.c1, c.tag),
                None => continue,
            };
            let (p0, p1) = (self.vpos(v0), self.vpos(v1));
            let nv0 = *vmap.entry(v0).or_insert_with(|| new.alloc_vertex(p0));
            let nv1 = *vmap.entry(v1).or_insert_with(|| new.alloc_vertex(p1));
            let ncid = new.alloc_curve(nv0, c0, c1, nv1, tag);
            cmap.insert(cid, ncid);
        }

        let mut fmap: HashMap<FillId, FillId> = HashMap::new();
        for &fid in fills {
            let (payload, anchor, loops) =
                match self.fills.get(fid.0 as usize).and_then(|o| o.as_ref()) {
                    Some(f) => (f.payload, f.anchor, f.loops.clone()),
                    None => continue,
                };
            let remapped: Vec<Vec<(CurveId, bool)>> = loops
                .iter()
                .map(|lp| lp.iter().filter_map(|&(c, d)| cmap.get(&c).map(|&nc| (nc, d))).collect())
                .collect();
            let nfid = FillId(new.fills.len() as u32);
            new.fills.push(Some(FillData { payload, anchor, loops: remapped }));
            fmap.insert(fid, nfid);
        }

        // 4. Remove from self everything that moved (inside minus duplicated
        //    boundary), plus the extracted fills; sweep orphaned vertices and
        //    re-derive the kept fills.
        for &cid in &inside {
            if !boundary.contains(&cid) {
                self.curves[cid.0 as usize] = None;
            }
        }
        for &fid in fills {
            if let Some(slot) = self.fills.get_mut(fid.0 as usize) {
                *slot = None;
            }
        }
        self.sweep_orphan_vertices();
        self.repair_fills();

        (new, cmap, fmap)
    }

    /// Merge every curve and fill of `other` into this scene, translated by
    /// `offset`. The seam is re-planarized (crossings between merged and existing
    /// geometry split; coincident vertices snap; duplicate curves dedupe), so the
    /// result is a valid arrangement. Tags and payloads are preserved. Returns
    /// old→new id maps for `other`'s curves and fills (mapping to the *pre*-
    /// integration ids; a mapped curve may since have been split further).
    pub fn merge(
        &mut self,
        other: &Scene,
        offset: Point,
    ) -> (HashMap<CurveId, CurveId>, HashMap<FillId, FillId>) {
        let mut vmap: HashMap<VertexId, VertexId> = HashMap::new();
        let mut cmap: HashMap<CurveId, CurveId> = HashMap::new();
        let mut new_curves: Vec<CurveId> = Vec::new();

        for cid in other.curve_ids() {
            let c = other.curve(cid);
            let (v0, v1, c0, c1, tag) = (c.v0, c.v1, c.c0, c.c1, c.tag);
            let p0 = other.vpos(v0) + offset;
            let p1 = other.vpos(v1) + offset;
            let nv0 = *vmap.entry(v0).or_insert_with(|| self.alloc_vertex(p0));
            let nv1 = *vmap.entry(v1).or_insert_with(|| self.alloc_vertex(p1));
            let ncid = self.alloc_curve(nv0, c0 + offset, c1 + offset, nv1, tag);
            cmap.insert(cid, ncid);
            new_curves.push(ncid);
        }

        let mut fmap: HashMap<FillId, FillId> = HashMap::new();
        for fid in other.fill_ids() {
            let (payload, anchor, loops) = {
                let f = other.fills[fid.0 as usize].as_ref().expect("live fill id");
                (f.payload, f.anchor + offset, f.loops.clone())
            };
            let remapped: Vec<Vec<(CurveId, bool)>> = loops
                .iter()
                .map(|lp| lp.iter().filter_map(|&(c, d)| cmap.get(&c).map(|&nc| (nc, d))).collect())
                .collect();
            let nfid = FillId(self.fills.len() as u32);
            self.fills.push(Some(FillData { payload, anchor, loops: remapped }));
            fmap.insert(fid, nfid);
        }

        // Re-planarize only the seam (the newly added curves and whatever they
        // touch), then re-derive all fills from their anchors.
        self.integrate(new_curves, None);
        self.sweep_orphan_vertices();
        self.repair_fills();

        (cmap, fmap)
    }
}
