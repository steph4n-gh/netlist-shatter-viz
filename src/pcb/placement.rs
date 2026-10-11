//! Bounded, deterministic placement previews. HPWL is a routing estimate, not routed length.
use super::*;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashSet};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PlacementConstraints {
    pub fixed: Vec<String>,
    pub max_move_mm: Option<f64>,
    /// Components in a group translate together; their relative positions/rotations never change.
    pub groups: Vec<Vec<String>>,
}
#[derive(Debug, Serialize)]
pub struct PlacementReport {
    pub before_hpwl_mm: f64,
    pub after_hpwl_mm: f64,
    pub moved_components: usize,
    pub accepted_moves: usize,
    pub elapsed_ms: f64,
    pub placement_issues: Vec<String>,
    pub removed_tracks: usize,
    pub removed_vias: usize,
}

/// Conservative fallback: critical ICs, capacitors and clocks stay fixed until groups are specified.
pub fn conservative_constraints(board: &PcbBoard) -> PlacementConstraints {
    PlacementConstraints {
        fixed: board
            .footprints
            .iter()
            .filter(|f| {
                ["J", "HDR", "U", "C", "Y", "X", "FID", "SW"]
                    .iter()
                    .any(|p| f.reference.starts_with(p))
            })
            .map(|f| f.reference.clone())
            .collect(),
        groups: vec![],
        max_move_mm: None,
    }
}

pub fn placement_hpwl(board: &PcbBoard) -> f64 {
    let mut bounds: BTreeMap<NetId, Rect> = BTreeMap::new();
    for f in &board.footprints {
        for p in &f.pads {
            if let Some(n) = p.net.filter(|n| *n != 0) {
                let q = Rect::from_point(f.pad_world_position(p));
                bounds
                    .entry(n)
                    .and_modify(|b| *b = b.union(&q))
                    .or_insert(q);
            }
        }
    }
    bounds
        .values()
        .map(|b| b.max.x - b.min.x + b.max.y - b.min.y)
        .sum()
}

fn same_side(a: &Footprint, b: &Footprint) -> bool {
    a.pads.iter().any(|p| {
        b.pads.iter().any(|q| match (p.pad_type, q.pad_type) {
            (PadType::Smd { layer: x }, PadType::Smd { layer: y }) => x == y,
            _ => true,
        })
    })
}

fn pad_envelope(f: &Footprint) -> Rect {
    f.pads.iter().map(|p| f.pad_world_bbox(p)).reduce(|a, b| a.union(&b))
        .unwrap_or_else(|| f.world_bounding_box())
}

pub fn placement_issues(board: &PcbBoard) -> Vec<String> {
    let boxes: Vec<_> = board
        .footprints
        .iter()
        .map(|f| f.world_bounding_box())
        .collect();
    let mut issues = vec![];
    for (i, b) in boxes.iter().enumerate() {
        // The edge setback constrains conductive lands. Connector/header
        // bodies may reach the edge; their envelopes still constrain neighbors.
        let copper = pad_envelope(&board.footprints[i]);
        if copper.min.x < 0.4
            || copper.min.y < 0.4
            || copper.max.x > board.width - 0.4
            || copper.max.y > board.height - 0.4
        {
            issues.push(format!(
                "{} pad copper exceeds the 0.4 mm edge margin",
                board.footprints[i].reference
            ));
        }
        for (j, a) in boxes.iter().enumerate().take(i) {
            if same_side(&board.footprints[i], &board.footprints[j])
                && b.inflate(0.15).intersects(a)
            {
                issues.push(format!(
                    "{} / {} envelopes overlap or have less than 0.15 mm separation",
                    board.footprints[j].reference, board.footprints[i].reference
                ));
            }
        }
    }
    issues
}

/// Coordinate descent with a seeded traversal order. No stale copper survives a placement change.
pub fn optimize_placement(
    board: &mut PcbBoard,
    constraints: &PlacementConstraints,
    seed: u64,
) -> Result<PlacementReport, String> {
    let start = std::time::Instant::now();
    if constraints
        .max_move_mm
        .is_some_and(|v| !v.is_finite() || v < 0.0 || v > 1000.0)
    {
        return Err("Invalid movement limit".into());
    }
    if board.footprints.len() > 300
        || ![board.width, board.height]
            .iter()
            .all(|v| v.is_finite() && *v > 0.8 && *v <= 1000.0)
    {
        return Err("Preview supports 1–1000 mm boards with at most 300 components".into());
    }
    if board.footprints.iter().any(|f| {
        ![f.position.x, f.position.y, f.rotation]
            .iter()
            .all(|v| v.is_finite())
            || !f.validate().is_empty()
    }) {
        return Err("Invalid component geometry".into());
    }
    let refs: BTreeMap<_, _> = board
        .footprints
        .iter()
        .enumerate()
        .map(|(i, f)| (f.reference.as_str(), i))
        .collect();
    if refs.len() != board.footprints.len() {
        return Err("Duplicate component references".into());
    }
    let mut fixed = HashSet::new();
    for r in &constraints.fixed {
        fixed.insert(
            *refs
                .get(r.as_str())
                .ok_or_else(|| format!("Unknown fixed component {r}"))?,
        );
    }
    let mut seen = HashSet::new();
    let mut groups = vec![];
    for group in &constraints.groups {
        if group.is_empty() {
            return Err("Empty placement group".into());
        }
        let mut g = vec![];
        for r in group {
            let i = *refs
                .get(r.as_str())
                .ok_or_else(|| format!("Unknown grouped component {r}"))?;
            if !seen.insert(i) {
                return Err(format!("Component {r} appears in multiple groups"));
            }
            g.push(i);
        }
        groups.push(g);
    }
    for i in 0..board.footprints.len() {
        if !seen.contains(&i) {
            groups.push(vec![i]);
        }
    }
    groups.retain(|g| !g.iter().any(|i| fixed.contains(i)));
    let original: Vec<_> = board.footprints.iter().map(|f| f.position).collect();
    let rotations: Vec<_> = board.footprints.iter().map(|f| f.rotation).collect();
    let before = placement_hpwl(board);
    let mut score = before;
    let mut accepted = 0;
    // Seed changes only traversal, never the circuit or dimensions.
    let mut rng = seed.wrapping_add(1);
    for i in (1..groups.len()).rev() {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
        groups.swap(i, (rng as usize) % (i + 1));
    }
    let mut boxes: Vec<_> = board
        .footprints
        .iter()
        .map(|f| f.world_bounding_box())
        .collect();
    let mut copper_boxes: Vec<_> = board.footprints.iter().map(pad_envelope).collect();
    for step in [2.0, 1.0, 0.5, 0.25] {
        for _ in 0..6 {
            let mut changed = false;
            for g in &groups {
                // A two-terminal resistor may flip without changing its body envelope.
                // Critical multi-part groups never rotate. A zero movement budget is a true baseline.
                if g.len() == 1 && constraints.max_move_mm != Some(0.0) {
                    let i = g[0];
                    if board.footprints[i].reference.starts_with('R')
                        && board.footprints[i].pads.len() == 2
                    {
                        let old = board.footprints[i].rotation;
                        board.footprints[i].rotation = (old + 180.0) % 360.0;
                        let b = board.footprints[i].world_bounding_box();
                        let copper = pad_envelope(&board.footprints[i]);
                        let legal = copper.min.x >= 0.4
                            && copper.min.y >= 0.4
                            && copper.max.x <= board.width - 0.4
                            && copper.max.y <= board.height - 0.4
                            && boxes.iter().enumerate().all(|(j, a)| {
                                i == j
                                    || !same_side(&board.footprints[i], &board.footprints[j])
                                    || !b.inflate(0.15).intersects(a)
                            });
                        let next = placement_hpwl(board);
                        if legal && next < score - 1e-7 {
                            score = next;
                            boxes[i] = b;
                            copper_boxes[i] = copper;
                            accepted += 1;
                            changed = true;
                        } else {
                            board.footprints[i].rotation = old;
                        }
                    }
                }
                let mut best = Point::zero();
                // Repair an invalid starting placement before minimizing wire
                // length. Otherwise every necessary separating move can lose
                // to a shorter, overlapping baseline forever.
                let current_legal = g.iter().all(|&i| {
                    let b = boxes[i];
                    let copper = copper_boxes[i];
                    copper.min.x >= 0.4 && copper.min.y >= 0.4
                        && copper.max.x <= board.width - 0.4 && copper.max.y <= board.height - 0.4
                        && boxes.iter().enumerate().all(|(j, a)| {
                            g.contains(&j)
                                || !same_side(&board.footprints[i], &board.footprints[j])
                                || !b.inflate(0.15).intersects(a)
                        })
                });
                let mut best_score = if current_legal { score } else { f64::INFINITY };
                for (dx, dy) in [
                    (1., 0.),
                    (-1., 0.),
                    (0., 1.),
                    (0., -1.),
                    (1., 1.),
                    (-1., 1.),
                    (1., -1.),
                    (-1., -1.),
                ] {
                    let delta = Point::new(dx * step, dy * step);
                    let valid = g.iter().all(|&i| {
                        if constraints.max_move_mm.is_some_and(|limit| {
                            (board.footprints[i].position + delta).distance_to(original[i])
                                > limit + 1e-9
                        }) {
                            return false;
                        }
                        let b = Rect::new(boxes[i].min + delta, boxes[i].max + delta);
                        let copper = Rect::new(copper_boxes[i].min + delta, copper_boxes[i].max + delta);
                        copper.min.x >= 0.4
                            && copper.min.y >= 0.4
                            && copper.max.x <= board.width - 0.4
                            && copper.max.y <= board.height - 0.4
                            && boxes.iter().enumerate().all(|(j, a)| {
                                g.contains(&j)
                                    || !same_side(&board.footprints[i], &board.footprints[j])
                                    || !b.inflate(0.15).intersects(a)
                            })
                    });
                    if !valid {
                        continue;
                    }
                    for &i in g {
                        board.footprints[i].position += delta;
                    }
                    let next = placement_hpwl(board);
                    for &i in g {
                        board.footprints[i].position -= delta;
                    }
                    if next < best_score - 1e-7 {
                        best_score = next;
                        best = delta;
                    }
                }
                if best != Point::zero() {
                    for &i in g {
                        board.footprints[i].position += best;
                        boxes[i] = Rect::new(boxes[i].min + best, boxes[i].max + best);
                        copper_boxes[i] = Rect::new(copper_boxes[i].min + best, copper_boxes[i].max + best);
                    }
                    score = best_score;
                    accepted += 1;
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
    }
    let moved = board
        .footprints
        .iter()
        .zip(original)
        .zip(rotations)
        .filter(|((f, p), r)| f.position.distance_to(*p) > 1e-6 || (f.rotation - r).abs() > 1e-6)
        .count();
    let (removed_tracks, removed_vias) = if moved > 0 {
        let counts = (board.tracks.len(), board.vias.len());
        board.tracks.clear();
        board.vias.clear();
        counts
    } else {
        (0, 0)
    };
    Ok(PlacementReport {
        before_hpwl_mm: before,
        after_hpwl_mm: placement_hpwl(board),
        moved_components: moved,
        accepted_moves: accepted,
        elapsed_ms: start.elapsed().as_secs_f64() * 1000.,
        placement_issues: placement_issues(board),
        removed_tracks,
        removed_vias,
    })
}
