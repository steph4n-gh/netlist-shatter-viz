//! Automated Design Rule Checker (DRC) Engine
//!
//! Validates:
//! - Minimum trace-to-trace clearance (2D capsule-to-capsule segment distance)
//! - Trace-to-pad clearance (capsule to rotated rectangle / circle / obround)
//! - Pad-to-pad clearance (rotated rectangle / circle / obround distance)
//! - Via-to-trace, via-to-pad, and via-to-via clearance
//! - Electrical short detection (intersecting copper belonging to different nets)
//! - Annular ring sizing (outer diameter - drill >= min_annular_ring, fatal breakout)
//! - Board edge keepouts (copper to Edge.Cuts clearance and bounds checking)
//! - Unrouted nets detection (graph connectivity via Union-Find / DSU)

use crate::pcb::core::*;
use crate::pcb::router::RouterConfig;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Taxonomy of Design Rule Checking (DRC) violation types.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DrcViolationKind {
    TraceTraceClearance,
    TracePadClearance,
    PadPadClearance,
    ViaTraceClearance,
    ViaPadClearance,
    ViaViaClearance,
    ShortCircuit,
    AnnularRingTooSmall,
    DrillBreakout,
    EdgeKeepoutViolation,
    OutsideBoardBoundary,
    UnroutedNet,
}

/// Detailed DRC violation report item.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DrcViolation {
    pub kind: DrcViolationKind,
    pub location: Point,
    pub layer: Option<Layer>,
    pub net1: Option<NetId>,
    pub net2: Option<NetId>,
    pub actual: f64,
    pub required: f64,
    pub message: String,
}

impl DrcViolation {
    pub fn new(
        kind: DrcViolationKind,
        location: Point,
        layer: Option<Layer>,
        net1: Option<NetId>,
        net2: Option<NetId>,
        actual: f64,
        required: f64,
        message: impl Into<String>,
    ) -> Self {
        Self {
            kind,
            location,
            layer,
            net1,
            net2,
            actual,
            required,
            message: message.into(),
        }
    }
}

/// Complete DRC verification result containing all violation categories.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DrcResult {
    pub clearance_violations: Vec<DrcViolation>,
    pub short_circuits: Vec<DrcViolation>,
    pub unrouted_nets: Vec<NetId>,
    pub annular_ring_violations: Vec<DrcViolation>,
    pub edge_keepout_violations: Vec<DrcViolation>,
}

/// Type alias providing compatibility with both DrcReport and DrcResult naming conventions.
pub type DrcReport = DrcResult;

impl DrcResult {
    pub fn new() -> Self {
        Self::default()
    }

    #[inline]
    pub fn is_clean(&self) -> bool {
        self.clearance_violations.is_empty()
            && self.short_circuits.is_empty()
            && self.unrouted_nets.is_empty()
            && self.annular_ring_violations.is_empty()
            && self.edge_keepout_violations.is_empty()
    }

    #[inline]
    pub fn total_violations(&self) -> usize {
        self.clearance_violations.len()
            + self.short_circuits.len()
            + self.unrouted_nets.len()
            + self.annular_ring_violations.len()
            + self.edge_keepout_violations.len()
    }
}

// ============================================================================
// Geometric Primitives & Distance Algorithms
// ============================================================================

/// Analytical distance between two 2D line segments P1P2 and Q1Q2.
/// Returns (minimum distance, midpoint of closest approach).
pub fn segment_to_segment_dist(p1: Point, p2: Point, q1: Point, q2: Point) -> (f64, Point) {
    let u = p2 - p1;
    let v = q2 - q1;
    let w = p1 - q1;
    let a = u.dot(u);
    let b = u.dot(v);
    let c = v.dot(v);
    let d = u.dot(w);
    let e = v.dot(w);
    let den = a * c - b * b;

    let sc: f64;
    let mut s_n: f64;
    let mut s_d: f64 = den;
    let tc: f64;
    let mut t_n: f64;
    let mut t_d: f64 = den;

    if den < 1e-12 {
        s_n = 0.0;
        s_d = 1.0;
        t_n = e;
        t_d = c;
    } else {
        s_n = b * e - c * d;
        t_n = a * e - b * d;
        if s_n < 0.0 {
            s_n = 0.0;
            t_n = e;
            t_d = c;
        } else if s_n > s_d {
            s_n = s_d;
            t_n = e + b;
            t_d = c;
        }
    }

    if t_n < 0.0 {
        tc = 0.0;
        if -d < 0.0 {
            sc = 0.0;
        } else if -d > a {
            sc = 1.0;
        } else {
            sc = -d / a;
        }
    } else if t_n > t_d {
        tc = 1.0;
        if (-d + b) < 0.0 {
            sc = 0.0;
        } else if (-d + b) > a {
            sc = 1.0;
        } else {
            sc = (-d + b) / a;
        }
    } else {
        tc = if t_d.abs() < 1e-12 { 0.0 } else { t_n / t_d };
        sc = if s_d.abs() < 1e-12 { 0.0 } else { s_n / s_d };
    }

    let sc = sc.clamp(0.0, 1.0);
    let tc = tc.clamp(0.0, 1.0);

    let closest_p = p1 + u * sc;
    let closest_q = q1 + v * tc;
    let dist = closest_p.distance_to(closest_q);
    (dist, closest_p.midpoint(closest_q))
}

/// Distance from a point to an oriented rectangle.
/// Returns (distance, closest point). Distance is 0.0 if inside.
pub fn point_to_oriented_rect_dist(
    p: Point,
    center: Point,
    width: f64,
    height: f64,
    rot_deg: f64,
) -> (f64, Point) {
    let p_rel = p - center;
    let p_local = p_rel.rotate(-rot_deg);

    let half_w = width / 2.0;
    let half_h = height / 2.0;

    let closest_local_x = p_local.x.clamp(-half_w, half_w);
    let closest_local_y = p_local.y.clamp(-half_h, half_h);
    let closest_local = Point::new(closest_local_x, closest_local_y);

    let dist = p_local.distance_to(closest_local);
    let closest_world = center + closest_local.rotate(rot_deg);
    (dist, closest_world)
}

/// Check if a 2D line segment intersects an axis-aligned box [-half_w, half_w] x [-half_h, half_h].
pub fn segment_intersects_aabb(p1: Point, p2: Point, half_w: f64, half_h: f64) -> bool {
    if (p1.x >= -half_w && p1.x <= half_w && p1.y >= -half_h && p1.y <= half_h)
        || (p2.x >= -half_w && p2.x <= half_w && p2.y >= -half_h && p2.y <= half_h)
    {
        return true;
    }

    let min_x = p1.x.min(p2.x);
    let max_x = p1.x.max(p2.x);
    let min_y = p1.y.min(p2.y);
    let max_y = p1.y.max(p2.y);

    if max_x < -half_w || min_x > half_w || max_y < -half_h || min_y > half_h {
        return false;
    }

    let edges = [
        (Point::new(-half_w, -half_h), Point::new(half_w, -half_h)),
        (Point::new(half_w, -half_h), Point::new(half_w, half_h)),
        (Point::new(half_w, half_h), Point::new(-half_w, half_h)),
        (Point::new(-half_w, half_h), Point::new(-half_w, -half_h)),
    ];

    for (e1, e2) in edges {
        let (d, _) = segment_to_segment_dist(p1, p2, e1, e2);
        if d < 1e-9 {
            return true;
        }
    }
    false
}

/// Distance from a line segment to an oriented rectangle.
/// Returns 0.0 if segment intersects or enters rectangle.
pub fn segment_to_oriented_rect_dist(
    p1: Point,
    p2: Point,
    center: Point,
    width: f64,
    height: f64,
    rot_deg: f64,
) -> f64 {
    let p1_local = (p1 - center).rotate(-rot_deg);
    let p2_local = (p2 - center).rotate(-rot_deg);

    let half_w = width / 2.0;
    let half_h = height / 2.0;

    if segment_intersects_aabb(p1_local, p2_local, half_w, half_h) {
        return 0.0;
    }

    let edges = [
        (Point::new(-half_w, -half_h), Point::new(half_w, -half_h)),
        (Point::new(half_w, -half_h), Point::new(half_w, half_h)),
        (Point::new(half_w, half_h), Point::new(-half_w, half_h)),
        (Point::new(-half_w, half_h), Point::new(-half_w, -half_h)),
    ];

    let mut min_d = f64::INFINITY;
    for (e1, e2) in edges {
        let (d, _) = segment_to_segment_dist(p1_local, p2_local, e1, e2);
        if d < min_d {
            min_d = d;
        }
    }
    min_d
}

/// Distance from a line segment to an obround pad (capsule).
pub fn segment_to_obround_dist(
    p1: Point,
    p2: Point,
    center: Point,
    width: f64,
    height: f64,
    rot_deg: f64,
) -> f64 {
    let (core1, core2, radius) = obround_core(center, width, height, rot_deg);
    let (d, _) = segment_to_segment_dist(p1, p2, core1, core2);
    (d - radius).max(0.0)
}

/// Obround core segment endpoints and radius.
fn obround_core(center: Point, width: f64, height: f64, rot_deg: f64) -> (Point, Point, f64) {
    if width >= height {
        let half_len = (width - height) / 2.0;
        let radius = height / 2.0;
        let c1 = center + Point::new(-half_len, 0.0).rotate(rot_deg);
        let c2 = center + Point::new(half_len, 0.0).rotate(rot_deg);
        (c1, c2, radius)
    } else {
        let half_len = (height - width) / 2.0;
        let radius = width / 2.0;
        let c1 = center + Point::new(0.0, -half_len).rotate(rot_deg);
        let c2 = center + Point::new(0.0, half_len).rotate(rot_deg);
        (c1, c2, radius)
    }
}

/// Rotated rectangle corners in world coordinates.
pub fn oriented_rect_corners(center: Point, width: f64, height: f64, rot_deg: f64) -> [Point; 4] {
    let hw = width / 2.0;
    let hh = height / 2.0;
    [
        center + Point::new(-hw, -hh).rotate(rot_deg),
        center + Point::new(hw, -hh).rotate(rot_deg),
        center + Point::new(hw, hh).rotate(rot_deg),
        center + Point::new(-hw, hh).rotate(rot_deg),
    ]
}

/// Separating Axis Theorem (SAT) check for two rotated rectangles.
/// Returns true if they intersect.
pub fn oriented_rects_intersect(
    c1: Point,
    w1: f64,
    h1: f64,
    r1: f64,
    c2: Point,
    w2: f64,
    h2: f64,
    r2: f64,
) -> bool {
    let corners1 = oriented_rect_corners(c1, w1, h1, r1);
    let corners2 = oriented_rect_corners(c2, w2, h2, r2);

    let axes = [
        Point::new(1.0, 0.0).rotate(r1),
        Point::new(0.0, 1.0).rotate(r1),
        Point::new(1.0, 0.0).rotate(r2),
        Point::new(0.0, 1.0).rotate(r2),
    ];

    for axis in axes {
        let mut min1 = f64::INFINITY;
        let mut max1 = f64::NEG_INFINITY;
        for &p in &corners1 {
            let proj = p.dot(axis);
            min1 = min1.min(proj);
            max1 = max1.max(proj);
        }

        let mut min2 = f64::INFINITY;
        let mut max2 = f64::NEG_INFINITY;
        for &p in &corners2 {
            let proj = p.dot(axis);
            min2 = min2.min(proj);
            max2 = max2.max(proj);
        }

        if max1 < min2 - 1e-9 || max2 < min1 - 1e-9 {
            return false; // Separating axis found
        }
    }
    true
}

/// Minimum surface distance between two oriented rectangles.
pub fn oriented_rect_to_rect_surface_dist(
    c1: Point,
    w1: f64,
    h1: f64,
    r1: f64,
    c2: Point,
    w2: f64,
    h2: f64,
    r2: f64,
) -> f64 {
    if oriented_rects_intersect(c1, w1, h1, r1, c2, w2, h2, r2) {
        return 0.0;
    }

    let corners1 = oriented_rect_corners(c1, w1, h1, r1);
    let corners2 = oriented_rect_corners(c2, w2, h2, r2);

    let edges1 = [
        (corners1[0], corners1[1]),
        (corners1[1], corners1[2]),
        (corners1[2], corners1[3]),
        (corners1[3], corners1[0]),
    ];

    let edges2 = [
        (corners2[0], corners2[1]),
        (corners2[1], corners2[2]),
        (corners2[2], corners2[3]),
        (corners2[3], corners2[0]),
    ];

    let mut min_d = f64::INFINITY;
    for &(e1a, e1b) in &edges1 {
        for &(e2a, e2b) in &edges2 {
            let (d, _) = segment_to_segment_dist(e1a, e1b, e2a, e2b);
            if d < min_d {
                min_d = d;
            }
        }
    }
    min_d
}

/// Distance from a line segment (with width) to a pad.
pub fn track_to_pad_surface_dist(track: &TrackSegment, pad: &Pad, pad_pos: Point, pad_rot: f64) -> f64 {
    let half_w = track.width / 2.0;
    match pad.shape {
        PadShape::Circle { diameter } => {
            let center_dist = track.distance_to_point(pad_pos);
            center_dist - (half_w + diameter / 2.0)
        }
        PadShape::Rect { width, height } => {
            let center_dist = segment_to_oriented_rect_dist(
                track.start,
                track.end,
                pad_pos,
                width,
                height,
                pad_rot,
            );
            center_dist - half_w
        }
        PadShape::Obround { width, height } => {
            let center_dist = segment_to_obround_dist(
                track.start,
                track.end,
                pad_pos,
                width,
                height,
                pad_rot,
            );
            center_dist - half_w
        }
    }
}

/// Distance between two pads.
pub fn pad_to_pad_surface_dist(
    p1: &Pad,
    pos1: Point,
    rot1: f64,
    p2: &Pad,
    pos2: Point,
    rot2: f64,
) -> f64 {
    match (&p1.shape, &p2.shape) {
        (PadShape::Circle { diameter: d1 }, PadShape::Circle { diameter: d2 }) => {
            let center_dist = pos1.distance_to(pos2);
            center_dist - (d1 / 2.0 + d2 / 2.0)
        }
        (PadShape::Rect { width: w1, height: h1 }, PadShape::Rect { width: w2, height: h2 }) => {
            oriented_rect_to_rect_surface_dist(pos1, *w1, *h1, rot1, pos2, *w2, *h2, rot2)
        }
        (PadShape::Circle { diameter }, PadShape::Rect { width, height }) => {
            let (d, _) = point_to_oriented_rect_dist(pos1, pos2, *width, *height, rot2);
            d - diameter / 2.0
        }
        (PadShape::Rect { width, height }, PadShape::Circle { diameter }) => {
            let (d, _) = point_to_oriented_rect_dist(pos2, pos1, *width, *height, rot1);
            d - diameter / 2.0
        }
        (PadShape::Obround { width: w1, height: h1 }, PadShape::Circle { diameter }) => {
            let (c1, c2, rad) = obround_core(pos1, *w1, *h1, rot1);
            let seg = TrackSegment::new(c1, c2, rad * 2.0, Layer::FCu, 0);
            seg.distance_to_point(pos2) - (rad + diameter / 2.0)
        }
        (PadShape::Circle { diameter }, PadShape::Obround { width: w2, height: h2 }) => {
            let (c1, c2, rad) = obround_core(pos2, *w2, *h2, rot2);
            let seg = TrackSegment::new(c1, c2, rad * 2.0, Layer::FCu, 0);
            seg.distance_to_point(pos1) - (rad + diameter / 2.0)
        }
        (PadShape::Obround { width: w1, height: h1 }, PadShape::Rect { width: w2, height: h2 }) => {
            let (c1, c2, rad) = obround_core(pos1, *w1, *h1, rot1);
            let d = segment_to_oriented_rect_dist(c1, c2, pos2, *w2, *h2, rot2);
            d - rad
        }
        (PadShape::Rect { width: w1, height: h1 }, PadShape::Obround { width: w2, height: h2 }) => {
            let (c1, c2, rad) = obround_core(pos2, *w2, *h2, rot2);
            let d = segment_to_oriented_rect_dist(c1, c2, pos1, *w1, *h1, rot1);
            d - rad
        }
        (PadShape::Obround { width: w1, height: h1 }, PadShape::Obround { width: w2, height: h2 }) => {
            let (c1a, c1b, rad1) = obround_core(pos1, *w1, *h1, rot1);
            let (c2a, c2b, rad2) = obround_core(pos2, *w2, *h2, rot2);
            let (d, _) = segment_to_segment_dist(c1a, c1b, c2a, c2b);
            d - (rad1 + rad2)
        }
    }
}

/// Distance from via to pad.
pub fn via_to_pad_surface_dist(via: &Via, pad: &Pad, pad_pos: Point, pad_rot: f64) -> f64 {
    let via_rad = via.size / 2.0;
    match pad.shape {
        PadShape::Circle { diameter } => {
            let d = via.at.distance_to(pad_pos);
            d - (via_rad + diameter / 2.0)
        }
        PadShape::Rect { width, height } => {
            let (d, _) = point_to_oriented_rect_dist(via.at, pad_pos, width, height, pad_rot);
            d - via_rad
        }
        PadShape::Obround { width, height } => {
            let (c1, c2, rad) = obround_core(pad_pos, width, height, pad_rot);
            let seg = TrackSegment::new(c1, c2, rad * 2.0, Layer::FCu, 0);
            seg.distance_to_point(via.at) - (via_rad + rad)
        }
    }
}

/// Check if two copper elements share at least one layer.
#[inline]
pub fn layers_share_copper(layer1: Layer, is_tht1: bool, layer2: Layer, is_tht2: bool) -> bool {
    if is_tht1 || is_tht2 {
        // THT pads and vias exist on both FCu and BCu
        (layer1.is_copper() || is_tht1) && (layer2.is_copper() || is_tht2)
    } else {
        layer1 == layer2 && layer1.is_copper()
    }
}

// ============================================================================
// DRC Verification Runner
// ============================================================================

/// Executes all Design Rule Checks on the board according to the given configuration.
pub fn run_drc(board: &PcbBoard, config: &RouterConfig) -> DrcResult {
    let mut result = DrcResult::new();

    // Flatten all pads with global positions
    struct GlobalPad<'a> {
        pad: &'a Pad,
        pos: Point,
        rot: f64,
        net: Option<NetId>,
        is_tht: bool,
        layer: Layer,
    }

    let mut pads = Vec::new();
    for fp in &board.footprints {
        for pad in &fp.pads {
            let pos = pad.global_position(fp.position, fp.rotation);
            let is_tht = pad.pad_type.is_through_hole();
            let layer = match pad.pad_type {
                PadType::Smd { layer } => layer,
                PadType::ThroughHole { .. } => Layer::FCu, // THT touches all copper
            };
            pads.push(GlobalPad {
                pad,
                pos,
                rot: fp.rotation,
                net: pad.net,
                is_tht,
                layer,
            });
        }
    }

    // ------------------------------------------------------------------------
    // 1. Annular Ring & Drill Breakout Checks
    // ------------------------------------------------------------------------
    const MIN_ANNULAR_RING: f64 = 0.15;

    for via in &board.vias {
        let ring = via.annular_ring();
        if via.drill >= via.size {
            result.annular_ring_violations.push(DrcViolation::new(
                DrcViolationKind::DrillBreakout,
                via.at,
                Some(Layer::FCu),
                Some(via.net),
                None,
                ring,
                MIN_ANNULAR_RING,
                format!(
                    "Fatal drill breakout: drill diameter ({:.2}mm) >= via diameter ({:.2}mm)",
                    via.drill, via.size
                ),
            ));
        } else if ring < MIN_ANNULAR_RING - 1e-6 {
            result.annular_ring_violations.push(DrcViolation::new(
                DrcViolationKind::AnnularRingTooSmall,
                via.at,
                Some(Layer::FCu),
                Some(via.net),
                None,
                ring,
                MIN_ANNULAR_RING,
                format!(
                    "Via annular ring too small: {:.3}mm < required {:.3}mm",
                    ring, MIN_ANNULAR_RING
                ),
            ));
        }
    }

    for gp in &pads {
        if let Some(drill) = gp.pad.pad_type.drill_diameter() {
            if let Some(ring) = gp.pad.annular_ring() {
                let outer = match gp.pad.pad_type {
                    PadType::ThroughHole { outer_diameter, .. } => outer_diameter,
                    _ => 0.0,
                };
                if drill >= outer {
                    result.annular_ring_violations.push(DrcViolation::new(
                        DrcViolationKind::DrillBreakout,
                        gp.pos,
                        Some(gp.layer),
                        gp.net,
                        None,
                        ring,
                        MIN_ANNULAR_RING,
                        format!(
                            "Fatal drill breakout on THT pad: drill ({:.2}mm) >= outer ({:.2}mm)",
                            drill, outer
                        ),
                    ));
                } else if ring < MIN_ANNULAR_RING - 1e-6 {
                    result.annular_ring_violations.push(DrcViolation::new(
                        DrcViolationKind::AnnularRingTooSmall,
                        gp.pos,
                        Some(gp.layer),
                        gp.net,
                        None,
                        ring,
                        MIN_ANNULAR_RING,
                        format!(
                            "Pad annular ring too small: {:.3}mm < required {:.3}mm",
                            ring, MIN_ANNULAR_RING
                        ),
                    ));
                }
            }
        }
    }

    // ------------------------------------------------------------------------
    // 2. Board Edge Keepout & Boundary Checks
    // ------------------------------------------------------------------------
    const MIN_EDGE_KEEPOUT: f64 = 0.40;

    let check_bbox_edge = |bbox: Rect, loc: Point, layer: Option<Layer>, net: Option<NetId>, res: &mut DrcResult| {
        if bbox.min.x < 0.0 || bbox.max.x > board.width || bbox.min.y < 0.0 || bbox.max.y > board.height {
            res.edge_keepout_violations.push(DrcViolation::new(
                DrcViolationKind::OutsideBoardBoundary,
                loc,
                layer,
                net,
                None,
                0.0,
                MIN_EDGE_KEEPOUT,
                "Copper geometry extends outside board outline boundary",
            ));
        } else {
            let edge_dist = bbox.min.x
                .min(board.width - bbox.max.x)
                .min(bbox.min.y)
                .min(board.height - bbox.max.y);
            if edge_dist < MIN_EDGE_KEEPOUT - 1e-6 {
                res.edge_keepout_violations.push(DrcViolation::new(
                    DrcViolationKind::EdgeKeepoutViolation,
                    loc,
                    layer,
                    net,
                    None,
                    edge_dist,
                    MIN_EDGE_KEEPOUT,
                    format!(
                        "Copper to board edge clearance violation: {:.3}mm < required {:.3}mm",
                        edge_dist, MIN_EDGE_KEEPOUT
                    ),
                ));
            }
        }
    };

    for track in &board.tracks {
        let half_w = track.width / 2.0;
        let min_x = track.start.x.min(track.end.x) - half_w;
        let max_x = track.start.x.max(track.end.x) + half_w;
        let min_y = track.start.y.min(track.end.y) - half_w;
        let max_y = track.start.y.max(track.end.y) + half_w;
        let bbox = Rect::from_coords(min_x, min_y, max_x, max_y);
        check_bbox_edge(bbox, track.midpoint(), Some(track.layer), Some(track.net), &mut result);
    }

    for via in &board.vias {
        let rad = via.size / 2.0;
        let bbox = Rect::from_coords(via.at.x - rad, via.at.y - rad, via.at.x + rad, via.at.y + rad);
        check_bbox_edge(bbox, via.at, Some(Layer::FCu), Some(via.net), &mut result);
    }

    // ------------------------------------------------------------------------
    // 3. Electrical Shorts & Clearances
    // ------------------------------------------------------------------------

    // A. Track vs Track
    for i in 0..board.tracks.len() {
        let t1 = &board.tracks[i];
        for j in (i + 1)..board.tracks.len() {
            let t2 = &board.tracks[j];
            if t1.net == t2.net || t1.layer != t2.layer {
                continue;
            }

            let (dist, mid) = segment_to_segment_dist(t1.start, t1.end, t2.start, t2.end);
            let surface_dist = dist - (t1.width / 2.0 + t2.width / 2.0);

            if surface_dist <= 1e-5 {
                result.short_circuits.push(DrcViolation::new(
                    DrcViolationKind::ShortCircuit,
                    mid,
                    Some(t1.layer),
                    Some(t1.net),
                    Some(t2.net),
                    surface_dist,
                    0.0,
                    format!("Short circuit between track of net {} and track of net {}", t1.net, t2.net),
                ));
            } else if surface_dist < config.clearance - 1e-5 {
                result.clearance_violations.push(DrcViolation::new(
                    DrcViolationKind::TraceTraceClearance,
                    mid,
                    Some(t1.layer),
                    Some(t1.net),
                    Some(t2.net),
                    surface_dist,
                    config.clearance,
                    format!(
                        "Trace-to-trace clearance violation: {:.3}mm < required {:.3}mm",
                        surface_dist, config.clearance
                    ),
                ));
            }
        }
    }

    // B. Track vs Pad
    for track in &board.tracks {
        for gp in &pads {
            if gp.net == Some(track.net) {
                continue;
            }
            if !layers_share_copper(track.layer, false, gp.layer, gp.is_tht) {
                continue;
            }

            let surface_dist = track_to_pad_surface_dist(track, gp.pad, gp.pos, gp.rot);

            if surface_dist <= 1e-5 {
                result.short_circuits.push(DrcViolation::new(
                    DrcViolationKind::ShortCircuit,
                    gp.pos,
                    Some(track.layer),
                    Some(track.net),
                    gp.net,
                    surface_dist,
                    0.0,
                    format!("Short circuit: track of net {} touches pad of net {:?}", track.net, gp.net),
                ));
            } else if surface_dist < config.clearance - 1e-5 {
                result.clearance_violations.push(DrcViolation::new(
                    DrcViolationKind::TracePadClearance,
                    gp.pos,
                    Some(track.layer),
                    Some(track.net),
                    gp.net,
                    surface_dist,
                    config.clearance,
                    format!(
                        "Trace-to-pad clearance violation: {:.3}mm < required {:.3}mm",
                        surface_dist, config.clearance
                    ),
                ));
            }
        }
    }

    // C. Pad vs Pad
    // Copper clearance applies both within and between packages.
    let min_pad_clearance = config.clearance;

    for i in 0..pads.len() {
        let p1 = &pads[i];
        for j in (i + 1)..pads.len() {
            let p2 = &pads[j];
            if p1.net == p2.net && p1.net.is_some() {
                continue;
            }
            if !layers_share_copper(p1.layer, p1.is_tht, p2.layer, p2.is_tht) {
                continue;
            }

            let surface_dist = pad_to_pad_surface_dist(p1.pad, p1.pos, p1.rot, p2.pad, p2.pos, p2.rot);

            if surface_dist <= 1e-5 {
                result.short_circuits.push(DrcViolation::new(
                    DrcViolationKind::ShortCircuit,
                    p1.pos.midpoint(p2.pos),
                    Some(p1.layer),
                    p1.net,
                    p2.net,
                    surface_dist,
                    0.0,
                    format!("Short circuit: pad of net {:?} overlaps pad of net {:?}", p1.net, p2.net),
                ));
            } else if surface_dist < min_pad_clearance - 1e-5 {
                result.clearance_violations.push(DrcViolation::new(
                    DrcViolationKind::PadPadClearance,
                    p1.pos.midpoint(p2.pos),
                    Some(p1.layer),
                    p1.net,
                    p2.net,
                    surface_dist,
                    min_pad_clearance,
                    format!(
                        "Pad-to-pad clearance violation: {:.3}mm < required {:.3}mm",
                        surface_dist, min_pad_clearance
                    ),
                ));
            }
        }
    }

    // D. Via vs Track
    for via in &board.vias {
        for track in &board.tracks {
            if via.net == track.net {
                continue;
            }
            if !via.connects_layer(track.layer) {
                continue;
            }

            let center_dist = track.distance_to_point(via.at);
            let surface_dist = center_dist - (via.size / 2.0 + track.width / 2.0);

            if surface_dist <= 1e-5 {
                result.short_circuits.push(DrcViolation::new(
                    DrcViolationKind::ShortCircuit,
                    via.at,
                    Some(track.layer),
                    Some(via.net),
                    Some(track.net),
                    surface_dist,
                    0.0,
                    format!("Short circuit: via of net {} collides with track of net {}", via.net, track.net),
                ));
            } else if surface_dist < config.clearance - 1e-5 {
                result.clearance_violations.push(DrcViolation::new(
                    DrcViolationKind::ViaTraceClearance,
                    via.at,
                    Some(track.layer),
                    Some(via.net),
                    Some(track.net),
                    surface_dist,
                    config.clearance,
                    format!(
                        "Via-to-trace clearance violation: {:.3}mm < required {:.3}mm",
                        surface_dist, config.clearance
                    ),
                ));
            }
        }
    }

    // E. Via vs Pad
    for via in &board.vias {
        for gp in &pads {
            if gp.net == Some(via.net) {
                continue;
            }
            if !layers_share_copper(Layer::FCu, true, gp.layer, gp.is_tht) {
                continue;
            }

            let surface_dist = via_to_pad_surface_dist(via, gp.pad, gp.pos, gp.rot);

            if surface_dist <= 1e-5 {
                result.short_circuits.push(DrcViolation::new(
                    DrcViolationKind::ShortCircuit,
                    via.at,
                    Some(gp.layer),
                    Some(via.net),
                    gp.net,
                    surface_dist,
                    0.0,
                    format!("Short circuit: via of net {} overlaps pad of net {:?}", via.net, gp.net),
                ));
            } else if surface_dist < config.clearance - 1e-5 {
                result.clearance_violations.push(DrcViolation::new(
                    DrcViolationKind::ViaPadClearance,
                    via.at,
                    Some(gp.layer),
                    Some(via.net),
                    gp.net,
                    surface_dist,
                    config.clearance,
                    format!(
                        "Via-to-pad clearance violation: {:.3}mm < required {:.3}mm",
                        surface_dist, config.clearance
                    ),
                ));
            }
        }
    }

    // F. Via vs Via
    for i in 0..board.vias.len() {
        let v1 = &board.vias[i];
        for j in (i + 1)..board.vias.len() {
            let v2 = &board.vias[j];
            if v1.net == v2.net {
                continue;
            }

            let center_dist = v1.at.distance_to(v2.at);
            let surface_dist = center_dist - (v1.size / 2.0 + v2.size / 2.0);

            if surface_dist <= 1e-5 {
                result.short_circuits.push(DrcViolation::new(
                    DrcViolationKind::ShortCircuit,
                    v1.at.midpoint(v2.at),
                    Some(Layer::FCu),
                    Some(v1.net),
                    Some(v2.net),
                    surface_dist,
                    0.0,
                    format!("Short circuit: via of net {} overlaps via of net {}", v1.net, v2.net),
                ));
            } else if surface_dist < config.clearance - 1e-5 {
                result.clearance_violations.push(DrcViolation::new(
                    DrcViolationKind::ViaViaClearance,
                    v1.at.midpoint(v2.at),
                    Some(Layer::FCu),
                    Some(v1.net),
                    Some(v2.net),
                    surface_dist,
                    config.clearance,
                    format!(
                        "Via-to-via clearance violation: {:.3}mm < required {:.3}mm",
                        surface_dist, config.clearance
                    ),
                ));
            }
        }
    }

    // ------------------------------------------------------------------------
    // 4. Unrouted Nets Detection (Union-Find / DSU)
    // ------------------------------------------------------------------------
    let mut net_pads: HashMap<NetId, Vec<usize>> = HashMap::new();
    for (idx, gp) in pads.iter().enumerate() {
        if let Some(net) = gp.net {
            if net != UNCONNECTED_NET {
                net_pads.entry(net).or_default().push(idx);
            }
        }
    }

    for (&net_id, pin_indices) in &net_pads {
        if pin_indices.len() < 2 {
            continue; // 0 or 1 pin cannot be unrouted
        }

        let num_pins = pin_indices.len();
        let net_tracks: Vec<&TrackSegment> = board.tracks.iter().filter(|t| t.net == net_id).collect();
        let net_vias: Vec<&Via> = board.vias.iter().filter(|v| v.net == net_id).collect();

        // Node indexing:
        // 0..num_pins: Pins
        // num_pins..(num_pins + num_tracks): Tracks
        // (num_pins + num_tracks)..(total): Vias
        let total_nodes = num_pins + net_tracks.len() + net_vias.len();
        let mut dsu = DisjointSet::new(total_nodes);

        // Connect tracks to pins
        for (t_idx, &track) in net_tracks.iter().enumerate() {
            let track_node = num_pins + t_idx;
            for (p_sub_idx, &p_idx) in pin_indices.iter().enumerate() {
                let gp = &pads[p_idx];
                if layers_share_copper(track.layer, false, gp.layer, gp.is_tht) {
                    let d = track_to_pad_surface_dist(track, gp.pad, gp.pos, gp.rot);
                    if d <= 1e-4 {
                        dsu.union(track_node, p_sub_idx);
                    }
                }
            }
        }

        // Connect vias to pins
        for (v_idx, &via) in net_vias.iter().enumerate() {
            let via_node = num_pins + net_tracks.len() + v_idx;
            for (p_sub_idx, &p_idx) in pin_indices.iter().enumerate() {
                let gp = &pads[p_idx];
                if layers_share_copper(Layer::FCu, true, gp.layer, gp.is_tht) {
                    let d = via_to_pad_surface_dist(via, gp.pad, gp.pos, gp.rot);
                    if d <= 1e-4 {
                        dsu.union(via_node, p_sub_idx);
                    }
                }
            }
        }

        // Connect tracks to tracks
        for t1_idx in 0..net_tracks.len() {
            let t1 = net_tracks[t1_idx];
            let node1 = num_pins + t1_idx;
            for t2_idx in (t1_idx + 1)..net_tracks.len() {
                let t2 = net_tracks[t2_idx];
                let node2 = num_pins + t2_idx;
                if t1.layer == t2.layer {
                    let (dist, _) = segment_to_segment_dist(t1.start, t1.end, t2.start, t2.end);
                    if dist <= (t1.width / 2.0 + t2.width / 2.0 + 1e-4) {
                        dsu.union(node1, node2);
                    }
                }
            }
        }

        // Connect tracks to vias
        for (t_idx, &track) in net_tracks.iter().enumerate() {
            let track_node = num_pins + t_idx;
            for (v_idx, &via) in net_vias.iter().enumerate() {
                let via_node = num_pins + net_tracks.len() + v_idx;
                if via.connects_layer(track.layer) {
                    let center_d = track.distance_to_point(via.at);
                    if center_d <= (track.width / 2.0 + via.size / 2.0 + 1e-4) {
                        dsu.union(track_node, via_node);
                    }
                }
            }
        }

        // Connect vias to vias
        for v1_idx in 0..net_vias.len() {
            let v1 = net_vias[v1_idx];
            let node1 = num_pins + net_tracks.len() + v1_idx;
            for v2_idx in (v1_idx + 1)..net_vias.len() {
                let v2 = net_vias[v2_idx];
                let node2 = num_pins + net_tracks.len() + v2_idx;
                if v1.at.distance_to(v2.at) <= (v1.size / 2.0 + v2.size / 2.0 + 1e-4) {
                    dsu.union(node1, node2);
                }
            }
        }

        // Check if all pins of this net are in the same set
        let root0 = dsu.find(0);
        let all_connected = (1..num_pins).all(|p| dsu.find(p) == root0);

        if !all_connected {
            result.unrouted_nets.push(net_id);
        }
    }

    result.unrouted_nets.sort_unstable();
    result.unrouted_nets.dedup();

    result
}

/// Disjoint Set Union (Union-Find) helper.
struct DisjointSet {
    parent: Vec<usize>,
    rank: Vec<usize>,
}

impl DisjointSet {
    fn new(size: usize) -> Self {
        Self {
            parent: (0..size).collect(),
            rank: vec![0; size],
        }
    }

    fn find(&mut self, x: usize) -> usize {
        if self.parent[x] != x {
            self.parent[x] = self.find(self.parent[x]);
        }
        self.parent[x]
    }

    fn union(&mut self, x: usize, y: usize) {
        let root_x = self.find(x);
        let root_y = self.find(y);
        if root_x == root_y {
            return;
        }
        if self.rank[root_x] < self.rank[root_y] {
            self.parent[root_x] = root_y;
        } else if self.rank[root_x] > self.rank[root_y] {
            self.parent[root_y] = root_x;
        } else {
            self.parent[root_y] = root_x;
            self.rank[root_x] += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_drc_result_is_clean_and_total_violations() {
        let mut res = DrcResult::new();
        assert!(res.is_clean());
        assert_eq!(res.total_violations(), 0);

        res.unrouted_nets.push(1);
        assert!(!res.is_clean());
        assert_eq!(res.total_violations(), 1);
    }

    #[test]
    fn test_segment_to_segment_dist_parallel_and_perpendicular() {
        // Parallel segments separated by 2.0 along Y
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(10.0, 0.0);
        let q1 = Point::new(0.0, 2.0);
        let q2 = Point::new(10.0, 2.0);
        let (d, _) = segment_to_segment_dist(p1, p2, q1, q2);
        assert!((d - 2.0).abs() < 1e-9);

        // Perpendicular crossing segments intersecting at (5.0, 5.0)
        let s1 = Point::new(0.0, 5.0);
        let s2 = Point::new(10.0, 5.0);
        let t1 = Point::new(5.0, 0.0);
        let t2 = Point::new(5.0, 10.0);
        let (d2, mid) = segment_to_segment_dist(s1, s2, t1, t2);
        assert!(d2 < 1e-9);
        assert!((mid.x - 5.0).abs() < 1e-9);
        assert!((mid.y - 5.0).abs() < 1e-9);
    }

    #[test]
    fn test_point_to_oriented_rect_dist_inside_and_outside() {
        let center = Point::new(10.0, 10.0);
        // Inside
        let (d_in, _) = point_to_oriented_rect_dist(Point::new(10.0, 10.0), center, 4.0, 2.0, 0.0);
        assert!(d_in < 1e-9);

        // Outside 3.0mm to the right: edge is at X = 12.0, point at X = 15.0 -> dist = 3.0
        let (d_out, _) = point_to_oriented_rect_dist(Point::new(15.0, 10.0), center, 4.0, 2.0, 0.0);
        assert!((d_out - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_via_to_via_short_and_clearance_detection() {
        let mut board = PcbBoard::new(50.0, 50.0);
        board.register_net(1, "N1");
        board.register_net(2, "N2");

        // Vias directly overlapping
        board.add_via(Via::new(Point::new(10.0, 10.0), 0.8, 0.4, 1));
        board.add_via(Via::new(Point::new(10.0, 10.0), 0.8, 0.4, 2));

        let config = RouterConfig::default();
        let drc = run_drc(&board, &config);
        assert!(!drc.short_circuits.is_empty());
    }
}
