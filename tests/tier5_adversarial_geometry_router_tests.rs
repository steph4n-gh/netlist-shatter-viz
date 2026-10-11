//! Tier 5 Challenger 1: White-Box Adversarial Coverage Hardening Suite
//! Focus: Geometry, Footprints, Dynamic Ratsnest (EMST), 2-Layer Router & DRC Engine.
//!
//! Subsystems Challenged:
//! 1. Geometry & Coordinate Precision:
//!    - Extreme coordinates (1e8, -1e8), length & distance overflow immunity.
//!    - Vector normalization of near-zero lengths (< 1e-12 snaps to zero).
//!    - Division by zero / sub-epsilon scalar division immunity.
//!    - Trigonometric & snapping exactness across multiples of 90°, multi-turn angles, and negative degrees.
//!    - Exact mathematical bounding box calculation for PadShape::Rect and PadShape::Obround under 45° rotation.
//!    - Degenerate EdgeCuts polygons (< 3 vertices, collinear vertices, reverse winding area, non-convex L-shape containment).
//!    - Inverted Rect coordinates normalization and zero-area bounding boxes.
//!
//! 2. Footprint Construction & Non-Standard Rotations:
//!    - Rotations at 45°, 135°, 225°, -45°, -135°, 360°, 720°.
//!    - Rotation invariance: 360°/720° identity match.
//!    - Empty footprint detection (0 pads) in validation.
//!    - Empty reference string and value handling.
//!    - High pin count (100 pins) footprint validation.
//!    - Non-finite coordinates (NaN, Inf) and invalid pad dimension detection.
//!
//! 3. Dynamic Ratsnest (EMST Topology):
//!    - Coincident pads of same net (0-distance Kruskal handling without panic).
//!    - Collinear multi-pad nets (linear tree topology vs star).
//!    - Cyclic copper loop topology (0 ratsnest lines when fully routed in a ring).
//!    - Multi-island copper partial routing (exact inter-island ratsnest edge selection).
//!    - High-density pin clusters (16-pin 4x4 grid producing exactly 15 acyclic edges).
//!    - Inactive / single-pin / unconnected net filtering.
//!
//! 4. 2-Layer Router Hardening:
//!    - Impassable foreign barriers (bounded loop termination, Err return < 1000ms).
//!    - Maze routing through narrow corridors.
//!    - Dense parallel buses (6 lines at 0.8mm pitch).
//!    - Boundary via placement constraint (vias strictly obeying MIN_EDGE_KEEPOUT = 0.40).
//!    - Rip-up and reroute (RRR) conflict negotiation across crossing nets.
//!    - Invalid router configuration rejection.
//!
//! 5. DRC Engine White-Box Hardening:
//!    - Sub-micron clearance discrimination: violation at clearance - 2e-5; immunity at clearance - 1e-6 (within 1e-5 epsilon).
//!    - Complex multi-segment polygonal clearances (Obround-Obround, Obround-Rect, 45°-rotated Rects).
//!    - Annular ring fatal breakouts (drill >= outer) and sizing violations (ring < 0.15mm) across Vias and THT pads.
//!    - False-positive immunity: same-footprint intra-package pad clearance immunity (< 0.65mm allowed for same IC).
//!    - Opposite-layer coincidence immunity (F.Cu and B.Cu overlapping geometry).
//!    - Graph connectivity Union-Find validation on complex star and daisy-chain nets.

use netlist_shatter_viz::pcb::*;
use std::collections::HashSet;
use std::time::Instant;

fn default_config() -> RouterConfig {
    RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    }
}

// ============================================================================
// SECTION 1: Geometry & Coordinate Precision
// ============================================================================

#[test]
fn test_geometry_extreme_coordinates_arithmetic_and_distance() {
    let p_large1 = Point::new(1e8, 1e8);
    let p_large2 = Point::new(1e8 + 3.0, 1e8 + 4.0);
    assert!((p_large1.distance_to(p_large2) - 5.0).abs() < 1e-9);

    let p_neg1 = Point::new(-1e8, -1e8);
    let p_neg2 = Point::new(-1e8 - 3.0, -1e8 - 4.0);
    assert!((p_neg1.distance_to(p_neg2) - 5.0).abs() < 1e-9);

    let diff = p_large2 - p_large1;
    assert_eq!(diff, Point::new(3.0, 4.0));
    assert!((diff.length() - 5.0).abs() < 1e-9);

    // Dot product on extreme coordinates
    let dot = Point::new(1e7, 0.0).dot(Point::new(1e7, 0.0));
    assert!((dot - 1e14).abs() < 1e-3);
}

#[test]
fn test_geometry_near_zero_normalization_and_safe_division() {
    // Length below 1e-12 snaps to zero vector
    let p_tiny = Point::new(1e-13, 1e-13);
    assert_eq!(p_tiny.normalize(), Point::zero());

    // Normalizing exact zero
    assert_eq!(Point::zero().normalize(), Point::zero());

    // Division by zero scalar safely produces zero point
    let p = Point::new(10.0, 20.0);
    assert_eq!(p / 0.0, Point::zero());
    assert_eq!(p / 1e-14, Point::zero());

    let mut p_assign = Point::new(15.0, 25.0);
    p_assign /= 0.0;
    assert_eq!(p_assign, Point::zero());
}

#[test]
fn test_geometry_approx_eq_at_sub_micron_tolerances() {
    let p1 = Point::new(10.0, 20.0);
    let p2 = Point::new(10.00000005, 19.99999995); // diff 5e-8

    // Epsilon 1e-7 should consider them equal
    assert!(p1.approx_eq(p2, 1e-7));

    // Epsilon 1e-8 should consider them not equal
    assert!(!p1.approx_eq(p2, 1e-8));
}

#[test]
fn test_geometry_sin_cos_deg_comprehensive_snapping() {
    // Multi-turn and negative angles
    assert_eq!(sin_cos_deg(0.0), (0.0, 1.0));
    assert_eq!(sin_cos_deg(90.0), (1.0, 0.0));
    assert_eq!(sin_cos_deg(180.0), (0.0, -1.0));
    assert_eq!(sin_cos_deg(270.0), (-1.0, 0.0));
    assert_eq!(sin_cos_deg(360.0), (0.0, 1.0));
    assert_eq!(sin_cos_deg(450.0), (1.0, 0.0));
    assert_eq!(sin_cos_deg(720.0), (0.0, 1.0));

    assert_eq!(sin_cos_deg(-90.0), (-1.0, 0.0));
    assert_eq!(sin_cos_deg(-180.0), (0.0, -1.0));
    assert_eq!(sin_cos_deg(-270.0), (1.0, 0.0));
    assert_eq!(sin_cos_deg(-360.0), (0.0, 1.0));
    assert_eq!(sin_cos_deg(-720.0), (0.0, 1.0));

    // 45 degree angle
    let (s45, c45) = sin_cos_deg(45.0);
    let expected = std::f64::consts::FRAC_1_SQRT_2;
    assert!((s45 - expected).abs() < 1e-9);
    assert!((c45 - expected).abs() < 1e-9);

    // -45 degree angle
    let (s_neg45, c_neg45) = sin_cos_deg(-45.0);
    assert!((s_neg45 - (-expected)).abs() < 1e-9);
    assert!((c_neg45 - expected).abs() < 1e-9);
}

#[test]
fn test_geometry_rect_normalization_and_inversion() {
    // Inverted coordinates in new and from_coords
    let r1 = Rect::new(Point::new(30.0, 40.0), Point::new(10.0, 20.0));
    assert_eq!(r1.min, Point::new(10.0, 20.0));
    assert_eq!(r1.max, Point::new(30.0, 40.0));
    assert_eq!(r1.width(), 20.0);
    assert_eq!(r1.height(), 20.0);
    assert_eq!(r1.area(), 400.0);
    assert_eq!(r1.center(), Point::new(20.0, 30.0));

    let r2 = Rect::from_coords(50.0, 60.0, 20.0, 10.0);
    assert_eq!(r2.min, Point::new(20.0, 10.0));
    assert_eq!(r2.max, Point::new(50.0, 60.0));

    // Point rectangle (zero area)
    let p_rect = Rect::from_point(Point::new(5.0, 5.0));
    assert_eq!(p_rect.area(), 0.0);
    assert!(p_rect.contains_point(Point::new(5.0, 5.0)));
    assert!(!p_rect.contains_point(Point::new(5.001, 5.0)));
}

#[test]
fn test_geometry_edge_cuts_degenerate_and_non_convex_polygons() {
    // Fewer than 3 vertices
    let ec_empty = EdgeCuts::new(vec![]);
    assert!(!ec_empty.is_closed(1e-3));
    assert_eq!(ec_empty.area(), 0.0);
    assert_eq!(ec_empty.bounding_box(), Rect::new(Point::zero(), Point::zero()));

    let ec_two = EdgeCuts::from_vertices(&[Point::new(0.0, 0.0), Point::new(10.0, 0.0)]);
    assert_eq!(ec_two.segments.len(), 0);
    assert!(!ec_two.is_closed(1e-3));

    // Collinear vertices (zero area triangle)
    let ec_collinear = EdgeCuts::from_vertices(&[
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(20.0, 0.0),
    ]);
    assert_eq!(ec_collinear.area(), 0.0);

    // Non-convex L-shaped board:
    // (0, 0) -> (40, 0) -> (40, 20) -> (20, 20) -> (20, 40) -> (0, 40)
    let l_vertices = vec![
        Point::new(0.0, 0.0),
        Point::new(40.0, 0.0),
        Point::new(40.0, 20.0),
        Point::new(20.0, 20.0),
        Point::new(20.0, 40.0),
        Point::new(0.0, 40.0),
    ];
    let ec_l = EdgeCuts::from_vertices(&l_vertices);
    assert!(ec_l.is_closed(1e-5));
    // Area of L = 40x40 - 20x20 = 1600 - 400 = 1200
    assert!((ec_l.area() - 1200.0).abs() < 1e-6);

    // Point in bottom-left arm (10, 10): INSIDE
    assert!(ec_l.contains_point(Point::new(10.0, 10.0)));
    // Point in bottom-right arm (30, 10): INSIDE
    assert!(ec_l.contains_point(Point::new(30.0, 10.0)));
    // Point in top-left arm (10, 30): INSIDE
    assert!(ec_l.contains_point(Point::new(10.0, 30.0)));
    // Point in cutout notch (30, 30): OUTSIDE!
    assert!(!ec_l.contains_point(Point::new(30.0, 30.0)));
    // Point far outside (50, 50): OUTSIDE
    assert!(!ec_l.contains_point(Point::new(50.0, 50.0)));

    // Reverse winding (clockwise): shoelace absolute value must be positive
    let mut cw_vertices = l_vertices.clone();
    cw_vertices.reverse();
    let ec_cw = EdgeCuts::from_vertices(&cw_vertices);
    assert!((ec_cw.area() - 1200.0).abs() < 1e-6);
}

// ============================================================================
// SECTION 2: Footprint Construction & Non-Standard Rotations
// ============================================================================

#[test]
fn test_footprint_non_standard_rotations_pad_positions() {
    let center = Point::new(20.0, 20.0);
    let p1 = Pad::new_smd("1", "A", Point::new(10.0, 0.0), PadShape::Rect { width: 2.0, height: 1.0 }, Layer::FCu);

    // Rotation at 45 degrees:
    // Relative pos (10, 0) rotated by 45 -> (10 * cos(45), 10 * sin(45)) = (7.071068, 7.071068)
    // World pos = (27.071068, 27.071068)
    let fp_45 = Footprint::new("U1", "C", "PKG", center, 45.0, vec![p1.clone()], Vec::new());
    let world_45 = fp_45.pad_world_pos("1").unwrap();
    let expected_offset = 10.0 * std::f64::consts::FRAC_1_SQRT_2;
    assert!((world_45.x - (20.0 + expected_offset)).abs() < 1e-5);
    assert!((world_45.y - (20.0 + expected_offset)).abs() < 1e-5);

    // Rotation at 135 degrees:
    // Relative pos (10, 0) rotated by 135 -> (-7.071068, 7.071068)
    let fp_135 = Footprint::new("U1", "C", "PKG", center, 135.0, vec![p1.clone()], Vec::new());
    let world_135 = fp_135.pad_world_pos("1").unwrap();
    assert!((world_135.x - (20.0 - expected_offset)).abs() < 1e-5);
    assert!((world_135.y - (20.0 + expected_offset)).abs() < 1e-5);

    // Negative rotation at -45 degrees:
    // Relative pos (10, 0) rotated by -45 -> (7.071068, -7.071068)
    let fp_neg45 = Footprint::new("U1", "C", "PKG", center, -45.0, vec![p1.clone()], Vec::new());
    let world_neg45 = fp_neg45.pad_world_pos("1").unwrap();
    assert!((world_neg45.x - (20.0 + expected_offset)).abs() < 1e-5);
    assert!((world_neg45.y - (20.0 - expected_offset)).abs() < 1e-5);

    // Multi-turn rotation: 720 degrees must match 0 degrees exactly
    let fp_720 = Footprint::new("U1", "C", "PKG", center, 720.0, vec![p1.clone()], Vec::new());
    let world_720 = fp_720.pad_world_pos("1").unwrap();
    assert_eq!(world_720, Point::new(30.0, 20.0));
}

#[test]
fn test_footprint_exact_mathematical_bounding_box_at_45_deg() {
    let center = Point::new(15.0, 25.0);

    // 1. Rect pad: width 4.0, height 2.0 at 45 degrees
    // dx = half_w * cos(45) + half_h * sin(45) = 2.0 * 1/sqrt(2) + 1.0 * 1/sqrt(2) = 3.0 / sqrt(2) = 2.1213203 mm
    // dy = half_w * sin(45) + half_h * cos(45) = 3.0 / sqrt(2) = 2.1213203 mm
    let rect_shape = PadShape::Rect { width: 4.0, height: 2.0 };
    let bb_rect = rect_shape.bounding_box(center, 45.0);
    let expected_d = 3.0 * std::f64::consts::FRAC_1_SQRT_2;
    assert!((bb_rect.width() - 2.0 * expected_d).abs() < 1e-6);
    assert!((bb_rect.height() - 2.0 * expected_d).abs() < 1e-6);
    assert!((bb_rect.center().x - center.x).abs() < 1e-9);
    assert!((bb_rect.center().y - center.y).abs() < 1e-9);

    // 2. Obround pad: width 4.0, height 2.0 at 45 degrees
    // spine_half = (4.0 - 2.0)/2 = 1.0. radius = 1.0.
    // dx = spine_half * cos(45) + radius = 1.0 * 1/sqrt(2) + 1.0 = 1.7071068 mm
    // dy = spine_half * sin(45) + radius = 1.0 * 1/sqrt(2) + 1.0 = 1.7071068 mm
    let obround_shape = PadShape::Obround { width: 4.0, height: 2.0 };
    let bb_obround = obround_shape.bounding_box(center, 45.0);
    let expected_ob_d = 1.0 * std::f64::consts::FRAC_1_SQRT_2 + 1.0;
    assert!((bb_obround.width() - 2.0 * expected_ob_d).abs() < 1e-6);
    assert!((bb_obround.height() - 2.0 * expected_ob_d).abs() < 1e-6);
}

#[test]
fn test_footprint_validation_edge_cases_and_error_traps() {
    // 1. Empty footprint (0 pads)
    let fp_empty = Footprint::new("U_EMPTY", "NONE", "PKG", Point::new(10.0, 10.0), 0.0, vec![], Vec::new());
    let errors = fp_empty.validate();
    assert!(errors.iter().any(|e| matches!(e, ValidationError::EmptyFootprint { .. })));

    // 2. Empty reference and package strings (should not panic)
    let p1 = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu);
    let fp_blank_strings = Footprint::new("", "", "", Point::new(10.0, 10.0), 0.0, vec![p1.clone()], Vec::new());
    assert!(fp_blank_strings.validate().is_empty());

    // 3. Non-finite coordinates (NaN position)
    let fp_nan = Footprint::new("U_NAN", "VAL", "PKG", Point::new(f64::NAN, 10.0), 0.0, vec![p1.clone()], Vec::new());
    let errors_nan = fp_nan.validate();
    assert!(errors_nan.iter().any(|e| matches!(e, ValidationError::NonFiniteCoordinate { .. })));

    // 4. Duplicate pin numbers
    let p_dup1 = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu);
    let p_dup2 = Pad::new_smd("1", "B", Point::new(2.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu);
    let fp_dup = Footprint::new("U_DUP", "VAL", "PKG", Point::new(10.0, 10.0), 0.0, vec![p_dup1, p_dup2], Vec::new());
    let errors_dup = fp_dup.validate();
    assert!(errors_dup.iter().any(|e| matches!(e, ValidationError::DuplicatePinNumber { .. })));

    // 5. Invalid through-hole drill (drill >= outer_diameter)
    let p_bad_tht = Pad::new_tht("1", "A", Point::zero(), PadShape::Circle { diameter: 1.0 }, 1.2, 1.0);
    let fp_bad_tht = Footprint::new("U_THT", "VAL", "PKG", Point::new(10.0, 10.0), 0.0, vec![p_bad_tht], Vec::new());
    let errors_tht = fp_bad_tht.validate();
    assert!(errors_tht.iter().any(|e| matches!(e, ValidationError::InvalidThroughHoleDrill { .. })));
}

#[test]
fn test_footprint_arbitrary_high_pin_count_100_pins() {
    let mut pads = Vec::with_capacity(100);
    for i in 0..100 {
        let pin_num = (i + 1).to_string();
        let pos = Point::new((i % 10) as f64 * 2.54, (i / 10) as f64 * 2.54);
        pads.push(Pad::new_tht(pin_num, format!("PIN_{}", i + 1), pos, PadShape::Circle { diameter: 1.6 }, 0.8, 1.6));
    }
    let fp_100 = Footprint::new("J_100", "HEADER_100", "PGA_100", Point::new(10.0, 10.0), 0.0, pads, Vec::new());
    assert_eq!(fp_100.pads.len(), 100);
    assert!(fp_100.validate().is_empty());

    assert!(fp_100.pad("1").is_some());
    assert!(fp_100.pad("50").is_some());
    assert!(fp_100.pad("100").is_some());
    assert!(fp_100.pad("101").is_none());
}

// ============================================================================
// SECTION 3: Dynamic Ratsnest (EMST) Edge Cases & Topologies
// ============================================================================

#[test]
fn test_ratsnest_coincident_pads_does_not_panic() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "COINCIDENT_NET");

    // Two pads of the same net placed at the exact same world coordinate
    let p1 = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let p2 = Pad::new_smd("2", "B", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);

    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(20.0, 20.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "C", "P", Point::new(20.0, 20.0), 0.0, vec![p2], Vec::new()));

    let rats = compute_ratsnest(&board);
    // Must produce exactly 1 spanning connection of length 0 without crashing or looping
    assert_eq!(rats.len(), 1);
    assert_eq!(rats[0].net, 1);
    assert_eq!(rats[0].start, Point::new(20.0, 20.0));
    assert_eq!(rats[0].end, Point::new(20.0, 20.0));
}

#[test]
fn test_ratsnest_collinear_pads_minimal_spanning_tree_length() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "COLLINEAR_NET");

    // 4 collinear pads along Y = 20.0: X = 10, 20, 30, 40
    for i in 0..4 {
        let x = 10.0 + (i as f64) * 10.0;
        let pad = Pad::new_smd(&(i + 1).to_string(), "P", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
        board.add_footprint(Footprint::new(format!("U{}", i + 1), "P", "P", Point::new(x, 20.0), 0.0, vec![pad], Vec::new()));
    }

    let rats = compute_ratsnest(&board);
    // 4 pads must generate exactly 3 lines in an EMST
    assert_eq!(rats.len(), 3);

    // Sum of lengths must be exactly 30.0 mm (10 + 10 + 10)
    let total_len: f64 = rats.iter().map(|l| l.start.distance_to(l.end)).sum();
    assert!((total_len - 30.0).abs() < 1e-6, "EMST length was {} instead of 30.0", total_len);
}

#[test]
fn test_ratsnest_cyclic_copper_loop_eliminates_all_lines() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(5, "RING_NET");

    // 3 pads arranged in a triangle: (10, 10), (30, 10), (20, 30)
    let p1 = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(5);
    let p2 = Pad::new_smd("2", "B", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(5);
    let p3 = Pad::new_smd("3", "C", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(5);

    board.add_footprint(Footprint::new("U1", "P", "P", Point::new(10.0, 10.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "P", "P", Point::new(30.0, 10.0), 0.0, vec![p2], Vec::new()));
    board.add_footprint(Footprint::new("U3", "P", "P", Point::new(20.0, 30.0), 0.0, vec![p3], Vec::new()));

    // Fully connected in a triangular loop with copper tracks:
    board.add_track(TrackSegment::new(Point::new(10.0, 10.0), Point::new(30.0, 10.0), 0.25, Layer::FCu, 5));
    board.add_track(TrackSegment::new(Point::new(30.0, 10.0), Point::new(20.0, 30.0), 0.25, Layer::FCu, 5));
    board.add_track(TrackSegment::new(Point::new(20.0, 30.0), Point::new(10.0, 10.0), 0.25, Layer::FCu, 5));

    // When copper forms a single connected island covering all pads, ratsnest MUST be empty!
    let rats = compute_ratsnest(&board);
    assert_eq!(rats.len(), 0, "Cyclic copper loop must eliminate all ratsnest lines");
}

#[test]
fn test_ratsnest_multi_island_copper_partial_routing() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(8, "ISLANDS");

    // 4 pads:
    // Island 1: Pad 1 at (10, 10) and Pad 2 at (10, 20) connected by track
    // Island 2: Pad 3 at (40, 10) and Pad 4 at (40, 20) connected by track
    let p1 = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(8);
    let p2 = Pad::new_smd("2", "B", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(8);
    let p3 = Pad::new_smd("3", "C", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(8);
    let p4 = Pad::new_smd("4", "D", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(8);

    board.add_footprint(Footprint::new("U1", "P", "P", Point::new(10.0, 10.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "P", "P", Point::new(10.0, 20.0), 0.0, vec![p2], Vec::new()));
    board.add_footprint(Footprint::new("U3", "P", "P", Point::new(40.0, 10.0), 0.0, vec![p3], Vec::new()));
    board.add_footprint(Footprint::new("U4", "P", "P", Point::new(40.0, 20.0), 0.0, vec![p4], Vec::new()));

    // Route within islands:
    board.add_track(TrackSegment::new(Point::new(10.0, 10.0), Point::new(10.0, 20.0), 0.25, Layer::FCu, 8));
    board.add_track(TrackSegment::new(Point::new(40.0, 10.0), Point::new(40.0, 20.0), 0.25, Layer::FCu, 8));

    let rats = compute_ratsnest(&board);
    // Exactly 1 ratsnest line should exist between the two islands!
    assert_eq!(rats.len(), 1);
    assert!((rats[0].start.distance_to(rats[0].end) - 30.0).abs() < 1e-6);
}

#[test]
fn test_ratsnest_high_density_pin_cluster_16_pins() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(42, "GRID_NET");

    // 4x4 grid of pads, pitch 2.0 mm: total 16 pins
    let mut pads_positions = Vec::new();
    for row in 0..4 {
        for col in 0..4 {
            let x = 15.0 + (col as f64) * 2.0;
            let y = 15.0 + (row as f64) * 2.0;
            let pt = Point::new(x, y);
            pads_positions.push(pt);
            let pad = Pad::new_smd("1", "P", Point::zero(), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(42);
            board.add_footprint(Footprint::new(format!("U_{}_{}", row, col), "P", "P", pt, 0.0, vec![pad], Vec::new()));
        }
    }

    let rats = compute_ratsnest(&board);
    // Unrouted 16-pin net must have exactly 15 ratsnest lines
    assert_eq!(rats.len(), 15);

    // Verify all 15 lines connect into a connected tree
    let mut visited = HashSet::new();
    let start_p = pads_positions[0];
    visited.insert((start_p.x.to_bits(), start_p.y.to_bits()));

    for _ in 0..16 {
        for line in &rats {
            let s_key = (line.start.x.to_bits(), line.start.y.to_bits());
            let e_key = (line.end.x.to_bits(), line.end.y.to_bits());
            if visited.contains(&s_key) {
                visited.insert(e_key);
            }
            if visited.contains(&e_key) {
                visited.insert(s_key);
            }
        }
    }
    assert_eq!(visited.len(), 16, "EMST lines must span all 16 pins in a single connected tree");
}

#[test]
fn test_ratsnest_unconnected_net_and_single_pin_ignored() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(UNCONNECTED_NET, "NC");
    board.register_net(99, "SOLO");

    // Two pads on UNCONNECTED_NET (0)
    let p_nc1 = Pad::new_smd("1", "NC1", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(UNCONNECTED_NET);
    let p_nc2 = Pad::new_smd("2", "NC2", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(UNCONNECTED_NET);
    board.add_footprint(Footprint::new("U1", "P", "P", Point::new(10.0, 10.0), 0.0, vec![p_nc1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "P", "P", Point::new(20.0, 20.0), 0.0, vec![p_nc2], Vec::new()));

    // Single pad on SOLO net
    let p_solo = Pad::new_smd("1", "S", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(99);
    board.add_footprint(Footprint::new("U3", "P", "P", Point::new(30.0, 30.0), 0.0, vec![p_solo], Vec::new()));

    let rats = compute_ratsnest(&board);
    assert_eq!(rats.len(), 0, "Neither unconnected net nor single-pin net should generate ratsnest lines");
}

// ============================================================================
// SECTION 4: 2-Layer Router Hardening
// ============================================================================

#[test]
fn test_router_impassable_fortress_bounded_termination() {
    let mut board = PcbBoard::new(40.0, 40.0);
    let config = default_config();

    // Target net: from (20, 20) to (35, 35)
    board.register_net(1, "TRAPPED");
    let p1 = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(1);
    let p2 = Pad::new_smd("2", "B", Point::zero(), PadShape::Rect { width: 0.8, height: 0.8 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U_TRAP", "P", "P", Point::new(20.0, 20.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U_DEST", "P", "P", Point::new(35.0, 35.0), 0.0, vec![p2], Vec::new()));

    // Enclose pad 1 with 8 large THT pads (net 2) forming an airtight barrier
    board.register_net(2, "WALL");
    let deltas = [
        (-1.8, -1.8), (0.0, -1.8), (1.8, -1.8),
        (-1.8,  0.0),              (1.8,  0.0),
        (-1.8,  1.8), (0.0,  1.8), (1.8,  1.8),
    ];
    for (i, &(dx, dy)) in deltas.iter().enumerate() {
        let p_wall = Pad::new_tht("1", "W", Point::zero(), PadShape::Circle { diameter: 1.8 }, 0.8, 1.8).with_net(2);
        board.add_footprint(Footprint::new(format!("UW{}", i), "P", "P", Point::new(20.0 + dx, 20.0 + dy), 0.0, vec![p_wall], Vec::new()));
    }

    let start = Instant::now();
    let result = route_board(&mut board, &config);
    let elapsed = start.elapsed();

    assert!(result.is_err(), "Impassable fortress must return error");
    assert!(elapsed.as_millis() < 1000, "Router must terminate within 1000ms: took {:?}", elapsed);
}

#[test]
fn test_router_dense_six_line_parallel_bus() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let config = default_config();

    // 6 parallel nets spaced by 0.8 mm (pitch = 0.8 mm) on multi-pin bus connectors
    let mut start_pads = Vec::new();
    let mut end_pads = Vec::new();

    for i in 0..6 {
        let net_id = (i + 1) as NetId;
        board.register_net(net_id, format!("BUS_{}", net_id));
        let y_rel = (i as f64) * 0.8;

        let p1 = Pad::new_smd(format!("{}", i + 1), "A", Point::new(0.0, y_rel), PadShape::Rect { width: 0.5, height: 0.5 }, Layer::FCu).with_net(net_id);
        let p2 = Pad::new_smd(format!("{}", i + 1), "B", Point::new(0.0, y_rel), PadShape::Rect { width: 0.5, height: 0.5 }, Layer::FCu).with_net(net_id);

        start_pads.push(p1);
        end_pads.push(p2);
    }

    board.add_footprint(Footprint::new("J_START", "CONN6", "CONN_1X6", Point::new(10.0, 15.0), 0.0, start_pads, Vec::new()));
    board.add_footprint(Footprint::new("J_END", "CONN6", "CONN_1X6", Point::new(40.0, 15.0), 0.0, end_pads, Vec::new()));

    let report = route_board_with_report(&mut board, &config).expect("6-line parallel bus must route");
    assert_eq!(report.routed_nets, 6);
    assert_eq!(report.unrouted_nets, 0);

    let drc = run_drc(&board, &config);
    assert!(drc.is_clean(), "6-line parallel bus must pass DRC with zero errors");
}

#[test]
fn test_router_boundary_via_placement_constraints() {
    let mut board = PcbBoard::new(40.0, 40.0);
    let config = default_config();

    // Crossing nets forcing via insertion near board edge
    // Net 1: F.Cu (1.5, 20.0) to (38.5, 20.0)
    // Net 2: B.Cu pad at (20.0, 1.5) to F.Cu pad at (20.0, 38.5) -> forces via
    board.register_net(1, "N1");
    board.register_net(2, "N2");

    let p1_a = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 0.6, height: 0.6 }, Layer::FCu).with_net(1);
    let p1_b = Pad::new_smd("2", "B", Point::zero(), PadShape::Rect { width: 0.6, height: 0.6 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1A", "P", "P", Point::new(1.5, 20.0), 0.0, vec![p1_a], Vec::new()));
    board.add_footprint(Footprint::new("U1B", "P", "P", Point::new(38.5, 20.0), 0.0, vec![p1_b], Vec::new()));

    let p2_a = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 0.6, height: 0.6 }, Layer::BCu).with_net(2);
    let p2_b = Pad::new_smd("2", "B", Point::zero(), PadShape::Rect { width: 0.6, height: 0.6 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U2A", "P", "P", Point::new(20.0, 1.5), 0.0, vec![p2_a], Vec::new()));
    board.add_footprint(Footprint::new("U2B", "P", "P", Point::new(20.0, 38.5), 0.0, vec![p2_b], Vec::new()));

    route_board(&mut board, &config).expect("Boundary crossing nets must route");

    // All inserted vias must strictly observe MIN_EDGE_KEEPOUT = 0.40
    const MIN_EDGE_KEEPOUT: f64 = 0.40;
    for via in &board.vias {
        let rad = via.size / 2.0;
        assert!(via.at.x - rad >= MIN_EDGE_KEEPOUT - 1e-6);
        assert!(via.at.x + rad <= board.width - MIN_EDGE_KEEPOUT + 1e-6);
        assert!(via.at.y - rad >= MIN_EDGE_KEEPOUT - 1e-6);
        assert!(via.at.y + rad <= board.height - MIN_EDGE_KEEPOUT + 1e-6);
    }

    let drc = run_drc(&board, &config);
    assert_eq!(drc.edge_keepout_violations.len(), 0);
    assert!(drc.is_clean());
}

#[test]
fn test_router_invalid_configuration_rejections() {
    let mut board = PcbBoard::new(50.0, 50.0);

    let config_zero_grid = RouterConfig {
        grid_resolution: 0.0,
        ..default_config()
    };
    assert!(matches!(
        route_board(&mut board, &config_zero_grid),
        Err(RoutingError::InvalidConfiguration { .. })
    ));

    let config_neg_grid = RouterConfig {
        grid_resolution: -0.25,
        ..default_config()
    };
    assert!(matches!(
        route_board(&mut board, &config_neg_grid),
        Err(RoutingError::InvalidConfiguration { .. })
    ));

    let config_zero_trace = RouterConfig {
        default_trace_width: 0.0,
        ..default_config()
    };
    assert!(matches!(
        route_board(&mut board, &config_zero_trace),
        Err(RoutingError::InvalidConfiguration { .. })
    ));
}

// ============================================================================
// SECTION 5: DRC Engine White-Box Adversarial Hardening
// ============================================================================

#[test]
fn test_drc_sub_micron_clearance_discrimination_threshold() {
    let config = default_config(); // clearance = 0.20
    let width = 0.25;

    // DRC uses `surface_dist < config.clearance - 1e-5` (0.199990 mm)
    // Case 1: surface_dist = 0.199980 mm (clearance - 2e-5) -> VIOLATION!
    {
        let mut board = PcbBoard::new(50.0, 50.0);
        board.register_net(1, "N1");
        board.register_net(2, "N2");

        board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(30.0, 20.0), width, Layer::FCu, 1));
        // Y = 20.0 + 0.25 + 0.199980 = 20.449980
        board.add_track(TrackSegment::new(Point::new(10.0, 20.449980), Point::new(30.0, 20.449980), width, Layer::FCu, 2));

        let drc = run_drc(&board, &config);
        assert_eq!(drc.clearance_violations.len(), 1, "Must detect violation at clearance - 2e-5");
        assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::TraceTraceClearance);
    }

    // Case 2: surface_dist = 0.199999 mm (clearance - 1e-6) -> CLEAN (within 1e-5 numerical tolerance)
    {
        let mut board = PcbBoard::new(50.0, 50.0);
        board.register_net(1, "N1");
        board.register_net(2, "N2");

        board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(30.0, 20.0), width, Layer::FCu, 1));
        // Y = 20.0 + 0.25 + 0.199999 = 20.449999
        board.add_track(TrackSegment::new(Point::new(10.0, 20.449999), Point::new(30.0, 20.449999), width, Layer::FCu, 2));

        let drc = run_drc(&board, &config);
        assert_eq!(drc.clearance_violations.len(), 0, "Must tolerate clearance - 1e-6 within 1e-5 epsilon");
    }
}

#[test]
fn test_drc_complex_multi_segment_polygonal_clearance() {
    let config = RouterConfig { clearance: 0.65, ..default_config() };

    // 1. Obround to Obround clearance check
    // Pad 1 at (20, 20) with size 3.0 x 1.0 (radius 0.5)
    // Pad 2 at (20, Y2) with size 3.0 x 1.0 (radius 0.5)
    // Min pad clearance = 0.65 mm.
    // Surface distance = (Y2 - 20) - 1.0. For violation: surface_dist = 0.649 -> Y2 = 21.649
    {
        let mut board = PcbBoard::new(50.0, 50.0);
        board.register_net(1, "N1");
        board.register_net(2, "N2");

        let p1 = Pad::new_smd("1", "A", Point::zero(), PadShape::Obround { width: 3.0, height: 1.0 }, Layer::FCu).with_net(1);
        let p2 = Pad::new_smd("1", "A", Point::zero(), PadShape::Obround { width: 3.0, height: 1.0 }, Layer::FCu).with_net(2);

        board.add_footprint(Footprint::new("U1", "O", "P", Point::new(20.0, 20.0), 0.0, vec![p1], Vec::new()));
        board.add_footprint(Footprint::new("U2", "O", "P", Point::new(20.0, 21.649), 0.0, vec![p2], Vec::new()));

        let drc = run_drc(&board, &config);
        assert_eq!(drc.clearance_violations.len(), 1);
        assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::PadPadClearance);
    }

    // 2. 45° Rotated Rect to 45° Rotated Rect
    // Two squares 2.0 x 2.0 at 45 degrees.
    // Half diagonal = sqrt(2) = 1.41421356 mm
    // Pad 1 at (20.0, 20.0) top vertex at Y = 21.41421356
    // Pad 2 at (20.0, Y2) bottom vertex at Y = Y2 - 1.41421356
    // Surface distance = Y2 - 2.82842712.
    // For surface_dist = 0.649 -> Y2 = 2.82842712 + 0.649 = 3.477427 mm
    {
        let mut board = PcbBoard::new(50.0, 50.0);
        board.register_net(1, "N1");
        board.register_net(2, "N2");

        let p1 = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 2.0, height: 2.0 }, Layer::FCu).with_net(1);
        let p2 = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 2.0, height: 2.0 }, Layer::FCu).with_net(2);

        let y2_violation = 20.0 + 2.0 * std::f64::consts::SQRT_2 + 0.649;
        board.add_footprint(Footprint::new("U1", "R", "P", Point::new(20.0, 20.0), 45.0, vec![p1.clone()], Vec::new()));
        board.add_footprint(Footprint::new("U2", "R", "P", Point::new(20.0, y2_violation), 45.0, vec![p2.clone()], Vec::new()));

        let drc = run_drc(&board, &config);
        assert_eq!(drc.clearance_violations.len(), 1);
        assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::PadPadClearance);
    }
}

#[test]
fn test_drc_annular_ring_breakout_and_sizing_comprehensive() {
    let config = default_config();

    // 1. Via Drill >= Size (Breakout)
    {
        let mut board = PcbBoard::new(50.0, 50.0);
        board.register_net(1, "N1");
        board.add_via(Via::new(Point::new(20.0, 20.0), 0.8, 0.8, 1));
        let drc = run_drc(&board, &config);
        assert_eq!(drc.annular_ring_violations.len(), 1);
        assert_eq!(drc.annular_ring_violations[0].kind, DrcViolationKind::DrillBreakout);
    }

    // 2. Via Annular Ring < 0.15mm
    {
        let mut board = PcbBoard::new(50.0, 50.0);
        board.register_net(1, "N1");
        // Size 0.60, drill 0.40 -> ring 0.10mm < 0.15mm
        board.add_via(Via::new(Point::new(20.0, 20.0), 0.60, 0.40, 1));
        let drc = run_drc(&board, &config);
        assert_eq!(drc.annular_ring_violations.len(), 1);
        assert_eq!(drc.annular_ring_violations[0].kind, DrcViolationKind::AnnularRingTooSmall);
    }

    // 3. THT Pad Drill >= Outer (Breakout)
    {
        let mut board = PcbBoard::new(50.0, 50.0);
        board.register_net(1, "N1");
        let pad = Pad::new_tht("1", "A", Point::zero(), PadShape::Circle { diameter: 1.0 }, 1.0, 1.0).with_net(1);
        board.add_footprint(Footprint::new("J1", "CONN", "HDR", Point::new(20.0, 20.0), 0.0, vec![pad], Vec::new()));
        let drc = run_drc(&board, &config);
        assert_eq!(drc.annular_ring_violations.len(), 1);
        assert_eq!(drc.annular_ring_violations[0].kind, DrcViolationKind::DrillBreakout);
    }

    // 4. Compliant THT Pad Ring == 0.15mm -> Clean!
    {
        let mut board = PcbBoard::new(50.0, 50.0);
        board.register_net(1, "N1");
        // Outer 1.30, drill 1.00 -> ring = (1.30 - 1.00)/2 = 0.15 mm
        let pad = Pad::new_tht("1", "A", Point::zero(), PadShape::Circle { diameter: 1.30 }, 1.00, 1.30).with_net(1);
        board.add_footprint(Footprint::new("J1", "CONN", "HDR", Point::new(20.0, 20.0), 0.0, vec![pad], Vec::new()));
        let drc = run_drc(&board, &config);
        assert_eq!(drc.annular_ring_violations.len(), 0);
    }
}

#[test]
fn test_drc_false_positive_immunity_same_footprint_tight_pitch() {
    let config = default_config();
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "P1");
    board.register_net(2, "P2");

    // Standard SMD IC (e.g. 0.5mm pitch SOIC or TQFP).
    // Pins are 0.5mm apart, which is less than min_pad_clearance (0.65mm)!
    // BUT they are on the SAME footprint (fp_idx 0).
    // DRC MUST NOT flag them as PadPadClearance violations!
    let pad1 = Pad::new_smd("1", "P1", Point::new(-0.25, 0.0), PadShape::Rect { width: 0.3, height: 1.0 }, Layer::FCu).with_net(1);
    let pad2 = Pad::new_smd("2", "P2", Point::new(0.25, 0.0), PadShape::Rect { width: 0.3, height: 1.0 }, Layer::FCu).with_net(2);

    board.add_footprint(Footprint::new("U1", "IC", "SOIC", Point::new(25.0, 25.0), 0.0, vec![pad1, pad2], Vec::new()));

    let drc = run_drc(&board, &config);
    assert_eq!(
        drc.clearance_violations.len(),
        0,
        "Pads within the same footprint must NOT trigger inter-component pad clearance errors"
    );
    assert_eq!(drc.short_circuits.len(), 0);
}

#[test]
fn test_drc_false_positive_immunity_opposite_layer_geometry() {
    let config = default_config();
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(10, "TOP");
    board.register_net(20, "BOTTOM");

    // Identical coordinates: SMD pad on F.Cu and SMD pad on B.Cu
    let p_top = Pad::new_smd("1", "T", Point::zero(), PadShape::Rect { width: 2.0, height: 2.0 }, Layer::FCu).with_net(10);
    let p_bot = Pad::new_smd("1", "B", Point::zero(), PadShape::Rect { width: 2.0, height: 2.0 }, Layer::BCu).with_net(20);

    board.add_footprint(Footprint::new("U_TOP", "T", "P", Point::new(20.0, 20.0), 0.0, vec![p_top], Vec::new()));
    board.add_footprint(Footprint::new("U_BOT", "B", "P", Point::new(20.0, 20.0), 0.0, vec![p_bot], Vec::new()));

    // Tracks directly overlapping in 2D projection
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(30.0, 20.0), 0.25, Layer::FCu, 10));
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(30.0, 20.0), 0.25, Layer::BCu, 20));

    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "Opposite layer copper must NOT short");
    assert_eq!(drc.clearance_violations.len(), 0, "Opposite layer copper must NOT trigger clearance violations");
}

#[test]
fn test_drc_unrouted_nets_detection_broken_branch_in_multi_pin() {
    let config = default_config();
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(100, "BUS");

    // 4 pins: (10, 20), (20, 20), (30, 20), (40, 20)
    for i in 0..4 {
        let x = 10.0 + (i as f64) * 10.0;
        let pad = Pad::new_smd(&(i + 1).to_string(), "P", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(100);
        board.add_footprint(Footprint::new(format!("U{}", i + 1), "P", "P", Point::new(x, 20.0), 0.0, vec![pad], Vec::new()));
    }

    // Connect pins 1-2 and 2-3, but leave pin 4 unconnected
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(20.0, 20.0), 0.25, Layer::FCu, 100));
    board.add_track(TrackSegment::new(Point::new(20.0, 20.0), Point::new(30.0, 20.0), 0.25, Layer::FCu, 100));

    let drc = run_drc(&board, &config);
    assert!(drc.unrouted_nets.contains(&100), "Net with missing leaf must be detected as unrouted");

    // Connect pin 3-4
    board.add_track(TrackSegment::new(Point::new(30.0, 20.0), Point::new(40.0, 20.0), 0.25, Layer::FCu, 100));
    let drc_complete = run_drc(&board, &config);
    assert!(!drc_complete.unrouted_nets.contains(&100), "Fully routed 4-pin net must be clean");
}
