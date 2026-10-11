//! Milestone 3 Challenger 2: Adversarial DRC Stress & Defect Injection Suite
//!
//! Rigorous empirical verification of the Design Rule Checker (`src/pcb/drc.rs`)
//! covering:
//! 1. Microscopic clearance violations (distance = clearance - 1e-4 mm):
//!    - Parallel horizontal traces
//!    - Parallel 45-degree diagonal traces
//!    - Perpendicular T-approach traces
//!    - Collinear head-to-head traces
//!    - Trace to SMD rectangular pad
//!    - Trace to 45-degree rotated SMD rectangular pad
//!    - Trace to circular THT pad
//!    - Trace to obround SMD pad
//!    - Pad to pad inter-component clearance
//!    - Via to trace clearance
//!    - Via to pad clearance
//!    - Via to via clearance
//!    - Razor-sharp threshold discrimination: (clearance - 1e-4) vs clearance vs (clearance + 1e-4)
//! 2. Annular ring breakout & sizing:
//!    - Via drill == size (breakout)
//!    - Via drill > size (breakout)
//!    - Via ring < 0.15 mm (annular ring too small)
//!    - Via ring == 0.15 mm (clean)
//!    - THT pad drill == outer diameter (breakout)
//!    - THT pad drill > outer diameter (breakout)
//!    - THT pad ring < 0.15 mm (annular ring too small)
//!    - THT pad ring == 0.15 mm (clean)
//! 3. Board edge keepout & boundary encroachment:
//!    - Left, right, bottom, top edge encroachment (distance = 0.3999 mm < 0.40 mm)
//!    - Exact boundary compliance (distance == 0.40 mm)
//!    - Out of bounds copper (coordinates outside board polygon)
//!    - Via edge encroachment
//! 4. Complex multi-point net open-circuits:
//!    - 5-pin star topology with isolated leaf
//!    - 6-pin net split into two internally-connected disjoint sub-islands
//!    - 4-pin multi-layer net with broken layer transition
//!    - Microscopic disconnect (gap = 0.001 mm > 1e-4 mm) between trace and pad
//!    - 8-pin daisy-chain net with broken middle link
//! 5. False-positive immunity:
//!    - Same-net 15-segment winding trace chain
//!    - Same-net redundant loops, cycles, and multi-way junctions
//!    - Multiple traces entering same-net pads from 8 directions and pass-through
//!    - Completely coincident opposite-layer traces (F.Cu and B.Cu)
//!    - Opposite-layer SMD pads at identical (X, Y) coordinates
//!    - Opposite-layer SMD pad vs passing trace

use netlist_shatter_viz::pcb::*;

fn test_config() -> RouterConfig {
    RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.8,
        via_drill: 0.4,
        clearance: 0.20,
    }
}

// ============================================================================
// SECTION 1: Microscopic Clearance Violations (distance = clearance - 1e-4 mm)
// ============================================================================

#[test]
fn test_microscopic_clearance_parallel_horizontal_traces() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "NET1");
    board.register_net(2, "NET2");

    let width = 0.25;

    // Net 1: Y = 20.0, bounds [19.875, 20.125]
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(30.0, 20.0), width, Layer::FCu, 1));

    // Net 2: Y = 20.0 + width + clearance - eps = 20.0 + 0.25 + 0.20 - 0.0001 = 20.4499
    // Surface distance = 20.4499 - 20.0 - 0.25 = 0.1999 mm (clearance - 1e-4 mm)
    board.add_track(TrackSegment::new(Point::new(10.0, 20.4499), Point::new(30.0, 20.4499), width, Layer::FCu, 2));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.short_circuits.len(), 0, "No short circuit at 0.1999mm separation");
    assert_eq!(drc.clearance_violations.len(), 1, "Must detect clearance violation at clearance - 1e-4");
    assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::TraceTraceClearance);
    assert!((drc.clearance_violations[0].actual - 0.1999).abs() < 1e-5);
}

#[test]
fn test_microscopic_clearance_boundary_discrimination() {
    let width = 0.25;

    // Test exact clearance (0.2000 mm) -> MUST BE CLEAN
    {
        let mut board = PcbBoard::new(60.0, 60.0);
        board.register_net(1, "NET1");
        board.register_net(2, "NET2");

        board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(30.0, 20.0), width, Layer::FCu, 1));
        // Y = 20.0 + 0.25 + 0.20 = 20.4500 -> surface_dist = 0.2000 mm
        board.add_track(TrackSegment::new(Point::new(10.0, 20.4500), Point::new(30.0, 20.4500), width, Layer::FCu, 2));

        let drc = run_drc(&board, &test_config());
        assert_eq!(drc.clearance_violations.len(), 0, "Exact clearance must NOT be flagged as violation");
    }

    // Test clearance + 1e-4 mm (0.2001 mm) -> MUST BE CLEAN
    {
        let mut board = PcbBoard::new(60.0, 60.0);
        board.register_net(1, "NET1");
        board.register_net(2, "NET2");

        board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(30.0, 20.0), width, Layer::FCu, 1));
        // Y = 20.0 + 0.25 + 0.20 + 0.0001 = 20.4501 -> surface_dist = 0.2001 mm
        board.add_track(TrackSegment::new(Point::new(10.0, 20.4501), Point::new(30.0, 20.4501), width, Layer::FCu, 2));

        let drc = run_drc(&board, &test_config());
        assert_eq!(drc.clearance_violations.len(), 0, "Clearance + 1e-4 must NOT be flagged as violation");
    }
}

#[test]
fn test_microscopic_clearance_parallel_45_degree_diagonal_traces() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "NET1");
    board.register_net(2, "NET2");

    let width = 0.25;
    let surface_dist = 0.20 - 1e-4; // 0.1999 mm
    let center_dist = surface_dist + width; // 0.4499 mm

    // Diagonal direction: (1, 1) / sqrt(2)
    // Normal direction: (-1, 1) / sqrt(2)
    let offset_x = -center_dist / std::f64::consts::SQRT_2;
    let offset_y = center_dist / std::f64::consts::SQRT_2;

    board.add_track(TrackSegment::new(Point::new(10.0, 10.0), Point::new(30.0, 30.0), width, Layer::FCu, 1));
    board.add_track(TrackSegment::new(
        Point::new(10.0 + offset_x, 10.0 + offset_y),
        Point::new(30.0 + offset_x, 30.0 + offset_y),
        width,
        Layer::FCu,
        2,
    ));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 1, "Must detect 45-degree parallel trace clearance violation");
    assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::TraceTraceClearance);
    assert!((drc.clearance_violations[0].actual - surface_dist).abs() < 1e-5);
}

#[test]
fn test_microscopic_clearance_perpendicular_t_approach() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "NET1");
    board.register_net(2, "NET2");

    let width = 0.25;
    let surface_dist = 0.20 - 1e-4; // 0.1999 mm

    // Track 1: horizontal along Y = 20.0 from X = 10.0 to X = 40.0. Top edge at Y = 20.125.
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(40.0, 20.0), width, Layer::FCu, 1));

    // Track 2: vertical along X = 25.0 from Y_start down towards Track 1.
    // Bottom edge of Track 2: Y_start - width/2.
    // Distance to top edge of Track 1: (Y_start - 0.125) - 20.125 = Y_start - 20.25.
    // For surface_dist = 0.1999 -> Y_start = 20.25 + 0.1999 = 20.4499.
    board.add_track(TrackSegment::new(Point::new(25.0, 20.4499), Point::new(25.0, 35.0), width, Layer::FCu, 2));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 1, "Must detect perpendicular T-approach clearance violation");
    assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::TraceTraceClearance);
    assert!((drc.clearance_violations[0].actual - surface_dist).abs() < 1e-5);
}

#[test]
fn test_microscopic_clearance_collinear_head_to_head_traces() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "NET1");
    board.register_net(2, "NET2");

    let width = 0.25;
    let surface_dist = 0.20 - 1e-4; // 0.1999 mm

    // Track 1: (10.0, 20.0) -> (20.0, 20.0). Capsule extends to X = 20.125.
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(20.0, 20.0), width, Layer::FCu, 1));

    // Track 2 starts at X = 20.0 + width + surface_dist = 20.4499 -> (20.4499, 20.0) to (30.0, 20.0).
    // Capsule extends left to 20.4499 - 0.125 = 20.3249.
    // Distance = 20.3249 - 20.125 = 0.1999 mm.
    board.add_track(TrackSegment::new(Point::new(20.4499, 20.0), Point::new(30.0, 20.0), width, Layer::FCu, 2));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 1, "Must detect collinear head-to-head clearance violation");
    assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::TraceTraceClearance);
    assert!((drc.clearance_violations[0].actual - surface_dist).abs() < 1e-5);
}

#[test]
fn test_microscopic_clearance_trace_to_rect_smd_pad() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "NET1");
    board.register_net(2, "NET2");

    // Pad of Net 1 at (20.0, 20.0), Rect 2.0 x 2.0 mm -> top edge at Y = 21.0
    let pad = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 2.0, height: 2.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(20.0, 20.0), 0.0, vec![pad], Vec::new()));

    // Track of Net 2 with width 0.25, horizontal:
    // Bottom edge at Y - 0.125.
    // We want distance to pad top edge (21.0) = 0.20 - 1e-4 = 0.1999 mm.
    // Y - 0.125 - 21.0 = 0.1999 -> Y = 21.3249
    board.add_track(TrackSegment::new(Point::new(10.0, 21.3249), Point::new(30.0, 21.3249), 0.25, Layer::FCu, 2));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 1, "Must detect trace-to-rect-pad clearance violation");
    assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::TracePadClearance);
    assert!((drc.clearance_violations[0].actual - 0.1999).abs() < 1e-5);
}

#[test]
fn test_microscopic_clearance_trace_to_rotated_smd_pad() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "NET1");
    board.register_net(2, "NET2");

    // Pad of Net 1 at (20.0, 20.0), Rect 2.0 x 2.0 mm rotated by 45 degrees.
    // Half diagonal of 2.0 x 2.0 square is sqrt(2) = 1.41421356...
    // Top-most vertex of pad is at (20.0, 20.0 + sqrt(2)) = (20.0, 21.41421356)
    let pad = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 2.0, height: 2.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(20.0, 20.0), 45.0, vec![pad], Vec::new()));

    let top_pad_y = 20.0 + std::f64::consts::SQRT_2;
    let track_y = top_pad_y + 0.125 + 0.1999;

    board.add_track(TrackSegment::new(Point::new(10.0, track_y), Point::new(30.0, track_y), 0.25, Layer::FCu, 2));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 1, "Must detect trace to 45-deg rotated pad clearance violation");
    assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::TracePadClearance);
    assert!((drc.clearance_violations[0].actual - 0.1999).abs() < 1e-4);
}

#[test]
fn test_microscopic_clearance_trace_to_circular_tht_pad() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "NET1");
    board.register_net(2, "NET2");

    // Circular THT pad at (20.0, 20.0), diameter 1.6 mm (radius 0.8 mm)
    let pad = Pad::new_tht("1", "A", Point::zero(), PadShape::Circle { diameter: 1.6 }, 0.8, 1.6).with_net(1);
    board.add_footprint(Footprint::new("J1", "CONN", "HDR", Point::new(20.0, 20.0), 0.0, vec![pad], Vec::new()));

    // Track on B.Cu (testing THT copper presence on B.Cu) with width 0.25
    // Surface distance = Y - 20.0 - (0.8 + 0.125) = Y - 20.925
    // For 0.1999 -> Y = 20.925 + 0.1999 = 21.1249
    board.add_track(TrackSegment::new(Point::new(10.0, 21.1249), Point::new(30.0, 21.1249), 0.25, Layer::BCu, 2));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 1, "Must detect trace to circular THT pad clearance violation on B.Cu");
    assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::TracePadClearance);
    assert!((drc.clearance_violations[0].actual - 0.1999).abs() < 1e-5);
}

#[test]
fn test_microscopic_clearance_trace_to_obround_pad() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "NET1");
    board.register_net(2, "NET2");

    // Obround pad at (20.0, 20.0), width 3.0, height 1.5. Radius = 0.75. Top edge at Y = 20.75.
    let pad = Pad::new_smd("1", "A", Point::zero(), PadShape::Obround { width: 3.0, height: 1.5 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(20.0, 20.0), 0.0, vec![pad], Vec::new()));

    // Track width 0.25 at Y = 20.75 + 0.125 + 0.1999 = 21.0749
    board.add_track(TrackSegment::new(Point::new(10.0, 21.0749), Point::new(30.0, 21.0749), 0.25, Layer::FCu, 2));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 1, "Must detect trace to obround pad clearance violation");
    assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::TracePadClearance);
    assert!((drc.clearance_violations[0].actual - 0.1999).abs() < 1e-5);
}

#[test]
fn test_microscopic_clearance_pad_to_pad_inter_component() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "NET1");
    board.register_net(2, "NET2");

    // Footprint 1 pad at (20.0, 20.0), Rect 1.0 x 1.0 mm (right edge at X = 20.50)
    let p1 = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "R", "0805", Point::new(20.0, 20.0), 0.0, vec![p1], Vec::new()));

    // Min pad clearance is 0.65 mm. Microscopic violation: 0.65 - 1e-4 = 0.6499 mm.
    // Footprint 2 pad at (X2, 20.0), Rect 1.0 x 1.0 mm (left edge at X2 - 0.50).
    // X2 - 0.50 - 20.50 = 0.6499 -> X2 = 21.6499
    let p2 = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U2", "R", "0805", Point::new(21.6499, 20.0), 0.0, vec![p2], Vec::new()));

    let drc = run_drc(&board, &RouterConfig { clearance: 0.65, ..test_config() });
    assert_eq!(drc.clearance_violations.len(), 1, "Must detect pad-to-pad clearance violation at 0.6499mm");
    assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::PadPadClearance);
    assert!((drc.clearance_violations[0].actual - 0.6499).abs() < 1e-5);
}

#[test]
fn test_microscopic_clearance_via_to_trace() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "NET1");
    board.register_net(2, "NET2");

    // Via of Net 1 at (20.0, 20.0), size 0.8 mm (radius 0.4 mm)
    board.add_via(Via::new(Point::new(20.0, 20.0), 0.8, 0.4, 1));

    // Track of Net 2 with width 0.25 (radius 0.125)
    // Surface distance = Y - 20.0 - (0.4 + 0.125) = Y - 20.525
    // For 0.1999 -> Y = 20.525 + 0.1999 = 20.7249
    board.add_track(TrackSegment::new(Point::new(10.0, 20.7249), Point::new(30.0, 20.7249), 0.25, Layer::FCu, 2));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 1, "Must detect via-to-trace clearance violation");
    assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::ViaTraceClearance);
    assert!((drc.clearance_violations[0].actual - 0.1999).abs() < 1e-5);
}

#[test]
fn test_microscopic_clearance_via_to_pad() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "NET1");
    board.register_net(2, "NET2");

    // Pad of Net 2 at (20.0, 20.0), Rect 2.0 x 2.0 mm (top edge at Y = 21.0)
    let pad = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 2.0, height: 2.0 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(20.0, 20.0), 0.0, vec![pad], Vec::new()));

    // Via of Net 1 at (20.0, Y_via), size 0.8 mm (radius 0.4 mm)
    // Surface distance = Y_via - 21.0 - 0.4 = Y_via - 21.4
    // For 0.1999 -> Y_via = 21.4 + 0.1999 = 21.5999
    board.add_via(Via::new(Point::new(20.0, 21.5999), 0.8, 0.4, 1));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 1, "Must detect via-to-pad clearance violation");
    assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::ViaPadClearance);
    assert!((drc.clearance_violations[0].actual - 0.1999).abs() < 1e-5);
}

#[test]
fn test_microscopic_clearance_via_to_via() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(1, "NET1");
    board.register_net(2, "NET2");

    // Via 1 at (20.0, 20.0), size 0.8
    board.add_via(Via::new(Point::new(20.0, 20.0), 0.8, 0.4, 1));

    // Via 2 at (20.0 + X, 20.0), size 0.8. Surface distance = X - 0.8.
    // For 0.1999 -> X = 0.8 + 0.1999 = 0.9999
    board.add_via(Via::new(Point::new(20.9999, 20.0), 0.8, 0.4, 2));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 1, "Must detect via-to-via clearance violation");
    assert_eq!(drc.clearance_violations[0].kind, DrcViolationKind::ViaViaClearance);
    assert!((drc.clearance_violations[0].actual - 0.1999).abs() < 1e-5);
}

// ============================================================================
// SECTION 2: Annular Ring Breakout & Sizing Validation
// ============================================================================

#[test]
fn test_annular_ring_breakout_via_drill_equals_size() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET1");

    // Drill = 0.80 mm, size = 0.80 mm -> Annular ring = 0.0 mm (drill = outer diameter)
    board.add_via(Via::new(Point::new(25.0, 25.0), 0.80, 0.80, 1));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.annular_ring_violations.len(), 1);
    assert_eq!(drc.annular_ring_violations[0].kind, DrcViolationKind::DrillBreakout);
    assert!(drc.annular_ring_violations[0].message.contains("breakout"));
}

#[test]
fn test_annular_ring_breakout_via_drill_exceeds_size() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET1");

    // Drill = 1.00 mm, size = 0.80 mm -> Negative ring
    board.add_via(Via::new(Point::new(25.0, 25.0), 0.80, 1.00, 1));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.annular_ring_violations.len(), 1);
    assert_eq!(drc.annular_ring_violations[0].kind, DrcViolationKind::DrillBreakout);
}

#[test]
fn test_annular_ring_too_small_via() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET1");

    // Size = 0.60, drill = 0.40 -> ring = (0.60 - 0.40)/2 = 0.10 mm < 0.15 mm
    board.add_via(Via::new(Point::new(25.0, 25.0), 0.60, 0.40, 1));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.annular_ring_violations.len(), 1);
    assert_eq!(drc.annular_ring_violations[0].kind, DrcViolationKind::AnnularRingTooSmall);
}

#[test]
fn test_annular_ring_clean_via() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET1");

    // Size = 0.70, drill = 0.40 -> ring = 0.15 mm == MIN_ANNULAR_RING
    board.add_via(Via::new(Point::new(25.0, 25.0), 0.70, 0.40, 1));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.annular_ring_violations.len(), 0, "Via with exactly 0.15mm ring must be clean");
}

#[test]
fn test_annular_ring_breakout_tht_pad_drill_equals_outer() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET1");

    // THT pad: drill = 1.0 mm, outer_diameter = 1.0 mm -> breakout!
    let pad = Pad::new_tht("1", "A", Point::zero(), PadShape::Circle { diameter: 1.0 }, 1.0, 1.0).with_net(1);
    board.add_footprint(Footprint::new("J1", "CONN", "HDR", Point::new(25.0, 25.0), 0.0, vec![pad], Vec::new()));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.annular_ring_violations.len(), 1);
    assert_eq!(drc.annular_ring_violations[0].kind, DrcViolationKind::DrillBreakout);
}

#[test]
fn test_annular_ring_breakout_tht_pad_drill_exceeds_outer() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET1");

    // THT pad: drill = 1.2 mm, outer_diameter = 1.0 mm -> negative ring
    let pad = Pad::new_tht("1", "A", Point::zero(), PadShape::Circle { diameter: 1.0 }, 1.2, 1.0).with_net(1);
    board.add_footprint(Footprint::new("J1", "CONN", "HDR", Point::new(25.0, 25.0), 0.0, vec![pad], Vec::new()));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.annular_ring_violations.len(), 1);
    assert_eq!(drc.annular_ring_violations[0].kind, DrcViolationKind::DrillBreakout);
}

#[test]
fn test_annular_ring_too_small_tht_pad() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET1");

    // THT pad: drill = 0.8 mm, outer_diameter = 1.0 mm -> ring = 0.10 mm < 0.15 mm
    let pad = Pad::new_tht("1", "A", Point::zero(), PadShape::Circle { diameter: 1.0 }, 0.8, 1.0).with_net(1);
    board.add_footprint(Footprint::new("J1", "CONN", "HDR", Point::new(25.0, 25.0), 0.0, vec![pad], Vec::new()));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.annular_ring_violations.len(), 1);
    assert_eq!(drc.annular_ring_violations[0].kind, DrcViolationKind::AnnularRingTooSmall);
}

// ============================================================================
// SECTION 3: Board Edge Encroachment & Keepout (trace distance < 0.40 mm)
// ============================================================================

#[test]
fn test_board_edge_encroachment_all_four_quadrants() {
    let board_w = 60.0;
    let board_h = 40.0;
    let track_w = 0.25;
    let half_w = track_w / 2.0; // 0.125 mm
    let margin = 0.3999; // 0.40 - 1e-4 mm

    // 1. Left edge encroachment: min_x = 0.3999 -> center X = 0.3999 + 0.125 = 0.5249
    {
        let mut board = PcbBoard::new(board_w, board_h);
        board.register_net(1, "N");
        let x_left = margin + half_w;
        board.add_track(TrackSegment::new(Point::new(x_left, 10.0), Point::new(x_left, 30.0), track_w, Layer::FCu, 1));
        let drc = run_drc(&board, &test_config());
        assert_eq!(drc.edge_keepout_violations.len(), 1, "Must flag left edge encroachment");
        assert_eq!(drc.edge_keepout_violations[0].kind, DrcViolationKind::EdgeKeepoutViolation);
        assert!((drc.edge_keepout_violations[0].actual - margin).abs() < 1e-5);
    }

    // 2. Right edge encroachment: max_x = board_w - 0.3999 -> center X = 60.0 - 0.125 - 0.3999 = 59.4751
    {
        let mut board = PcbBoard::new(board_w, board_h);
        board.register_net(1, "N");
        let x_right = board_w - half_w - margin;
        board.add_track(TrackSegment::new(Point::new(x_right, 10.0), Point::new(x_right, 30.0), track_w, Layer::FCu, 1));
        let drc = run_drc(&board, &test_config());
        assert_eq!(drc.edge_keepout_violations.len(), 1, "Must flag right edge encroachment");
        assert_eq!(drc.edge_keepout_violations[0].kind, DrcViolationKind::EdgeKeepoutViolation);
        assert!((drc.edge_keepout_violations[0].actual - margin).abs() < 1e-5);
    }

    // 3. Bottom edge encroachment: min_y = 0.3999 -> center Y = 0.3999 + 0.125 = 0.5249
    {
        let mut board = PcbBoard::new(board_w, board_h);
        board.register_net(1, "N");
        let y_bottom = margin + half_w;
        board.add_track(TrackSegment::new(Point::new(10.0, y_bottom), Point::new(30.0, y_bottom), track_w, Layer::FCu, 1));
        let drc = run_drc(&board, &test_config());
        assert_eq!(drc.edge_keepout_violations.len(), 1, "Must flag bottom edge encroachment");
        assert_eq!(drc.edge_keepout_violations[0].kind, DrcViolationKind::EdgeKeepoutViolation);
        assert!((drc.edge_keepout_violations[0].actual - margin).abs() < 1e-5);
    }

    // 4. Top edge encroachment: max_y = board_h - 0.3999 -> center Y = 40.0 - 0.125 - 0.3999 = 39.4751
    {
        let mut board = PcbBoard::new(board_w, board_h);
        board.register_net(1, "N");
        let y_top = board_h - half_w - margin;
        board.add_track(TrackSegment::new(Point::new(10.0, y_top), Point::new(30.0, y_top), track_w, Layer::FCu, 1));
        let drc = run_drc(&board, &test_config());
        assert_eq!(drc.edge_keepout_violations.len(), 1, "Must flag top edge encroachment");
        assert_eq!(drc.edge_keepout_violations[0].kind, DrcViolationKind::EdgeKeepoutViolation);
        assert!((drc.edge_keepout_violations[0].actual - margin).abs() < 1e-5);
    }
}

#[test]
fn test_board_edge_keepout_exact_boundary_clean() {
    let board_w = 60.0;
    let board_h = 40.0;
    let track_w = 0.25;

    // Center X = 0.4000 + 0.125 = 0.5250 mm -> edge_dist = 0.4000 mm exactly
    let mut board = PcbBoard::new(board_w, board_h);
    board.register_net(1, "N");
    board.add_track(TrackSegment::new(Point::new(0.5250, 10.0), Point::new(0.5250, 30.0), track_w, Layer::FCu, 1));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.edge_keepout_violations.len(), 0, "Distance of exactly 0.40mm must NOT trigger edge keepout violation");
}

#[test]
fn test_board_edge_keepout_outside_boundary() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "N");

    // Track extending partially outside the left boundary: start at X = -0.1
    board.add_track(TrackSegment::new(Point::new(-0.1, 10.0), Point::new(10.0, 10.0), 0.25, Layer::FCu, 1));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.edge_keepout_violations.len(), 1);
    assert_eq!(drc.edge_keepout_violations[0].kind, DrcViolationKind::OutsideBoardBoundary);
}

#[test]
fn test_board_edge_keepout_via_encroachment() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "N");

    // Via size 0.8 mm (radius 0.4 mm). Center at X = 0.40 - 1e-4 + 0.4 = 0.7999 mm.
    // Edge distance = 0.7999 - 0.4 = 0.3999 mm < 0.40 mm!
    board.add_via(Via::new(Point::new(0.7999, 25.0), 0.8, 0.4, 1));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.edge_keepout_violations.len(), 1);
    assert_eq!(drc.edge_keepout_violations[0].kind, DrcViolationKind::EdgeKeepoutViolation);
}

// ============================================================================
// SECTION 4: Complex Multi-Point Net Open-Circuits
// ============================================================================

#[test]
fn test_complex_open_circuit_5_pin_star_topology() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(10, "STAR_NET");

    // 5 pins: Center (25, 25), West (15, 25), East (35, 25), South (25, 15), North (25, 35)
    let p_c = Pad::new_smd("1", "C", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(10);
    let p_w = Pad::new_smd("2", "W", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(10);
    let p_e = Pad::new_smd("3", "E", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(10);
    let p_s = Pad::new_smd("4", "S", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(10);
    let p_n = Pad::new_smd("5", "N", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(10);

    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(25.0, 25.0), 0.0, vec![p_c], Vec::new()));
    board.add_footprint(Footprint::new("U2", "W", "P", Point::new(15.0, 25.0), 0.0, vec![p_w], Vec::new()));
    board.add_footprint(Footprint::new("U3", "E", "P", Point::new(35.0, 25.0), 0.0, vec![p_e], Vec::new()));
    board.add_footprint(Footprint::new("U4", "S", "P", Point::new(25.0, 15.0), 0.0, vec![p_s], Vec::new()));
    board.add_footprint(Footprint::new("U5", "N", "P", Point::new(25.0, 35.0), 0.0, vec![p_n], Vec::new()));

    // Route West, East, South branches to Center
    board.add_track(TrackSegment::new(Point::new(15.0, 25.0), Point::new(25.0, 25.0), 0.25, Layer::FCu, 10));
    board.add_track(TrackSegment::new(Point::new(25.0, 25.0), Point::new(35.0, 25.0), 0.25, Layer::FCu, 10));
    board.add_track(TrackSegment::new(Point::new(25.0, 25.0), Point::new(25.0, 15.0), 0.25, Layer::FCu, 10));
    // North branch (U5) is LEFT UNROUTED!

    let drc = run_drc(&board, &test_config());
    assert!(
        drc.unrouted_nets.contains(&10),
        "DRC must detect open-circuit in 5-pin star topology when North pin is unconnected"
    );

    // Now connect North pin and verify clean
    board.add_track(TrackSegment::new(Point::new(25.0, 25.0), Point::new(25.0, 35.0), 0.25, Layer::FCu, 10));
    let drc_solved = run_drc(&board, &test_config());
    assert!(
        !drc_solved.unrouted_nets.contains(&10),
        "DRC must clear unrouted status once all 5 pins of star topology are connected"
    );
}

#[test]
fn test_complex_open_circuit_disjoint_sub_islands() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(20, "ISLAND_NET");

    // 6-pin net split into Island A (pins 1,2,3) and Island B (pins 4,5,6)
    // Island A at bottom-left:
    let p1 = Pad::new_smd("1", "A1", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(20);
    let p2 = Pad::new_smd("2", "A2", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(20);
    let p3 = Pad::new_smd("3", "A3", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(20);

    board.add_footprint(Footprint::new("U1", "A1", "P", Point::new(10.0, 10.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "A2", "P", Point::new(10.0, 20.0), 0.0, vec![p2], Vec::new()));
    board.add_footprint(Footprint::new("U3", "A3", "P", Point::new(20.0, 10.0), 0.0, vec![p3], Vec::new()));

    // Connect Island A pins together:
    board.add_track(TrackSegment::new(Point::new(10.0, 10.0), Point::new(10.0, 20.0), 0.25, Layer::FCu, 20));
    board.add_track(TrackSegment::new(Point::new(10.0, 10.0), Point::new(20.0, 10.0), 0.25, Layer::FCu, 20));

    // Island B at top-right:
    let p4 = Pad::new_smd("4", "B1", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(20);
    let p5 = Pad::new_smd("5", "B2", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(20);
    let p6 = Pad::new_smd("6", "B3", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(20);

    board.add_footprint(Footprint::new("U4", "B1", "P", Point::new(40.0, 40.0), 0.0, vec![p4], Vec::new()));
    board.add_footprint(Footprint::new("U5", "B2", "P", Point::new(40.0, 50.0), 0.0, vec![p5], Vec::new()));
    board.add_footprint(Footprint::new("U6", "B3", "P", Point::new(50.0, 40.0), 0.0, vec![p6], Vec::new()));

    // Connect Island B pins together:
    board.add_track(TrackSegment::new(Point::new(40.0, 40.0), Point::new(40.0, 50.0), 0.25, Layer::FCu, 20));
    board.add_track(TrackSegment::new(Point::new(40.0, 40.0), Point::new(50.0, 40.0), 0.25, Layer::FCu, 20));

    // Even though both Island A and Island B are internally connected, there is no bridge between them!
    let drc = run_drc(&board, &test_config());
    assert!(
        drc.unrouted_nets.contains(&20),
        "DRC must detect open net when two clusters are internally connected but disjoint from each other"
    );

    // Add bridge connecting (20, 10) to (40, 40)
    board.add_track(TrackSegment::new(Point::new(20.0, 10.0), Point::new(40.0, 10.0), 0.25, Layer::FCu, 20));
    board.add_track(TrackSegment::new(Point::new(40.0, 10.0), Point::new(40.0, 40.0), 0.25, Layer::FCu, 20));

    let drc_bridged = run_drc(&board, &test_config());
    assert!(
        !drc_bridged.unrouted_nets.contains(&20),
        "DRC must confirm net connectivity after bridge is placed"
    );
}

#[test]
fn test_complex_open_circuit_multi_layer_broken_via() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(30, "MULTI_LAYER_NET");

    // Pin 1 (FCu) at (10, 20) and Pin 2 (BCu) at (30, 20)
    let p1 = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(30);
    let p2 = Pad::new_smd("2", "B", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::BCu).with_net(30);

    board.add_footprint(Footprint::new("U1", "A", "P", Point::new(10.0, 20.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "B", "P", Point::new(30.0, 20.0), 0.0, vec![p2], Vec::new()));

    // Track on F.Cu from (10, 20) to (19.0, 20.0) -> stops short of via at (20, 20) by 0.5 mm!
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(19.0, 20.0), 0.25, Layer::FCu, 30));
    // Via at (20, 20) size 0.8
    board.add_via(Via::new(Point::new(20.0, 20.0), 0.8, 0.4, 30));
    // Track on B.Cu from (20, 20) to (30, 20)
    board.add_track(TrackSegment::new(Point::new(20.0, 20.0), Point::new(30.0, 20.0), 0.25, Layer::BCu, 30));

    let drc = run_drc(&board, &test_config());
    assert!(
        drc.unrouted_nets.contains(&30),
        "DRC must detect open circuit when track stops short of via"
    );

    // Fix connection by extending track to (20.0, 20.0)
    board.tracks[0] = TrackSegment::new(Point::new(10.0, 20.0), Point::new(20.0, 20.0), 0.25, Layer::FCu, 30);
    let drc_fixed = run_drc(&board, &test_config());
    assert!(
        !drc_fixed.unrouted_nets.contains(&30),
        "DRC must verify multi-layer net connection through via"
    );
}

#[test]
fn test_complex_open_circuit_microscopic_gap_to_pad() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(40, "MICRO_GAP_NET");

    // Pad 1 at (10.0, 20.0), 1.0 x 1.0 mm (right edge at X = 10.500)
    let p1 = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(40);
    let p2 = Pad::new_smd("2", "B", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(40);

    board.add_footprint(Footprint::new("U1", "A", "P", Point::new(10.0, 20.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "B", "P", Point::new(30.0, 20.0), 0.0, vec![p2], Vec::new()));

    // Track width is 0.25 mm (half_w = 0.125 mm). Pad edge is at X = 10.500.
    // For a 1 micron (0.001 mm) copper gap: track start = 10.500 + 0.125 + 0.001 = 10.626 mm.
    // Track copper capsule boundary is at 10.626 - 0.125 = 10.501 mm.
    // Distance from track copper to pad edge = 0.001 mm > 1e-4 mm threshold -> unconnected!
    board.add_track(TrackSegment::new(Point::new(10.626, 20.0), Point::new(30.0, 20.0), 0.25, Layer::FCu, 40));

    let drc = run_drc(&board, &test_config());
    assert!(
        drc.unrouted_nets.contains(&40),
        "DRC must detect unrouted net when track stops 1 micron (0.001mm) short of pad"
    );

    // Close the 1 micron gap (start at 10.625 mm so copper touches pad edge, or pad center 10.0 mm)
    board.tracks[0] = TrackSegment::new(Point::new(10.625, 20.0), Point::new(30.0, 20.0), 0.25, Layer::FCu, 40);
    let drc_closed = run_drc(&board, &test_config());
    assert!(
        !drc_closed.unrouted_nets.contains(&40),
        "DRC must verify connectivity when track makes physical contact with pad"
    );
}

#[test]
fn test_complex_open_circuit_8_pin_daisy_chain() {
    let mut board = PcbBoard::new(100.0, 50.0);
    board.register_net(50, "DAISY_NET");

    // 8 pins placed along X = 10, 20, 30, 40, 50, 60, 70, 80 at Y = 25.0
    for i in 0..8 {
        let pad = Pad::new_smd(&(i + 1).to_string(), "P", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(50);
        board.add_footprint(Footprint::new(&format!("U{}", i + 1), "P", "P", Point::new(10.0 * (i + 1) as f64, 25.0), 0.0, vec![pad], Vec::new()));
    }

    // Connect segments: (1-2), (2-3), (3-4), [4-5 MISSING], (5-6), (6-7), (7-8)
    board.add_track(TrackSegment::new(Point::new(10.0, 25.0), Point::new(20.0, 25.0), 0.25, Layer::FCu, 50));
    board.add_track(TrackSegment::new(Point::new(20.0, 25.0), Point::new(30.0, 25.0), 0.25, Layer::FCu, 50));
    board.add_track(TrackSegment::new(Point::new(30.0, 25.0), Point::new(40.0, 25.0), 0.25, Layer::FCu, 50));
    // Missing: (40, 25) to (50, 25)
    board.add_track(TrackSegment::new(Point::new(50.0, 25.0), Point::new(60.0, 25.0), 0.25, Layer::FCu, 50));
    board.add_track(TrackSegment::new(Point::new(60.0, 25.0), Point::new(70.0, 25.0), 0.25, Layer::FCu, 50));
    board.add_track(TrackSegment::new(Point::new(70.0, 25.0), Point::new(80.0, 25.0), 0.25, Layer::FCu, 50));

    let drc = run_drc(&board, &test_config());
    assert!(
        drc.unrouted_nets.contains(&50),
        "DRC must detect open net in 8-pin daisy chain when link 4-5 is missing"
    );

    // Insert missing link
    board.add_track(TrackSegment::new(Point::new(40.0, 25.0), Point::new(50.0, 25.0), 0.25, Layer::FCu, 50));
    let drc_complete = run_drc(&board, &test_config());
    assert!(
        !drc_complete.unrouted_nets.contains(&50),
        "DRC must confirm 8-pin daisy chain is 100% connected"
    );
}

// ============================================================================
// SECTION 5: False-Positive Immunity Validation
// ============================================================================

#[test]
fn test_immunity_same_net_15_segment_winding_trace_chain() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(100, "SNAKE_NET");

    let p_start = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(100);
    let p_end = Pad::new_smd("2", "B", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(100);

    board.add_footprint(Footprint::new("U1", "A", "P", Point::new(5.0, 5.0), 0.0, vec![p_start], Vec::new()));
    board.add_footprint(Footprint::new("U2", "B", "P", Point::new(50.0, 50.0), 0.0, vec![p_end], Vec::new()));

    // 15 contiguous segments with 45 and 90 degree transitions
    let points = [
        Point::new(5.0, 5.0),
        Point::new(10.0, 5.0),
        Point::new(15.0, 10.0),
        Point::new(20.0, 10.0),
        Point::new(25.0, 15.0),
        Point::new(25.0, 25.0),
        Point::new(20.0, 30.0),
        Point::new(15.0, 30.0),
        Point::new(15.0, 35.0),
        Point::new(25.0, 35.0),
        Point::new(30.0, 40.0),
        Point::new(40.0, 40.0),
        Point::new(45.0, 45.0),
        Point::new(48.0, 45.0),
        Point::new(50.0, 47.0),
        Point::new(50.0, 50.0),
    ];

    for i in 0..(points.len() - 1) {
        board.add_track(TrackSegment::new(points[i], points[i + 1], 0.25, Layer::FCu, 100));
    }

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 0, "Same-net winding trace chain must NOT trigger clearance violations");
    assert_eq!(drc.short_circuits.len(), 0, "Same-net trace joints must NOT trigger short circuits");
    assert_eq!(drc.unrouted_nets.len(), 0, "15-segment chain must be recognized as fully connected");
    assert!(drc.is_clean(), "Board with valid same-net trace chain must be 100% clean");
}

#[test]
fn test_immunity_same_net_loops_and_junctions() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(101, "RING_NET");

    let p1 = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(101);
    let p2 = Pad::new_smd("2", "B", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(101);

    board.add_footprint(Footprint::new("U1", "A", "P", Point::new(10.0, 10.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "B", "P", Point::new(30.0, 30.0), 0.0, vec![p2], Vec::new()));

    // Complete circular loop of traces (ring routing)
    board.add_track(TrackSegment::new(Point::new(10.0, 10.0), Point::new(30.0, 10.0), 0.25, Layer::FCu, 101));
    board.add_track(TrackSegment::new(Point::new(30.0, 10.0), Point::new(30.0, 30.0), 0.25, Layer::FCu, 101));
    board.add_track(TrackSegment::new(Point::new(30.0, 30.0), Point::new(10.0, 30.0), 0.25, Layer::FCu, 101));
    board.add_track(TrackSegment::new(Point::new(10.0, 30.0), Point::new(10.0, 10.0), 0.25, Layer::FCu, 101));

    // Internal diagonal cross-track creating multiple T-junctions
    board.add_track(TrackSegment::new(Point::new(10.0, 10.0), Point::new(30.0, 30.0), 0.25, Layer::FCu, 101));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 0, "Same-net loop must NOT trigger clearance violations");
    assert_eq!(drc.short_circuits.len(), 0, "Same-net junctions must NOT trigger short circuits");
    assert_eq!(drc.unrouted_nets.len(), 0, "All pads in ring net must be connected");
}

#[test]
fn test_immunity_traces_entering_same_net_pads_multi_angle() {
    let mut board = PcbBoard::new(60.0, 60.0);
    board.register_net(102, "PAD_ENTRY_NET");

    // Central pad at (30.0, 30.0), Rect 2.0 x 2.0 mm
    let center_pad = Pad::new_smd("1", "CTR", Point::zero(), PadShape::Rect { width: 2.0, height: 2.0 }, Layer::FCu).with_net(102);
    board.add_footprint(Footprint::new("U_CTR", "C", "P", Point::new(30.0, 30.0), 0.0, vec![center_pad], Vec::new()));

    // Traces entering from 4 cardinal directions and 4 diagonals
    let angles: [f64; 8] = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0];
    for deg in angles {
        let rad = deg.to_radians();
        let start = Point::new(30.0 + 10.0 * rad.cos(), 30.0 + 10.0 * rad.sin());
        let end = Point::new(30.0, 30.0); // Ends directly at pad center
        board.add_track(TrackSegment::new(start, end, 0.25, Layer::FCu, 102));
    }

    // Continuous trace passing completely through pad from (15, 30) to (45, 30)
    board.add_track(TrackSegment::new(Point::new(15.0, 30.0), Point::new(45.0, 30.0), 0.25, Layer::FCu, 102));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 0, "Multi-angle traces entering same-net pad must NOT trigger clearance violations");
    assert_eq!(drc.short_circuits.len(), 0, "Traces entering same-net pad must NOT trigger short circuits");
}

#[test]
fn test_immunity_completely_coincident_opposite_layer_traces() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(201, "TOP_NET");
    board.register_net(202, "BOTTOM_NET");

    // Identical start and end coordinates, completely overlapping in 2D projection
    // Net 201 on F.Cu
    board.add_track(TrackSegment::new(Point::new(10.0, 25.0), Point::new(40.0, 25.0), 0.25, Layer::FCu, 201));
    // Net 202 on B.Cu
    board.add_track(TrackSegment::new(Point::new(10.0, 25.0), Point::new(40.0, 25.0), 0.25, Layer::BCu, 202));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 0, "Coincident opposite-layer traces must NOT trigger clearance violation");
    assert_eq!(drc.short_circuits.len(), 0, "Coincident opposite-layer traces must NOT trigger short circuit");
}

#[test]
fn test_immunity_opposite_layer_smd_pads_at_identical_position() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(301, "TOP_PAD_NET");
    board.register_net(302, "BOTTOM_PAD_NET");

    // Top SMD pad on F.Cu at (25.0, 25.0)
    let p_top = Pad::new_smd("1", "T", Point::zero(), PadShape::Rect { width: 2.0, height: 2.0 }, Layer::FCu).with_net(301);
    board.add_footprint(Footprint::new("U_TOP", "T", "P", Point::new(25.0, 25.0), 0.0, vec![p_top], Vec::new()));

    // Bottom SMD pad on B.Cu at (25.0, 25.0)
    let p_bot = Pad::new_smd("1", "B", Point::zero(), PadShape::Rect { width: 2.0, height: 2.0 }, Layer::BCu).with_net(302);
    board.add_footprint(Footprint::new("U_BOT", "B", "P", Point::new(25.0, 25.0), 0.0, vec![p_bot], Vec::new()));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 0, "Opposite-layer SMD pads must NOT trigger clearance violation");
    assert_eq!(drc.short_circuits.len(), 0, "Opposite-layer SMD pads must NOT trigger short circuit");
}

#[test]
fn test_immunity_opposite_layer_smd_pad_vs_passing_trace() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(401, "PAD_NET");
    board.register_net(402, "TRACE_NET");

    // Top SMD pad on F.Cu at (25.0, 25.0)
    let p_top = Pad::new_smd("1", "T", Point::zero(), PadShape::Rect { width: 2.0, height: 2.0 }, Layer::FCu).with_net(401);
    board.add_footprint(Footprint::new("U_TOP", "T", "P", Point::new(25.0, 25.0), 0.0, vec![p_top], Vec::new()));

    // Bottom copper trace running right underneath the pad on B.Cu
    board.add_track(TrackSegment::new(Point::new(10.0, 25.0), Point::new(40.0, 25.0), 0.25, Layer::BCu, 402));

    let drc = run_drc(&board, &test_config());
    assert_eq!(drc.clearance_violations.len(), 0, "Opposite-layer SMD pad vs trace must NOT trigger clearance violation");
    assert_eq!(drc.short_circuits.len(), 0, "Opposite-layer SMD pad vs trace must NOT trigger short circuit");
}
