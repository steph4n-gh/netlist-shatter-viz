//! E2E Test Suite: DRC Violation Injection & Defect Matrix (Tiers 2 & 3)
//!
//! Verifies:
//! - Deliberate short-circuit injection detection (trace-trace, trace-pad, pad-pad, via-trace) (F13)
//! - Deliberate clearance violation detection (trace-trace, trace-pad, pad-pad) (F11)
//! - Deliberate annular ring violation detection (drill >= outer diameter, thin annular ring) (F12)
//! - Deliberate board edge keepout violation detection (copper near or outside Edge.Cuts) (F12)
//! - Deliberate unrouted net detection (open netlist, partial routing) (F13)
//! - False positive immunity: same-net copper overlap, trace entering same-net pad, multi-layer crossings

use netlist_shatter_viz::pcb::*;
use std::collections::HashMap;

fn strict_drc_config() -> RouterConfig {
    RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.8,
        via_drill: 0.4,
        clearance: 0.20, // 0.20 mm minimum clearance
    }
}

// ============================================================================
// 1. Electrical Short Circuit Injections
// ============================================================================

#[test]
fn test_inject_crossing_traces_same_layer_short() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_A");
    board.register_net(2, "NET_B");

    // Net 1: horizontal trace on F.Cu from (10, 20) to (30, 20)
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(30.0, 20.0), 0.25, Layer::FCu, 1));
    // Net 2: vertical trace on F.Cu from (20, 10) to (20, 30) -> crosses Net 1 at (20, 20)!
    board.add_track(TrackSegment::new(Point::new(20.0, 10.0), Point::new(20.0, 30.0), 0.25, Layer::FCu, 2));

    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        !drc.short_circuits.is_empty(),
        "DRC must detect short circuit when two traces of different nets cross on the same layer"
    );
}

#[test]
fn test_inject_overlapping_pads_different_nets_short() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_1");
    board.register_net(2, "NET_2");

    // Pad 1 at (20, 20) for Net 1
    let pad1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 2.0, height: 2.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(20.0, 20.0), 0.0, vec![pad1], Vec::new()));

    // Pad 2 at (20.5, 20.5) for Net 2 -> physical overlap with Pad 1!
    let pad2 = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 2.0, height: 2.0 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U2", "C", "P", Point::new(20.5, 20.5), 0.0, vec![pad2], Vec::new()));

    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        !drc.short_circuits.is_empty(),
        "DRC must detect short circuit when pads of different nets physically overlap"
    );
}

#[test]
fn test_inject_trace_terminating_inside_different_net_pad_short() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_A");
    board.register_net(2, "NET_B");

    // Pad of Net 2 at (25.0, 25.0), size 1.5x1.5mm
    let pad2 = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.5, height: 1.5 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(25.0, 25.0), 0.0, vec![pad2], Vec::new()));

    // Trace of Net 1 on F.Cu ending directly at (25.0, 25.0) -> touches Net 2 pad!
    board.add_track(TrackSegment::new(Point::new(10.0, 25.0), Point::new(25.0, 25.0), 0.25, Layer::FCu, 1));

    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        !drc.short_circuits.is_empty(),
        "DRC must detect short circuit when a trace terminates inside a pad of a different net"
    );
}

#[test]
fn test_inject_via_colliding_with_different_net_trace_short() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_A");
    board.register_net(2, "NET_B");

    // Trace of Net 1 from (10, 20) to (30, 20) on F.Cu
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(30.0, 20.0), 0.25, Layer::FCu, 1));
    // Via of Net 2 placed at (20.0, 20.0) -> center directly on Net 1 track!
    board.add_via(Via::new(Point::new(20.0, 20.0), 0.8, 0.4, 2));

    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        !drc.short_circuits.is_empty(),
        "DRC must detect short circuit when a via of Net 2 collides with a trace of Net 1"
    );
}

// ============================================================================
// 2. Clearance Violation Injections
// ============================================================================

#[test]
fn test_inject_trace_to_trace_clearance_violation() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_1");
    board.register_net(2, "NET_2");

    // Net 1 trace: Y = 20.0, width 0.25 mm (extends Y: 19.875 to 20.125)
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(30.0, 20.0), 0.25, Layer::FCu, 1));

    // Net 2 trace: Y = 20.35, width 0.25 mm (extends Y: 20.225 to 20.475)
    // Edge-to-edge clearance = 20.225 - 20.125 = 0.10 mm.
    // Minimum required clearance = 0.20 mm.
    // 0.10 mm < 0.20 mm -> CLEARANCE VIOLATION (but not a short, since clearance > 0)
    board.add_track(TrackSegment::new(Point::new(10.0, 20.35), Point::new(30.0, 20.35), 0.25, Layer::FCu, 2));

    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        !drc.clearance_violations.is_empty(),
        "DRC must report clearance violation for parallel traces spaced 0.10mm apart (required 0.20mm)"
    );
    assert_eq!(drc.short_circuits.len(), 0, "No physical short should be reported for 0.10mm separation");
}

#[test]
fn test_inject_trace_to_pad_clearance_violation() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_A");
    board.register_net(2, "NET_B");

    // Pad of Net 1 at (20.0, 20.0), size 1.0x1.0mm (bounds X: 19.5 to 20.5, Y: 19.5 to 20.5)
    let pad = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(20.0, 20.0), 0.0, vec![pad], Vec::new()));

    // Trace of Net 2 on F.Cu, width 0.25mm, along Y = 20.65 from (10, 20.65) to (30, 20.65)
    // Trace inner edge = 20.65 - 0.125 = 20.525.
    // Distance to pad edge (20.50) = 0.025 mm < 0.20 mm clearance!
    board.add_track(TrackSegment::new(Point::new(10.0, 20.65), Point::new(30.0, 20.65), 0.25, Layer::FCu, 2));

    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        !drc.clearance_violations.is_empty(),
        "DRC must report clearance violation when trace passes 0.025mm from pad of different net"
    );
}

#[test]
fn test_inject_pad_to_pad_clearance_violation() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_1");
    board.register_net(2, "NET_2");

    // Pad 1 at (10.0, 20.0), size 1.0x1.0mm (right edge at 10.50)
    let p1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(10.0, 20.0), 0.0, vec![p1], Vec::new()));

    // Pad 2 at (11.10, 20.0), size 1.0x1.0mm (left edge at 11.10)
    // Clearance between pads = 11.10 - 10.50 = 0.10 mm < 0.20 mm requirement!
    let p2 = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U2", "C", "P", Point::new(11.10, 20.0), 0.0, vec![p2], Vec::new()));

    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        !drc.clearance_violations.is_empty(),
        "DRC must report clearance violation when adjacent pads of different nets are 0.10mm apart"
    );
}

// ============================================================================
// 3. Annular Ring Injections
// ============================================================================

#[test]
fn test_inject_via_annular_ring_too_small() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_VIA");

    // Standard via: diameter 0.8, drill 0.4 -> ring = 0.20 mm (valid)
    // Defective via: diameter 0.50, drill 0.40 -> ring = (0.50 - 0.40)/2 = 0.05 mm!
    // Minimum annular ring required is 0.15 mm.
    board.add_via(Via::new(Point::new(20.0, 20.0), 0.50, 0.40, 1));

    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        !drc.annular_ring_violations.is_empty(),
        "DRC must detect annular ring violation when via ring is 0.05mm < 0.15mm"
    );
}

#[test]
fn test_inject_drill_exceeds_pad_outer_diameter_breakout() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_BREAKOUT");

    // Fatal breakout: drill 1.0 mm exceeds via diameter 0.8 mm (ring = -0.10 mm)
    board.add_via(Via::new(Point::new(20.0, 20.0), 0.80, 1.00, 1));

    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        !drc.annular_ring_violations.is_empty(),
        "DRC must flag fatal breakout where drill exceeds pad diameter"
    );
}

// ============================================================================
// 4. Board Edge Keepout Injections
// ============================================================================

#[test]
fn test_inject_track_violating_board_edge_keepout() {
    let mut board = PcbBoard::new(60.0, 35.0);
    board.register_net(1, "NET_EDGE");

    // Board boundary: X in [0.0, 60.0], Y in [0.0, 35.0]
    // Standard edge keepout is 0.40 mm.
    // Place a track at X = 0.10 mm (distance to edge = 0.10 mm < 0.40 mm)
    board.add_track(TrackSegment::new(Point::new(0.10, 5.0), Point::new(0.10, 30.0), 0.25, Layer::FCu, 1));

    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        !drc.edge_keepout_violations.is_empty(),
        "DRC must detect board edge keepout violation when track is placed 0.10mm from board edge"
    );
}

#[test]
fn test_inject_copper_outside_board_boundary() {
    let mut board = PcbBoard::new(60.0, 35.0);
    board.register_net(1, "NET_OUTSIDE");

    // Place a track entirely outside the board: X = 70.0 mm on a 60.0 mm board!
    board.add_track(TrackSegment::new(Point::new(70.0, 5.0), Point::new(70.0, 25.0), 0.25, Layer::FCu, 1));

    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        !drc.edge_keepout_violations.is_empty(),
        "DRC must flag copper geometry placed outside board boundary"
    );
}

// ============================================================================
// 5. Unrouted Net Injections
// ============================================================================

#[test]
fn test_inject_unrouted_two_pin_net() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(42, "NET_OPEN");

    let p1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(42);
    let p2 = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(42);

    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(10.0, 10.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "C", "P", Point::new(30.0, 10.0), 0.0, vec![p2], Vec::new()));

    // No tracks added: Net 42 is completely open
    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        drc.unrouted_nets.contains(&42),
        "DRC must list unrouted net ID 42 when no tracks connect its pads"
    );
}

#[test]
fn test_inject_partially_routed_multi_pin_net() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(7, "NET_PARTIAL");

    let p1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(7);
    let p2 = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(7);
    let p3 = Pad::new_smd("1", "C", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(7);

    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(10.0, 10.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "C", "P", Point::new(20.0, 10.0), 0.0, vec![p2], Vec::new()));
    board.add_footprint(Footprint::new("U3", "C", "P", Point::new(40.0, 10.0), 0.0, vec![p3], Vec::new()));

    // Only route between U1 and U2; U3 is left disconnected
    board.add_track(TrackSegment::new(Point::new(10.0, 10.0), Point::new(20.0, 10.0), 0.25, Layer::FCu, 7));

    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        drc.unrouted_nets.contains(&7),
        "DRC must flag Net 7 as unrouted when only 2 of its 3 pins are connected"
    );
}

// ============================================================================
// 6. False Positive Immunity (Same Net & Multi-Layer)
// ============================================================================

#[test]
fn test_immunity_same_net_overlapping_tracks() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_SAME");

    // Two intersecting tracks of the SAME net on F.Cu
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(30.0, 20.0), 0.25, Layer::FCu, 1));
    board.add_track(TrackSegment::new(Point::new(20.0, 10.0), Point::new(20.0, 30.0), 0.25, Layer::FCu, 1));

    let drc = run_drc(&board, &strict_drc_config());
    assert_eq!(
        drc.short_circuits.len(),
        0,
        "Same-net overlapping tracks must NEVER be reported as a short circuit"
    );
    assert_eq!(
        drc.clearance_violations.len(),
        0,
        "Same-net overlapping tracks must NEVER be reported as a clearance violation"
    );
}

#[test]
fn test_immunity_track_entering_same_net_pad() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_1");

    let pad = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.5, height: 1.5 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(20.0, 20.0), 0.0, vec![pad], Vec::new()));

    // Track ends right in the center of pad (overlap = 0.75mm)
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(20.0, 20.0), 0.25, Layer::FCu, 1));

    let drc = run_drc(&board, &strict_drc_config());
    assert_eq!(drc.short_circuits.len(), 0);
    assert_eq!(drc.clearance_violations.len(), 0, "Trace entering same-net pad must NOT trigger clearance violation");
}

#[test]
fn test_immunity_traces_crossing_on_different_layers() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_TOP");
    board.register_net(2, "NET_BOTTOM");

    // Net 1 on F.Cu (Top copper)
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(30.0, 20.0), 0.25, Layer::FCu, 1));
    // Net 2 on B.Cu (Bottom copper)
    board.add_track(TrackSegment::new(Point::new(20.0, 10.0), Point::new(20.0, 30.0), 0.25, Layer::BCu, 2));

    let drc = run_drc(&board, &strict_drc_config());
    assert_eq!(
        drc.short_circuits.len(),
        0,
        "Traces of different nets crossing on opposite copper layers must NOT be flagged as short circuit"
    );
}

#[test]
fn test_immunity_single_pin_net_not_flagged_as_unrouted() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(99, "NET_SINGLE_PIN");

    let pad = Pad::new_smd("1", "TEST", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(99);
    board.add_footprint(Footprint::new("U1", "TEST", "TEST", Point::new(25.0, 25.0), 0.0, vec![pad], Vec::new()));

    let drc = run_drc(&board, &strict_drc_config());
    assert!(
        !drc.unrouted_nets.contains(&99),
        "A net with only 1 pin must NOT be flagged as an unrouted net violation"
    );
}
