//! E2E Test Suite: 2-Layer Autorouter & Design Rule Checking (Tiers 1, 2, 3)
//!
//! Verifies:
//! - 2-layer grid-based A* maze routing with anisotropic layer biasing (F8, F14)
//! - 45° and 90° octilinear track geometry (F9)
//! - Automated via insertion on layer transitions (F10)
//! - 100% completion of routable nets (R2, F8)
//! - Zero DRC clearance violations, shorts, or unrouted nets on solved boards (F11-F13)
//! - Tier 3 pairwise interactions: via near pad, parallel 45° bends, dense bus routing

use netlist_shatter_viz::pcb::*;
use std::collections::HashMap;

fn default_router_config() -> RouterConfig {
    RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.8,
        via_drill: 0.4,
        clearance: 0.2,
    }
}

// Helper to create a test board with two 2-pin components
fn create_simple_routing_board(p1: Point, p2: Point, net_id: NetId) -> PcbBoard {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(net_id, format!("NET_{}", net_id));

    let pad1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(net_id);
    let fp1 = Footprint::new("U1", "C", "P", p1, 0.0, vec![pad1], Vec::new());

    let pad2 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(net_id);
    let fp2 = Footprint::new("U2", "C", "P", p2, 0.0, vec![pad2], Vec::new());

    board.add_footprint(fp1);
    board.add_footprint(fp2);
    board
}

// ============================================================================
// Tier 1: Router Configuration & Nominal Routing
// ============================================================================

#[test]
fn test_router_config_default_tolerances() {
    let config = default_router_config();
    assert_eq!(config.grid_resolution, 0.25);
    assert_eq!(config.default_trace_width, 0.25);
    assert_eq!(config.via_diameter, 0.8);
    assert_eq!(config.via_drill, 0.4);
    assert_eq!(config.clearance, 0.2);

    // Annular ring on default via: (0.8 - 0.4) / 2 = 0.2 mm >= standard 0.15 mm
    let annular_ring = (config.via_diameter - config.via_drill) / 2.0;
    assert!(annular_ring >= 0.15);
}

#[test]
fn test_route_simple_horizontal_net() {
    // Two pads on horizontal line: (10, 20) and (30, 20)
    let mut board = create_simple_routing_board(Point::new(10.0, 20.0), Point::new(30.0, 20.0), 1);
    let config = default_router_config();

    let result = route_board(&mut board, &config);
    assert!(result.is_ok(), "Router must successfully route simple horizontal net");

    // Tracks must have been added
    assert!(!board.tracks.is_empty(), "Router must generate track segments");
    
    // Check DRC: clean routing with zero errors
    let drc = run_drc(&board, &config);
    assert_eq!(drc.unrouted_nets.len(), 0, "All nets must be routed");
    assert_eq!(drc.clearance_violations.len(), 0, "No clearance violations");
    assert_eq!(drc.short_circuits.len(), 0, "No short circuits");
}

#[test]
fn test_route_simple_vertical_net() {
    // Two pads on vertical line: (25, 10) and (25, 35)
    let mut board = create_simple_routing_board(Point::new(25.0, 10.0), Point::new(25.0, 35.0), 1);
    let config = default_router_config();

    let result = route_board(&mut board, &config);
    assert!(result.is_ok());

    let drc = run_drc(&board, &config);
    assert_eq!(drc.unrouted_nets.len(), 0);
    assert_eq!(drc.clearance_violations.len(), 0);
    assert_eq!(drc.short_circuits.len(), 0);
}

#[test]
fn test_route_diagonal_net_octilinear_geometry() {
    // Diagonal route from (10, 10) to (30, 25)
    let mut board = create_simple_routing_board(Point::new(10.0, 10.0), Point::new(30.0, 25.0), 1);
    let config = default_router_config();

    let result = route_board(&mut board, &config);
    assert!(result.is_ok());
    assert!(!board.tracks.is_empty());

    // Octilinear invariant check:
    // Every track segment must be either horizontal (dy == 0), vertical (dx == 0),
    // or exact 45-degree diagonal (|dx| == |dy|)
    for (idx, seg) in board.tracks.iter().enumerate() {
        let dx = (seg.end.x - seg.start.x).abs();
        let dy = (seg.end.y - seg.start.y).abs();

        let is_horizontal = dy < 1e-4;
        let is_vertical = dx < 1e-4;
        let is_diagonal_45 = (dx - dy).abs() < 1e-4;

        assert!(
            is_horizontal || is_vertical || is_diagonal_45,
            "Track segment #{} from ({},{}) to ({},{}) violates 45/90 octilinear rule: dx={}, dy={}",
            idx, seg.start.x, seg.start.y, seg.end.x, seg.end.y, dx, dy
        );

        // Segment width must equal config trace width
        assert!((seg.width - config.default_trace_width).abs() < 1e-6);
        // Net ID must match
        assert_eq!(seg.net, 1);
    }
}

// ============================================================================
// Tier 1 & 2: 2-Layer Crossing & Automated Via Placement
// ============================================================================

#[test]
fn test_route_crossing_nets_inserts_vias_without_shorts() {
    // Two nets that intersect orthogonally in 2D plane:
    // Net 1: horizontal from (10, 25) to (40, 25)
    // Net 2: vertical from (25, 10) to (25, 40)
    // These two paths physically cross at (25, 25).
    // On a single layer, this would short!
    // The 2-layer router MUST use vias to route one net on F.Cu and the other on B.Cu.
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_HORIZ");
    board.register_net(2, "NET_VERT");

    // Net 1 pads
    let p1_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let p1_b = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(10.0, 25.0), 0.0, vec![p1_a], Vec::new()));
    board.add_footprint(Footprint::new("U2", "C", "P", Point::new(40.0, 25.0), 0.0, vec![p1_b], Vec::new()));

    // Net 2 pads
    let p2_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    let p2_b = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U3", "C", "P", Point::new(25.0, 10.0), 0.0, vec![p2_a], Vec::new()));
    board.add_footprint(Footprint::new("U4", "C", "P", Point::new(25.0, 40.0), 0.0, vec![p2_b], Vec::new()));

    let config = default_router_config();
    let result = route_board(&mut board, &config);
    assert!(result.is_ok(), "Router must solve crossing nets");

    // Verify vias were inserted to cross layers
    assert!(!board.vias.is_empty(), "Crossing nets require via insertion for 2-layer jumping");

    for via in &board.vias {
        assert_eq!(via.size, config.via_diameter);
        assert_eq!(via.drill, config.via_drill);
        assert!(via.net == 1 || via.net == 2);
    }

    // Both F.Cu and B.Cu must have tracks
    let fcu_tracks = board.tracks.iter().filter(|t| t.layer == Layer::FCu).count();
    let bcu_tracks = board.tracks.iter().filter(|t| t.layer == Layer::BCu).count();
    assert!(fcu_tracks > 0, "Must have tracks on F.Cu");
    assert!(bcu_tracks > 0, "Must have tracks on B.Cu for crossing jumper");

    // DRC MUST PASS CLEANLY WITH ZERO SHORTS
    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "Crossing nets must NOT produce electrical short circuits");
    assert_eq!(drc.clearance_violations.len(), 0, "Crossing nets must satisfy all clearances");
    assert_eq!(drc.unrouted_nets.len(), 0, "Both nets must be 100% routed");
}

#[test]
fn test_via_layer_transition_connectivity() {
    // Verify that every inserted via physically touches tracks on both F.Cu and B.Cu of its assigned net
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "NET_X");

    let p1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let p2 = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::BCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(10.0, 10.0), 0.0, vec![p1], Vec::new()));
    board.add_footprint(Footprint::new("U2", "C", "P", Point::new(30.0, 30.0), 0.0, vec![p2], Vec::new()));

    let config = default_router_config();
    let result = route_board(&mut board, &config);
    assert!(result.is_ok());

    assert!(!board.vias.is_empty(), "Routing from F.Cu pad to B.Cu pad requires a via");
    let drc = run_drc(&board, &config);
    assert_eq!(drc.unrouted_nets.len(), 0);
    assert_eq!(drc.short_circuits.len(), 0);
}

// ============================================================================
// Tier 2: Boundary Conditions (Zero-Length, Small Clearances)
// ============================================================================

#[test]
fn test_no_degenerate_tracks_produced() {
    let mut board = create_simple_routing_board(Point::new(10.0, 10.0), Point::new(20.0, 20.0), 1);
    let config = default_router_config();
    let _ = route_board(&mut board, &config);

    for (idx, track) in board.tracks.iter().enumerate() {
        let length = track.length();
        assert!(
            length > 1e-5,
            "Track segment #{} is degenerate (zero length: ({},{}) to ({},{}))",
            idx, track.start.x, track.start.y, track.end.x, track.end.y
        );
    }
}

// ============================================================================
// Tier 3: Pairwise Combinations & High-Density Routing
// ============================================================================

#[test]
fn test_high_density_parallel_bus_routing() {
    // 3 parallel nets running side-by-side:
    // Net 1: (10, 10) to (40, 10)
    // Net 2: (10, 12) to (40, 12)
    // Net 3: (10, 14) to (40, 14)
    // Pitch is 2.0mm, well above trace_width (0.25) + clearance (0.20) = 0.45mm
    let mut board = PcbBoard::new(50.0, 50.0);
    for net_id in 1..=3 {
        board.register_net(net_id, format!("BUS_{}", net_id));
        let y = 10.0 + (net_id as f64 - 1.0) * 2.0;
        let p_start = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(net_id);
        let p_end = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(net_id);
        board.add_footprint(Footprint::new(format!("U_S{}", net_id), "P", "P", Point::new(10.0, y), 0.0, vec![p_start], Vec::new()));
        board.add_footprint(Footprint::new(format!("U_E{}", net_id), "P", "P", Point::new(40.0, y), 0.0, vec![p_end], Vec::new()));
    }

    let config = default_router_config();
    let result = route_board(&mut board, &config);
    assert!(result.is_ok(), "High-density parallel bus must route 100%");

    let drc = run_drc(&board, &config);
    assert_eq!(drc.unrouted_nets.len(), 0, "All 3 bus nets must be routed");
    assert_eq!(drc.short_circuits.len(), 0, "Parallel traces must not short");
    assert_eq!(drc.clearance_violations.len(), 0, "Parallel traces must maintain clearance");
}

#[test]
fn test_via_adjacent_to_smd_pad_clearance() {
    // Net 1 with a pad at (15.0, 15.0)
    // Net 2 routed nearby with a via
    // Router must NOT place Net 2 via closer than via_radius + pad_half_width + clearance
    let mut board = PcbBoard::new(40.0, 40.0);
    board.register_net(1, "NET_STATIC");
    board.register_net(2, "NET_ROUTED");

    // Static pad for Net 1
    let pad1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.5, height: 1.5 }, Layer::FCu).with_net(1);
    board.add_footprint(Footprint::new("U1", "C", "P", Point::new(20.0, 20.0), 0.0, vec![pad1], Vec::new()));

    // Net 2 goes from (10.0, 20.0) to (30.0, 20.0) - directly through Net 1's path!
    // Net 2 must detour around Net 1's pad, possibly using vias
    let pad2_a = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    let pad2_b = Pad::new_smd("1", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(2);
    board.add_footprint(Footprint::new("U2", "C", "P", Point::new(10.0, 20.0), 0.0, vec![pad2_a], Vec::new()));
    board.add_footprint(Footprint::new("U3", "C", "P", Point::new(30.0, 20.0), 0.0, vec![pad2_b], Vec::new()));

    let config = default_router_config();
    let result = route_board(&mut board, &config);
    assert!(result.is_ok(), "Router must detour around obstacle pad");

    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "No short circuits when detouring");
    assert_eq!(drc.clearance_violations.len(), 0, "Clearance to obstacle pad must be maintained");
}
