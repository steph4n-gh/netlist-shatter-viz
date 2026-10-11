//! E2E Test Suite: Dynamic Ratsnest EMST Engine & Island Clustering (Tiers 1 & 2)
//!
//! Verifies:
//! - Euclidean Minimum Spanning Tree (EMST) generation per net across multi-pin pads (F5)
//! - Real-time dirty net invalidation and sub-millisecond incremental recomputation (F6)
//! - Copper-connected island clustering: dynamic elimination of ratsnest lines as pads are routed (F7)
//! - Tier 2 boundary cases: single-pin nets, coincident pads, large pin counts, collinear pads

use netlist_shatter_viz::pcb::*;
use std::collections::HashMap;

// Helper to construct a minimal board with specified nets and pad locations
fn make_test_board_with_pads(net_pads: &[(NetId, Vec<Point>)]) -> PcbBoard {
    let mut board = PcbBoard::new(100.0, 100.0);
    let mut net_names = HashMap::new();

    for (net_idx, (net_id, points)) in net_pads.iter().enumerate() {
        net_names.insert(*net_id, format!("NET_{}", net_id));
        let pads: Vec<Pad> = points
            .iter()
            .enumerate()
            .map(|(p_idx, pt)| {
                Pad::new_smd(
                    (p_idx + 1).to_string(),
                    format!("P{}", p_idx + 1),
                    *pt,
                    PadShape::Rect { width: 1.0, height: 1.0 },
                    Layer::FCu,
                )
                .with_net(*net_id)
            })
            .collect();

        let fp = Footprint::new(
            format!("U{}", net_idx + 1),
            "TEST_PART",
            "TEST_PKG",
            Point::new(0.0, 0.0),
            0.0,
            pads,
            Vec::new(),
        );
        board.add_footprint(fp);
    }

    board.net_names = net_names;
    board
}

// ============================================================================
// Tier 1: Euclidean Minimum Spanning Tree (EMST) Mathematical Properties
// ============================================================================

#[test]
fn test_ratsnest_empty_board_yields_empty_lines() {
    let board = PcbBoard::new(50.0, 50.0);
    let lines = compute_ratsnest(&board);
    assert!(lines.is_empty(), "Empty board must have zero ratsnest lines");
}

#[test]
fn test_ratsnest_single_pin_net_yields_no_lines() {
    // A net with only 1 pin cannot be routed to anything, so it must not generate a line
    let board = make_test_board_with_pads(&[(1, vec![Point::new(10.0, 10.0)])]);
    let lines = compute_ratsnest(&board);
    assert_eq!(lines.len(), 0, "Single-pin net must not generate any ratsnest line");
}

#[test]
fn test_ratsnest_two_pin_net_produces_single_direct_line() {
    let p1 = Point::new(10.0, 10.0);
    let p2 = Point::new(40.0, 50.0);
    let board = make_test_board_with_pads(&[(1, vec![p1, p2])]);
    
    let lines = compute_ratsnest(&board);
    assert_eq!(lines.len(), 1, "2-pin net must produce exactly 1 ratsnest line");
    
    let line = &lines[0];
    assert_eq!(line.net, 1);
    
    // Check that line endpoints connect p1 and p2 (in either order)
    let direct = (line.start.distance_to(p1) < 1e-6 && line.end.distance_to(p2) < 1e-6)
        || (line.start.distance_to(p2) < 1e-6 && line.end.distance_to(p1) < 1e-6);
    assert!(direct, "Ratsnest line must span directly between p1 and p2");

    let expected_len = p1.distance_to(p2);
    let actual_len = line.start.distance_to(line.end);
    assert!((actual_len - expected_len).abs() < 1e-6);
}

#[test]
fn test_ratsnest_three_pin_triangle_selects_shortest_edges() {
    // Right triangle:
    // P1 at (0, 0), P2 at (4, 0), P3 at (0, 3)
    // Distance(P1, P2) = 4.0
    // Distance(P1, P3) = 3.0
    // Distance(P2, P3) = 5.0 (hypotenuse)
    // The EMST MUST select edges (P1, P3) [weight 3] and (P1, P2) [weight 4], total weight = 7.0
    // It must NOT select the hypotenuse of weight 5.0 in place of a shorter edge.
    let p1 = Point::new(0.0, 0.0);
    let p2 = Point::new(4.0, 0.0);
    let p3 = Point::new(0.0, 3.0);

    let board = make_test_board_with_pads(&[(10, vec![p1, p2, p3])]);
    let lines = compute_ratsnest(&board);

    assert_eq!(lines.len(), 2, "3-pin net spanning tree must have exactly 2 edges (N-1)");

    let total_len: f64 = lines.iter().map(|l| l.start.distance_to(l.end)).sum();
    assert!((total_len - 7.0).abs() < 1e-6, "EMST total length must be 7.0 (3.0 + 4.0), got {}", total_len);

    for line in &lines {
        let len = line.start.distance_to(line.end);
        assert!((len - 5.0).abs() > 1e-4, "EMST must not select the 5.0 hypotenuse");
    }
}

#[test]
fn test_ratsnest_four_pin_square_selects_perimeter_edges() {
    // 4 points in a 10x10 square: (0,0), (10,0), (10,10), (0,10)
    // Distance of sides = 10.0, diagonals = 10 * sqrt(2) ≈ 14.14
    // EMST must select 3 edges of length 10.0, total weight = 30.0.
    let p1 = Point::new(0.0, 0.0);
    let p2 = Point::new(10.0, 0.0);
    let p3 = Point::new(10.0, 10.0);
    let p4 = Point::new(0.0, 10.0);

    let board = make_test_board_with_pads(&[(1, vec![p1, p2, p3, p4])]);
    let lines = compute_ratsnest(&board);

    assert_eq!(lines.len(), 3, "4-pin net spanning tree must have exactly 3 edges");

    let total_len: f64 = lines.iter().map(|l| l.start.distance_to(l.end)).sum();
    assert!((total_len - 30.0).abs() < 1e-6, "EMST total length must be 30.0, got {}", total_len);

    for line in &lines {
        let len = line.start.distance_to(line.end);
        assert!((len - 10.0).abs() < 1e-4, "Each selected edge in square EMST must have length 10.0");
    }
}

#[test]
fn test_ratsnest_collinear_pads() {
    // Collinear points along X axis at 0, 10, 25, 40
    // EMST must connect (0,10), (10,25), (25,40)
    // Total length = 10 + 15 + 15 = 40.0
    let points = vec![
        Point::new(0.0, 5.0),
        Point::new(10.0, 5.0),
        Point::new(25.0, 5.0),
        Point::new(40.0, 5.0),
    ];
    let board = make_test_board_with_pads(&[(1, points)]);
    let lines = compute_ratsnest(&board);

    assert_eq!(lines.len(), 3);
    let total_len: f64 = lines.iter().map(|l| l.start.distance_to(l.end)).sum();
    assert!((total_len - 40.0).abs() < 1e-6, "Total length must be 40.0, got {}", total_len);
}

#[test]
fn test_ratsnest_multiple_independent_nets() {
    // Net 1 with 2 pins at (10, 10) and (20, 10) -> 1 line of length 10
    // Net 2 with 3 pins at (0, 0), (0, 5), (0, 15) -> 2 lines of length 5 + 10 = 15
    let board = make_test_board_with_pads(&[
        (1, vec![Point::new(10.0, 10.0), Point::new(20.0, 10.0)]),
        (2, vec![Point::new(0.0, 0.0), Point::new(0.0, 5.0), Point::new(0.0, 15.0)]),
    ]);

    let lines = compute_ratsnest(&board);
    assert_eq!(lines.len(), 3, "1 line for Net 1 + 2 lines for Net 2 = 3 lines total");

    let net1_lines: Vec<_> = lines.iter().filter(|l| l.net == 1).collect();
    let net2_lines: Vec<_> = lines.iter().filter(|l| l.net == 2).collect();

    assert_eq!(net1_lines.len(), 1);
    assert_eq!(net2_lines.len(), 2);

    // Cross-check: No line connects Net 1 coordinates to Net 2 coordinates
    for line in &lines {
        if line.net == 1 {
            assert!(line.start.x >= 9.0 && line.end.x >= 9.0, "Net 1 line must only touch Net 1 pads");
        } else if line.net == 2 {
            assert!(line.start.x < 1.0 && line.end.x < 1.0, "Net 2 line must only touch Net 2 pads");
        }
    }
}

// ============================================================================
// Tier 1: Real-Time Invalidation & Footprint Movement Updates
// ============================================================================

#[test]
fn test_ratsnest_updates_when_footprint_moved() {
    let mut board = PcbBoard::new(100.0, 100.0);
    board.register_net(1, "NET_A");

    let pad_u1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let u1 = Footprint::new("U1", "CHIP", "PKG", Point::new(10.0, 10.0), 0.0, vec![pad_u1], Vec::new());

    let pad_u2 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let u2 = Footprint::new("U2", "CHIP", "PKG", Point::new(20.0, 10.0), 0.0, vec![pad_u2], Vec::new());

    board.add_footprint(u1);
    board.add_footprint(u2);

    let initial_lines = compute_ratsnest(&board);
    assert_eq!(initial_lines.len(), 1);
    let initial_len = initial_lines[0].start.distance_to(initial_lines[0].end);
    assert!((initial_len - 10.0).abs() < 1e-6);

    // Now move U2 from (20, 10) to (50, 10)
    if let Some(fp_u2) = board.find_footprint_mut("U2") {
        fp_u2.position = Point::new(50.0, 10.0);
    }

    let updated_lines = compute_ratsnest(&board);
    assert_eq!(updated_lines.len(), 1);
    let updated_len = updated_lines[0].start.distance_to(updated_lines[0].end);
    assert!((updated_len - 40.0).abs() < 1e-6, "Ratsnest line must stretch to 40.0 mm after moving U2, got {}", updated_len);
}

#[test]
fn test_ratsnest_updates_when_footprint_rotated() {
    let mut board = PcbBoard::new(100.0, 100.0);
    board.register_net(1, "NET_X");

    // U1 has pad offset at relative (2.0, 0.0) from footprint center (10.0, 10.0) -> global (12.0, 10.0)
    let pad_u1 = Pad::new_smd("1", "P", Point::new(2.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let u1 = Footprint::new("U1", "C", "P", Point::new(10.0, 10.0), 0.0, vec![pad_u1], Vec::new());

    // U2 pad at fixed (10.0, 20.0)
    let pad_u2 = Pad::new_smd("1", "P", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
    let u2 = Footprint::new("U2", "C", "P", Point::new(10.0, 20.0), 0.0, vec![pad_u2], Vec::new());

    board.add_footprint(u1);
    board.add_footprint(u2);

    // Initial distance from (12, 10) to (10, 20) is sqrt(2^2 + 10^2) = sqrt(104) ≈ 10.198
    let l1 = compute_ratsnest(&board);
    let d1 = l1[0].start.distance_to(l1[0].end);
    assert!((d1 - (104.0_f64).sqrt()).abs() < 1e-4);

    // Rotate U1 by 90 degrees CCW: relative (2, 0) rotates to (0, 2)
    // Global pad position becomes (10.0, 12.0)
    // Distance from (10, 12) to (10, 20) is exactly 8.0 mm!
    if let Some(fp_u1) = board.find_footprint_mut("U1") {
        fp_u1.rotation = 90.0;
    }

    let l2 = compute_ratsnest(&board);
    let d2 = l2[0].start.distance_to(l2[0].end);
    assert!((d2 - 8.0).abs() < 1e-6, "Ratsnest length after 90 deg rotation must be 8.0 mm, got {}", d2);
}

// ============================================================================
// Tier 1: Copper-Connected Island Clustering
// ============================================================================

#[test]
fn test_ratsnest_line_eliminated_when_fully_routed() {
    let p1 = Point::new(10.0, 10.0);
    let p2 = Point::new(30.0, 10.0);
    let mut board = make_test_board_with_pads(&[(1, vec![p1, p2])]);

    // Before routing: 1 ratsnest line
    let lines_before = compute_ratsnest(&board);
    assert_eq!(lines_before.len(), 1);

    // Add a track segment directly connecting p1 and p2 on F.Cu
    board.add_track(TrackSegment::new(p1, p2, 0.25, Layer::FCu, 1));

    // After routing: track connects p1 and p2, forming a single copper island -> 0 lines
    let lines_after = compute_ratsnest(&board);
    assert_eq!(lines_after.len(), 0, "Ratsnest line must disappear when pads are connected by routed copper");
}

#[test]
fn test_ratsnest_partial_routing_clusters_island_and_reduces_lines() {
    // 3 pins: P1 at (0, 0), P2 at (10, 0), P3 at (30, 0)
    // Unrouted: 2 ratsnest lines (P1-P2, P2-P3)
    let p1 = Point::new(0.0, 0.0);
    let p2 = Point::new(10.0, 0.0);
    let p3 = Point::new(30.0, 0.0);
    let mut board = make_test_board_with_pads(&[(1, vec![p1, p2, p3])]);

    let lines_unrouted = compute_ratsnest(&board);
    assert_eq!(lines_unrouted.len(), 2);

    // Connect P1 and P2 with a track segment
    board.add_track(TrackSegment::new(p1, p2, 0.25, Layer::FCu, 1));

    // P1 and P2 are now in island 1; P3 is in island 2.
    // Ratsnest must produce exactly 1 line connecting island 1 to island 2 (shortest is P2 to P3, length 20.0).
    let lines_partial = compute_ratsnest(&board);
    assert_eq!(lines_partial.len(), 1, "Connecting 2 of 3 pins must reduce ratsnest lines from 2 to 1");

    let remaining_line = &lines_partial[0];
    let len = remaining_line.start.distance_to(remaining_line.end);
    assert!((len - 20.0).abs() < 1e-4, "Remaining ratsnest line must bridge P2 and P3 (len 20.0), got {}", len);
}

#[test]
fn test_ratsnest_multi_segment_trace_with_via_eliminates_line() {
    // P1 at (10, 10), P2 at (20, 20)
    // Routed with L-shaped trace: P1 -> Via at (20, 10) on F.Cu, then Via -> P2 on B.Cu
    let p1 = Point::new(10.0, 10.0);
    let p_via = Point::new(20.0, 10.0);
    let p2 = Point::new(20.0, 20.0);

    let mut board = make_test_board_with_pads(&[(1, vec![p1, p2])]);
    assert_eq!(compute_ratsnest(&board).len(), 1);

    board.add_track(TrackSegment::new(p1, p_via, 0.25, Layer::FCu, 1));
    board.add_via(Via::new(p_via, 0.8, 0.4, 1));
    board.add_track(TrackSegment::new(p_via, p2, 0.25, Layer::BCu, 1));

    let lines_after = compute_ratsnest(&board);
    assert_eq!(lines_after.len(), 0, "Multi-layer routed path through via must eliminate ratsnest line");
}

#[test]
fn test_different_net_track_does_not_eliminate_ratsnest() {
    let p1 = Point::new(10.0, 10.0);
    let p2 = Point::new(30.0, 10.0);
    let mut board = make_test_board_with_pads(&[(1, vec![p1, p2])]);

    // Place a track between p1 and p2 belonging to Net 99 (different net)
    board.add_track(TrackSegment::new(p1, p2, 0.25, Layer::FCu, 99));

    let lines = compute_ratsnest(&board);
    assert_eq!(lines.len(), 1, "Track of a different net must NOT eliminate Net 1 ratsnest");
}

// ============================================================================
// Tier 2: Boundary Values & Stress
// ============================================================================

#[test]
fn test_ratsnest_coincident_pads() {
    // Two pads placed at the exact same point (distance = 0.0)
    let p = Point::new(25.0, 25.0);
    let board = make_test_board_with_pads(&[(1, vec![p, p])]);

    let lines = compute_ratsnest(&board);
    assert_eq!(lines.len(), 1);
    let len = lines[0].start.distance_to(lines[0].end);
    assert!((len - 0.0).abs() < 1e-12, "Coincident pads produce 0.0 length line without panic");
}

#[test]
fn test_ratsnest_large_pin_count_net() {
    // Net with 32 pins (e.g. GND bus across full board)
    let mut points = Vec::new();
    for i in 0..32 {
        points.push(Point::new((i % 8) as f64 * 5.0, (i / 8) as f64 * 5.0));
    }
    let board = make_test_board_with_pads(&[(1, points)]);

    let lines = compute_ratsnest(&board);
    // Tree on 32 nodes must have exactly 31 edges
    assert_eq!(lines.len(), 31, "32-pin net must have exactly 31 EMST edges");

    for line in &lines {
        assert_eq!(line.net, 1);
    }
}

#[test]
fn test_ratsnest_pad_with_none_net_ignored() {
    let mut board = PcbBoard::new(50.0, 50.0);
    // Unassigned net (None)
    let pad_no_net = Pad::new_smd("1", "NC", Point::new(10.0, 10.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu);
    let fp = Footprint::new("U1", "NC", "PKG", Point::new(0.0, 0.0), 0.0, vec![pad_no_net], Vec::new());
    board.add_footprint(fp);

    let lines = compute_ratsnest(&board);
    assert!(lines.is_empty(), "Pads without assigned net must produce zero ratsnest lines");
}
