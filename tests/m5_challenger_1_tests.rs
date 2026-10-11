//! Milestone 5 Challenger 1 Empirical Test Suite
//!
//! Rigorously challenges and stress-tests:
//! 1. Pad Boundary Checks: All 98 pads within 65.0 x 32.0 mm Edge.Cuts with 0 margin violations.
//! 2. Component Rotation & Placement Sanity: J2/J3 horizontal orientation (270.0°) and collision clearance.
//! 3. Dynamic Ratsnest Stress: EMST stability, component perturbation, island clustering, reversibility.
//! 4. Netlist Graph Connectivity & Electrical Integrity: GND 10 distinct components, 3V3/5V power trees,
//!    tactile reset switch pull-up logic and dissipation safety.
//! 5. Adversarial Board Mutation: Probing failure detection upon injected boundary and collision defects.

use netlist_shatter_viz::pcb::*;
use std::collections::HashSet;

// ============================================================================
// 1. Pad Boundary & Outline Verification
// ============================================================================

#[test]
fn test_m5_challenge_pad_boundaries_strictly_inside_board_outline() {
    let board = create_reference_board();

    assert_eq!(board.width, 65.0, "Board width must be 65.0 mm");
    assert_eq!(board.height, 32.0, "Board height must be 32.0 mm");

    let mut total_pad_count = 0;
    let mut min_edge_margin = f64::INFINITY;
    let mut tightest_pad_info = String::new();

    // Board bounding rectangle
    let board_rect = Rect::new(Point::new(0.0, 0.0), Point::new(board.width, board.height));

    for fp in &board.footprints {
        for pad in &fp.pads {
            total_pad_count += 1;

            let pad_center = fp.pad_world_position(pad);
            let pad_bbox = fp.pad_world_bbox(pad);

            // Pad center must be strictly inside board outline
            assert!(
                board_rect.contains_point(pad_center),
                "Pad center ({:.3}, {:.3}) of {}.{} is outside board outline",
                pad_center.x, pad_center.y, fp.reference, pad.pin_number
            );

            // Pad bounding box must be within board outline [0.0..65.0, 0.0..32.0]
            assert!(
                pad_bbox.min.x >= -1e-6,
                "Pad {}.{} min.x ({:.4}) < 0.0 mm",
                fp.reference, pad.pin_number, pad_bbox.min.x
            );
            assert!(
                pad_bbox.min.y >= -1e-6,
                "Pad {}.{} min.y ({:.4}) < 0.0 mm",
                fp.reference, pad.pin_number, pad_bbox.min.y
            );
            assert!(
                pad_bbox.max.x <= board.width + 1e-6,
                "Pad {}.{} max.x ({:.4}) > board width ({:.4})",
                fp.reference, pad.pin_number, pad_bbox.max.x, board.width
            );
            assert!(
                pad_bbox.max.y <= board.height + 1e-6,
                "Pad {}.{} max.y ({:.4}) > board height ({:.4})",
                fp.reference, pad.pin_number, pad_bbox.max.y, board.height
            );

            // Compute exact margin to all four edges
            let margin_left = pad_bbox.min.x;
            let margin_right = board.width - pad_bbox.max.x;
            let margin_bottom = pad_bbox.min.y;
            let margin_top = board.height - pad_bbox.max.y;

            let pad_margin = margin_left.min(margin_right).min(margin_bottom).min(margin_top);
            assert!(
                pad_margin >= 0.0,
                "Pad {}.{} margin violation: margin = {:.4} mm",
                fp.reference, pad.pin_number, pad_margin
            );

            if pad_margin < min_edge_margin {
                min_edge_margin = pad_margin;
                tightest_pad_info = format!("{}.{} (margin = {:.3} mm)", fp.reference, pad.pin_number, pad_margin);
            }
        }
    }

    // Verify pad count matches specification exactly
    assert_eq!(total_pad_count, 98, "Reference board must have exactly 98 physical pads");

    // Board validation report must return zero errors
    let report = board.validate();
    assert_eq!(report.errors.len(), 0, "Board validation reported errors: {:?}", report.errors);
    assert_eq!(report.total_pads, 98);
    assert!(min_edge_margin > 0.50, "Tightest pad edge margin should be >= 0.50 mm, found {}", tightest_pad_info);
}

// ============================================================================
// 2. Component Rotation & Placement Sanity
// ============================================================================

#[test]
fn test_m5_challenge_header_rotations_and_non_collision() {
    let board = create_reference_board();

    let j2 = board.find_footprint("J2").expect("J2 (CONN_TOP) must exist");
    let j3 = board.find_footprint("J3").expect("J3 (CONN_BOT) must exist");

    // Rotation must be exactly 270.0 degrees (yielding horizontal pin layout)
    assert_eq!(j2.rotation, 270.0, "J2 must be rotated 270.0 deg");
    assert_eq!(j3.rotation, 270.0, "J3 must be rotated 270.0 deg");

    // Position checks
    assert_eq!(j2.position, Point::new(14.72, 2.54));
    assert_eq!(j3.position, Point::new(14.72, 29.46));

    // Verify horizontal orientation: all pads must have identical Y coordinates
    let j2_world_pads = j2.all_pad_world_positions();
    assert_eq!(j2_world_pads.len(), 15);
    for (i, (_pad, pos)) in j2_world_pads.iter().enumerate() {
        assert!(
            (pos.y - 2.54).abs() < 1e-6,
            "J2 pad {} Y coordinate ({:.4}) must be 2.54 mm",
            i + 1, pos.y
        );
        let expected_x = 14.72 + (i as f64) * 2.54;
        assert!(
            (pos.x - expected_x).abs() < 1e-6,
            "J2 pad {} X coordinate ({:.4}) must match expected ({:.4})",
            i + 1, pos.x, expected_x
        );
    }

    let j3_world_pads = j3.all_pad_world_positions();
    assert_eq!(j3_world_pads.len(), 15);
    for (i, (_pad, pos)) in j3_world_pads.iter().enumerate() {
        assert!(
            (pos.y - 29.46).abs() < 1e-6,
            "J3 pad {} Y coordinate ({:.4}) must be 29.46 mm",
            i + 1, pos.y
        );
        let expected_x = 14.72 + (i as f64) * 2.54;
        assert!(
            (pos.x - expected_x).abs() < 1e-6,
            "J3 pad {} X coordinate ({:.4}) must match expected ({:.4})",
            i + 1, pos.x, expected_x
        );
    }

    // Clearance and Non-Collision Verification:
    // Headers must not collide with U1, U2, J1, Y1, or passives.
    let other_components: Vec<&Footprint> = board
        .footprints
        .iter()
        .filter(|fp| fp.reference != "J2" && fp.reference != "J3")
        .collect();

    for header in [j2, j3] {
        let h_bbox = header.world_bounding_box();
        for &other in &other_components {
            let o_bbox = other.world_bounding_box();
            assert!(
                !h_bbox.intersects(&o_bbox),
                "Header {} bounding box {:?} intersects with {} bounding box {:?}",
                header.reference, h_bbox, other.reference, o_bbox
            );

            // Pad-to-pad Euclidean surface distance must exceed 1.0 mm
            for (h_pad, h_pos) in header.all_pad_world_positions() {
                for (o_pad, o_pos) in other.all_pad_world_positions() {
                    let center_dist = h_pos.distance_to(o_pos);
                    assert!(
                        center_dist > 1.5,
                        "Header {}.{} and {}.{} center distance ({:.3}mm) too close",
                        header.reference, h_pad.pin_number, other.reference, o_pad.pin_number, center_dist
                    );
                }
            }
        }
    }

    // Footprint pairwise clearance check across ALL components
    for i in 0..board.footprints.len() {
        for j in (i + 1)..board.footprints.len() {
            let fp1 = &board.footprints[i];
            let fp2 = &board.footprints[j];
            for (p1, pos1) in fp1.all_pad_world_positions() {
                for (p2, pos2) in fp2.all_pad_world_positions() {
                    let dist = pos1.distance_to(pos2);
                    assert!(
                        dist > 1.0,
                        "Pads {}.{} and {}.{} are dangerously close: {:.3}mm",
                        fp1.reference, p1.pin_number, fp2.reference, p2.pin_number, dist
                    );
                }
            }
        }
    }
}

// ============================================================================
// 3. Dynamic Ratsnest Stress & Invalidation
// ============================================================================

#[test]
fn test_m5_challenge_dynamic_ratsnest_perturbation_and_island_clustering() {
    let mut board = create_reference_board();

    // Baseline ratsnest
    let baseline_ratsnest = compute_ratsnest(&board);
    assert_eq!(baseline_ratsnest.len(), 31, "Baseline ratsnest must have 31 lines");

    let total_baseline_length: f64 = baseline_ratsnest
        .iter()
        .map(|l| l.start.distance_to(l.end))
        .sum();
    assert!(total_baseline_length > 100.0, "Total baseline ratsnest length must be > 100mm");

    // Invariant: Non-zero length on all lines
    for line in &baseline_ratsnest {
        assert!(line.start.distance_to(line.end) > 0.01, "Zero-length line found");
        assert!(line.net > 0, "Invalid net on ratsnest line");
    }

    // 1. Single Component Perturbation: Move U1 by (+1.0, +0.5) mm
    let original_u1_pos = board.find_footprint("U1").unwrap().position;
    board.find_footprint_mut("U1").unwrap().position = original_u1_pos + Point::new(1.0, 0.5);

    let perturbed_ratsnest = compute_ratsnest(&board);
    assert_eq!(perturbed_ratsnest.len(), 31, "Perturbed board should retain 31 lines");

    // Lines not touching U1 should be completely invariant (as a set of undirected edges)
    let net_5v = board.get_or_create_net("5V");
    let canonical_edge = |l: &RatsnestLine| {
        let p1 = ((l.start.x * 1000.0).round() as i64, (l.start.y * 1000.0).round() as i64);
        let p2 = ((l.end.x * 1000.0).round() as i64, (l.end.y * 1000.0).round() as i64);
        if p1 <= p2 { (p1, p2) } else { (p2, p1) }
    };

    let baseline_5v_set: HashSet<_> = baseline_ratsnest
        .iter()
        .filter(|l| l.net == net_5v)
        .map(canonical_edge)
        .collect();
    let perturbed_5v_set: HashSet<_> = perturbed_ratsnest
        .iter()
        .filter(|l| l.net == net_5v)
        .map(canonical_edge)
        .collect();

    assert_eq!(
        baseline_5v_set, perturbed_5v_set,
        "5V ratsnest edges must be completely unchanged when U1 moves"
    );

    // Restore U1 to original position -> Reversibility check
    board.find_footprint_mut("U1").unwrap().position = original_u1_pos;
    let restored_ratsnest = compute_ratsnest(&board);
    assert_eq!(restored_ratsnest.len(), 31);
    let total_restored_length: f64 = restored_ratsnest
        .iter()
        .map(|l| l.start.distance_to(l.end))
        .sum();
    assert!(
        (total_baseline_length - total_restored_length).abs() < 1e-6,
        "Ratsnest must be perfectly reversible"
    );

    // 2. Copper Island Clustering Stress:
    // Connect GND pads with copper tracks and observe dynamic ratsnest line count reduction
    let net_gnd = board.get_or_create_net("GND");
    let initial_gnd_lines = baseline_ratsnest.iter().filter(|l| l.net == net_gnd).count();
    assert_eq!(initial_gnd_lines, 10, "GND connects 11 pads, so EMST must have 10 lines");

    // Add a copper track between C1 pin 2 and U2 pin 1 (both GND)
    let c1_gnd_pos = board.find_footprint("C1").unwrap().pad_world_pos("2").unwrap();
    let u2_gnd_pos = board.find_footprint("U2").unwrap().pad_world_pos("1").unwrap();
    board.tracks.push(TrackSegment::new(c1_gnd_pos, u2_gnd_pos, 0.25, Layer::FCu, net_gnd));

    let routed_1_ratsnest = compute_ratsnest(&board);
    let routed_1_gnd_lines = routed_1_ratsnest.iter().filter(|l| l.net == net_gnd).count();
    assert_eq!(
        routed_1_gnd_lines,
        initial_gnd_lines - 1,
        "Adding 1 copper connection must reduce ratsnest by 1 line"
    );

    // Add another track connecting U2 pin 1 to J1 SH1
    let j1_gnd_pos = board.find_footprint("J1").unwrap().pad_world_pos("SH1").unwrap();
    board.tracks.push(TrackSegment::new(u2_gnd_pos, j1_gnd_pos, 0.25, Layer::FCu, net_gnd));

    let routed_2_ratsnest = compute_ratsnest(&board);
    let routed_2_gnd_lines = routed_2_ratsnest.iter().filter(|l| l.net == net_gnd).count();
    assert_eq!(
        routed_2_gnd_lines,
        initial_gnd_lines - 2,
        "Adding 2 copper connections must reduce ratsnest by 2 lines"
    );

    // Remove tracks -> returns to original count
    board.tracks.clear();
    let unrouted_ratsnest = compute_ratsnest(&board);
    let unrouted_gnd_lines = unrouted_ratsnest.iter().filter(|l| l.net == net_gnd).count();
    assert_eq!(unrouted_gnd_lines, 10, "Clearing tracks must restore 10 GND ratsnest lines");
}

// ============================================================================
// 4. Netlist Graph Connectivity & Electrical Integrity
// ============================================================================

#[test]
fn test_m5_challenge_netlist_electrical_integrity_and_reset_pullup() {
    let mut board = create_reference_board();

    // 1. GND Net: Must connect to 10 distinct components
    let gnd_id = board
        .net_names
        .iter()
        .find(|(_, name)| *name == "GND")
        .map(|(&id, _)| id)
        .expect("GND net must exist");

    let mut gnd_components: HashSet<&str> = HashSet::new();
    let mut gnd_pad_details: Vec<(&str, &str)> = Vec::new();

    for fp in &board.footprints {
        for pad in &fp.pads {
            if pad.net == Some(gnd_id) {
                gnd_components.insert(fp.reference.as_str());
                gnd_pad_details.push((fp.reference.as_str(), pad.pin_number.as_str()));
            }
        }
    }

    let expected_gnd_refs: HashSet<&str> = [
        "U1", "U2", "J1", "Y1", "C1", "C2", "C3", "SW1", "J2", "J3",
    ]
    .iter()
    .copied()
    .collect();

    assert_eq!(
        gnd_components, expected_gnd_refs,
        "GND must connect to exactly these 10 components: {:?}",
        expected_gnd_refs
    );
    assert_eq!(gnd_components.len(), 10, "GND must connect to 10 distinct components");

    // 2. 3V3 Power Rail: Connects LDO VOUT, caps, MCU, pullup R1, and J3
    let net_3v3_id = board
        .net_names
        .iter()
        .find(|(_, name)| *name == "3V3")
        .map(|(&id, _)| id)
        .expect("3V3 net must exist");

    let mut net_3v3_pads: Vec<(&str, &str)> = Vec::new();
    for fp in &board.footprints {
        for pad in &fp.pads {
            if pad.net == Some(net_3v3_id) {
                net_3v3_pads.push((fp.reference.as_str(), pad.pin_number.as_str()));
            }
        }
    }

    // U2 pin 2 (LDO VOUT), C2 pin 1 (Filter Cap), U1 pin 4 (MCU VCC),
    // C3 pin 1 (Bypass Cap), R1 pin 1 (Pullup), J3 pin 1 (Expansion)
    assert_eq!(net_3v3_pads.len(), 6, "3V3 net must connect exactly 6 pads");
    assert!(net_3v3_pads.contains(&("U2", "2")), "3V3 must connect U2.2 (LDO OUT)");
    assert!(net_3v3_pads.contains(&("C2", "1")), "3V3 must connect C2.1 (Filter cap)");
    assert!(net_3v3_pads.contains(&("U1", "4")), "3V3 must connect U1.4 (MCU VCC)");
    assert!(net_3v3_pads.contains(&("C3", "1")), "3V3 must connect C3.1 (Bypass cap)");
    assert!(net_3v3_pads.contains(&("R1", "1")), "3V3 must connect R1.1 (Pullup)");
    assert!(net_3v3_pads.contains(&("J3", "1")), "3V3 must connect J3.1 (Header)");

    // 3. 5V Rail: Connects LDO VIN, C1, and J2
    let net_5v_id = board
        .net_names
        .iter()
        .find(|(_, name)| *name == "5V")
        .map(|(&id, _)| id)
        .expect("5V net must exist");

    let mut net_5v_pads: Vec<(&str, &str)> = Vec::new();
    for fp in &board.footprints {
        for pad in &fp.pads {
            if pad.net == Some(net_5v_id) {
                net_5v_pads.push((fp.reference.as_str(), pad.pin_number.as_str()));
            }
        }
    }
    assert_eq!(net_5v_pads.len(), 3, "5V rail must connect 3 pads (U2.3, C1.1, J2.1)");
    assert!(net_5v_pads.contains(&("U2", "3")));
    assert!(net_5v_pads.contains(&("C1", "1")));
    assert!(net_5v_pads.contains(&("J2", "1")));

    // 4. RESET Pull-Up Circuit Electrical Logic
    let net_reset_id = board
        .net_names
        .iter()
        .find(|(_, name)| *name == "RESET")
        .map(|(&id, _)| id)
        .expect("RESET net must exist");

    let r1 = board.find_footprint("R1").expect("R1 must exist");
    let sw1 = board.find_footprint("SW1").expect("SW1 must exist");

    let r1_pin1_net = r1.pad("1").unwrap().net;
    let r1_pin2_net = r1.pad("2").unwrap().net;
    let sw1_pin1_net = sw1.pad("1").unwrap().net;
    let sw1_pin2_net = sw1.pad("2").unwrap().net;

    assert_eq!(r1_pin1_net, Some(net_3v3_id), "R1 pin 1 must connect to 3V3 rail");
    assert_eq!(r1_pin2_net, Some(net_reset_id), "R1 pin 2 must connect to RESET");
    assert_eq!(sw1_pin1_net, Some(gnd_id), "SW1 pin 1 must connect to GND");
    assert_eq!(sw1_pin2_net, Some(net_reset_id), "SW1 pin 2 must connect to RESET");

    // Electrical circuit sanity check:
    // Assert R1 does not bridge 3V3 directly to GND (no dead short)
    assert_ne!(r1_pin1_net, r1_pin2_net, "R1 must not have both pins shorted together");
    assert_ne!(sw1_pin1_net, sw1_pin2_net, "SW1 must not short GND directly to GND");

    // Resistor value verification: 10k pull-up
    assert_eq!(r1.value, "10k", "R1 value must be 10k");
    let r_pullup = 10_000.0; // 10 kOhms
    let v_supply = 3.3; // 3.3 Volts
    let i_press = v_supply / r_pullup; // Current when switch pressed
    let p_dissipation = i_press * i_press * r_pullup; // Power dissipation in R1
    assert!(i_press < 0.001, "Switch press current ({:.4}A) must be < 1mA", i_press);
    assert!(p_dissipation < 0.005, "Pull-up dissipation ({:.4}W) well within 0805 125mW rating", p_dissipation);

    // 5. Oscillator Circuit: XTAL1 & XTAL2
    let net_xtal1 = board.get_or_create_net("XTAL1");
    let net_xtal2 = board.get_or_create_net("XTAL2");
    let u1 = board.find_footprint("U1").unwrap();
    let y1 = board.find_footprint("Y1").unwrap();
    assert_eq!(u1.pad("7").unwrap().net, Some(net_xtal1), "U1 pin 7 is XTAL1");
    assert_eq!(y1.pad("1").unwrap().net, Some(net_xtal1), "Y1 pin 1 is XTAL1");
    assert_eq!(u1.pad("8").unwrap().net, Some(net_xtal2), "U1 pin 8 is XTAL2");
    assert_eq!(y1.pad("3").unwrap().net, Some(net_xtal2), "Y1 pin 3 is XTAL2");
}

// ============================================================================
// 5. Adversarial Board Mutation & Failure Ingestion
// ============================================================================

#[test]
fn test_m5_challenge_adversarial_mutation_catches_violations() {
    // Mutation 1: Move a component outside board boundary -> Board validation must detect it
    let mut board = create_reference_board();
    // Shift J2 far to the right beyond 65.0 mm
    board.find_footprint_mut("J2").unwrap().position = Point::new(60.0, 2.54);
    let report = board.validate();
    assert!(
        !report.errors.is_empty(),
        "Shifting J2 past board edge must produce validation errors"
    );
    assert!(
        report.errors.iter().any(|e| matches!(e, ValidationError::PadOutOfBounds { .. })),
        "Must flag PadOutOfBounds error"
    );

    // Mutation 2: Collision injection -> Move SW1 directly on top of U1 pad 1
    let mut board = create_reference_board();
    let u1_pad1_pos = board.find_footprint("U1").unwrap().pad_world_pos("1").unwrap();
    board.find_footprint_mut("SW1").unwrap().position = u1_pad1_pos;

    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    };
    let drc = run_drc(&board, &config);
    assert!(
        !drc.short_circuits.is_empty() || !drc.clearance_violations.is_empty(),
        "Placing SW1 on top of U1 pad 1 must trigger DRC short or clearance violations"
    );

    // Mutation 3: Artificial short injection between 5V and GND
    let mut board = create_reference_board();
    let net_5v = board.get_or_create_net("5V");
    let net_gnd = board.get_or_create_net("GND");
    let p_5v = board.find_footprint("J2").unwrap().pad_world_pos("1").unwrap();
    let p_gnd = board.find_footprint("J2").unwrap().pad_world_pos("4").unwrap();

    // Direct track on F.Cu bridging 5V pad and GND pad
    board.tracks.push(TrackSegment::new(p_5v, p_gnd, 0.25, Layer::FCu, net_5v));
    let drc = run_drc(&board, &config);
    assert!(
        drc.short_circuits.iter().any(|s| {
            (s.net1 == Some(net_5v) && s.net2 == Some(net_gnd))
                || (s.net1 == Some(net_gnd) && s.net2 == Some(net_5v))
        }),
        "DRC must detect 5V-to-GND track-to-pad short circuit"
    );
}

// ============================================================================
// 6. BOM Inventory & Pad Geometry Classification Audit
// ============================================================================

#[test]
fn test_m5_challenge_per_component_pad_inventory_and_bom_integrity() {
    let board = create_reference_board();

    let expected_bom: [(&str, &str, usize, Point); 11] = [
        ("U1", "ATmega328P-AU", 32, Point::new(32.5, 16.0)),
        ("U2", "AMS1117-3.3", 4, Point::new(14.0, 16.0)),
        ("J1", "USB-C", 16, Point::new(4.5, 16.0)),
        ("Y1", "16MHz", 4, Point::new(24.0, 10.0)),
        ("C1", "10uF", 2, Point::new(14.0, 8.5)),
        ("C2", "10uF", 2, Point::new(14.0, 23.5)),
        ("C3", "0.1uF", 2, Point::new(24.0, 22.0)),
        ("R1", "10k", 2, Point::new(40.0, 10.0)),
        ("SW1", "RESET", 4, Point::new(48.0, 16.0)),
        ("J2", "CONN_TOP", 15, Point::new(14.72, 2.54)),
        ("J3", "CONN_BOT", 15, Point::new(14.72, 29.46)),
    ];

    let mut tht_pad_count = 0;
    let mut smd_pad_count = 0;

    for (reference, value, expected_pads, pos) in expected_bom {
        let fp = board.find_footprint(reference).unwrap_or_else(|| panic!("Missing {}", reference));
        assert_eq!(fp.value, value, "Footprint {} value mismatch", reference);
        assert_eq!(fp.pads.len(), expected_pads, "Footprint {} pad count mismatch", reference);
        assert!(
            fp.position.approx_eq(pos, 1e-6),
            "Footprint {} position mismatch: expected {:?}, got {:?}",
            reference, pos, fp.position
        );

        for pad in &fp.pads {
            match pad.pad_type {
                PadType::ThroughHole { .. } => tht_pad_count += 1,
                PadType::Smd { layer } => {
                    smd_pad_count += 1;
                    assert_eq!(layer, Layer::FCu, "SMD pad must be on Top Copper (F.Cu)");
                }
            }
        }
    }

    assert_eq!(tht_pad_count, 34, "Must have exactly 34 through-hole pins (15 J2 + 15 J3 + 4 J1 tabs)");
    assert_eq!(smd_pad_count, 64, "Must have exactly 64 surface mount pads");
    assert_eq!(tht_pad_count + smd_pad_count, 98, "Total pads must equal 98");
}

// ============================================================================
// 7. Header Vertical Alignment & Symmetrical Row Spacing
// ============================================================================

#[test]
fn test_m5_challenge_header_vertical_alignment_and_pin_spacing() {
    let board = create_reference_board();

    let j2 = board.find_footprint("J2").unwrap();
    let j3 = board.find_footprint("J3").unwrap();

    let j2_pads = j2.all_pad_world_positions();
    let j3_pads = j3.all_pad_world_positions();

    assert_eq!(j2_pads.len(), 15);
    assert_eq!(j3_pads.len(), 15);

    // Row-to-row spacing
    let row_spacing: f64 = 29.46 - 2.54;
    assert!((row_spacing - 26.92).abs() < 1e-6, "Row spacing must be 26.92 mm");

    for i in 0..15 {
        let (_p2, pos2) = &j2_pads[i];
        let (_p3, pos3) = &j3_pads[i];

        // X coordinate of pin i in J2 must equal X coordinate of pin i in J3
        assert!(
            (pos2.x - pos3.x).abs() < 1e-6,
            "Pin {} vertical misalignment: J2.x = {:.4}, J3.x = {:.4}",
            i + 1, pos2.x, pos3.x
        );

        // Distance between corresponding top and bottom pins must be exactly 26.92 mm
        let vertical_span = pos3.y - pos2.y;
        assert!(
            (vertical_span - row_spacing).abs() < 1e-6,
            "Pin {} vertical span mismatch: expected {:.4}, got {:.4}",
            i + 1, row_spacing, vertical_span
        );
    }
}

// ============================================================================
// 8. Continuous Ratsnest Grid Sweep & Topological Invariant
// ============================================================================

#[test]
fn test_m5_challenge_ratsnest_continuous_grid_sweep_and_topological_invariants() {
    let mut board = create_reference_board();

    // 1. Verify net-by-net ratsnest line count breakdown
    let baseline = compute_ratsnest(&board);
    assert_eq!(baseline.len(), 31);

    let mut net_edge_counts: std::collections::HashMap<NetId, usize> = std::collections::HashMap::new();
    for line in &baseline {
        *net_edge_counts.entry(line.net).or_insert(0) += 1;
    }

    let gnd_id = board.get_or_create_net("GND");
    let net_3v3 = board.get_or_create_net("3V3");
    let net_5v = board.get_or_create_net("5V");
    let net_reset = board.get_or_create_net("RESET");
    let net_xtal1 = board.get_or_create_net("XTAL1");
    let net_xtal2 = board.get_or_create_net("XTAL2");

    assert_eq!(net_edge_counts.get(&gnd_id).copied().unwrap_or(0), 10, "GND must have 10 lines");
    assert_eq!(net_edge_counts.get(&net_3v3).copied().unwrap_or(0), 5, "3V3 must have 5 lines");
    assert_eq!(net_edge_counts.get(&net_5v).copied().unwrap_or(0), 2, "5V must have 2 lines");
    assert_eq!(net_edge_counts.get(&net_reset).copied().unwrap_or(0), 2, "RESET must have 2 lines");
    assert_eq!(net_edge_counts.get(&net_xtal1).copied().unwrap_or(0), 1, "XTAL1 must have 1 line");
    assert_eq!(net_edge_counts.get(&net_xtal2).copied().unwrap_or(0), 1, "XTAL2 must have 1 line");

    // 2. Continuous 2D grid sweep of MCU U1 displacement
    let original_u1_pos = board.find_footprint("U1").unwrap().position;
    let offsets = [-1.0, -0.5, 0.0, 0.5, 1.0];

    for &dx in &offsets {
        for &dy in &offsets {
            board.find_footprint_mut("U1").unwrap().position = original_u1_pos + Point::new(dx, dy);
            let ratsnest = compute_ratsnest(&board);
            assert_eq!(ratsnest.len(), 31, "Grid sweep ({}, {}) line count drifted", dx, dy);
            for line in &ratsnest {
                assert!(line.start.distance_to(line.end) > 0.001, "Degenerate zero-length ratsnest line");
                assert!(line.net > 0);
            }
        }
    }

    // Restore U1
    board.find_footprint_mut("U1").unwrap().position = original_u1_pos;
}

// ============================================================================
// 9. Fabrication Package Drill Hits & S-Expression CAM Audit
// ============================================================================

#[test]
fn test_m5_challenge_cam_exports_drill_counts_and_s_expr_structure() {
    let board = create_reference_board();

    // 1. Excellon Drill Export: Count drill hits on unrouted board
    let drl = export_excellon_drill(&board);
    let hit_count = drl.lines().filter(|l| l.starts_with('X') && l.contains('Y')).count();
    assert_eq!(
        hit_count, 34,
        "Unrouted reference board drill file must have exactly 34 drill hits (15 J2 + 15 J3 + 4 J1 tabs)"
    );

    // 2. KiCad PCB S-Expression Export
    let kicad = export_kicad_pcb(&board);
    assert!(kicad.starts_with("(kicad_pcb"), "Must start with (kicad_pcb");
    assert!(kicad.contains("(version 20240108)"), "Must declare KiCad version 20240108");

    // Footprint package names verification
    let expected_packages = [
        "TQFP-32_7x7mm_P0.8mm",
        "SOT-223-3_TabPin2",
        "USB_C_Receptacle_16Pin",
        "Crystal_SMD_3225-4Pin",
        "C_0805_2012Metric",
        "R_0805_2012Metric",
        "SW_Push_SPST_NO_3x4mm",
        "PinHeader_1x15_P2.54mm_Vertical",
    ];

    for pkg in expected_packages {
        assert!(kicad.contains(pkg), "KiCad export missing package '{}'", pkg);
    }
}

// ============================================================================
// 10. End-to-End Autorouting & Post-Route DRC Stress Verification
// ============================================================================

#[test]
fn test_m5_challenge_full_autoroute_and_post_route_drc_stress() {
    let mut board = create_reference_board();

    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    };

    let result = route_board(&mut board, &config);
    assert!(result.is_ok(), "Autorouting reference board must succeed: {:?}", result);

    assert!(board.tracks.len() >= 80, "Must generate >= 80 track segments, found {}", board.tracks.len());
    assert!(board.vias.len() >= 20, "Must generate >= 20 vias, found {}", board.vias.len());

    let drc = run_drc(&board, &config);
    assert!(drc.is_clean(), "Post-route DRC must be 100% clean: {:?}", drc);
    assert_eq!(drc.total_violations(), 0);
    assert_eq!(drc.unrouted_nets.len(), 0);

    // Verify CNC drill deduplication consistency:
    // The exact via count varies with routing strategy — what matters is that the
    // drill file accurately reflects the board's unique via locations.
    let mut unique_via_locs: Vec<Point> = board.vias.iter().map(|v| v.at).collect();
    unique_via_locs.sort_by(|a, b| a.y.partial_cmp(&b.y).unwrap().then(a.x.partial_cmp(&b.x).unwrap()));
    unique_via_locs.dedup_by(|a, b| (a.x - b.x).abs() < 1e-4 && (a.y - b.y).abs() < 1e-4);

    let unique_via_count = unique_via_locs.len();
    assert!(unique_via_count >= 30, "Must have >= 30 unique via drill locations, found {}", unique_via_count);

    let routed_drl = export_excellon_drill(&board);
    let routed_hits = routed_drl.lines().filter(|l| l.starts_with('X') && l.contains('Y')).count();
    let expected_hits = 34 + unique_via_count;
    assert_eq!(
        routed_hits, expected_hits,
        "Routed drill file hits ({}) must equal 34 THT + {} unique vias = {}",
        routed_hits, unique_via_count, expected_hits
    );
}

