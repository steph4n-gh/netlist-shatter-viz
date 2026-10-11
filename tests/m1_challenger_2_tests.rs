//! Empirical Challenger 2 Test Harness for Milestone 1
//!
//! Stress-tests:
//! 1. All 7 parametric footprint generators (pad counts, layer assignments, pin uniqueness, geometry).
//! 2. Pin mapping & schematic netlist assignment collisions.
//! 3. PcbBoard::validate catching out-of-bounds pads (including rotated footprints),
//!    negative/zero annular rings, duplicate pin numbers, and degenerate geometries.
//! 4. Edge cases: Boundary tolerances, multi-pin net assignment, idempotence,
//!    custom pin names, negative angle rotations, and non-standard geometries.

use netlist_shatter_viz::pcb::core::{
    Footprint, IngestionError, Layer, NetPinRef, Pad, PadShape, PadType,
    PcbBoard, Point, SchematicNet, SchematicNetlist, ValidationError,
};
use netlist_shatter_viz::pcb::footprint::*;
use std::collections::HashSet;

// ============================================================================
// 1. Footprint Generators & Layer Assignment Checks
// ============================================================================

#[test]
fn test_all_7_parametric_footprints_validity_and_counts() {
    // 1. TQFP-32
    let tqfp = tqfp32("U1", "ATmega328P");
    assert_eq!(tqfp.pads.len(), 32, "TQFP-32 must have exactly 32 pads");
    assert_eq!(tqfp.reference, "U1");
    let mut seen_pins = HashSet::new();
    for (i, pad) in tqfp.pads.iter().enumerate() {
        assert_eq!(pad.pin_number, (i + 1).to_string());
        assert!(seen_pins.insert(&pad.pin_number), "Pin numbers must be unique");
        assert_eq!(pad.pad_type, PadType::Smd { layer: Layer::FCu });
        match pad.shape {
            PadShape::Rect { width, height } => {
                assert!(width > 0.0 && height > 0.0);
            }
            _ => panic!("TQFP-32 pads must be Rect"),
        }
    }
    assert!(tqfp.validate().is_empty(), "TQFP-32 footprint must pass validation");
    assert!(!tqfp.silkscreen_lines.is_empty(), "Silkscreen lines should be present");

    // 2. SOT-223
    let sot223_fp = sot223("U2", "AMS1117-3.3");
    assert_eq!(sot223_fp.pads.len(), 4, "SOT-223 must have 4 pads (3 signal + 1 tab)");
    let mut seen_sot223 = HashSet::new();
    for pad in &sot223_fp.pads {
        assert!(seen_sot223.insert(&pad.pin_number));
        assert_eq!(pad.pad_type, PadType::Smd { layer: Layer::FCu });
        match pad.shape {
            PadShape::Rect { width, height } => {
                assert!(width > 0.0 && height > 0.0);
            }
            _ => panic!("SOT-223 pads must be Rect"),
        }
    }
    assert!(sot223_fp.validate().is_empty(), "SOT-223 footprint must pass validation");

    // 3. SOT-23
    let sot23_fp = sot23("Q1", "MMBT3904");
    assert_eq!(sot23_fp.pads.len(), 3, "SOT-23 must have 3 pads");
    let mut seen_sot23 = HashSet::new();
    for pad in &sot23_fp.pads {
        assert!(seen_sot23.insert(&pad.pin_number));
        assert_eq!(pad.pad_type, PadType::Smd { layer: Layer::FCu });
    }
    assert!(sot23_fp.validate().is_empty(), "SOT-23 footprint must pass validation");

    // 4. USB-C 16-pin
    let usb = usb_c_16pin("J1", "USB-C-16P");
    assert_eq!(usb.pads.len(), 16, "USB-C must have 16 pads");
    let smd_count = usb.pads.iter().filter(|p| p.pad_type.is_smd()).count();
    let tht_count = usb.pads.iter().filter(|p| p.pad_type.is_through_hole()).count();
    assert_eq!(smd_count, 12, "USB-C must have 12 SMD pads");
    assert_eq!(tht_count, 4, "USB-C must have 4 THT shield tabs");
    let mut seen_usb = HashSet::new();
    for pad in &usb.pads {
        assert!(seen_usb.insert(&pad.pin_number));
        if let PadType::Smd { layer } = pad.pad_type {
            assert_eq!(layer, Layer::FCu);
        } else if let PadType::ThroughHole { drill, outer_diameter } = pad.pad_type {
            assert!(drill > 0.0);
            assert!(outer_diameter > drill);
            assert!(pad.annular_ring().unwrap() >= 0.3);
        }
    }
    assert!(usb.validate().is_empty(), "USB-C footprint must pass validation");

    // 5. 0805 Passives (Resistor, Capacitor, LED)
    let r0805 = resistor_0805("R1", "10k");
    assert_eq!(r0805.pads.len(), 2, "0805 Resistor must have 2 pads");
    assert!(r0805.validate().is_empty());
    for pad in &r0805.pads {
        assert_eq!(pad.pad_type, PadType::Smd { layer: Layer::FCu });
    }

    let c0805 = capacitor_0805("C1", "100nF");
    assert_eq!(c0805.pads.len(), 2, "0805 Capacitor must have 2 pads");
    assert!(c0805.validate().is_empty());
    for pad in &c0805.pads {
        assert_eq!(pad.pad_type, PadType::Smd { layer: Layer::FCu });
    }

    let d0805 = led_0805("D1", "Green");
    assert_eq!(d0805.pads.len(), 2, "0805 LED must have 2 pads");
    assert_eq!(d0805.pads[0].pin_name, "K");
    assert_eq!(d0805.pads[1].pin_name, "A");
    assert!(d0805.validate().is_empty());
    for pad in &d0805.pads {
        assert_eq!(pad.pad_type, PadType::Smd { layer: Layer::FCu });
    }

    // 6. Crystal Resonators (HC-49/US SMD and SMD 3225)
    let xtal_hc49 = crystal_hc49_smd("Y1", "16MHz");
    assert_eq!(xtal_hc49.pads.len(), 3, "HC-49 SMD crystal must have 3 pads");
    assert!(xtal_hc49.validate().is_empty());
    for pad in &xtal_hc49.pads {
        assert_eq!(pad.pad_type, PadType::Smd { layer: Layer::FCu });
    }

    let xtal_3225 = crystal_smd_3225("Y2", "16MHz");
    assert_eq!(xtal_3225.pads.len(), 4, "3225 SMD crystal must have 4 pads");
    assert!(xtal_3225.validate().is_empty());
    for pad in &xtal_3225.pads {
        assert_eq!(pad.pad_type, PadType::Smd { layer: Layer::FCu });
    }

    // 7. 0.1" (2.54mm) Headers (Single and Dual 1x15)
    let hdr = header_1x15("J2", "CONN_15");
    assert_eq!(hdr.pads.len(), 15, "1x15 header must have 15 pads");
    assert!(hdr.validate().is_empty());
    for (i, pad) in hdr.pads.iter().enumerate() {
        assert!(pad.pad_type.is_through_hole());
        let ring = pad.annular_ring().expect("Must have annular ring");
        assert!(ring >= 0.35, "Annular ring must be at least 0.35mm");
        if i == 0 {
            assert!(matches!(pad.shape, PadShape::Rect { .. }), "Pin 1 must be square");
        } else {
            assert!(matches!(pad.shape, PadShape::Circle { .. }), "Pins 2..15 must be circular");
        }
    }

    let (hdr_left, hdr_right) = dual_headers_1x15("J_L", "J_R", 15.24, &[], &[]);
    assert_eq!(hdr_left.pads.len(), 15);
    assert_eq!(hdr_right.pads.len(), 15);
    assert!(hdr_left.validate().is_empty());
    assert!(hdr_right.validate().is_empty());

    // Additional: Tactile Switch
    let sw = tactile_switch_3x4("SW1", "RESET");
    assert_eq!(sw.pads.len(), 4);
    assert!(sw.validate().is_empty());
    for pad in &sw.pads {
        assert_eq!(pad.pad_type, PadType::Smd { layer: Layer::FCu });
    }
}

// ============================================================================
// 2. Netlist Ingestion & Assignment Collisions
// ============================================================================

#[test]
fn test_netlist_pin_mapping_and_collision_detection() {
    let mut board = PcbBoard::new(50.0, 50.0);
    let r1 = resistor_0805("R1", "10k").at_position(Point::new(10.0, 10.0));
    let r2 = resistor_0805("R2", "10k").at_position(Point::new(20.0, 10.0));
    board.add_footprint(r1);
    board.add_footprint(r2);

    // Initial assignment by pin_number
    assert!(board.assign_pad_net("R1", "1", "VCC").is_ok());
    assert!(board.assign_pad_net("R1", "2", "GND").is_ok());

    // Re-assignment of same net to same pin succeeds (idempotent)
    assert!(board.assign_pad_net("R1", "1", "VCC").is_ok());

    // Re-assignment of DIFFERENT net to same pin triggers DuplicatePinAssignment error
    let collision_err = board.assign_pad_net("R1", "1", "NET_SIGNAL");
    match collision_err {
        Err(IngestionError::DuplicatePinAssignment {
            component_ref,
            pin_id,
            existing_net_name,
            new_net_name,
        }) => {
            assert_eq!(component_ref, "R1");
            assert_eq!(pin_id, "1");
            assert_eq!(existing_net_name, "VCC");
            assert_eq!(new_net_name, "NET_SIGNAL");
        }
        other => panic!("Expected DuplicatePinAssignment error, got {:?}", other),
    }

    // Assignment to nonexistent component
    let no_comp = board.assign_pad_net("R99", "1", "VCC");
    assert!(matches!(no_comp, Err(IngestionError::ComponentNotFound { .. })));

    // Assignment to nonexistent pin
    let no_pin = board.assign_pad_net("R1", "99", "VCC");
    assert!(matches!(no_pin, Err(IngestionError::PinNotFound { .. })));

    // Ingestion via SchematicNetlist
    let netlist = SchematicNetlist {
        nets: vec![
            SchematicNet {
                name: "DATA".to_string(),
                pins: vec![
                    NetPinRef {
                        component_ref: "R2".to_string(),
                        pin_id: "1".to_string(),
                    },
                ],
            },
            // Colliding entry: assigning R2 pin 1 to CLK
            SchematicNet {
                name: "CLK".to_string(),
                pins: vec![
                    NetPinRef {
                        component_ref: "R2".to_string(),
                        pin_id: "1".to_string(),
                    },
                ],
            },
        ],
    };

    let result = board.apply_netlist(&netlist);
    assert!(result.is_err(), "Colliding netlist must fail");
    assert!(matches!(
        result,
        Err(IngestionError::DuplicatePinAssignment { .. })
    ));
}

#[test]
fn test_multi_pin_same_net_assignment_and_pin_name_aliasing() {
    let mut board = PcbBoard::new(60.0, 60.0);
    let u1 = tqfp32("U1", "MCU").at_position(Point::new(30.0, 30.0));
    board.add_footprint(u1);

    // TQFP-32 has multiple GND pins (pin 3, 5, 21)
    // Multiple pins assigned to same net "GND" MUST succeed without collision
    assert!(board.assign_pad_net("U1", "3", "GND").is_ok());
    assert!(board.assign_pad_net("U1", "5", "GND").is_ok());
    assert!(board.assign_pad_net("U1", "21", "GND").is_ok());

    // Pin name mapping: U1 pin 4 has pin_name "VCC"
    assert!(board.assign_pad_net("U1", "VCC", "VCC_NET").is_ok());

    // Verification of assigned nets
    let fp = board.find_footprint("U1").unwrap();
    let pad_gnd3 = fp.pad("3").unwrap();
    let pad_gnd5 = fp.pad("5").unwrap();
    let pad_vcc = fp.pad("4").unwrap();

    assert!(pad_gnd3.net.is_some());
    assert_eq!(pad_gnd3.net, pad_gnd5.net, "Pads assigned to same net must share NetId");
    assert_ne!(pad_gnd3.net, pad_vcc.net, "Different nets must have different NetIds");

    // Idempotent re-application of netlist
    let netlist = SchematicNetlist {
        nets: vec![
            SchematicNet {
                name: "GND".to_string(),
                pins: vec![
                    NetPinRef { component_ref: "U1".to_string(), pin_id: "3".to_string() },
                    NetPinRef { component_ref: "U1".to_string(), pin_id: "5".to_string() },
                ],
            },
        ],
    };
    assert!(board.apply_netlist(&netlist).is_ok(), "Applying same netlist twice must be idempotent");
}

// ============================================================================
// 3. Board Geometry Validation: Out-of-bounds Pads & Rotations
// ============================================================================

#[test]
fn test_board_validate_catches_out_of_bounds_pads() {
    let mut board = PcbBoard::new(20.0, 20.0);

    // Place resistor centered at (10.0, 10.0) -> fully within 20x20 board
    let r1 = resistor_0805("R1", "10k").at_position(Point::new(10.0, 10.0));
    board.add_footprint(r1);
    let report = board.validate();
    assert!(report.is_valid(), "Well-placed footprint must be valid: {:?}", report.errors);

    // Place footprint partially hanging off the right edge:
    // Center at (19.8, 10.0). Resistor pads extend to +0.95 + 0.50 = +1.45 mm from center.
    // 19.8 + 1.45 = 21.25 > 20.0 (board width).
    let r_off_right = resistor_0805("R_RIGHT", "10k").at_position(Point::new(19.8, 10.0));
    let mut board2 = PcbBoard::new(20.0, 20.0);
    board2.add_footprint(r_off_right);
    let report2 = board2.validate();
    assert!(!report2.is_valid(), "Footprint hanging off right edge must fail");
    let oob_found = report2.errors.iter().any(|e| matches!(e, ValidationError::PadOutOfBounds { component_ref, .. } if component_ref == "R_RIGHT"));
    assert!(oob_found, "Must report PadOutOfBounds for R_RIGHT");

    // Place footprint partially hanging off the left edge (negative X)
    let r_off_left = resistor_0805("R_LEFT", "10k").at_position(Point::new(0.2, 10.0));
    let mut board3 = PcbBoard::new(20.0, 20.0);
    board3.add_footprint(r_off_left);
    let report3 = board3.validate();
    assert!(!report3.is_valid(), "Footprint hanging off left edge must fail");
    let oob_found_left = report3.errors.iter().any(|e| matches!(e, ValidationError::PadOutOfBounds { component_ref, .. } if component_ref == "R_LEFT"));
    assert!(oob_found_left, "Must report PadOutOfBounds for R_LEFT");

    // Place footprint at negative coordinates completely outside board
    let r_neg = resistor_0805("R_NEG", "10k").at_position(Point::new(-5.0, -5.0));
    let mut board4 = PcbBoard::new(20.0, 20.0);
    board4.add_footprint(r_neg);
    let report4 = board4.validate();
    assert!(!report4.is_valid());
    let oob_count = report4.errors.iter().filter(|e| matches!(e, ValidationError::PadOutOfBounds { .. })).count();
    assert_eq!(oob_count, 2, "Both pads of R_NEG must be out of bounds");
}

#[test]
fn test_board_validate_catches_rotation_induced_out_of_bounds() {
    let mut board = PcbBoard::new(20.0, 20.0);

    // TQFP-32 has pads extending out to ~4.25 + 0.75 = 5.0 mm from center.
    // If placed at (4.5, 4.5), pads extend from -0.5 to 9.5 -> pad at -0.5 is OOB.
    let tqfp = tqfp32("U1", "MCU").at_position(Point::new(4.5, 4.5));
    board.add_footprint(tqfp);
    let report = board.validate();
    assert!(!report.is_valid());
    assert!(report.errors.iter().any(|e| matches!(e, ValidationError::PadOutOfBounds { .. })));

    // Now test rotation effect:
    // Header 1x15 has length (14 * 2.54) = 35.56 mm.
    // On a 30 x 40 mm board:
    // When orientation is vertical (0 deg), Y length is ~37 mm -> fits in 40 mm height.
    // But when rotated 90 deg, X length becomes ~37 mm -> exceeds 30 mm width!
    let mut board_rot = PcbBoard::new(30.0, 40.0);
    let hdr_rot = header_1x15("J1", "EXP").at_position(Point::new(5.0, 2.0)).with_rotation(90.0);
    board_rot.add_footprint(hdr_rot);
    let rot_report = board_rot.validate();
    assert!(!rot_report.is_valid(), "Rotated header exceeding board width must fail");
    assert!(rot_report.errors.iter().any(|e| matches!(e, ValidationError::PadOutOfBounds { .. })));

    // Negative angle rotation test (-90 degrees)
    let mut board_neg_rot = PcbBoard::new(30.0, 40.0);
    let hdr_neg_rot = header_1x15("J2", "EXP").at_position(Point::new(5.0, 38.0)).with_rotation(-90.0);
    board_neg_rot.add_footprint(hdr_neg_rot);
    let neg_rot_report = board_neg_rot.validate();
    assert!(!neg_rot_report.is_valid(), "Negative rotated header exceeding width must fail");
    assert!(neg_rot_report.errors.iter().any(|e| matches!(e, ValidationError::PadOutOfBounds { .. })));
}

#[test]
fn test_board_validate_boundary_tolerances() {
    let board_w = 20.0;
    let board_h = 20.0;
    let mut board = PcbBoard::new(board_w, board_h);

    // Create a 1.0 x 1.0 mm pad whose bounding box touches x = 0.0 exactly:
    // Center at (0.5, 10.0), width 1.0 -> min.x = 0.0, max.x = 1.0.
    // This should PASS (within board boundary).
    let pad_flush = Pad::new_smd("1", "FLUSH", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu);
    let fp_flush = Footprint::new("FLUSH", "V", "PKG", Point::new(0.5, 10.0), 0.0, vec![pad_flush], vec![]);
    board.add_footprint(fp_flush);

    // Another pad whose bounding box touches x = 20.0 exactly:
    // Center at (19.5, 10.0), width 1.0 -> min.x = 19.0, max.x = 20.0.
    let pad_flush_max = Pad::new_smd("1", "FLUSH_MAX", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu);
    let fp_flush_max = Footprint::new("FLUSH_MAX", "V", "PKG", Point::new(19.5, 10.0), 0.0, vec![pad_flush_max], vec![]);
    board.add_footprint(fp_flush_max);

    let report = board.validate();
    assert!(report.is_valid(), "Pads flush with board boundaries must be valid: {:?}", report.errors);

    // Now move pad 1.1e-6 outside boundary (beyond 1e-6 epsilon tolerance)
    let mut board_oob = PcbBoard::new(board_w, board_h);
    let pad_slight_oob = Pad::new_smd("1", "SLIGHT_OOB", Point::zero(), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu);
    let fp_slight_oob = Footprint::new("SLIGHT_OOB", "V", "PKG", Point::new(0.5 - 2e-6, 10.0), 0.0, vec![pad_slight_oob], vec![]);
    board_oob.add_footprint(fp_slight_oob);

    let report_oob = board_oob.validate();
    assert!(!report_oob.is_valid(), "Pad exceeding boundary beyond epsilon must be caught");
    assert!(report_oob.errors.iter().any(|e| matches!(e, ValidationError::PadOutOfBounds { .. })));
}

// ============================================================================
// 4. Board Geometry Validation: Annular Rings & Invalid Drill
// ============================================================================

#[test]
fn test_board_validate_catches_negative_and_zero_annular_rings() {
    let mut board = PcbBoard::new(30.0, 30.0);

    // Create a footprint with an invalid through-hole pad: drill = 1.0, outer = 0.8 (outer < drill)
    let bad_pad = Pad::new_tht(
        "1",
        "P1",
        Point::zero(),
        PadShape::Circle { diameter: 0.8 },
        1.0, // drill > outer
        0.8,
    );
    let bad_fp = Footprint::new(
        "BAD_THT",
        "BAD",
        "TEST_PKG",
        Point::new(15.0, 15.0),
        0.0,
        vec![bad_pad],
        vec![],
    );
    board.add_footprint(bad_fp);
    let report = board.validate();
    assert!(!report.is_valid());
    let drill_err = report.errors.iter().find(|e| matches!(e, ValidationError::InvalidThroughHoleDrill { .. }));
    assert!(drill_err.is_some(), "Must report InvalidThroughHoleDrill when drill > outer_diameter");

    // Drill == outer_diameter (zero annular ring)
    let mut board2 = PcbBoard::new(30.0, 30.0);
    let zero_ring_pad = Pad::new_tht(
        "1",
        "P1",
        Point::zero(),
        PadShape::Circle { diameter: 1.0 },
        1.0, // drill == outer
        1.0,
    );
    let zero_ring_fp = Footprint::new(
        "ZERO_RING",
        "BAD",
        "TEST_PKG",
        Point::new(15.0, 15.0),
        0.0,
        vec![zero_ring_pad],
        vec![],
    );
    board2.add_footprint(zero_ring_fp);
    let report2 = board2.validate();
    assert!(!report2.is_valid());
    assert!(report2.errors.iter().any(|e| matches!(e, ValidationError::InvalidThroughHoleDrill { .. })));

    // Negative drill diameter (drill <= 0.0)
    let mut board3 = PcbBoard::new(30.0, 30.0);
    let neg_drill_pad = Pad::new_tht(
        "1",
        "P1",
        Point::zero(),
        PadShape::Circle { diameter: 1.5 },
        -0.5,
        1.5,
    );
    let neg_drill_fp = Footprint::new(
        "NEG_DRILL",
        "BAD",
        "TEST_PKG",
        Point::new(15.0, 15.0),
        0.0,
        vec![neg_drill_pad],
        vec![],
    );
    board3.add_footprint(neg_drill_fp);
    let report3 = board3.validate();
    assert!(!report3.is_valid());
    assert!(report3.errors.iter().any(|e| matches!(e, ValidationError::InvalidThroughHoleDrill { .. })));
}

// ============================================================================
// 5. Board Geometry Validation: Duplicate Pin Numbers & Degenerate Geometries
// ============================================================================

#[test]
fn test_board_validate_catches_duplicate_pin_numbers() {
    let mut board = PcbBoard::new(30.0, 30.0);
    let dup_pad1 = Pad::new_smd("1", "PIN_A", Point::new(-1.0, 0.0), PadShape::Circle { diameter: 1.0 }, Layer::FCu);
    let dup_pad2 = Pad::new_smd("1", "PIN_B", Point::new(1.0, 0.0), PadShape::Circle { diameter: 1.0 }, Layer::FCu);
    let dup_fp = Footprint::new(
        "DUP_PINS",
        "BAD",
        "TEST_PKG",
        Point::new(15.0, 15.0),
        0.0,
        vec![dup_pad1, dup_pad2],
        vec![],
    );
    board.add_footprint(dup_fp);
    let report = board.validate();
    assert!(!report.is_valid());
    let dup_err = report.errors.iter().find(|e| matches!(e, ValidationError::DuplicatePinNumber { component_ref, pin_number } if component_ref == "DUP_PINS" && pin_number == "1"));
    assert!(dup_err.is_some(), "Must report DuplicatePinNumber error");
}

#[test]
fn test_board_validate_catches_degenerate_geometries() {
    // 1. Zero or negative pad dimensions
    let mut board1 = PcbBoard::new(30.0, 30.0);
    let zero_pad = Pad::new_smd("1", "P1", Point::zero(), PadShape::Rect { width: 0.0, height: 1.0 }, Layer::FCu);
    let zero_fp = Footprint::new("ZERO_PAD", "V", "PKG", Point::new(15.0, 15.0), 0.0, vec![zero_pad], vec![]);
    board1.add_footprint(zero_fp);
    let report1 = board1.validate();
    assert!(!report1.is_valid());
    assert!(report1.errors.iter().any(|e| matches!(e, ValidationError::InvalidPadDimensions { .. })));

    let mut board2 = PcbBoard::new(30.0, 30.0);
    let neg_circle = Pad::new_smd("1", "P1", Point::zero(), PadShape::Circle { diameter: -2.0 }, Layer::FCu);
    let neg_fp = Footprint::new("NEG_PAD", "V", "PKG", Point::new(15.0, 15.0), 0.0, vec![neg_circle], vec![]);
    board2.add_footprint(neg_fp);
    let report2 = board2.validate();
    assert!(!report2.is_valid());
    assert!(report2.errors.iter().any(|e| matches!(e, ValidationError::InvalidPadDimensions { .. })));

    let mut board2b = PcbBoard::new(30.0, 30.0);
    let neg_obround = Pad::new_smd("1", "P1", Point::zero(), PadShape::Obround { width: 1.0, height: 0.0 }, Layer::FCu);
    let neg_obround_fp = Footprint::new("NEG_OBROUND", "V", "PKG", Point::new(15.0, 15.0), 0.0, vec![neg_obround], vec![]);
    board2b.add_footprint(neg_obround_fp);
    let report2b = board2b.validate();
    assert!(!report2b.is_valid());
    assert!(report2b.errors.iter().any(|e| matches!(e, ValidationError::InvalidPadDimensions { .. })));

    // 2. Empty footprint
    let mut board3 = PcbBoard::new(30.0, 30.0);
    let empty_fp = Footprint::new("EMPTY", "V", "PKG", Point::new(15.0, 15.0), 0.0, vec![], vec![]);
    board3.add_footprint(empty_fp);
    let report3 = board3.validate();
    assert!(!report3.is_valid());
    assert!(report3.errors.iter().any(|e| matches!(e, ValidationError::EmptyFootprint { .. })));

    // 3. Invalid board dimensions (zero or negative)
    let bad_board = PcbBoard::new(0.0, 10.0);
    let report4 = bad_board.validate();
    assert!(!report4.is_valid());
    assert!(report4.errors.iter().any(|e| matches!(e, ValidationError::InvalidBoardDimensions { .. })));

    let bad_board_neg = PcbBoard::new(-10.0, -10.0);
    let report5 = bad_board_neg.validate();
    assert!(!report5.is_valid());
    assert!(report5.errors.iter().any(|e| matches!(e, ValidationError::InvalidBoardDimensions { .. })));

    // 4. Non-finite coordinates (NaN / Inf)
    let mut board6 = PcbBoard::new(30.0, 30.0);
    let nan_pad = Pad::new_smd("1", "P1", Point::new(f64::NAN, 0.0), PadShape::Circle { diameter: 1.0 }, Layer::FCu);
    let nan_fp = Footprint::new("NAN_FP", "V", "PKG", Point::new(15.0, 15.0), 0.0, vec![nan_pad], vec![]);
    board6.add_footprint(nan_fp);
    let report6 = board6.validate();
    assert!(!report6.is_valid());
    assert!(report6.errors.iter().any(|e| matches!(e, ValidationError::NonFiniteCoordinate { .. })));

    // 5. Invalid SMD pad layer (e.g. EdgeCuts or FSilkscreen assigned to SMD pad)
    let mut board7 = PcbBoard::new(30.0, 30.0);
    let bad_layer_pad = Pad::new_smd("1", "P1", Point::zero(), PadShape::Circle { diameter: 1.0 }, Layer::EdgeCuts);
    let bad_layer_fp = Footprint::new("BAD_LAYER", "V", "PKG", Point::new(15.0, 15.0), 0.0, vec![bad_layer_pad], vec![]);
    board7.add_footprint(bad_layer_fp);
    let report7 = board7.validate();
    assert!(!report7.is_valid());
    assert!(report7.errors.iter().any(|e| matches!(e, ValidationError::InvalidPadLayer { .. })));
}
