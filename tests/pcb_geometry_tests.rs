//! E2E Test Suite: PCB Physical Geometry, Footprints & Transformations (Tiers 1 & 2)
//!
//! Verifies:
//! - Canonical mm-based float points, vectors, and exact orthogonal rotations (F1)
//! - Millimeter-accurate Rect, Circle, and Obround pad geometries with drill/annular rings (F2)
//! - Parametric component footprint generators (TQFP-32, SOT-223, SOT-23, USB-C, 0805, Crystal, Header 1x15) (F3)
//! - Closed Edge.Cuts board outline polygon definition, shoelace area, containment, and edge distance (F4)
//! - Tier 2 boundary cases: degenerate dimensions, extreme coordinates, multi-turn rotations

use netlist_shatter_viz::pcb::*;
use std::collections::HashMap;

// ============================================================================
// Tier 1: Canonical Points, Vectors, and Exact Orthogonal Rotations
// ============================================================================

#[test]
fn test_point_instantiation_and_fields() {
    let pt = Point { x: 12.5, y: -8.25 };
    assert_eq!(pt.x, 12.5);
    assert_eq!(pt.y, -8.25);

    let pt_new = Point::new(3.0, 4.0);
    assert_eq!(pt_new.x, 3.0);
    assert_eq!(pt_new.y, 4.0);
}

#[test]
fn test_point_euclidean_distance() {
    let p1 = Point::new(0.0, 0.0);
    let p2 = Point::new(3.0, 4.0);
    assert!((p1.distance_to(p2) - 5.0).abs() < 1e-9);
    assert!((p2.distance_to(p1) - 5.0).abs() < 1e-9);

    let p3 = Point::new(1.0, 1.0);
    assert!((p3.distance_to(p3) - 0.0).abs() < 1e-12);
}

#[test]
fn test_point_midpoint() {
    let p1 = Point::new(2.0, 6.0);
    let p2 = Point::new(8.0, 10.0);
    let mid = p1.midpoint(p2);
    assert!((mid.x - 5.0).abs() < 1e-9);
    assert!((mid.y - 8.0).abs() < 1e-9);
}

#[test]
fn test_point_vector_arithmetic() {
    let p1 = Point::new(5.0, 10.0);
    let p2 = Point::new(2.0, 3.0);
    
    let sum = p1 + p2;
    assert_eq!(sum.x, 7.0);
    assert_eq!(sum.y, 13.0);

    let diff = p1 - p2;
    assert_eq!(diff.x, 3.0);
    assert_eq!(diff.y, 7.0);

    let scaled = p2.scale(2.5);
    assert_eq!(scaled.x, 5.0);
    assert_eq!(scaled.y, 7.5);

    assert_eq!(p1.dot(p2), 5.0 * 2.0 + 10.0 * 3.0);
    assert_eq!(p2.length_sq(), 4.0 + 9.0);
}

#[test]
fn test_exact_orthogonal_rotation_drift_elimination() {
    // Rotating (1.0, 0.0) 4 times by 90 degrees must return EXACTLY (1.0, 0.0) without float drift
    let origin = Point::new(0.0, 0.0);
    let p = Point::new(5.0, 0.0);

    let r90 = p.rotate_around(origin, 90.0);
    assert!((r90.x - 0.0).abs() < 1e-12, "Expected x=0 for 90 deg rotation, got {}", r90.x);
    assert!((r90.y - 5.0).abs() < 1e-12, "Expected y=5 for 90 deg rotation, got {}", r90.y);

    let r180 = p.rotate_around(origin, 180.0);
    assert!((r180.x - (-5.0)).abs() < 1e-12);
    assert!((r180.y - 0.0).abs() < 1e-12);

    let r270 = p.rotate_around(origin, 270.0);
    assert!((r270.x - 0.0).abs() < 1e-12);
    assert!((r270.y - (-5.0)).abs() < 1e-12);

    let r360 = p.rotate_around(origin, 360.0);
    assert!((r360.x - 5.0).abs() < 1e-12);
    assert!((r360.y - 0.0).abs() < 1e-12);
}

#[test]
fn test_rotation_around_arbitrary_center() {
    let center = Point::new(10.0, 20.0);
    let p = Point::new(12.0, 20.0); // 2mm to the right of center

    let rotated = p.rotate_around(center, 90.0);
    assert!((rotated.x - 10.0).abs() < 1e-9);
    assert!((rotated.y - 22.0).abs() < 1e-9); // 2mm above center

    let rotated180 = p.rotate_around(center, 180.0);
    assert!((rotated180.x - 8.0).abs() < 1e-9); // 2mm to the left of center
    assert!((rotated180.y - 20.0).abs() < 1e-9);
}

// ============================================================================
// Tier 1: Pad Geometry & Annular Rings
// ============================================================================

#[test]
fn test_rectangular_smd_pad_geometry() {
    let pad = Pad::new_smd(
        "1",
        "VCC",
        Point::new(1.0, 2.0),
        PadShape::Rect { width: 1.2, height: 0.5 },
        Layer::FCu,
    );

    assert_eq!(pad.pin_number, "1");
    assert_eq!(pad.pin_name, "VCC");
    assert_eq!(pad.relative_pos.x, 1.0);
    assert_eq!(pad.relative_pos.y, 2.0);

    match pad.shape {
        PadShape::Rect { width, height } => {
            assert_eq!(width, 1.2);
            assert_eq!(height, 0.5);
        }
        _ => panic!("Expected PadShape::Rect"),
    }

    match pad.pad_type {
        PadType::Smd { layer } => assert_eq!(layer, Layer::FCu),
        _ => panic!("Expected PadType::Smd"),
    }

    assert_eq!(pad.annular_ring(), None);
}

#[test]
fn test_circular_tht_pad_geometry_and_annular_ring() {
    let pad = Pad::new_tht(
        "2",
        "GND",
        Point::new(0.0, 0.0),
        PadShape::Circle { diameter: 1.6 },
        0.9,  // drill diameter mm
        1.6,  // outer diameter mm
    );

    assert_eq!(pad.pin_number, "2");
    match pad.shape {
        PadShape::Circle { diameter } => assert_eq!(diameter, 1.6),
        _ => panic!("Expected PadShape::Circle"),
    }

    match pad.pad_type {
        PadType::ThroughHole { drill, outer_diameter } => {
            assert_eq!(drill, 0.9);
            assert_eq!(outer_diameter, 1.6);
        }
        _ => panic!("Expected PadType::ThroughHole"),
    }

    let ring = pad.annular_ring().expect("THT pad must have an annular ring");
    // (outer_diameter - drill) / 2 = (1.6 - 0.9) / 2 = 0.35 mm
    assert!((ring - 0.35).abs() < 1e-9);
}

#[test]
fn test_obround_pad_geometry() {
    let pad = Pad::new_smd(
        "A1",
        "GND",
        Point::new(2.5, 3.5),
        PadShape::Obround { width: 2.0, height: 1.0 },
        Layer::FCu,
    );

    match pad.shape {
        PadShape::Obround { width, height } => {
            assert_eq!(width, 2.0);
            assert_eq!(height, 1.0);
        }
        _ => panic!("Expected PadShape::Obround"),
    }
}

#[test]
fn test_pad_bounding_box_unrotated() {
    let center = Point::new(10.0, 10.0);

    let rect_shape = PadShape::Rect { width: 2.0, height: 1.0 };
    let bbox_rect = rect_shape.bounding_box(center, 0.0);
    assert!((bbox_rect.min.x - 9.0).abs() < 1e-9);
    assert!((bbox_rect.max.x - 11.0).abs() < 1e-9);
    assert!((bbox_rect.min.y - 9.5).abs() < 1e-9);
    assert!((bbox_rect.max.y - 10.5).abs() < 1e-9);
    assert!((bbox_rect.width() - 2.0).abs() < 1e-9);
    assert!((bbox_rect.height() - 1.0).abs() < 1e-9);

    let circle_shape = PadShape::Circle { diameter: 1.6 };
    let bbox_circle = circle_shape.bounding_box(center, 0.0);
    assert!((bbox_circle.width() - 1.6).abs() < 1e-9);
    assert!((bbox_circle.height() - 1.6).abs() < 1e-9);
}

#[test]
fn test_pad_bounding_box_rotated_90() {
    let center = Point::new(10.0, 10.0);
    let rect_shape = PadShape::Rect { width: 2.0, height: 1.0 };
    let bbox_rot90 = rect_shape.bounding_box(center, 90.0);

    // At 90 deg, width and height swap: width becomes 1.0, height becomes 2.0
    assert!((bbox_rot90.width() - 1.0).abs() < 1e-9);
    assert!((bbox_rot90.height() - 2.0).abs() < 1e-9);
}

#[test]
fn test_pad_global_position_transformation() {
    let fp_pos = Point::new(20.0, 30.0);
    let pad = Pad::new_smd(
        "1",
        "SIG",
        Point::new(2.0, 1.0),
        PadShape::Rect { width: 1.0, height: 1.0 },
        Layer::FCu,
    );

    // At 0 deg: (20 + 2, 30 + 1) = (22, 31)
    let g0 = pad.global_position(fp_pos, 0.0);
    assert!((g0.x - 22.0).abs() < 1e-9);
    assert!((g0.y - 31.0).abs() < 1e-9);

    // At 90 deg CCW: relative (2, 1) rotates to (-1, 2)
    // Global: (20 - 1, 30 + 2) = (19, 32)
    let g90 = pad.global_position(fp_pos, 90.0);
    assert!((g90.x - 19.0).abs() < 1e-9);
    assert!((g90.y - 32.0).abs() < 1e-9);

    // At 180 deg: relative (2, 1) rotates to (-2, -1) -> (18, 29)
    let g180 = pad.global_position(fp_pos, 180.0);
    assert!((g180.x - 18.0).abs() < 1e-9);
    assert!((g180.y - 29.0).abs() < 1e-9);

    // At 270 deg: relative (2, 1) rotates to (1, -2) -> (21, 28)
    let g270 = pad.global_position(fp_pos, 270.0);
    assert!((g270.x - 21.0).abs() < 1e-9);
    assert!((g270.y - 28.0).abs() < 1e-9);
}

// ============================================================================
// Tier 1: Parametric Component Footprint Library
// ============================================================================

#[test]
fn test_footprint_tqfp32() {
    let fp = tqfp32("U1", "ATmega328P");
    assert_eq!(fp.reference, "U1");
    assert_eq!(fp.value, "ATmega328P");
    assert_eq!(fp.package, "TQFP-32_7x7mm_P0.8mm");
    assert_eq!(fp.pads.len(), 32, "TQFP-32 must have exactly 32 pads");

    // All pads must be SMD on F.Cu with Rect shape
    for pad in &fp.pads {
        match pad.pad_type {
            PadType::Smd { layer } => assert_eq!(layer, Layer::FCu),
            _ => panic!("TQFP-32 pad must be SMD"),
        }
        match pad.shape {
            PadShape::Rect { width, height } => {
                assert!(width > 0.0 && height > 0.0);
            }
            _ => panic!("TQFP-32 pad shape must be Rect"),
        }
    }

    // Verify pin 1 and pin 32 exist and can be looked up
    assert!(fp.pad("1").is_some());
    assert!(fp.pad("32").is_some());
    assert!(fp.pad("33").is_none());

    // Verify pad pitch: adjacent pads on bottom row must be spaced 0.8 mm apart
    let p1 = fp.pad("1").unwrap();
    let p2 = fp.pad("2").unwrap();
    let pad_dist = p1.relative_pos.distance_to(p2.relative_pos);
    assert!((pad_dist - 0.8).abs() < 1e-4, "TQFP-32 pitch must be 0.8mm, got {}", pad_dist);

    // Silkscreen outline must be present
    assert!(!fp.silkscreen_lines.is_empty(), "TQFP-32 must include silkscreen lines");
}

#[test]
fn test_footprint_sot223() {
    let fp = sot223("U2", "AMS1117-3.3");
    assert_eq!(fp.reference, "U2");
    assert_eq!(fp.value, "AMS1117-3.3");
    assert_eq!(fp.package, "SOT-223-3_TabPin2");
    assert_eq!(fp.pads.len(), 4, "SOT-223 has 3 signal pins plus 1 heat tab");

    assert!(fp.pad("1").is_some());
    assert!(fp.pad("2").is_some());
    assert!(fp.pad("3").is_some());
    assert!(fp.pad("4").is_some() || fp.pad("TAB").is_some());

    // Pitch between pins 1 and 2 is 2.30mm
    let p1 = fp.pad("1").unwrap();
    let p2 = fp.pad("2").unwrap();
    let pitch = p1.relative_pos.distance_to(p2.relative_pos);
    assert!((pitch - 2.30).abs() < 1e-3, "SOT-223 pitch must be 2.30mm, got {}", pitch);
}

#[test]
fn test_footprint_sot23() {
    let fp = sot23("Q1", "MMBT3904");
    assert_eq!(fp.reference, "Q1");
    assert!(fp.pads.len() >= 3, "SOT-23 must have at least 3 pads");
}

#[test]
fn test_footprint_usb_c_16pin() {
    let fp = usb_c_16pin("J1", "USB-C");
    assert_eq!(fp.reference, "J1");
    assert_eq!(fp.package, "USB_C_Receptacle_16Pin");
    // USB-C 16-pin receptacle: 12 or 16 SMD signal pins plus 4 THT shield tabs
    assert!(fp.pads.len() >= 16, "USB-C receptacle must have at least 16 pads, got {}", fp.pads.len());

    let mut smd_count = 0;
    let mut tht_count = 0;
    for pad in &fp.pads {
        match pad.pad_type {
            PadType::Smd { .. } => smd_count += 1,
            PadType::ThroughHole { .. } => tht_count += 1,
        }
    }
    assert!(smd_count >= 12, "USB-C must have SMD signal pins");
    assert!(tht_count >= 4, "USB-C must have at least 4 THT shell retention tabs");
}

#[test]
fn test_footprint_resistor_and_capacitor_0805() {
    let r = resistor_0805("R1", "10k");
    assert_eq!(r.reference, "R1");
    assert_eq!(r.value, "10k");
    assert_eq!(r.pads.len(), 2);
    assert!(r.pad("1").is_some());
    assert!(r.pad("2").is_some());

    let c = capacitor_0805("C1", "0.1uF");
    assert_eq!(c.reference, "C1");
    assert_eq!(c.value, "0.1uF");
    assert_eq!(c.pads.len(), 2);

    // 0805 pad spacing pitch is nominally ~1.9mm
    let p1 = r.pad("1").unwrap();
    let p2 = r.pad("2").unwrap();
    let pitch = p1.relative_pos.distance_to(p2.relative_pos);
    assert!((pitch - 1.9).abs() < 0.2, "0805 pad pitch must be ~1.9mm, got {}", pitch);
}

#[test]
fn test_footprint_led_0805() {
    let led = led_0805("D1", "Green");
    assert_eq!(led.reference, "D1");
    assert_eq!(led.pads.len(), 2);
    // Silkscreen should indicate cathode mark
    assert!(!led.silkscreen_lines.is_empty(), "LED must have silkscreen polarity marking");
}

#[test]
fn test_footprint_crystal() {
    let y = crystal_smd_3225("Y1", "16MHz");
    assert_eq!(y.reference, "Y1");
    assert_eq!(y.pads.len(), 4, "3225 SMD crystal must have 4 pads");
    assert!(y.pad("1").is_some());
    assert!(y.pad("2").is_some());
    assert!(y.pad("3").is_some());
    assert!(y.pad("4").is_some());
}

#[test]
fn test_footprint_header_1x15() {
    let hdr = header_1x15("J2", "CONN_01x15");
    assert_eq!(hdr.reference, "J2");
    assert_eq!(hdr.package, "PinHeader_1x15_P2.54mm_Vertical");
    assert_eq!(hdr.pads.len(), 15, "1x15 header must have 15 pads");

    // All pads must be ThroughHole
    for pad in &hdr.pads {
        match pad.pad_type {
            PadType::ThroughHole { drill, outer_diameter } => {
                assert!((drill - 1.0).abs() < 0.1, "Drill should be ~1.0mm, got {}", drill);
                assert!(outer_diameter > drill, "Outer diameter must exceed drill");
            }
            _ => panic!("Header pad must be ThroughHole"),
        }
    }

    // Pitch between consecutive pins must be 2.54mm (100 mil)
    for i in 1..15 {
        let p_curr = hdr.pad(&i.to_string()).unwrap();
        let p_next = hdr.pad(&(i + 1).to_string()).unwrap();
        let pitch = p_curr.relative_pos.distance_to(p_next.relative_pos);
        assert!((pitch - 2.54).abs() < 1e-3, "Header pitch must be 2.54mm, got {}", pitch);
    }
}

#[test]
fn test_footprint_tactile_switch() {
    let sw = tactile_switch_3x4("SW1", "Reset");
    assert_eq!(sw.reference, "SW1");
    assert_eq!(sw.pads.len(), 4, "Tactile switch SPST has 4 terminals");
}

// ============================================================================
// Tier 1: Board Outline (Edge.Cuts) Closed Polygon
// ============================================================================

#[test]
fn test_board_outline_creation_and_bounds() {
    let board = PcbBoard::new(60.0, 35.0);
    assert_eq!(board.width, 60.0);
    assert_eq!(board.height, 35.0);

    let outline = board.edge_cuts();
    assert_eq!(outline.segments.len(), 4, "Rectangular Edge.Cuts must have 4 segments");
    assert!(outline.is_closed(1e-6), "Edge.Cuts outline must form a closed loop");
}

#[test]
fn test_board_outline_shoelace_area() {
    let outline = EdgeCuts::from_rect(60.0, 35.0);
    let area = outline.area();
    assert!((area - 60.0 * 35.0).abs() < 1e-6, "Area must equal 2100 mm^2, got {}", area);
}

#[test]
fn test_board_outline_point_containment() {
    let outline = EdgeCuts::from_rect(60.0, 35.0);

    // Inside points
    assert!(outline.contains_point(Point::new(30.0, 17.5)));
    assert!(outline.contains_point(Point::new(1.0, 1.0)));
    assert!(outline.contains_point(Point::new(59.0, 34.0)));

    // Outside points
    assert!(!outline.contains_point(Point::new(-1.0, 10.0)));
    assert!(!outline.contains_point(Point::new(70.0, 20.0)));
    assert!(!outline.contains_point(Point::new(30.0, -5.0)));
    assert!(!outline.contains_point(Point::new(30.0, 40.0)));
}

#[test]
fn test_board_outline_distance_to_edge() {
    let outline = EdgeCuts::from_rect(60.0, 35.0);

    // Point at (5.0, 10.0): closest edge is x=0 at distance 5.0
    let dist = outline.distance_to_point(Point::new(5.0, 10.0));
    assert!((dist - 5.0).abs() < 1e-6);

    // Point at (58.0, 30.0): closest edge is x=60 at distance 2.0
    let dist2 = outline.distance_to_point(Point::new(58.0, 30.0));
    assert!((dist2 - 2.0).abs() < 1e-6);
}

// ============================================================================
// Tier 2: Boundary Value Analysis & Geometric Stress
// ============================================================================

#[test]
fn test_point_extreme_coordinates() {
    let p_min = Point::new(-1000.0, -1000.0);
    let p_max = Point::new(1000.0, 1000.0);
    let dist = p_min.distance_to(p_max);
    assert!((dist - 2000.0 * std::f64::consts::SQRT_2).abs() < 1e-6);
}

#[test]
fn test_rotation_negative_and_multi_turn_angles() {
    let origin = Point::new(0.0, 0.0);
    let p = Point::new(10.0, 0.0);

    // -90 degrees CCW = 270 degrees CCW -> (0, -10)
    let r_neg90 = p.rotate_around(origin, -90.0);
    assert!((r_neg90.x - 0.0).abs() < 1e-9);
    assert!((r_neg90.y - (-10.0)).abs() < 1e-9);

    // 450 degrees = 360 + 90 = 90 degrees -> (0, 10)
    let r_450 = p.rotate_around(origin, 450.0);
    assert!((r_450.x - 0.0).abs() < 1e-9);
    assert!((r_450.y - 10.0).abs() < 1e-9);

    // 720 degrees = 2 full turns -> (10, 0)
    let r_720 = p.rotate_around(origin, 720.0);
    assert!((r_720.x - 10.0).abs() < 1e-9);
    assert!((r_720.y - 0.0).abs() < 1e-9);
}

#[test]
fn test_degenerate_pad_geometry_zero_dimensions() {
    let pad_zero_rect = Pad::new_smd(
        "1",
        "T1",
        Point::new(0.0, 0.0),
        PadShape::Rect { width: 0.0, height: 0.0 },
        Layer::FCu,
    );
    let bbox = pad_zero_rect.bounding_box(Point::new(5.0, 5.0), 0.0);
    assert_eq!(bbox.width(), 0.0);
    assert_eq!(bbox.height(), 0.0);
}

#[test]
fn test_annular_ring_boundary_drill_equals_outer() {
    let pad = Pad::new_tht(
        "1",
        "TEST",
        Point::new(0.0, 0.0),
        PadShape::Circle { diameter: 1.0 },
        1.0, // drill == outer
        1.0,
    );
    let ring = pad.annular_ring().expect("THT must yield annular ring");
    assert_eq!(ring, 0.0, "Drill == outer must yield 0.0 annular ring");
}

#[test]
fn test_annular_ring_drill_exceeds_outer_negative() {
    let pad = Pad::new_tht(
        "1",
        "TEST",
        Point::new(0.0, 0.0),
        PadShape::Circle { diameter: 0.8 },
        1.0, // drill 1.0 > outer 0.8
        0.8,
    );
    let ring = pad.annular_ring().expect("THT must yield annular ring");
    assert!(ring < 0.0, "Drill > outer must yield negative annular ring");
}

#[test]
fn test_footprint_rotation_at_position_chain() {
    let fp = resistor_0805("R1", "1k")
        .at_position(Point::new(15.0, 25.0))
        .with_rotation(90.0);

    assert_eq!(fp.position.x, 15.0);
    assert_eq!(fp.position.y, 25.0);
    assert_eq!(fp.rotation, 90.0);

    // Look up pad positions in world coordinates
    let pos_p1 = fp.pad_world_pos("1").expect("Pin 1 must exist");
    let pos_p2 = fp.pad_world_pos("2").expect("Pin 2 must exist");

    // At 90 deg rotation, pads spaced along Y instead of X
    let dy = (pos_p1.y - pos_p2.y).abs();
    let dx = (pos_p1.x - pos_p2.x).abs();
    assert!(dy > 1.5, "Pads should be separated along Y axis when rotated 90 deg");
    assert!(dx < 1e-4, "Pads should have near-zero X separation when rotated 90 deg");
}

#[test]
fn test_footprint_net_assignment_chain() {
    let fp = resistor_0805("R1", "1k")
        .with_net_assigned("1", 42)
        .with_net_assigned("2", 99);

    assert_eq!(fp.pad("1").unwrap().net, Some(42));
    assert_eq!(fp.pad("2").unwrap().net, Some(99));
}

#[test]
fn test_board_add_footprint_and_lookup() {
    let mut board = PcbBoard::new(50.0, 30.0);
    let fp = resistor_0805("R1", "10k");
    board.add_footprint(fp);

    assert_eq!(board.footprints.len(), 1);
    assert!(board.find_footprint("R1").is_some());
    assert!(board.find_footprint("R2").is_none());
}
