//! Small RP2040 bring-up board: USB, complete QSPI, SWD, reset, boot, GPIO LED.
//! This is the first hardware validation target. Fabrication does not imply bench validation.
use super::*;

pub fn create_rp2040_bringup_board() -> PcbBoard {
    let mut b = PcbBoard::new(60.0, 42.0);
    for fp in [
        qfn56("U1", "RP2040").at_position(Point::new(28.0, 21.0)),
        soic8("U2", "W25Q128JVSSIQ").at_position(Point::new(28.0, 8.0)),
        ap2112k_33("U3").at_position(Point::new(10.0, 10.0)),
        usb_c_16pin("J1", "USB-C").at_position(Point::new(8.0, 3.5)),
        header_1x10("J2", "GPIO-SWD").at_position(Point::new(53.0, 8.0)),
        crystal_smd_3225("Y1", "ABM8-272-T3").at_position(Point::new(25.0, 30.0)),
        tactile_switch_3x4("SW1", "RESET").at_position(Point::new(43.0, 30.0)),
        tactile_switch_3x4("SW2", "BOOTSEL").at_position(Point::new(40.0, 8.0)),
        led_0805("D1", "GREEN").at_position(Point::new(44.0, 20.0)),
        capacitor_0805("C1", "10uF").at_position(Point::new(7.0, 15.0)),
        capacitor_0805("C2", "10uF").at_position(Point::new(14.0, 15.0)),
        capacitor_0805("C3", "1uF").at_position(Point::new(21.0, 8.0)),
        capacitor_0805("C4", "15pF")
            .at_position(Point::new(21.0, 30.0))
            .with_rotation(90.0),
        capacitor_0805("C5", "15pF")
            .at_position(Point::new(29.0, 32.0))
            .with_rotation(90.0),
        resistor_0805("R1", "5.1k")
            .at_position(Point::new(5.0, 10.0))
            .with_rotation(90.0),
        resistor_0805("R2", "5.1k")
            .at_position(Point::new(15.0, 10.0))
            .with_rotation(90.0),
        resistor_0805("R3", "27R").at_position(Point::new(20.0, 12.0)),
        resistor_0805("R4", "27R").at_position(Point::new(20.0, 16.0)),
        resistor_0805("R5", "1k").at_position(Point::new(32.5, 28.5)),
        resistor_0805("R6", "1k").at_position(Point::new(36.0, 12.0)),
        resistor_0805("R7", "10k").at_position(Point::new(43.0, 26.0)),
        resistor_0805("R8", "1k").at_position(Point::new(44.0, 24.0)),
    ] {
        b.add_footprint(fp);
    }
    for (name, pins) in [
        (
            "GND",
            vec![
                ("U3", "2"),
                ("C1", "2"),
                ("C2", "2"),
                ("C3", "2"),
                ("C4", "2"),
                ("C5", "2"),
                ("R1", "2"),
                ("R2", "2"),
                ("Y1", "2"),
                ("Y1", "4"),
                ("U2", "4"),
                ("J2", "2"),
                ("J2", "10"),
                ("SW1", "1"),
                ("SW2", "1"),
                ("D1", "1"),
            ],
        ),
        (
            "5V_VBUS",
            vec![("J1", "A4"), ("U3", "1"), ("U3", "3"), ("C1", "1")],
        ),
        (
            "3V3",
            vec![
                ("U3", "5"),
                ("C2", "1"),
                ("U2", "8"),
                ("R7", "1"),
                ("J2", "1"),
            ],
        ),
        ("1V1_CORE", vec![("U1", "45"), ("C3", "1")]),
        ("CC1", vec![("J1", "A5"), ("R1", "1")]),
        ("CC2", vec![("J1", "B5"), ("R2", "1")]),
        ("USB_CONN_DP", vec![("J1", "A6"), ("R3", "1")]),
        ("USB_CONN_DM", vec![("J1", "A7"), ("R4", "1")]),
        ("USB_DP", vec![("R3", "2"), ("U1", "47")]),
        ("USB_DM", vec![("R4", "2"), ("U1", "46")]),
        ("QSPI_CLK", vec![("U1", "52"), ("U2", "6")]),
        ("QSPI_D0", vec![("U1", "53"), ("U2", "5")]),
        ("QSPI_D1", vec![("U1", "55"), ("U2", "2")]),
        ("QSPI_CS", vec![("U1", "56"), ("U2", "1"), ("R6", "1")]),
        ("BOOT_BUTTON", vec![("R6", "2"), ("SW2", "2")]),
        ("XIN", vec![("U1", "20"), ("Y1", "1"), ("C4", "1")]),
        ("XOUT", vec![("U1", "21"), ("R5", "1")]),
        ("XTAL_DRIVE", vec![("R5", "2"), ("Y1", "3"), ("C5", "1")]),
        (
            "RUN_RESET",
            vec![("U1", "26"), ("SW1", "2"), ("R7", "2"), ("J2", "9")],
        ),
        ("SWCLK", vec![("U1", "24"), ("J2", "7")]),
        ("SWDIO", vec![("U1", "25"), ("J2", "8")]),
        ("GPIO0", vec![("U1", "2"), ("J2", "3")]),
        ("GPIO1", vec![("U1", "3"), ("J2", "4")]),
        ("GPIO4", vec![("U1", "6"), ("J2", "5")]),
        ("GPIO5", vec![("U1", "7"), ("J2", "6")]),
        ("GPIO25_LED", vec![("U1", "37"), ("R8", "1")]),
        ("LED_A", vec![("R8", "2"), ("D1", "2")]),
    ] {
        let net = b.get_or_create_net(name);
        for (r, p) in pins {
            b.connect(r, p, net);
        }
    }
    rp2040::complete_core(&mut b);
    // Library outlines are illustrative; omit strokes that would print on exposed pads.
    let geometry = b.clone();
    for f in &mut b.footprints {
        let origin = f.position;
        let rotation = f.rotation;
        f.silkscreen_lines.retain(|(a, z)| {
            let stroke = TrackSegment::new(
                origin + a.rotate(rotation),
                origin + z.rotate(rotation),
                0.12,
                Layer::FSilkscreen,
                0,
            );
            geometry.footprints.iter().all(|other| {
                other.pads.iter().all(|p| {
                    matches!(p.pad_type, PadType::Smd { layer: Layer::BCu })
                        || drc::track_to_pad_surface_dist(
                            &stroke,
                            p,
                            other.pad_world_position(p),
                            other.rotation,
                        ) >= 0.15
                })
            })
        });
    }
    b
}
