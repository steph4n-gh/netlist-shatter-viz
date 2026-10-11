use netlist_shatter_viz::pcb::esp32_cam_pro::*;
use netlist_shatter_viz::pcb::*;
use std::collections::HashSet;
fn net<'a>(b: &'a PcbBoard, r: &str, p: &str) -> Option<&'a str> {
    b.find_footprint(r)?
        .pad(p)?
        .net
        .and_then(|n| b.get_net_name(n))
}
fn same(b: &PcbBoard, a: (&str, &str), z: (&str, &str)) -> bool {
    net(b, a.0, a.1).is_some() && net(b, a.0, a.1) == net(b, z.0, z.1)
}
fn change(b: &mut PcbBoard, r: &str, p: &str, name: &str) {
    let n = b.get_or_create_net(name);
    b.find_footprint_mut(r).unwrap().pad_mut(p).unwrap().net = Some(n);
}
fn rails_valid(b: &PcbBoard) -> bool {
    [
        ("J2", "14", "3V3_SYS"),
        ("J2", "15", "CAM_DVDD"),
        ("J2", "21", "CAM_AVDD"),
        ("U2", "4", "3V3_SYS"),
        ("U3", "1", "3V3_SYS"),
        ("U3", "3", "3V3_SYS"),
        ("U5", "1", "3V3_SYS"),
        ("U5", "3", "3V3_SYS"),
    ]
    .iter()
    .all(|&(r, p, n)| net(b, r, p) == Some(n))
}
fn auto_valid(b: &PcbBoard) -> bool {
    // Independent Espressif DevKitCv4 cross-coupled reset topology, actual MMBT B1/E2/C3.
    [
        ("U4", "28", "R3", "1"),
        ("U4", "28", "Q2", "2"),
        ("U4", "24", "R4", "1"),
        ("U4", "24", "Q1", "2"),
        ("R3", "2", "Q1", "1"),
        ("R4", "2", "Q2", "1"),
        ("Q1", "3", "U1", "3"),
        ("Q2", "3", "U1", "25"),
    ]
    .iter()
    .all(|&(r, p, s, q)| same(b, (r, p), (s, q)))
        && !same(b, ("U4", "28"), ("U4", "24"))
        && !same(b, ("U1", "3"), ("U1", "25"))
        && b.find_footprint("R3").unwrap().value == "10k"
        && b.find_footprint("R4").unwrap().value == "10k"
}
#[test]
fn complete_camera_manufacturer_connector_and_firmware_mapping() {
    let b = create_esp32_cam_pro_board();
    // Ai-Thinker original V1.6 connector, not order of firmware struct fields.
    for (pin, name, mcu) in [
        ("3", "CAM_D2", "31"),
        ("4", "CAM_D1", "30"),
        ("5", "CAM_D3", "33"),
        ("6", "CAM_D0", "29"),
        ("7", "CAM_D4", "4"),
        ("8", "CAM_PCLK", "36"),
        ("9", "CAM_D5", "5"),
        ("11", "CAM_D6", "6"),
        ("12", "ESP_IO0", "25"),
        ("13", "CAM_D7", "7"),
        ("16", "CAM_HREF", "37"),
        ("17", "CAM_PWDN", "8"),
        ("18", "CAM_VSYNC", "10"),
        ("20", "CAM_SIOC", "12"),
        ("22", "CAM_SIOD", "11"),
    ] {
        assert_eq!(net(&b, "J2", pin), Some(name));
        assert!(same(&b, ("J2", pin), ("U1", mcu)));
    }
    for (pin, n) in [
        ("10", "GND"),
        ("14", "3V3_SYS"),
        ("15", "CAM_DVDD"),
        ("19", "CAM_RESET_N"),
        ("21", "CAM_AVDD"),
        ("23", "GND"),
        ("MP1", "GND"),
        ("MP2", "GND"),
    ] {
        assert_eq!(net(&b, "J2", pin), Some(n));
    }
    for p in ["1", "2", "24"] {
        assert_eq!(net(&b, "J2", p), None);
    }
    assert_eq!(CAMERA_DATA_GPIOS, [5, 18, 19, 21, 36, 39, 34, 35]);
    assert_eq!(CAMERA_XCLK_GPIO, 0);
    assert_eq!(CAMERA_PWDN_GPIO, 32);
    assert_eq!(
        b.find_footprint("J2").unwrap().pad("4").unwrap().pin_name,
        "Y3"
    );
}
#[test]
fn regulators_are_enabled_decoupled_and_keep_three_camera_rails_distinct() {
    let mut b = create_esp32_cam_pro_board();
    assert!(rails_valid(&b));
    for u in ["U3", "U5"] {
        for p in ["1", "3"] {
            assert_eq!(net(&b, u, p), Some("3V3_SYS"));
        }
        assert_eq!(net(&b, u, "2"), Some("GND"));
        assert_eq!(net(&b, u, "4"), None);
    }
    for (u, c1, c2, n) in [
        ("U3", "C4", "C5", "CAM_AVDD"),
        ("U5", "C7", "C8", "CAM_DVDD"),
    ] {
        for (r, p) in [(u, "5"), (c1, "1"), (c2, "1")] {
            assert_eq!(net(&b, r, p), Some(n));
        }
    }
    assert_eq!(net(&b, "U2", "4"), Some("3V3_SYS"));
    assert_eq!(net(&b, "C15", "1"), Some("ESP_EN"));
    assert_eq!(net(&b, "C16", "1"), Some("CAM_RESET_N"));
    assert!(b.find_footprint("L1").is_none());
    assert!(!same(&b, ("J2", "14"), ("J2", "15")));
    assert!(!same(&b, ("J2", "15"), ("J2", "21")));
    change(&mut b, "J2", "15", "CAM_AVDD");
    assert!(!rails_valid(&b), "wrong camera core rail must fail");
    let mut disabled = create_esp32_cam_pro_board();
    disabled
        .find_footprint_mut("U3")
        .unwrap()
        .pad_mut("3")
        .unwrap()
        .net = None;
    assert!(!rails_valid(&disabled), "floating LP5907 enable must fail");
    for (r, p) in [("R10", "20"), ("R11", "22")] {
        assert!(same(&disabled, (r, "2"), ("J2", p)));
        assert_eq!(net(&disabled, r, "1"), Some("3V3_SYS"));
        assert_eq!(disabled.find_footprint(r).unwrap().value, "4.7k");
    }
}
#[test]
fn genuine_bridge_uart_direction_and_crosscoupled_autoboot_negatives() {
    let b = create_esp32_cam_pro_board();
    assert!(auto_valid(&b));
    assert_eq!(b.find_footprint("U4").unwrap().value, "CP2102N-A02-GQFN28");
    assert!(same(&b, ("U4", "26"), ("U1", "34")));
    assert!(same(&b, ("U4", "25"), ("U1", "35")));
    for (r, p, n) in [
        ("Q1", "1", "RTS_BASE"),
        ("Q2", "2", "UART_RTS"),
        ("Q1", "3", "ESP_IO0"),
        ("R3", "2", "GND"),
    ] {
        let mut broken = b.clone();
        change(&mut broken, r, p, n);
        assert!(!auto_valid(&broken), "{r}.{p} broken variant must fail");
    }
    // Voltage truth-table of independent collector pull-downs: equal inputs turn both off.
    for (d, r, en, boot) in [
        (true, true, true, true),
        (false, false, true, true),
        (true, false, false, true),
        (false, true, true, false),
    ] {
        assert_eq!(!(d && !r), en);
        assert_eq!(!(r && !d), boot);
    }
}
#[test]
fn self_powered_usb_never_backfeeds_and_both_orientations_connect() {
    let mut b = create_esp32_cam_pro_board();
    assert_eq!(net(&b, "J1", "A4"), Some("USB_VBUS"));
    assert!(same(&b, ("J1", "A4"), ("J1", "B4")));
    assert_eq!(net(&b, "U2", "3"), Some("EXT_5V"));
    assert_eq!(net(&b, "U4", "7"), Some("EXT_5V"));
    assert!(!same(&b, ("J1", "A4"), ("HDR1", "1")));
    assert!(!same(&b, ("U4", "6"), ("U1", "2")));
    assert_eq!(net(&b, "U4", "8"), Some("VBUS_SENSE"));
    for (a, z, p) in [("A6", "B6", "4"), ("A7", "B7", "5")] {
        assert!(same(&b, ("J1", a), ("J1", z)));
        assert!(same(&b, ("J1", a), ("U4", p)));
    }
    for (p, r) in [("A5", "R5"), ("B5", "R6")] {
        assert!(same(&b, ("J1", p), (r, "1")));
        assert_eq!(net(&b, r, "2"), Some("GND"));
        assert_eq!(b.find_footprint(r).unwrap().value, "5.1k");
    }
    change(&mut b, "U2", "3", "USB_VBUS");
    assert!(
        same(&b, ("J1", "A4"), ("U2", "3")),
        "negative must reveal bus-power regression"
    );
    assert_eq!(EXTERNAL_5V_SUPPLY_MIN_MA, 1000);
    assert_eq!(DECLARED_TOTAL_ALLOCATION_MA, 800);
    assert!(QUALIFICATION_HOLDS.iter().any(|s| s.contains("thermal")));
}
#[test]
fn spi_storage_pullups_do_not_drive_flash_voltage_or_download_straps() {
    let b = create_esp32_cam_pro_board();
    assert_eq!(SD_SPI_GPIOS, [12, 13, 33, 15]);
    for (socket, mcu, n) in [
        ("5", "14", "SD_CLK"),
        ("7", "16", "SD_MISO"),
        ("3", "9", "SD_MOSI"),
        ("2", "23", "SD_CS"),
    ] {
        assert_eq!(net(&b, "J3", socket), Some(n));
        assert!(same(&b, ("J3", socket), ("U1", mcu)));
    }
    for (r, p) in [
        ("R16", "7"),
        ("R17", "3"),
        ("R18", "2"),
        ("R19", "8"),
        ("R20", "1"),
    ] {
        assert!(same(&b, (r, "2"), ("J3", p)));
        assert_eq!(net(&b, r, "1"), Some("3V3_SYS"));
        assert_eq!(b.find_footprint(r).unwrap().value, "10k");
    }
    assert_eq!(net(&b, "R15", "2"), Some("GND"));
    assert!(same(&b, ("R15", "1"), ("U1", "14")));
    assert_eq!(net(&b, "R14", "2"), Some("GND"));
    assert!(same(&b, ("R14", "1"), ("U1", "24")));
    assert!(!same(&b, ("J3", "1"), ("U1", "14")));
    assert!(!same(&b, ("J3", "8"), ("U1", "26")));
    assert_eq!(net(&b, "Q3", "2"), Some("GND"));
    assert!(same(&b, ("Q3", "1"), ("R21", "2")));
    assert!(same(&b, ("LED1", "1"), ("Q3", "3")));
    assert_eq!(b.find_footprint("R23").unwrap().value, "330R");
}
#[test]
fn manufacturer_module_bridge_and_camera_land_dimensions() {
    let b = create_esp32_cam_pro_board();
    let u = b.find_footprint("U1").unwrap();
    assert_eq!(u.pads.len(), 47);
    assert_eq!(u.pad("1").unwrap().relative_pos, Point::new(-8.75, -8.26));
    assert!((u.pad("19").unwrap().relative_pos.y - 14.6).abs() < 1e-9);
    assert_eq!(u.pad("EP5").unwrap().relative_pos, Point::new(-1.5, -0.46));
    for p in ["17", "18", "19", "20", "21", "22", "27", "28", "32"] {
        assert_eq!(net(&b, "U1", p), None);
    }
    for p in [
        "1", "15", "38", "EP1", "EP2", "EP3", "EP4", "EP5", "EP6", "EP7", "EP8", "EP9",
    ] {
        assert_eq!(net(&b, "U1", p), Some("GND"));
    }
    let cp = b.find_footprint("U4").unwrap();
    assert_eq!(cp.pads.len(), 29);
    assert_eq!(cp.pad("1").unwrap().relative_pos, Point::new(-2.4, -1.5));
    assert_eq!(
        cp.pad("1").unwrap().shape,
        PadShape::Rect {
            width: 0.95,
            height: 0.30
        }
    );
    assert_eq!(
        cp.pad("29").unwrap().shape,
        PadShape::Rect {
            width: 3.35,
            height: 3.35
        }
    );
    let camera = b.find_footprint("J2").unwrap();
    assert_eq!(
        camera.pad("1").unwrap().relative_pos,
        Point::new(-5.75, -1.85)
    );
    assert_eq!(
        camera.pad("24").unwrap().relative_pos,
        Point::new(5.75, -1.85)
    );
    assert_eq!(
        camera.pad("1").unwrap().shape,
        PadShape::Rect {
            width: 0.3,
            height: 1.3
        }
    );
    assert!(
        u.pads
            .iter()
            .chain(cp.pads.iter())
            .chain(camera.pads.iter())
            .all(|p| p.pad_type.is_smd())
    );
}
#[test]
fn all_active_pins_and_headers_connected_without_fake_bus_aliases() {
    let b = create_esp32_cam_pro_board();
    assert_eq!(b.footprints.len(), 61);
    assert_eq!(
        b.footprints.iter().map(|f| f.pads.len()).sum::<usize>(),
        283
    );
    for f in &b.footprints {
        for p in &f.pads {
            let nc = match f.reference.as_str() {
                "U1" => ["17", "18", "19", "20", "21", "22", "27", "28", "32"]
                    .contains(&p.pin_number.as_str()),
                "U3" | "U5" => p.pin_number == "4",
                "U4" => [
                    "1", "2", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20",
                    "21", "22", "27",
                ]
                .contains(&p.pin_number.as_str()),
                "J2" => ["1", "2", "24"].contains(&p.pin_number.as_str()),
                "J3" => ["9", "10"].contains(&p.pin_number.as_str()),
                "J1" => ["A8", "B8"].contains(&p.pin_number.as_str()),
                _ => false,
            };
            assert_eq!(
                p.net.is_none(),
                nc,
                "{}.{} must have declared connection/NC",
                f.reference,
                p.pin_number
            );
        }
    }
    assert!(!b.net_names.values().any(|s| s.starts_with("BUS_")));
    for h in ["HDR1", "HDR2"] {
        for p in &b.find_footprint(h).unwrap().pads {
            assert!(p.net.is_some());
        }
    }
}
#[test]
fn bom_and_positions_follow_actual_board_and_do_not_invent_part_numbers() {
    let mut b = create_esp32_cam_pro_board();
    let bom = generate_esp32_cam_pro_bom_for_board(&b);
    let refs: HashSet<_> = bom.iter().map(|e| e.designator.as_str()).collect();
    assert_eq!(refs.len(), b.footprints.len());
    assert_eq!(generate_esp32_cam_pro_cpl().len(), b.footprints.len());
    assert_eq!(
        bom.iter().find(|r| r.designator == "U1").unwrap().mpn,
        "ESP32-WROVER-E-N4R8"
    );
    assert!(
        bom.iter()
            .find(|r| r.designator == "U2")
            .unwrap()
            .mpn
            .is_empty()
    );
    b.find_footprint_mut("U1").unwrap().value = "unreviewed_variant".into();
    assert!(
        generate_esp32_cam_pro_bom_for_board(&b)
            .iter()
            .find(|r| r.designator == "U1")
            .unwrap()
            .mpn
            .is_empty()
    );
}

#[test]
fn source_placement_passes_the_same_envelope_gate_as_fresh_app_routing() {
    let board = create_esp32_cam_pro_board();
    assert!(placement::placement_issues(&board).is_empty());
}
