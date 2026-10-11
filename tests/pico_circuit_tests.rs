use netlist_shatter_viz::pcb::pico_ultimate::{
    create_pico_ultimate_board, generate_pico_ultimate_bom, generate_pico_ultimate_bom_for_board,
    generate_pico_ultimate_cpl,
};
use netlist_shatter_viz::pcb::*;
use std::collections::HashSet;

fn net<'a>(b: &'a PcbBoard, r: &str, p: &str) -> &'a str {
    b.find_footprint(r)
        .unwrap()
        .pads
        .iter()
        .find(|x| x.pin_number == p || x.pin_name == p)
        .and_then(|p| p.net)
        .and_then(|id| b.get_net_name(id))
        .unwrap()
}
fn value<'a>(b: &'a PcbBoard, r: &str) -> &'a str {
    &b.find_footprint(r).unwrap().value
}

#[test]
fn all_forty_official_header_contacts_and_internal_services() {
    let b = create_pico_ultimate_board();
    // Independent Pico datasheet Fig19 mapping; lower physical numbering is40..21.
    let expected = [
        "GPIO0",
        "GPIO1",
        "GND",
        "GPIO2",
        "GPIO3",
        "I2C_SDA",
        "I2C_SCL",
        "GND",
        "GPIO6",
        "GPIO7",
        "GPIO8",
        "GPIO9",
        "GND",
        "GPIO10",
        "GPIO11",
        "GPIO12",
        "GPIO13",
        "GND",
        "GPIO14",
        "GPIO15",
        "RGB_DATA",
        "GPIO17",
        "GND",
        "GPIO18",
        "GPIO19",
        "GPIO20",
        "GPIO21",
        "GND",
        "GPIO22",
        "RUN_RESET",
        "GPIO26",
        "GPIO27",
        "GND",
        "GPIO28",
        "ADC_VREF",
        "3V3",
        "3V3_EN",
        "GND",
        "VSYS",
        "5V_VBUS",
    ];
    for (i, name) in expected.iter().enumerate() {
        let (r, p) = if i < 20 {
            ("HDR1", i + 1)
        } else {
            ("HDR2", 40 - i)
        };
        assert_eq!(
            net(&b, r, &p.to_string()),
            *name,
            "official header pin{}",
            i + 1
        );
    }
    for (pin, name) in [
        ("35", "BUCK_PS"),
        ("36", "VBUS_DETECT"),
        ("37", "USER_LED"),
        ("41", "VSYS_SENSE"),
        ("24", "SWCLK"),
        ("25", "SWDIO"),
    ] {
        assert_eq!(net(&b, "U1", pin), name);
    }
    for (r, p, name) in [
        ("J4", "1", "SWCLK"),
        ("J4", "2", "GND"),
        ("J4", "3", "SWDIO"),
        ("Q2", "1", "3V3"),
        ("Q2", "2", "VSYS_SENSE"),
        ("Q2", "3", "VSYS_DIV"),
    ] {
        assert_eq!(net(&b, r, p), name);
    }
    assert_eq!(value(&b, "R19"), "5.6k");
    assert_eq!(net(&b, "R19", "1"), "5V_VBUS");
    assert_eq!(net(&b, "R19", "2"), "VBUS_DETECT");
    assert_eq!(value(&b, "R22"), "10k");
    assert_eq!(net(&b, "R22", "1"), "VBUS_DETECT");
    assert_eq!(net(&b, "R22", "2"), "GND");
    assert_eq!(value(&b, "R20"), "200k");
    assert_eq!(value(&b, "R21"), "100k");
    assert_eq!(value(&b, "C9"), "100nF");
}

#[test]
fn charger_power_path_polarity_and_every_converter_pin() {
    let b = create_pico_ultimate_board();
    assert_eq!(value(&b, "U3"), "MCP73831T-2ACI/OT");
    for (p, n) in [
        ("1", "CHG_STAT"),
        ("2", "GND"),
        ("3", "VBAT"),
        ("4", "5V_VBUS"),
        ("5", "CHG_PROG"),
    ] {
        assert_eq!(net(&b, "U3", p), n);
    }
    assert_eq!(value(&b, "R11"), "10k");
    assert_eq!(net(&b, "R11", "1"), "CHG_PROG");
    assert_eq!(net(&b, "R11", "2"), "GND");
    assert_eq!(net(&b, "LED3", "K"), "CHG_STAT");
    assert_eq!(net(&b, "LED3", "A"), net(&b, "R1", "1"));
    assert_eq!(net(&b, "R1", "2"), "5V_VBUS");
    assert_eq!(value(&b, "R1"), "2.0k");
    // Reversed P-MOSFET intrinsic diode must feed BAT -> VSYS, and gate followsVBUS.
    for (p, n) in [("1", "5V_VBUS"), ("2", "VSYS"), ("3", "VBAT")] {
        assert_eq!(net(&b, "Q1", p), n);
    }
    assert_eq!(net(&b, "D1", "A"), "5V_VBUS");
    assert_eq!(net(&b, "D1", "K"), "VSYS");
    assert_eq!(value(&b, "R12"), "100k");
    assert_eq!(net(&b, "R12", "1"), "5V_VBUS");
    assert_eq!(net(&b, "R12", "2"), "GND");
    let expected = [
        "3V3", "BUCK_LX2", "GND", "BUCK_LX1", "VSYS", "3V3_EN", "BUCK_PS", "VSYS", "GND", "3V3",
        "GND",
    ];
    for (i, n) in expected.iter().enumerate() {
        assert_eq!(net(&b, "U4", &(i + 1).to_string()), *n);
    }
    assert_eq!(value(&b, "L1"), "2.2uH");
    assert_eq!(net(&b, "L1", "1"), "BUCK_LX1");
    assert_eq!(net(&b, "L1", "2"), "BUCK_LX2");
    assert_eq!(value(&b, "C2"), "10uF");
    assert_eq!(value(&b, "C3"), "47uF");
    assert_eq!(net(&b, "C2", "1"), "VSYS");
    assert_eq!(net(&b, "C3", "1"), "3V3");
    for (r, n) in [("R16", "3V3_EN"), ("R17", "BUCK_PS")] {
        assert_eq!(value(&b, r), "100k");
        assert!(
            b.find_footprint(r)
                .unwrap()
                .pads
                .iter()
                .any(|p| p.net.and_then(|id| b.get_net_name(id)) == Some(n))
        );
    }
    let issues = check_electrical(&b);
    assert!(issues.is_empty(), "{issues:?}");
    // Meaningful polarity rejection: swapping the actual source/drain must not pass.
    let mut bad = b.clone();
    let q = bad
        .footprints
        .iter_mut()
        .find(|f| f.reference == "Q1")
        .unwrap();
    let s = q.pads[1].net;
    q.pads[1].net = q.pads[2].net;
    q.pads[2].net = s;
    assert!(
        check_electrical(&bad)
            .iter()
            .any(|i| i.message.contains("orientation"))
    );
}

#[test]
fn crystal_boot_led_and_filtered_adc_have_real_series_parts() {
    let b = create_pico_ultimate_board();
    assert_eq!(value(&b, "Y1"), "ABM8-272-T3 12MHz");
    assert_eq!(net(&b, "U1", "20"), net(&b, "Y1", "1"));
    assert_eq!(net(&b, "U1", "21"), net(&b, "R13", "1"));
    assert_ne!(net(&b, "U1", "21"), net(&b, "Y1", "3"));
    assert_eq!(net(&b, "R13", "2"), net(&b, "Y1", "3"));
    assert_eq!(value(&b, "R13"), "1k");
    for (r, p) in [("C5", "1"), ("C6", "3")] {
        assert_eq!(value(&b, r), "15pF");
        assert_eq!(net(&b, r, "1"), net(&b, "Y1", p));
        assert_eq!(net(&b, r, "2"), "GND");
    }
    assert_eq!(value(&b, "R14"), "1k");
    assert_eq!(net(&b, "U1", "56"), net(&b, "U2", "1"));
    assert_ne!(net(&b, "U1", "56"), net(&b, "SW2", "2"));
    assert_eq!(net(&b, "R14", "2"), net(&b, "SW2", "2"));
    assert_eq!(net(&b, "SW2", "1"), "GND");
    assert_eq!(value(&b, "R15"), "1k");
    assert_eq!(net(&b, "R15", "2"), net(&b, "LED2", "A"));
    assert_eq!(net(&b, "LED2", "K"), "GND");
    assert_eq!(value(&b, "R18"), "200R");
    assert_eq!(net(&b, "R18", "1"), "3V3");
    assert_eq!(net(&b, "R18", "2"), "ADC_VREF");
    assert_eq!(net(&b, "U1", "43"), "ADC_VREF");
    assert_eq!(value(&b, "C8"), "2.2uF");
    assert_eq!(net(&b, "C8", "1"), "ADC_VREF");
    assert_eq!(net(&b, "C27", "1"), "ADC_VREF");
    assert_eq!(value(&b, "LED1"), "WS2812B-V6");
    for (p, n) in [("1", "3V3"), ("3", "GND"), ("4", "RGB_DATA")] {
        assert_eq!(net(&b, "LED1", p), n);
    }
}

#[test]
fn unique_complete_refs_pad_roles_and_collision_free_official_row_spacing() {
    let b = create_pico_ultimate_board();
    let refs: HashSet<_> = b.footprints.iter().map(|f| &f.reference).collect();
    assert_eq!(refs.len(), b.footprints.len());
    for f in &b.footprints {
        let pins: HashSet<_> = f.pads.iter().map(|p| &p.pin_number).collect();
        assert_eq!(pins.len(), f.pads.len(), "{}", f.reference);
        for p in &f.pads {
            assert!(
                p.net.is_some()
                    || (f.reference == "LED1" && p.pin_number == "2")
                    || (f.reference == "J1" && ["A8", "B8"].contains(&p.pin_number.as_str())),
                "{}.{} unassigned",
                f.reference,
                p.pin_number
            );
        }
    }
    let h1 = b.find_footprint("HDR1").unwrap();
    let h2 = b.find_footprint("HDR2").unwrap();
    for i in 0..20 {
        assert!(
            (h1.pad_world_position(&h1.pads[i])
                .distance_to(h2.pad_world_position(&h2.pads[i]))
                - 17.78)
                .abs()
                < 1e-9
        );
    }
    let issues = placement::placement_issues(&b);
    assert!(issues.is_empty(), "{issues:?}");
    let q = b.find_footprint("Q1").unwrap();
    for pad in &q.pads {
        assert_eq!(
            pad.shape,
            PadShape::Rect {
                width: 0.8,
                height: 0.9
            }
        );
        assert_eq!(pad.relative_pos.y.abs(), 1.0);
    }
    let reg = b.find_footprint("U4").unwrap();
    assert_eq!(reg.pads.len(), 11);
    assert_eq!(
        reg.pads[10].shape,
        PadShape::Rect {
            width: 1.2,
            height: 2.0
        }
    );
    for i in 1..5 {
        assert!((reg.pads[i].relative_pos.y - reg.pads[i - 1].relative_pos.y - 0.5).abs() < 1e-9);
    }
    println!(
        "Pico actual parts={} pads={} nets={}",
        b.footprints.len(),
        b.footprints.iter().map(|f| f.pads.len()).sum::<usize>(),
        b.net_names.len()
    );
}

#[test]
fn actual_board_bom_and_positions_include_core_and_new_power_parts() {
    let b = create_pico_ultimate_board();
    let bom = generate_pico_ultimate_bom();
    let cpl = generate_pico_ultimate_cpl();
    assert_eq!(bom.len(), b.footprints.len());
    assert_eq!(cpl.len(), b.footprints.len());
    for f in &b.footprints {
        let row = bom.iter().find(|x| x.designator == f.reference).unwrap();
        assert_eq!(row.value, f.value);
        assert_eq!(row.package, f.package);
        assert_eq!(row.quantity, 1);
        let pos = cpl.iter().find(|x| x.designator == f.reference).unwrap();
        assert!((pos.mid_x - f.position.x).abs() < 1e-4);
        assert!((pos.mid_y - f.position.y).abs() < 1e-4);
        assert_eq!(pos.rotation, f.rotation);
    }
    for i in 20..=31 {
        assert!(bom.iter().any(|x| x.designator == format!("C{i}")));
    }
    assert_eq!(
        bom.iter().find(|x| x.designator == "L1").unwrap().mpn,
        "VLS252012HBX-2R2M-1"
    );
    assert!(
        !bom.iter()
            .any(|x| x.mpn == "AMS1117-3.3" || x.mpn == "WS2812B-B")
    );
    let mut edited = b.clone();
    edited
        .footprints
        .iter_mut()
        .find(|f| f.reference == "U3")
        .unwrap()
        .value = "OTHER_CHARGER".into();
    edited
        .footprints
        .iter_mut()
        .find(|f| f.reference == "LED1")
        .unwrap()
        .package = "OTHER_RGB_LANDS".into();
    let changed = generate_pico_ultimate_bom_for_board(&edited);
    for r in ["U3", "LED1"] {
        assert!(
            changed
                .iter()
                .find(|x| x.designator == r)
                .unwrap()
                .mpn
                .is_empty()
        );
    }
    if let Ok(path) = std::env::var("PICO_CIRCUIT_EVIDENCE") {
        let path = std::path::Path::new(&path);
        std::fs::create_dir_all(path).unwrap();
        std::fs::write(
            path.join("board.json"),
            serde_json::to_string_pretty(&b).unwrap(),
        )
        .unwrap();
        std::fs::write(path.join("board.kicad_pcb"), export_kicad_pcb(&b)).unwrap();
        std::fs::write(path.join("BOM.csv"), assembly::export_bom_csv(&bom)).unwrap();
        std::fs::write(path.join("CPL.csv"), assembly::export_cpl_csv(&cpl)).unwrap();
    }
}
