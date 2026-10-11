//! Integration Test Suite: Manufacturing Export & CAM Packages
//!
//! Validates:
//! 1. Draft export and validation gates for ESP32-CAM Pro and Pico Ultimate
//! 2. KiCad 7/8 .kicad_pcb export syntax and balanced s-expression parentheses
//! 3. Standard RS-274X Gerber layer headers, aperture tables, draw/flash commands, and M02* EOF
//! 4. Excellon NC drill export format (M48, METRIC, tool table, hits, M30)
//! 5. BOM CSV and CPL CSV generation and format compliance
//! 6. Production ZIP archive creation with valid PK\x03\x04 headers and complete layer sets

use netlist_shatter_viz::pcb::*;

#[test]
fn reviewed_regulator_sourcing_requires_the_actual_part_and_lands() {
    let board = create_rp2040_bringup_board();
    let sourced = generate_bom_from_board(&board);
    let regulator = sourced.iter().find(|row| row.designator == "U3").unwrap();
    assert_eq!(regulator.manufacturer, "Diodes Incorporated");
    assert_eq!(regulator.mpn, "AP2112K-3.3TRG1");
    assert!(regulator.description.contains("hardware unqualified"));
    assert!(sourced.iter().filter(|row| row.designator != "U3").all(|row| row.mpn.is_empty()));

    for change in 0..3 {
        let mut edited = board.clone();
        let footprint = edited.find_footprint_mut("U3").unwrap();
        match change {
            0 => footprint.value = "AP2112K-3.3".into(),
            1 => footprint.package = "SO-8".into(),
            _ => footprint.pads[0].relative_pos.x += 0.05,
        }
        let bom = generate_bom_from_board(&edited);
        let regulator = bom.iter().find(|row| row.designator == "U3").unwrap();
        assert!(regulator.mpn.is_empty());
        assert!(regulator.manufacturer.is_empty());
        assert!(regulator.description.contains("sourcing/land pattern unqualified"));
    }
}

#[test]
fn test_esp32_cam_pro_manufacturing_pipeline() {
    let config = prototype_rules();
    let board = create_esp32_cam_pro_board();
    let report = validate_board(&board, &config);
    assert!(
        report.electrical_issues.is_empty(),
        "{:?}",
        report.electrical_issues
    );
    assert!(!report.software_checks_pass);
    assert!(!report.hardware_validated);

    // Unrouted drafts must not pass the fabrication gate.
    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "Zero short circuits");
    assert_eq!(
        drc.clearance_violations.len(),
        0,
        "Zero clearance violations"
    );
    assert_eq!(
        drc.annular_ring_violations.len(),
        0,
        "Zero annular ring violations"
    );
    assert_eq!(
        drc.edge_keepout_violations.len(),
        0,
        "Zero edge keepout violations"
    );
    assert!(!drc.unrouted_nets.is_empty());

    // 2. KiCad PCB Export
    let kicad_text = export_kicad_pcb(&board);
    assert!(kicad_text.starts_with("(kicad_pcb (version 20240108)"));
    assert!(kicad_text.ends_with(")\n"));
    let open_parens = kicad_text.chars().filter(|&c| c == '(').count();
    let close_parens = kicad_text.chars().filter(|&c| c == ')').count();
    assert_eq!(
        open_parens, close_parens,
        "KiCad s-expression parentheses must balance"
    );
    let ground_id = board
        .net_names
        .iter()
        .find_map(|(id, name)| (name == "GND").then_some(*id))
        .expect("Ground net must exist");
    assert!(kicad_text.contains(&format!("(net {ground_id} \"GND\")")));
    assert!(kicad_text.contains("ESP32-WROVER-E-N4R8"));

    // 3. CAM Gerbers and Drill Export
    let cam_files = export_all_gerbers_and_drill(&board);
    assert_eq!(
        cam_files.len(),
        7,
        "CAM package must contain 6 Gerber layers + 1 drill file"
    );

    let required_layers = [
        "F_Cu.gbr",
        "B_Cu.gbr",
        "F_SilkS.gbr",
        "Edge_Cuts.gbr",
        "F_Mask.gbr",
        "B_Mask.gbr",
    ];
    for &layer in &required_layers {
        let content = cam_files
            .get(layer)
            .expect("Layer must exist in CAM output");
        assert!(
            content.contains("%FSLAX46Y46*%"),
            "Layer {} must have RS-274X format header",
            layer
        );
        assert!(
            content.contains("%MOMM*%"),
            "Layer {} must specify metric units",
            layer
        );
        assert!(
            content.ends_with("M02*\n"),
            "Layer {} must end with M02* EOF",
            layer
        );
    }

    let drill = cam_files.get("drill.drl").expect("Drill file must exist");
    assert!(drill.contains("M48"), "Drill file must have M48 header");
    assert!(drill.contains("METRIC"), "Drill file must specify METRIC");
    assert!(drill.contains("M30"), "Drill file must end with M30");

    // 4. BOM and CPL
    let bom = generate_esp32_cam_pro_bom();
    let bom_csv = export_bom_csv(&bom);
    assert!(
        bom_csv.starts_with("Item,Designator,Value,Package,Quantity,Description,Manufacturer,MPN")
    );
    assert!(bom_csv.contains("ESP32-WROVER-E-N4R8"));
    assert!(bom_csv.contains("LP5907MFX-2.8"));
    assert!(bom_csv.contains("LP5907MFX-1.2"));
    assert!(bom_csv.contains("CP2102N-A02-GQFN28"));
    assert!(!bom_csv.contains("CH340N"));

    let cpl = generate_cpl_from_board(&board);
    let cpl_csv = export_cpl_csv(&cpl);
    assert!(cpl_csv.starts_with("Designator,Mid X,Mid Y,Layer,Rotation"));
    assert_eq!(cpl.len(), board.footprints.len());

    // 5. Production ZIP Archive
    let mut file_entries = Vec::new();
    let mut string_cache = Vec::new();
    for (name, content) in &cam_files {
        string_cache.push((name.clone(), content.clone()));
    }
    for (name, content) in &string_cache {
        file_entries.push((name.as_str(), content.as_bytes()));
    }
    let zip_bytes = create_zip_archive(&file_entries);
    assert!(zip_bytes.len() > 1000);
    assert_eq!(
        &zip_bytes[0..4],
        &[0x50, 0x4B, 0x03, 0x04],
        "ZIP archive must begin with PK header"
    );
}

#[test]
fn test_pico_ultimate_manufacturing_pipeline() {
    let config = RouterConfig {
        grid_resolution: 0.20,
        default_trace_width: 0.20,
        via_diameter: 0.60,
        via_drill: 0.30,
        clearance: 0.15,
    };

    let board = create_pico_ultimate_board();
    let report = validate_board(&board, &config);
    assert!(
        !report.software_checks_pass,
        "Unrouted circuits must not pass the manufacturing gate"
    );
    assert!(
        report.electrical_issues.is_empty(),
        "{:?}",
        report.electrical_issues
    );
    assert!(!report.drc.unrouted_nets.is_empty());
    assert!(!report.hardware_validated);

    // Export still supports draft review; only a passing validation can proceed.
    // 2. KiCad PCB Export
    let kicad_text = export_kicad_pcb(&board);
    assert!(kicad_text.starts_with("(kicad_pcb (version 20240108)"));
    assert!(kicad_text.ends_with(")\n"));
    let open_parens = kicad_text.chars().filter(|&c| c == '(').count();
    let close_parens = kicad_text.chars().filter(|&c| c == ')').count();
    assert_eq!(
        open_parens, close_parens,
        "KiCad s-expression parentheses must balance"
    );
    let ground_id = board
        .net_names
        .iter()
        .find_map(|(id, name)| (name == "GND").then_some(*id))
        .expect("Ground net must exist");
    assert!(kicad_text.contains(&format!("(net {ground_id} \"GND\")")));
    assert!(kicad_text.contains("RP2040-B2"));
    assert!(kicad_text.contains("W25Q128JVS"));

    // 3. CAM Gerbers and Drill Export
    let cam_files = export_all_gerbers_and_drill(&board);
    assert_eq!(
        cam_files.len(),
        7,
        "CAM package must contain 6 Gerber layers + 1 drill file"
    );

    let required_layers = [
        "F_Cu.gbr",
        "B_Cu.gbr",
        "F_SilkS.gbr",
        "Edge_Cuts.gbr",
        "F_Mask.gbr",
        "B_Mask.gbr",
    ];
    for &layer in &required_layers {
        let content = cam_files
            .get(layer)
            .expect("Layer must exist in CAM output");
        assert!(
            content.contains("%FSLAX46Y46*%"),
            "Layer {} must have RS-274X format header",
            layer
        );
        assert!(
            content.contains("%MOMM*%"),
            "Layer {} must specify metric units",
            layer
        );
        assert!(
            content.ends_with("M02*\n"),
            "Layer {} must end with M02* EOF",
            layer
        );
    }

    let drill = cam_files.get("drill.drl").expect("Drill file must exist");
    assert!(drill.contains("M48"), "Drill file must have M48 header");
    assert!(drill.contains("METRIC"), "Drill file must specify METRIC");
    assert!(drill.contains("M30"), "Drill file must end with M30");

    // 4. BOM and CPL
    let bom = generate_pico_ultimate_bom();
    let bom_csv = export_bom_csv(&bom);
    assert!(
        bom_csv.starts_with("Item,Designator,Value,Package,Quantity,Description,Manufacturer,MPN")
    );
    assert!(bom_csv.contains("RP2040"));
    assert!(bom_csv.contains("W25Q128JVS"));
    assert!(bom_csv.contains("MCP73831T-2ACI/OT"));
    assert!(bom_csv.contains("WS2812B-V6"));
    assert!(bom_csv.contains("RT6150B-33GQW"));
    assert!(!bom_csv.contains("AMS1117-3.3"));

    let cpl = generate_cpl_from_board(&board);
    let cpl_csv = export_cpl_csv(&cpl);
    assert!(cpl_csv.starts_with("Designator,Mid X,Mid Y,Layer,Rotation"));
    assert_eq!(cpl.len(), board.footprints.len());

    // 5. Production ZIP Archive
    let mut file_entries = Vec::new();
    let mut string_cache = Vec::new();
    for (name, content) in &cam_files {
        string_cache.push((name.clone(), content.clone()));
    }
    for (name, content) in &string_cache {
        file_entries.push((name.as_str(), content.as_bytes()));
    }
    let zip_bytes = create_zip_archive(&file_entries);
    assert!(zip_bytes.len() > 1000);
    assert_eq!(
        &zip_bytes[0..4],
        &[0x50, 0x4B, 0x03, 0x04],
        "ZIP archive must begin with PK header"
    );
}

#[test]
fn test_framework_card_manufacturing_pipeline() {
    let config = RouterConfig {
        grid_resolution: 0.20,
        default_trace_width: 0.20,
        via_diameter: 0.60,
        via_drill: 0.30,
        clearance: 0.15,
    };

    let board = create_framework_card_board();
    let report = validate_board(&board, &config);
    assert!(
        !report.software_checks_pass,
        "Unrouted circuits must not pass the manufacturing gate"
    );
    assert!(
        report.electrical_issues.is_empty(),
        "{:?}",
        report.electrical_issues
    );
    assert!(!report.drc.unrouted_nets.is_empty());

    // Export still supports draft review; only a passing validation can proceed.
    // 2. KiCad PCB Export
    let kicad_text = export_kicad_pcb(&board);
    assert!(kicad_text.starts_with("(kicad_pcb (version 20240108)"));
    assert!(kicad_text.ends_with(")\n"));
    let open_parens = kicad_text.chars().filter(|&c| c == '(').count();
    let close_parens = kicad_text.chars().filter(|&c| c == ')').count();
    assert_eq!(
        open_parens, close_parens,
        "KiCad s-expression parentheses must balance"
    );
    assert!(kicad_text.contains("RP2040"));
    assert!(kicad_text.contains("W25Q128JVS"));

    // 3. CAM Gerbers and Drill Export
    let cam_files = export_all_gerbers_and_drill(&board);
    assert_eq!(
        cam_files.len(),
        7,
        "CAM package must contain 6 Gerber layers + 1 drill file"
    );

    let required_layers = [
        "F_Cu.gbr",
        "B_Cu.gbr",
        "F_SilkS.gbr",
        "Edge_Cuts.gbr",
        "F_Mask.gbr",
        "B_Mask.gbr",
    ];
    for &layer in &required_layers {
        let content = cam_files
            .get(layer)
            .expect("Layer must exist in CAM output");
        assert!(content.contains("%FSLAX46Y46*%"));
        assert!(content.contains("%MOMM*%"));
        assert!(content.ends_with("M02*\n"));
    }

    let drill = cam_files.get("drill.drl").expect("Drill file must exist");
    assert!(drill.contains("M48"));
    assert!(drill.contains("METRIC"));
    assert!(drill.contains("M30"));

    // 4. BOM and CPL
    let bom = generate_framework_card_bom();
    let bom_csv = export_bom_csv(&bom);
    assert!(bom_csv.contains("RP2040"));
    assert!(bom_csv.contains("W25Q128JVS"));
    assert!(bom_csv.contains("AP2112K-3.3"));
    assert!(bom_csv.contains("ABM8-272-T3"));
    let bom_refs: std::collections::BTreeSet<_> =
        bom.iter().map(|row| row.designator.as_str()).collect();
    let board_refs: std::collections::BTreeSet<_> = board
        .footprints
        .iter()
        .map(|fp| fp.reference.as_str())
        .collect();
    assert_eq!(
        bom_refs, board_refs,
        "The BOM must include every actual circuit part"
    );

    let cpl = generate_framework_card_cpl(&board);
    let cpl_csv = export_cpl_csv(&cpl);
    assert!(cpl_csv.starts_with("Designator,Mid X,Mid Y,Layer,Rotation"));
    assert_eq!(cpl.len(), board.footprints.len());

    // 5. Production ZIP Archive
    let mut file_entries = Vec::new();
    let mut string_cache = Vec::new();
    for (name, content) in &cam_files {
        string_cache.push((name.clone(), content.clone()));
    }
    for (name, content) in &string_cache {
        file_entries.push((name.as_str(), content.as_bytes()));
    }
    let zip_bytes = create_zip_archive(&file_entries);
    assert!(zip_bytes.len() > 1000);
    assert_eq!(&zip_bytes[0..4], &[0x50, 0x4B, 0x03, 0x04]);
}

#[test]
fn test_m64_cart_pro_manufacturing_pipeline() {
    let config = RouterConfig {
        grid_resolution: 0.20,
        default_trace_width: 0.20,
        via_diameter: 0.60,
        via_drill: 0.30,
        clearance: 0.15,
    };

    let board = solve_m64_cart_pro_board(&config).expect("M64 Cart Pro must solve");

    // 1. DRC Verification
    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "Zero short circuits");
    assert_eq!(
        drc.clearance_violations.len(),
        0,
        "Zero clearance violations"
    );
    assert_eq!(
        drc.annular_ring_violations.len(),
        0,
        "Zero annular ring violations"
    );
    assert_eq!(
        drc.edge_keepout_violations.len(),
        0,
        "Zero edge keepout violations"
    );
    assert_eq!(drc.unrouted_nets.len(), 0, "Zero unrouted nets");

    // 2. KiCad PCB Export
    let kicad_text = export_kicad_pcb(&board);
    assert!(kicad_text.starts_with("(kicad_pcb (version 20240108)"));
    assert!(kicad_text.ends_with(")\n"));
    let open_parens = kicad_text.chars().filter(|&c| c == '(').count();
    let close_parens = kicad_text.chars().filter(|&c| c == ')').count();
    assert_eq!(
        open_parens, close_parens,
        "KiCad s-expression parentheses must balance"
    );
    assert!(kicad_text.contains("GW1N-UV9QN88"));
    assert!(kicad_text.contains("MB85RS256B"));

    // 3. CAM Gerbers and Drill Export
    let cam_files = export_all_gerbers_and_drill(&board);
    assert_eq!(
        cam_files.len(),
        7,
        "CAM package must contain 6 Gerber layers + 1 drill file"
    );

    let required_layers = [
        "F_Cu.gbr",
        "B_Cu.gbr",
        "F_SilkS.gbr",
        "Edge_Cuts.gbr",
        "F_Mask.gbr",
        "B_Mask.gbr",
    ];
    for &layer in &required_layers {
        let content = cam_files
            .get(layer)
            .expect("Layer must exist in CAM output");
        assert!(content.contains("%FSLAX46Y46*%"));
        assert!(content.contains("%MOMM*%"));
        assert!(content.ends_with("M02*\n"));
    }

    let drill = cam_files.get("drill.drl").expect("Drill file must exist");
    assert!(drill.contains("M48"));
    assert!(drill.contains("METRIC"));
    assert!(drill.contains("M30"));

    // 4. BOM and CPL
    let bom = generate_m64_cart_pro_bom();
    let bom_csv = export_bom_csv(&bom);
    assert!(bom_csv.contains("GW1N-UV9QN88"));
    assert!(bom_csv.contains("MB85RS256B"));
    assert!(bom_csv.contains("AP3428WT-7"));
    assert!(bom_csv.contains("FX8-50P-SV"));

    let cpl = generate_m64_cart_pro_cpl(&board);
    let cpl_csv = export_cpl_csv(&cpl);
    assert!(cpl_csv.starts_with("Designator,Mid X,Mid Y,Layer,Rotation"));
    assert_eq!(cpl.len(), board.footprints.len());

    // 5. Production ZIP Archive
    let mut file_entries = Vec::new();
    let mut string_cache = Vec::new();
    for (name, content) in &cam_files {
        string_cache.push((name.clone(), content.clone()));
    }
    for (name, content) in &string_cache {
        file_entries.push((name.as_str(), content.as_bytes()));
    }
    let zip_bytes = create_zip_archive(&file_entries);
    assert!(zip_bytes.len() > 1000);
    assert_eq!(&zip_bytes[0..4], &[0x50, 0x4B, 0x03, 0x04]);
}
