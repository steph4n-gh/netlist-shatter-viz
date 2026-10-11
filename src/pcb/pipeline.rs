//! Reproducible design export and a fail-closed software validation gate.
use super::*;
use serde::Serialize;
use std::{fs, path::Path};

pub fn prototype_rules() -> RouterConfig {
    RouterConfig {
        grid_resolution: 0.1,
        default_trace_width: 0.15,
        clearance: 0.15,
        via_diameter: 0.6,
        via_drill: 0.3,
    }
}

pub fn board_by_name(name: &str) -> Result<PcbBoard, String> {
    Ok(match name {
        "bringup" => create_rp2040_bringup_board(),
        "framework" => create_framework_card_board(),
        "pico" => create_pico_ultimate_board(),
        "esp32" => create_esp32_cam_pro_board(),
        "m64" => create_m64_cart_pro_board(),
        "reference" => create_reference_board(),
        _ => {
            return Err(format!(
                "Unknown board {name}. Choose bringup, framework, pico, esp32, m64, reference."
            ));
        }
    })
}

#[derive(Debug, Serialize)]
pub struct ValidationReport {
    pub software_checks_pass: bool,
    pub hardware_validated: bool,
    pub drc: DrcResult,
    pub electrical_issues: Vec<ElectricalIssue>,
    pub geometry_issues: Vec<String>,
    pub components: usize,
    pub tracks: usize,
    pub vias: usize,
    pub limitations: Vec<String>,
}

pub fn validate_board(board: &PcbBoard, rules: &RouterConfig) -> ValidationReport {
    let drc = run_drc(board, rules);
    let electrical_issues = check_electrical(board);
    let mut geometry_issues: Vec<String> = board
        .validate()
        .errors
        .iter()
        .map(|e| format!("{e:?}"))
        .collect();
    if ![board.width, board.height, board.thickness]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0)
    {
        geometry_issues.push("Board dimensions must be positive and finite".into());
    }
    for (i, t) in board.tracks.iter().enumerate() {
        if ![t.start.x, t.start.y, t.end.x, t.end.y, t.width]
            .iter()
            .all(|v| v.is_finite())
            || t.width + 0.00001 < rules.default_trace_width
        {
            geometry_issues.push(format!(
                "Track {i}: invalid geometry or width below {} mm",
                rules.default_trace_width
            ));
        }
    }
    let mut refs = std::collections::HashSet::new();
    for f in &board.footprints {
        if !refs.insert(&f.reference) {
            geometry_issues.push(format!("Duplicate reference {}", f.reference));
        }
        for p in &f.pads {
            if p.net
                .is_some_and(|n| n != 0 && !board.net_names.contains_key(&n))
            {
                geometry_issues.push(format!(
                    "{}-{} references an undeclared net",
                    f.reference, p.pin_number
                ));
            }
        }
    }
    let mut limitations = vec!["Prototype only: part-specific land patterns, assembly sourcing, power integrity, USB signal integrity and physical bring-up still require review.".into(),"Electrical assertions cover selected known requirements; they are not a complete schematic ERC.".into()];
    if board.footprints.iter().any(|f| f.value == "QWIIC-I2C")
        && board.footprints.iter().any(|f| f.value == "RP2040")
    {
        limitations.push("Framework draft: replace the generic USB receptacle with the official plug/mechanical design; qualify crystal/power loop placement, I2C pull-ups and ESD protection before fabrication.".into());
    }
    if board
        .footprints
        .iter()
        .any(|f| f.value.starts_with("MCP73831"))
    {
        limitations.push("Pico battery design: requires an externally protected 4.2V-charge 1S pack with qualified polarity/current/temperature limits. Nominal charging is 100mA; USB source/load budget, battery temperature protection, switching power/thermal loops and exact inductor/capacitor/connector land patterns require qualification. Header pinout/spacing do not establish a Pico-compatible outline.".into());
        limitations.push("RGB supply envelope: the selected LED's minimum voltage must be checked against regulator tolerance, transients, load and temperature; nominal 3.3V alone is not a guaranteed operating margin.".into());
    }
    if board
        .footprints
        .iter()
        .any(|f| f.value == "ESP32-WROVER-E-N4R8")
    {
        limitations.extend(
            super::esp32_cam_pro::QUALIFICATION_HOLDS
                .iter()
                .map(|s| s.to_string()),
        );
    }
    ValidationReport {
        software_checks_pass: drc.is_clean()
            && electrical_issues.is_empty()
            && geometry_issues.is_empty(),
        hardware_validated: false,
        drc,
        electrical_issues,
        geometry_issues,
        components: board.footprints.len(),
        tracks: board.tracks.len(),
        vias: board.vias.len(),
        limitations,
    }
}

pub fn dsn_for_board(board: &PcbBoard, name: &str, rules: &RouterConfig) -> String {
    export_specctra_dsn_with_rules(
        board,
        &SpecctraDsnExportConfig {
            pcb_name: name.into(),
            unit: SpecctraUnit::Um,
            resolution_factor: 10.0,
            top_layer_name: "F.Cu".into(),
            bot_layer_name: "B.Cu".into(),
        },
        rules,
    )
}

/// Writes reviewable files for both passing and blocked designs. Never emits an order archive.
pub fn write_design(
    board: &PcbBoard,
    name: &str,
    directory: &Path,
    rules: &RouterConfig,
) -> Result<ValidationReport, Box<dyn std::error::Error>> {
    fs::create_dir_all(directory)?;
    let report = validate_board(board, rules);
    fs::write(
        directory.join("board.json"),
        serde_json::to_string_pretty(board)?,
    )?;
    fs::write(
        directory.join("report.json"),
        serde_json::to_string_pretty(&report)?,
    )?;
    fs::write(
        directory.join("rules.json"),
        serde_json::to_string_pretty(rules)?,
    )?;
    fs::write(
        directory.join(format!("{name}.dsn")),
        dsn_for_board(board, name, rules),
    )?;
    fs::write(
        directory.join(format!("{name}.kicad_pcb")),
        export_kicad_pcb(board),
    )?;
    // Keep KiCad's independent checker on the exact same numerical design rules.
    let project = serde_json::json!({"board":{"design_settings":{"rules":{
        "min_clearance":rules.clearance,"min_track_width":rules.default_trace_width,
        "min_via_diameter":rules.via_diameter,"min_through_hole_diameter":rules.via_drill,
        "min_hole_clearance":rules.clearance,"min_copper_edge_clearance":0.4,"min_via_annular_width":0.15
    }}},"net_settings":{"classes":[{"name":"Default","clearance":rules.clearance,"track_width":rules.default_trace_width,"via_diameter":rules.via_diameter,"via_drill":rules.via_drill}]}});
    fs::write(
        directory.join(format!("{name}.kicad_pro")),
        serde_json::to_string_pretty(&project)?,
    )?;
    for (name, content) in export_all_gerbers_and_drill(board) {
        fs::write(directory.join(name), content)?;
    }
    let bom = if name == "pico" {
        generate_pico_ultimate_bom_for_board(board)
    } else if name == "esp32" {
        generate_esp32_cam_pro_bom_for_board(board)
    } else {
        generate_bom_from_board(board)
    };
    fs::write(directory.join("BOM.csv"), export_bom_csv(&bom))?;
    fs::write(
        directory.join("CPL.csv"),
        export_cpl_csv(&generate_cpl_from_board(board)),
    )?;
    fs::write(
        directory.join("PROTOTYPE.txt"),
        "ENGINEERING REVIEW OUTPUT — NOT AN APPROVED FABRICATION ORDER\nRead report.json and independent KiCad/CAM reports. No board has been physically validated.\nCPL coordinates: board origin at top left, +Y down, clockwise degrees; convert to your assembler's convention.\n",
    )?;
    Ok(report)
}
