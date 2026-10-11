use netlist_shatter_viz::pcb::*;
use std::{env, fs, path::PathBuf};
fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut name = "bringup".to_string();
    let mut out = None;
    let mut session = None;
    let mut input = None;
    let mut rules_path = None;
    let mut draft = false;
    let mut route = false;
    let mut cleanup = false;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--board" => name = args.next().ok_or("--board needs a name")?,
            "--out" => out = Some(PathBuf::from(args.next().ok_or("--out needs a directory")?)),
            "--session" => session = Some(args.next().ok_or("--session needs a .ses file")?),
            "--input" => input = Some(args.next().ok_or("--input needs board.json")?),
            "--rules" => rules_path = Some(args.next().ok_or("--rules needs rules.json")?),
            "--draft" => draft = true,
            "--route" => route = true,
            "--cleanup-route" => cleanup = true,
            "--help" | "-h" => {
                println!(
                    "export_manufacturing [--board bringup|framework|pico|esp32|m64|reference] [--input board.json] [--rules rules.json] [--session route.ses | --route] [--out directory] [--draft] [--cleanup-route]\nExports JSON, KiCad, Specctra, Gerbers, drill, engineering BOM/CPL and validation report.\nWithout --draft, failing software checks return an error. These outputs remain unqualified hardware prototypes.\nUse scripts/build_board.py for routing, independent checks and a review archive."
                );
                return Ok(());
            }
            _ => return Err(format!("Unknown option {arg}; use --help").into()),
        }
    }
    if route && session.is_some() {
        return Err("Choose --route or --session".into());
    }
    if !["bringup", "framework", "pico", "esp32", "m64", "reference"].contains(&name.as_str()) {
        return Err("Unknown board name".into());
    }
    let mut b = if let Some(p) = input {
        serde_json::from_str(&fs::read_to_string(p)?)?
    } else {
        board_by_name(&name)?
    };
    let rules: RouterConfig = if let Some(path) = rules_path {
        serde_json::from_str(&fs::read_to_string(path)?)?
    } else {
        prototype_rules()
    };
    rules.validate()?;
    if route {
        route_board(&mut b, &rules)?;
    }
    if let Some(p) = session {
        import_specctra_ses(&mut b, &fs::read_to_string(p)?)?;
    }
    let out = out.unwrap_or_else(|| PathBuf::from(format!("build/{name}")));
    let cleanup_report=if cleanup {Some(cleanup_route(&mut b,&rules))}else{None};
    let report = write_design(&b, &name, &out, &rules)?;
    if let Some(r)=cleanup_report {fs::write(out.join("route-cleanup.json"),serde_json::to_string_pretty(&r)?)?;}
    println!(
        "{}: {} components, {} tracks, {} vias; {} DRC violations, {} electrical blockers, {} geometry errors.\nFiles: {}",
        name,
        report.components,
        report.tracks,
        report.vias,
        report.drc.total_violations(),
        report.electrical_issues.len(),
        report.geometry_issues.len(),
        out.display()
    );
    if !draft && !report.software_checks_pass {
        return Err(
            "Validation failed. Read report.json; use --draft only for inspection/routing input."
                .into(),
        );
    }
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("Error: {e}");
        std::process::exit(1);
    }
}
