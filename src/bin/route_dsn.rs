//! CLI Autorouting Tool for Specctra DSN & SES files.
//!
//! Usage:
//!   cargo run --bin route_dsn -- [OPTIONS] <input.dsn> <output.ses>
//!
//! Options:
//!   -h, --help            Print help information
//!   -v, --verbose         Print detailed progress and netlist diagnostics
//!   --resolution <mm>     Override router grid resolution in mm (default: derived from DSN)
//!   --width <mm>          Override default trace width in mm
//!   --clearance <mm>      Override default clearance in mm
//!   --drc                 Run full Design Rule Check (DRC) and fail if violations exist

use netlist_shatter_viz::pcb::{
    export_specctra_ses, parse_specctra_dsn, route_board_with_report, run_drc,
};
use std::env;
use std::fs;
use std::path::Path;
use std::process;
use std::time::Instant;

fn print_help() {
    println!(
        r#"Netlist Shatter - Universal Specctra DSN Autorouter

USAGE:
    route_dsn [OPTIONS] <INPUT.dsn> <OUTPUT.ses>

ARGS:
    <INPUT.dsn>     Path to input Specctra DSN design file (KiCad, Altium, Eagle)
    <OUTPUT.ses>    Path to output Specctra SES session file

OPTIONS:
    -h, --help            Print this help information
    -v, --verbose         Enable verbose logging of nets, layers, and routing iterations
    --resolution <val>    Override router grid resolution (mm, e.g. 0.25)
    --width <val>         Override default trace width (mm, e.g. 0.25)
    --clearance <val>     Override default trace clearance (mm, e.g. 0.20)
    --drc                 Enforce zero DRC clearance and short-circuit violations

EXAMPLE:
    cargo run --bin route_dsn -- board.dsn board.ses
    cargo run --bin route_dsn -- --verbose --drc input.dsn output.ses
"#
    );
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();

    if args.is_empty() {
        print_help();
        process::exit(1);
    }

    let mut input_path: Option<String> = None;
    let mut output_path: Option<String> = None;
    let mut verbose = false;
    let mut enforce_drc = false;
    let mut override_resolution: Option<f64> = None;
    let mut override_width: Option<f64> = None;
    let mut override_clearance: Option<f64> = None;
    let mut override_via: Option<f64> = None;
    let mut override_drill: Option<f64> = None;

    let mut i = 0;
    while i < args.len() {
        let arg = &args[i];
        match arg.as_str() {
            "-h" | "--help" => {
                print_help();
                process::exit(0);
            }
            "-v" | "--verbose" => {
                verbose = true;
                i += 1;
            }
            "--drc" => {
                enforce_drc = true;
                i += 1;
            }
            "--resolution" => {
                if i + 1 >= args.len() {
                    eprintln!("Error: --resolution requires a numeric value in mm");
                    process::exit(1);
                }
                override_resolution = args[i + 1].parse::<f64>().ok();
                if override_resolution.is_none() {
                    eprintln!("Error: invalid value for --resolution: '{}'", args[i + 1]);
                    process::exit(1);
                }
                i += 2;
            }
            "--width" => {
                if i + 1 >= args.len() {
                    eprintln!("Error: --width requires a numeric value in mm");
                    process::exit(1);
                }
                override_width = args[i + 1].parse::<f64>().ok();
                if override_width.is_none() {
                    eprintln!("Error: invalid value for --width: '{}'", args[i + 1]);
                    process::exit(1);
                }
                i += 2;
            }
            "--clearance" => {
                if i + 1 >= args.len() {
                    eprintln!("Error: --clearance requires a numeric value in mm");
                    process::exit(1);
                }
                override_clearance = args[i + 1].parse::<f64>().ok();
                if override_clearance.is_none() {
                    eprintln!("Error: invalid value for --clearance: '{}'", args[i + 1]);
                    process::exit(1);
                }
                i += 2;
            }
            "--via" => {
                if i + 1 >= args.len() {
                    eprintln!("Error: --via requires a numeric value in mm");
                    process::exit(1);
                }
                override_via = args[i + 1].parse::<f64>().ok();
                if override_via.is_none() {
                    eprintln!("Error: invalid value for --via: '{}'", args[i + 1]);
                    process::exit(1);
                }
                i += 2;
            }
            "--drill" => {
                if i + 1 >= args.len() {
                    eprintln!("Error: --drill requires a numeric value in mm");
                    process::exit(1);
                }
                override_drill = args[i + 1].parse::<f64>().ok();
                if override_drill.is_none() {
                    eprintln!("Error: invalid value for --drill: '{}'", args[i + 1]);
                    process::exit(1);
                }
                i += 2;
            }
            val if val.starts_with('-') => {
                eprintln!("Error: unknown option '{}'", val);
                print_help();
                process::exit(1);
            }
            val => {
                if input_path.is_none() {
                    input_path = Some(val.to_string());
                } else if output_path.is_none() {
                    output_path = Some(val.to_string());
                } else {
                    eprintln!("Error: unexpected positional argument '{}'", val);
                    process::exit(1);
                }
                i += 1;
            }
        }
    }

    let in_file = match input_path {
        Some(p) => p,
        None => {
            eprintln!("Error: missing <INPUT.dsn> path");
            print_help();
            process::exit(1);
        }
    };

    let out_file = match output_path {
        Some(p) => p,
        None => {
            eprintln!("Error: missing <OUTPUT.ses> path");
            print_help();
            process::exit(1);
        }
    };

    println!("============================================================");
    println!(" Netlist Shatter - Universal Specctra DSN Autorouter");
    println!("============================================================");
    println!("Input DSN : {}", in_file);
    println!("Output SES: {}", out_file);

    let start_time = Instant::now();

    // 1. Read input DSN file
    let dsn_content = match fs::read_to_string(&in_file) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Fatal Error: failed to read input file '{}': {}", in_file, e);
            process::exit(1);
        }
    };

    // 2. Parse DSN
    println!("\n[1/4] Parsing Specctra DSN document...");
    let dsn = match parse_specctra_dsn(&dsn_content) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("Fatal Error: failed to parse DSN: {}", e);
            process::exit(1);
        }
    };

    println!("  - Design Name      : {}", dsn.pcb_name);
    println!(
        "  - Unit / Resolution: {} (factor {})",
        dsn.resolution.unit.as_str(),
        dsn.resolution.factor
    );
    println!(
        "  - Board Dimensions : {:.2} mm x {:.2} mm",
        dsn.bounds.width(),
        dsn.bounds.height()
    );
    println!("  - Copper Layers    : {}", dsn.layers.len());
    println!("  - Placed Components: {}", dsn.components.len());
    println!("  - Nets Declared    : {}", dsn.nets.len());
    println!("  - Keepout Areas    : {}", dsn.keepouts.len());
    if dsn.has_prerouted_wiring {
        println!("  ⚠ WARNING: DSN contains pre-routed wiring. All traces will be re-routed from scratch.");
    }

    // 3. Convert to PcbBoard and configure router
    println!("\n[2/4] Ingesting into PCB routing geometry...");
    let (mut board, default_config) = match dsn.to_board() {
        Ok(res) => res,
        Err(e) => {
            eprintln!("Fatal Error: failed to ingest DSN into board: {}", e);
            process::exit(1);
        }
    };

    let mut router_config = default_config;
    if let Some(res) = override_resolution {
        router_config.grid_resolution = res;
    }
    if let Some(w) = override_width {
        router_config.default_trace_width = w;
    }
    if let Some(c) = override_clearance {
        router_config.clearance = c;
    }
    if let Some(v) = override_via {
        router_config.via_diameter = v;
    }
    if let Some(d) = override_drill {
        router_config.via_drill = d;
    }

    println!(
        "  - Grid Resolution  : {:.3} mm",
        router_config.grid_resolution
    );
    println!(
        "  - Trace Width      : {:.3} mm",
        router_config.default_trace_width
    );
    println!(
        "  - Trace Clearance  : {:.3} mm",
        router_config.clearance
    );
    println!(
        "  - Via Size / Drill : {:.2} mm / {:.2} mm",
        router_config.via_diameter, router_config.via_drill
    );

    // 4. Run Autorouter
    println!("\n[3/4] Executing 2-Layer Maze Routing with Rip-Up & Reroute (RRR)...");
    let route_start = Instant::now();
    let report = match route_board_with_report(&mut board, &router_config) {
        Ok(rep) => rep,
        Err(e) => {
            eprintln!("Routing Failure: {}", e);
            process::exit(2);
        }
    };
    let route_duration = route_start.elapsed();

    println!("  - Routed Nets      : {} / {}", report.routed_nets, report.routed_nets + report.unrouted_nets);
    println!("  - Tracks Generated : {}", report.total_tracks);
    println!("  - Vias Inserted    : {}", report.total_vias);
    println!("  - RRR Iterations   : {}", report.rrr_iterations);
    println!("  - Routing Duration : {:.3} s", route_duration.as_secs_f64());

    // Optional DRC Check
    if enforce_drc || verbose {
        println!("\nDesign Rule Checking (DRC) Verification:");
        let drc = run_drc(&board, &router_config);
        println!("  - Unrouted Nets        : {}", drc.unrouted_nets.len());
        println!("  - Clearance Violations : {}", drc.clearance_violations.len());
        println!("  - Short Circuits       : {}", drc.short_circuits.len());

        if enforce_drc && (!drc.unrouted_nets.is_empty() || !drc.clearance_violations.is_empty() || !drc.short_circuits.is_empty()) {
            eprintln!("DRC Check Failed: violations detected");
            process::exit(3);
        }
    }

    // 5. Export Specctra Session (.ses)
    println!("\n[4/4] Generating Specctra Session (.ses) file...");
    let ses_content = export_specctra_ses(&board, &dsn);

    if let Some(parent) = Path::new(&out_file).parent() {
        if !parent.as_os_str().is_empty() {
            let _ = fs::create_dir_all(parent);
        }
    }

    if let Err(e) = fs::write(&out_file, &ses_content) {
        eprintln!("Fatal Error: failed to write output SES to '{}': {}", out_file, e);
        process::exit(1);
    }

    let total_duration = start_time.elapsed();
    println!("Successfully exported Specctra session to: {}", out_file);
    println!("Total Elapsed Time: {:.3} s", total_duration.as_secs_f64());
    println!("Done. Re-import into KiCad with: File -> Import -> Specctra Session File\n");
}
