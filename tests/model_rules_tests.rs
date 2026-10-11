use netlist_shatter_viz::pcb::*;
use std::{fs, path::PathBuf, process::Command};

fn directory(label: &str) -> PathBuf {
    let stamp=std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos();
    let dir=std::env::temp_dir().join(format!("model-rules-{label}-{}-{stamp}",std::process::id()));
    fs::create_dir_all(&dir).unwrap();dir
}
fn command() -> Command { Command::new(env!("CARGO_BIN_EXE_export_manufacturing")) }
fn rules() -> RouterConfig {
    RouterConfig {grid_resolution:0.1,default_trace_width:0.2032,clearance:0.254,via_diameter:0.8128,via_drill:0.4064}
}
#[test]
fn numeric_rules_reuse_router_invariants() {
    let good=rules();assert!(good.validate().is_ok());
    let mut zero_clearance=good;zero_clearance.clearance=0.0;assert!(zero_clearance.validate().is_ok());
    for index in 0..5 {
        for value in [f64::NAN,f64::INFINITY,-1.0] {
            let mut r=good;
            match index {0=>r.grid_resolution=value,1=>r.default_trace_width=value,2=>r.via_diameter=value,3=>r.via_drill=value,_=>r.clearance=value}
            assert!(r.validate().is_err());
        }
    }
    assert!(RouterConfig {via_drill:good.via_diameter,..good}.validate().is_err());
    assert!(RouterConfig {grid_resolution:0.0,..good}.validate().is_err());
}
#[test]
fn malformed_or_extra_rules_cannot_be_silently_accepted() {
    for text in ["null","[]","{}","{",r#"{"grid_resolution":NaN}"#,
                 r#"{"grid_resolution":1e999,"default_trace_width":0.2,"clearance":0.2,"via_diameter":0.8,"via_drill":0.4}"#,
                 r#"{"grid_resolution":true,"default_trace_width":0.2,"clearance":0.2,"via_diameter":0.8,"via_drill":0.4}"#] {
        assert!(serde_json::from_str::<RouterConfig>(text).is_err(),"{text}");
    }
    let mut extra=serde_json::to_value(rules()).unwrap();extra["copper_edge_clearance"]=serde_json::json!(0.4);
    assert!(serde_json::from_value::<RouterConfig>(extra).is_err());
}
#[test]
fn supplied_rules_match_draft_session_and_native_project() {
    let dir=directory("positive");let board=dir.join("input.json");let provided=dir.join("provided.json");let session=dir.join("route.ses");
    fs::write(&board,serde_json::to_string(&PcbBoard::new(10.0,10.0)).unwrap()).unwrap();
    let supplied=serde_json::to_vec_pretty(&rules()).unwrap();fs::write(&provided,&supplied).unwrap();
    fs::write(&session,"(session reference (routes (resolution mm 1000000) (network_out)))").unwrap();
    for label in ["draft","session"] {
        let out=dir.join(label);let mut cmd=command();cmd.args(["--board","reference","--draft","--input"]).arg(&board).arg("--rules").arg(&provided).arg("--out").arg(&out);
        if label=="session" {cmd.arg("--session").arg(&session);}
        let result=cmd.output().unwrap();assert!(result.status.success(),"{}",String::from_utf8_lossy(&result.stderr));
        let r:RouterConfig=serde_json::from_slice(&fs::read(out.join("rules.json")).unwrap()).unwrap();assert_eq!(r,rules());
        let project:serde_json::Value=serde_json::from_slice(&fs::read(out.join("reference.kicad_pro")).unwrap()).unwrap();
        let native=&project["board"]["design_settings"]["rules"];
        for (name,value) in [("min_clearance",r.clearance),("min_track_width",r.default_trace_width),("min_via_diameter",r.via_diameter),("min_through_hole_diameter",r.via_drill)] {assert_eq!(native[name].as_f64(),Some(value));}
        let (_,dsn_rules)=parse_specctra_dsn(&fs::read_to_string(out.join("reference.dsn")).unwrap()).unwrap().to_board().unwrap();
        assert!((dsn_rules.default_trace_width-r.default_trace_width).abs()<1e-9);assert!((dsn_rules.clearance-r.clearance).abs()<1e-9);
        assert_eq!(fs::read(&provided).unwrap(),supplied);
    }
    assert_eq!(fs::read(dir.join("draft/rules.json")).unwrap(),fs::read(dir.join("session/rules.json")).unwrap());
    fs::remove_dir_all(dir).unwrap();
}
#[test]
fn invalid_rules_fail_even_for_draft_before_any_export() {
    let dir=directory("negative");let provided=dir.join("bad.json");let out=dir.join("out");
    for text in ["{",r#"{"grid_resolution":0,"default_trace_width":0.2,"clearance":0.2,"via_diameter":0.8,"via_drill":0.4}"#,
                 r#"{"grid_resolution":0.1,"default_trace_width":0.2,"clearance":0.2,"via_diameter":0.8,"via_drill":0.8}"#,
                 r#"{"grid_resolution":0.1,"default_trace_width":0.2,"clearance":0.2,"via_diameter":0.8,"via_drill":0.4,"unknown_rule":1}"#] {
        fs::write(&provided,text).unwrap();let result=command().args(["--draft","--rules"]).arg(&provided).arg("--out").arg(&out).output().unwrap();
        assert!(!result.status.success());assert!(!out.exists());
    }
    fs::remove_dir_all(dir).unwrap();
}
