use netlist_shatter_viz::*;
fn main() {
    let kind = std::env::args().nth(1).unwrap_or("framework".into());
    let mut b = match kind.as_str() { "bringup" => create_rp2040_bringup_board(), "pico" => create_pico_ultimate_board(), "esp32" => create_esp32_cam_pro_board(), "m64" => create_m64_cart_pro_board(), _=>create_framework_card_board() };
    let c = RouterConfig { grid_resolution: std::env::args().nth(2).unwrap_or("0.1".into()).parse().unwrap(), default_trace_width:0.15, clearance:0.15, via_diameter:0.6, via_drill:0.3 };
    std::fs::write(format!("/tmp/{kind}.dsn"), export_specctra_dsn_with_rules(&b,&SpecctraDsnExportConfig {pcb_name:kind.clone(),unit:SpecctraUnit::Um,resolution_factor:10.0,top_layer_name:"F.Cu".into(),bot_layer_name:"B.Cu".into()},&c)).unwrap();
    let pre = run_drc(&b,&c);
    for v in pre.short_circuits.iter().chain(&pre.clearance_violations).chain(&pre.edge_keepout_violations) { eprintln!("PRE: {v:?}"); }
    if std::env::var_os("NO_ROUTE").is_none() { println!("ROUTE: {:?}",route_board_with_report(&mut b,&c)); }
    if let Ok(path)=std::env::var("IMPORT_SES") { import_specctra_ses(&mut b,&std::fs::read_to_string(path).unwrap()).unwrap(); }
    let d = run_drc(&b,&c);
    println!("DRC {}",d.total_violations());
    for v in d.short_circuits.iter().chain(&d.clearance_violations).chain(&d.edge_keepout_violations) {eprintln!("POST: {v:?}");}
    println!("UNROUTED: {:?}",d.unrouted_nets);
    std::fs::write(format!("/tmp/{kind}-repaired.kicad_pcb"),export_kicad_pcb(&b)).unwrap();
    std::fs::write(format!("/tmp/{kind}-repaired.json"),serde_json::to_string_pretty(&b).unwrap()).unwrap();
}
