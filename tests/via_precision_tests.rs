use netlist_shatter_viz::pcb::*;

const ID: &str = "Via[0-1]_812800:406400_nm";
const DSN: &str = include_str!("fixtures/via-precision.dsn");
const SES: &str = include_str!("fixtures/via-precision.ses");
fn rules() -> RouterConfig {
    RouterConfig {grid_resolution:0.1,default_trace_width:0.2032,clearance:0.254,via_diameter:0.8128,via_drill:0.4064}
}
fn close(actual:f64, expected:f64) { assert!((actual-expected).abs()<1e-12,"{actual} != {expected}"); }

#[test]
fn pinned_freerouting_actual_forced_via_recovers_exact_dimensions() {
    // Actual output from pinned jar SHA52471120...: opposite-layer SMD pins require a via.
    let dsn=parse_specctra_dsn(DSN).unwrap();
    let (mut board,derived)=dsn.to_board().unwrap();
    close(derived.via_diameter,0.8128);close(derived.via_drill,0.4064);
    let frames=board.footprints.clone();
    import_specctra_ses(&mut board,SES).unwrap();
    assert_eq!(board.footprints,frames);assert_eq!(board.vias.len(),1);
    close(board.vias[0].size,0.8128);close(board.vias[0].drill,0.4064);
    assert_eq!(board.tracks.len(),6);
    let native=export_kicad_pcb(&board);
    assert!(native.contains("(size 0.8128) (drill 0.4064)"));
    let report=validate_board(&board,&rules());assert!(report.software_checks_pass,"{report:?}");
}
#[test]
fn both_exported_via_names_and_session_fallback_round_trip() {
    let supplied=rules();let original=supplied;
    let board=PcbBoard::new(20.0,12.0);
    let exported=dsn_for_board(&board,"precision",&supplied);
    assert_eq!(exported.matches(ID).count(),2);
    let (_,derived)=parse_specctra_dsn(&exported).unwrap().to_board().unwrap();
    close(derived.via_diameter,supplied.via_diameter);close(derived.via_drill,supplied.via_drill);
    let mut board=board;board.register_net(1,"signal");
    board.vias.push(Via::new(Point::new(10.0,6.0),supplied.via_diameter,supplied.via_drill,1));
    let exported=export_specctra_ses(&board,&SpecctraDsn::default());
    assert!(exported.contains(ID));
    board.vias.clear();import_specctra_ses(&mut board,&exported).unwrap();
    close(board.vias[0].size,supplied.via_diameter);close(board.vias[0].drill,supplied.via_drill);
    assert_eq!(supplied,original);
}
#[test]
fn legacy_um_imports_remain_exact_and_sub_nm_drills_round_outward() {
    let old="Via[0-1]_812:406_um";
    let mut board=parse_specctra_dsn(DSN).unwrap().to_board().unwrap().0;
    import_specctra_ses(&mut board,&SES.replace(ID,old)).unwrap();
    close(board.vias[0].size,0.8128);close(board.vias[0].drill,0.406);
    let dsn=parse_specctra_dsn(&DSN.replace(ID,old)).unwrap();
    close(dsn.get_via_dimensions(0.2).1,0.406);
    let exported=export_specctra_ses(&board,&dsn);assert!(exported.contains(old));
    let r=RouterConfig {via_drill:0.4064001,..rules()};
    let exported=dsn_for_board(&PcbBoard::new(20.0,12.0),"precision",&r);
    assert!(exported.contains("Via[0-1]_812800:406401_nm"));
    let (_,derived)=parse_specctra_dsn(&exported).unwrap().to_board().unwrap();
    assert!(derived.via_drill>=r.via_drill);close(derived.via_drill,0.406401);
}
#[test]
fn malformed_nm_labels_fail_dsn_and_session_before_copper_changes() {
    for bad in ["Via[0-1]_812800:406.4_nm","Via[0-1]_812800:NaN_nm",
                "Via[0-1]_812800:406400_nm_extra","Via[0-1]_812800:+406400_nm",
                "Via[0-1]_812800:0_nm","Via[0-1]_812800:812800_nm",
                "Via[0-1]_812800:99999999999999999999999999_nm",
                "Via[1-0]_812800:406400_nm","Via[0-1]_812800:406400_NM"] {
        assert!(parse_specctra_dsn(&DSN.replace(ID,bad)).is_err(),"{bad}");
        let mut board=parse_specctra_dsn(DSN).unwrap().to_board().unwrap().0;
        let original=board.clone();
        assert!(import_specctra_ses(&mut board,&SES.replace(ID,bad)).is_err(),"{bad}");
        assert_eq!(board,original,"{bad}");
    }
}
#[test]
fn named_nm_fallback_without_library_keeps_exact_dimensions() {
    let mut dsn=SpecctraDsn::default();dsn.via_padstack_ids=vec![ID.into()];
    let (diameter,drill)=dsn.get_via_dimensions(0.2);
    close(diameter,0.8128);close(drill,0.4064);
}
