//! Empirical Challenger 1 Test Suite for Milestone 4:
//! KiCad S-Expression Exporter (`src/pcb/kicad.rs`)
//!
//! Rigorously verifies:
//! 1. S-expression recursive grammar validation with full tokenizer & AST parser
//!    (tracks exact parenthesis depth, handles escaped quotes, parens inside strings)
//! 2. Netlist cross-referencing: pads, tracks, and vias against declared net tables
//! 3. Footprint property verification: Reference, Value, Package, Pad shapes/sizes, Silkscreen
//! 4. Edge cases & Stress testing: Special characters, empty boards, dense 50-net layouts,
//!    extreme floating point values, closed Edge.Cuts polygon validation, undeclared net detection probe.

use netlist_shatter_viz::pcb::*;
use std::collections::{HashMap, HashSet};

// ============================================================================
// Recursive S-Expression Tokenizer and AST Parser
// ============================================================================

#[derive(Debug, PartialEq, Clone)]
pub enum Token {
    OpenParen,
    CloseParen,
    QuotedString(String),
    Atom(String),
}

#[derive(Debug, PartialEq, Clone)]
pub enum SExpr {
    Atom(String),
    QuotedString(String),
    List(Vec<SExpr>),
}

impl SExpr {
    pub fn as_list(&self) -> Option<&[SExpr]> {
        match self {
            SExpr::List(list) => Some(list.as_slice()),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            SExpr::Atom(s) | SExpr::QuotedString(s) => Some(s.as_str()),
            _ => None,
        }
    }

    pub fn tag(&self) -> Option<&str> {
        self.as_list()
            .and_then(|l| l.first())
            .and_then(|item| item.as_str())
    }

    pub fn find_sublist(&self, tag: &str) -> Option<&[SExpr]> {
        self.as_list().and_then(|list| {
            for item in list {
                if let SExpr::List(sub) = item {
                    if let Some(first) = sub.first() {
                        if first.as_str() == Some(tag) {
                            return Some(sub.as_slice());
                        }
                    }
                }
            }
            None
        })
    }

    pub fn find_all_sublists(&self, tag: &str) -> Vec<&[SExpr]> {
        let mut results = Vec::new();
        if let SExpr::List(list) = self {
            for item in list {
                if let SExpr::List(sub) = item {
                    if let Some(first) = sub.first() {
                        if first.as_str() == Some(tag) {
                            results.push(sub.as_slice());
                        }
                    }
                }
            }
        }
        results
    }

    pub fn find_all_recursive(&self, tag: &str) -> Vec<&[SExpr]> {
        let mut results = Vec::new();
        self.collect_recursive(tag, &mut results);
        results
    }

    fn collect_recursive<'a>(&'a self, tag: &str, results: &mut Vec<&'a [SExpr]>) {
        if let SExpr::List(list) = self {
            if let Some(first) = list.first() {
                if first.as_str() == Some(tag) {
                    results.push(list.as_slice());
                }
            }
            for item in list {
                item.collect_recursive(tag, results);
            }
        }
    }
}

pub struct SExprParser;

impl SExprParser {
    pub fn tokenize(input: &str) -> Result<Vec<Token>, String> {
        let mut tokens = Vec::new();
        let chars: Vec<char> = input.chars().collect();
        let mut i = 0;
        let mut paren_depth: i32 = 0;

        while i < chars.len() {
            let c = chars[i];
            if c.is_whitespace() {
                i += 1;
                continue;
            }

            if c == '(' {
                paren_depth += 1;
                tokens.push(Token::OpenParen);
                i += 1;
            } else if c == ')' {
                paren_depth -= 1;
                if paren_depth < 0 {
                    return Err(format!(
                        "Premature closing parenthesis encountered at char index {}",
                        i
                    ));
                }
                tokens.push(Token::CloseParen);
                i += 1;
            } else if c == '"' {
                // Quoted string literal with escape sequence support
                i += 1;
                let mut s = String::new();
                let mut closed = false;
                while i < chars.len() {
                    let sc = chars[i];
                    if sc == '\\' {
                        i += 1;
                        if i >= chars.len() {
                            return Err("Dangling escape backslash at EOF".to_string());
                        }
                        let esc = chars[i];
                        match esc {
                            'n' => s.push('\n'),
                            'r' => s.push('\r'),
                            't' => s.push('\t'),
                            '\\' => s.push('\\'),
                            '"' => s.push('"'),
                            other => s.push(other),
                        }
                        i += 1;
                    } else if sc == '"' {
                        closed = true;
                        i += 1;
                        break;
                    } else {
                        s.push(sc);
                        i += 1;
                    }
                }
                if !closed {
                    return Err("Unclosed string literal".to_string());
                }
                tokens.push(Token::QuotedString(s));
            } else {
                // Atom (symbol / number / identifier)
                let mut atom = String::new();
                while i < chars.len() && !chars[i].is_whitespace() && chars[i] != '(' && chars[i] != ')' && chars[i] != '"' {
                    atom.push(chars[i]);
                    i += 1;
                }
                tokens.push(Token::Atom(atom));
            }
        }

        if paren_depth != 0 {
            return Err(format!(
                "Unbalanced parentheses at EOF: net depth was {}",
                paren_depth
            ));
        }

        Ok(tokens)
    }

    pub fn parse(tokens: &[Token]) -> Result<SExpr, String> {
        let mut idx = 0;
        let expr = Self::parse_expr(tokens, &mut idx)?;
        if idx != tokens.len() {
            return Err(format!(
                "Trailing unparsed tokens at index {} of {}",
                idx,
                tokens.len()
            ));
        }
        Ok(expr)
    }

    fn parse_expr(tokens: &[Token], idx: &mut usize) -> Result<SExpr, String> {
        if *idx >= tokens.len() {
            return Err("Unexpected EOF while parsing S-expression".to_string());
        }

        match &tokens[*idx] {
            Token::OpenParen => {
                *idx += 1;
                let mut list = Vec::new();
                while *idx < tokens.len() {
                    if let Token::CloseParen = &tokens[*idx] {
                        *idx += 1;
                        return Ok(SExpr::List(list));
                    }
                    let child = Self::parse_expr(tokens, idx)?;
                    list.push(child);
                }
                Err("Unterminated list: missing closing parenthesis".to_string())
            }
            Token::CloseParen => Err(format!(
                "Unexpected closing parenthesis at token index {}",
                *idx
            )),
            Token::QuotedString(s) => {
                let res = SExpr::QuotedString(s.clone());
                *idx += 1;
                Ok(res)
            }
            Token::Atom(s) => {
                let res = SExpr::Atom(s.clone());
                *idx += 1;
                Ok(res)
            }
        }
    }

    pub fn parse_string(input: &str) -> Result<SExpr, String> {
        let tokens = Self::tokenize(input)?;
        Self::parse(&tokens)
    }
}

// ============================================================================
// Netlist Cross-Reference Auditor
// ============================================================================

#[derive(Debug, Default)]
pub struct NetlistAuditReport {
    pub declared_nets: HashMap<u32, String>,
    pub undeclared_pad_nets: Vec<(String, String, u32)>, // (comp_ref, pin_number, net_id)
    pub mismatched_pad_net_names: Vec<(String, String, u32, String, String)>, // (comp_ref, pin, net_id, found_name, decl_name)
    pub undeclared_track_nets: Vec<(usize, u32)>,        // (track_index, net_id)
    pub undeclared_via_nets: Vec<(usize, u32)>,          // (via_index, net_id)
}

impl NetlistAuditReport {
    pub fn is_clean(&self) -> bool {
        self.undeclared_pad_nets.is_empty()
            && self.mismatched_pad_net_names.is_empty()
            && self.undeclared_track_nets.is_empty()
            && self.undeclared_via_nets.is_empty()
    }
}

pub fn audit_kicad_netlist(ast: &SExpr) -> Result<NetlistAuditReport, String> {
    let root = ast
        .as_list()
        .ok_or_else(|| "Root AST node is not a list".to_string())?;

    let mut report = NetlistAuditReport::default();

    // 1. Collect all declared nets from top-level `(net <id> "<name>")`
    for item in root {
        if let SExpr::List(sub) = item {
            if sub.len() >= 3 && sub[0].as_str() == Some("net") {
                if let Some(id_str) = sub[1].as_str() {
                    if let Ok(id) = id_str.parse::<u32>() {
                        let name = sub[2].as_str().unwrap_or("").to_string();
                        report.declared_nets.insert(id, name);
                    }
                }
            }
        }
    }

    // KiCad mandates net 0 as (net 0 "")
    if !report.declared_nets.contains_key(&0) {
        return Err("KiCad S-expression missing mandatory (net 0 \"\") declaration".to_string());
    }

    let mut track_idx = 0;
    let mut via_idx = 0;

    // 2. Audit all elements
    for item in root {
        if let SExpr::List(sub) = item {
            let tag = match sub.first().and_then(|s| s.as_str()) {
                Some(t) => t,
                None => continue,
            };

            match tag {
                "footprint" => {
                    // Extract reference property
                    let mut comp_ref = "UNKNOWN".to_string();
                    for fp_elem in sub.iter() {
                        if let SExpr::List(prop_sub) = fp_elem {
                            if prop_sub.len() >= 3 && prop_sub[0].as_str() == Some("property") {
                                if prop_sub[1].as_str() == Some("Reference") {
                                    if let Some(r) = prop_sub[2].as_str() {
                                        comp_ref = r.to_string();
                                    }
                                }
                            }
                        }
                    }

                    // Audit pads
                    for fp_elem in sub.iter() {
                        if let SExpr::List(pad_sub) = fp_elem {
                            if pad_sub.len() >= 2 && pad_sub[0].as_str() == Some("pad") {
                                let pin_number = pad_sub[1].as_str().unwrap_or("?").to_string();
                                // Look for (net <id> "<name>") inside pad
                                for pad_field in pad_sub.iter() {
                                    if let SExpr::List(net_field) = pad_field {
                                        if net_field.len() >= 3 && net_field[0].as_str() == Some("net") {
                                            if let Some(id_str) = net_field[1].as_str() {
                                                if let Ok(net_id) = id_str.parse::<u32>() {
                                                    let net_name = net_field[2].as_str().unwrap_or("");
                                                    if let Some(decl_name) = report.declared_nets.get(&net_id) {
                                                        if decl_name != net_name {
                                                            report.mismatched_pad_net_names.push((
                                                                comp_ref.clone(),
                                                                pin_number.clone(),
                                                                net_id,
                                                                net_name.to_string(),
                                                                decl_name.clone(),
                                                            ));
                                                        }
                                                    } else {
                                                        report.undeclared_pad_nets.push((
                                                            comp_ref.clone(),
                                                            pin_number.clone(),
                                                            net_id,
                                                        ));
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                "segment" => {
                    // (segment ... (net <id>))
                    for seg_field in sub.iter() {
                        if let SExpr::List(net_field) = seg_field {
                            if net_field.len() >= 2 && net_field[0].as_str() == Some("net") {
                                if let Some(id_str) = net_field[1].as_str() {
                                    if let Ok(net_id) = id_str.parse::<u32>() {
                                        if !report.declared_nets.contains_key(&net_id) {
                                            report.undeclared_track_nets.push((track_idx, net_id));
                                        }
                                    }
                                }
                            }
                        }
                    }
                    track_idx += 1;
                }
                "via" => {
                    // (via ... (net <id>))
                    for via_field in sub.iter() {
                        if let SExpr::List(net_field) = via_field {
                            if net_field.len() >= 2 && net_field[0].as_str() == Some("net") {
                                if let Some(id_str) = net_field[1].as_str() {
                                    if let Ok(net_id) = id_str.parse::<u32>() {
                                        if !report.declared_nets.contains_key(&net_id) {
                                            report.undeclared_via_nets.push((via_idx, net_id));
                                        }
                                    }
                                }
                            }
                        }
                    }
                    via_idx += 1;
                }
                _ => {}
            }
        }
    }

    Ok(report)
}

// ============================================================================
// TEST 1: S-Expression Tokenizer, Parens Depth, and Escape Semantics
// ============================================================================

#[test]
fn test_challenger_sexpr_tokenizer_and_escape_grammar() {
    // 1. Parens inside string literals must NOT alter parenthesis depth
    let sexpr_str = r#"(test "hello (world) with (nested) parens" (child 123 "another (paren)"))"#;
    let tokens = SExprParser::tokenize(sexpr_str).expect("Tokenizer must succeed");
    let parsed = SExprParser::parse(&tokens).expect("Parser must succeed");

    assert_eq!(parsed.tag(), Some("test"));
    let list = parsed.as_list().unwrap();
    assert_eq!(list.len(), 3);
    assert_eq!(
        list[1].as_str(),
        Some("hello (world) with (nested) parens")
    );

    // 2. Escaped quotes \" inside string literals
    let escaped_str = r#"(prop "key" "value with \"quotes\" and \\backslash")"#;
    let parsed_esc = SExprParser::parse_string(escaped_str).expect("Must parse escaped quotes");
    let esc_list = parsed_esc.as_list().unwrap();
    assert_eq!(esc_list[2].as_str(), Some("value with \"quotes\" and \\backslash"));

    // 3. Premature close must fail immediately
    let premature_close = "(foo) ) (bar)";
    let err = SExprParser::tokenize(premature_close);
    assert!(err.is_err(), "Premature close parenthesis must trigger error");

    // 4. Unclosed list at EOF must fail
    let unclosed = "(foo (bar 123)";
    let err_unclosed = SExprParser::tokenize(unclosed);
    assert!(err_unclosed.is_err(), "Unclosed list at EOF must trigger error");
}

// ============================================================================
// TEST 2: KiCad S-Expression Recursive Grammar & Root Structure
// ============================================================================

#[test]
fn test_challenger_kicad_grammar_and_structural_compliance() {
    let mut board = PcbBoard::new(60.0, 35.0);
    board.register_net(1, "GND");
    board.register_net(2, "+5V_USB");
    board.add_footprint(
        resistor_0805("R1", "10k")
            .at_position(Point::new(15.0, 15.0))
            .with_rotation(90.0)
            .with_net_assigned("1", 1)
            .with_net_assigned("2", 2),
    );
    board.add_track(TrackSegment::new(
        Point::new(15.0, 15.0),
        Point::new(25.0, 15.0),
        0.25,
        Layer::FCu,
        1,
    ));
    board.add_via(Via::new(Point::new(25.0, 15.0), 0.8, 0.4, 1));

    let output = export_kicad_pcb(&board);

    // Parse into recursive AST
    let ast = SExprParser::parse_string(&output).expect("KiCad output must be valid S-expression AST");

    assert_eq!(ast.tag(), Some("kicad_pcb"), "Root tag must be 'kicad_pcb'");

    // Verify version & generator
    let version_sub = ast.find_sublist("version").expect("Must contain version list");
    assert_eq!(version_sub[1].as_str(), Some("20240108"));

    let gen_sub = ast.find_sublist("generator").expect("Must contain generator list");
    assert_eq!(gen_sub[1].as_str(), Some("netlist_shatter_viz"));

    // Verify general thickness
    let general_sub = ast.find_sublist("general").expect("Must contain general section");
    let gen_ast = SExpr::List(general_sub.to_vec());
    let thickness_sub = gen_ast.find_sublist("thickness").expect("Must have thickness");
    assert_eq!(thickness_sub[1].as_str(), Some("1.6"));

    // Verify paper
    let paper_sub = ast.find_sublist("paper").expect("Must have paper");
    assert_eq!(paper_sub[1].as_str(), Some("A4"));

    // Verify standard layers are all declared
    let layers_sub = ast.find_sublist("layers").expect("Must declare layers");
    let layers_ast = SExpr::List(layers_sub.to_vec());
    let all_layer_lists = layers_ast.as_list().unwrap();
    let mut found_layers = HashSet::new();
    for l_item in all_layer_lists {
        if let SExpr::List(l_sub) = l_item {
            if l_sub.len() >= 2 {
                if let Some(layer_name) = l_sub[1].as_str() {
                    found_layers.insert(layer_name.to_string());
                }
            }
        }
    }
    assert!(found_layers.contains("F.Cu"), "F.Cu must be declared");
    assert!(found_layers.contains("B.Cu"), "B.Cu must be declared");
    assert!(found_layers.contains("F.SilkS"), "F.SilkS must be declared");
    assert!(found_layers.contains("B.SilkS"), "B.SilkS must be declared");
    assert!(found_layers.contains("F.Mask"), "F.Mask must be declared");
    assert!(found_layers.contains("B.Mask"), "B.Mask must be declared");
    assert!(found_layers.contains("Edge.Cuts"), "Edge.Cuts must be declared");
}

// ============================================================================
// TEST 3: Netlist Cross-Referencing across Pads, Tracks, and Vias
// ============================================================================

#[test]
fn test_challenger_netlist_cross_referencing() {
    let mut board = PcbBoard::new(80.0, 50.0);
    board.register_net(1, "GND");
    board.register_net(2, "+3V3");
    board.register_net(3, "VBUS");
    board.register_net(4, "RESET");
    board.register_net(5, "SPI_SCK");
    board.register_net(6, "SPI_MOSI");

    // Add components with net bindings
    let r1 = resistor_0805("R1", "10k")
        .at_position(Point::new(20.0, 20.0))
        .with_net_assigned("1", 2)
        .with_net_assigned("2", 4);
    board.add_footprint(r1);

    let j1 = header_1x15("J1", "CONN")
        .at_position(Point::new(10.0, 10.0))
        .with_net_assigned("1", 1)
        .with_net_assigned("2", 3)
        .with_net_assigned("3", 5)
        .with_net_assigned("4", 6);
    board.add_footprint(j1);

    // Add tracks referencing nets 1, 2, 5, 6
    board.add_track(TrackSegment::new(Point::new(10.0, 10.0), Point::new(15.0, 10.0), 0.25, Layer::FCu, 1));
    board.add_track(TrackSegment::new(Point::new(15.0, 10.0), Point::new(20.0, 10.0), 0.25, Layer::BCu, 2));
    board.add_track(TrackSegment::new(Point::new(10.0, 15.0), Point::new(15.0, 15.0), 0.25, Layer::FCu, 5));

    // Add vias referencing nets 1 and 2
    board.add_via(Via::new(Point::new(15.0, 10.0), 0.8, 0.4, 1));
    board.add_via(Via::new(Point::new(20.0, 10.0), 0.8, 0.4, 2));

    let output = export_kicad_pcb(&board);
    let ast = SExprParser::parse_string(&output).expect("Must parse valid KiCad S-expr");

    let audit = audit_kicad_netlist(&ast).expect("Netlist audit must complete");

    assert!(
        audit.is_clean(),
        "Netlist audit failed with inconsistencies: {:?}",
        audit
    );

    // Check that declared nets match registered nets exactly
    assert_eq!(audit.declared_nets.get(&0), Some(&"".to_string()));
    assert_eq!(audit.declared_nets.get(&1), Some(&"GND".to_string()));
    assert_eq!(audit.declared_nets.get(&2), Some(&"+3V3".to_string()));
    assert_eq!(audit.declared_nets.get(&3), Some(&"VBUS".to_string()));
    assert_eq!(audit.declared_nets.get(&4), Some(&"RESET".to_string()));
    assert_eq!(audit.declared_nets.get(&5), Some(&"SPI_SCK".to_string()));
    assert_eq!(audit.declared_nets.get(&6), Some(&"SPI_MOSI".to_string()));
}

// ============================================================================
// TEST 4: Undeclared Net Detection Oracle (Adversarial Challenge)
// ============================================================================

#[test]
fn test_challenger_undeclared_net_detection_oracle() {
    // Construct KiCad S-expression directly with an undeclared net 999 on a track and pad
    let bad_kicad_pcb = r#"(kicad_pcb (version 20240108) (generator netlist_shatter_viz)
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "")
  (net 1 "GND")

  (footprint "R_0805" (layer "F.Cu") (at 10 10)
    (property "Reference" "R1" (at 0 -2 0) (layer "F.SilkS"))
    (property "Value" "10k" (at 0 2 0) (layer "F.SilkS"))
    (pad "1" smd rect (at -1 0) (size 1 1) (layers "F.Cu" "F.Mask" "F.SilkS") (net 999 "UNDECLARED"))
  )
  (segment (start 10 10) (end 20 10) (width 0.25) (layer "F.Cu") (net 888))
  (via (at 20 10) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net 777))
)
"#;

    let ast = SExprParser::parse_string(bad_kicad_pcb).expect("Parse bad test AST");
    let audit = audit_kicad_netlist(&ast).expect("Audit completes");

    assert!(!audit.is_clean(), "Oracle must catch undeclared nets");
    assert_eq!(audit.undeclared_pad_nets.len(), 1);
    assert_eq!(audit.undeclared_pad_nets[0].2, 999);
    assert_eq!(audit.undeclared_track_nets.len(), 1);
    assert_eq!(audit.undeclared_track_nets[0].1, 888);
    assert_eq!(audit.undeclared_via_nets.len(), 1);
    assert_eq!(audit.undeclared_via_nets[0].1, 777);
}

// ============================================================================
// TEST 5: Footprint Property & Pad Geometry Verification
// ============================================================================

#[test]
fn test_challenger_footprint_properties_and_pad_geometries() {
    let mut board = PcbBoard::new(120.0, 80.0);
    board.register_net(1, "GND");
    board.register_net(2, "+5V");

    // Add diverse footprints
    let u1 = tqfp32("U1", "ATmega328P")
        .at_position(Point::new(30.0, 40.0))
        .with_rotation(45.0)
        .with_net_assigned("4", 1)
        .with_net_assigned("5", 2);
    board.add_footprint(u1);

    let j1 = usb_c_16pin("J1", "TYPE-C-16P")
        .at_position(Point::new(10.0, 40.0))
        .with_rotation(0.0)
        .with_net_assigned("A1", 1);
    board.add_footprint(j1);

    let q1 = sot223("U2", "AMS1117-3.3")
        .at_position(Point::new(60.0, 40.0))
        .with_rotation(180.0);
    board.add_footprint(q1);

    let sw1 = tactile_switch_3x4("SW1", "RESET_BUTTON")
        .at_position(Point::new(80.0, 40.0))
        .with_rotation(270.0);
    board.add_footprint(sw1);

    let output = export_kicad_pcb(&board);
    let ast = SExprParser::parse_string(&output).expect("Must parse AST cleanly");

    let _root = ast.as_list().unwrap();
    let footprints = ast.find_all_sublists("footprint");
    assert_eq!(footprints.len(), 4, "Must serialize all 4 footprints");

    for fp_sub in footprints {
        let package_name = fp_sub[1].as_str().unwrap();

        // Find (property "Reference" ...) and (property "Value" ...)
        let mut ref_val = None;
        let mut val_val = None;
        let mut at_pos = None;
        let mut pads_found = Vec::new();
        let mut silk_lines_count = 0;

        for item in fp_sub {
            if let SExpr::List(sub) = item {
                let tag = sub.first().and_then(|s| s.as_str()).unwrap_or("");
                match tag {
                    "property" => {
                        if sub.len() >= 3 {
                            if sub[1].as_str() == Some("Reference") {
                                ref_val = sub[2].as_str().map(|s| s.to_string());
                            } else if sub[1].as_str() == Some("Value") {
                                val_val = sub[2].as_str().map(|s| s.to_string());
                            }
                        }
                    }
                    "at" => {
                        let x = sub[1].as_str().unwrap().parse::<f64>().unwrap();
                        let y = sub[2].as_str().unwrap().parse::<f64>().unwrap();
                        let rot = if sub.len() >= 4 {
                            sub[3].as_str().unwrap().parse::<f64>().unwrap()
                        } else {
                            0.0
                        };
                        at_pos = Some((x, y, rot));
                    }
                    "pad" => {
                        let pin = sub[1].as_str().unwrap().to_string();
                        let pad_type = sub[2].as_str().unwrap().to_string();
                        let shape = sub[3].as_str().unwrap().to_string();
                        pads_found.push((pin, pad_type, shape));
                    }
                    "fp_line" => {
                        silk_lines_count += 1;
                    }
                    _ => {}
                }
            }
        }

        let ref_str = ref_val.expect("Footprint must have Reference property");
        let val_str = val_val.expect("Footprint must have Value property");
        let (pos_x, pos_y, rot_deg) = at_pos.expect("Footprint must have (at ...) tag");

        match ref_str.as_str() {
            "U1" => {
                assert_eq!(val_str, "ATmega328P");
                assert_eq!(package_name, "TQFP-32_7x7mm_P0.8mm");
                assert!((pos_x - 30.0).abs() < 1e-3);
                assert!((pos_y - 40.0).abs() < 1e-3);
                assert!((rot_deg + 45.0).abs() < 1e-3);
                assert_eq!(pads_found.len(), 32);
                for (_, pad_type, shape) in &pads_found {
                    assert_eq!(pad_type, "smd");
                    assert_eq!(shape, "rect");
                }
            }
            "J1" => {
                assert_eq!(val_str, "TYPE-C-16P");
                assert_eq!(package_name, "USB_C_Receptacle_16Pin");
                assert!((pos_x - 10.0).abs() < 1e-3);
                assert!((pos_y - 40.0).abs() < 1e-3);
                assert_eq!(pads_found.len(), 16); // 12 SMD + 4 THT tabs
                let smd_count = pads_found.iter().filter(|(_, t, _)| t == "smd").count();
                let tht_count = pads_found.iter().filter(|(_, t, _)| t == "thru_hole").count();
                assert_eq!(smd_count, 12);
                assert_eq!(tht_count, 4);
            }
            "U2" => {
                assert_eq!(val_str, "AMS1117-3.3");
                assert_eq!(package_name, "SOT-223-3_TabPin2");
                assert!((pos_x - 60.0).abs() < 1e-3);
                assert!((pos_y - 40.0).abs() < 1e-3);
                assert!((rot_deg + 180.0).abs() < 1e-3);
                assert_eq!(pads_found.len(), 4);
            }
            "SW1" => {
                assert_eq!(val_str, "RESET_BUTTON");
                assert_eq!(package_name, "SW_Push_SPST_NO_3x4mm");
                assert!((pos_x - 80.0).abs() < 1e-3);
                assert!((pos_y - 40.0).abs() < 1e-3);
                assert!((rot_deg + 270.0).abs() < 1e-3);
                assert_eq!(pads_found.len(), 4);
            }
            other => panic!("Unexpected footprint reference: {}", other),
        }

        assert!(silk_lines_count > 0, "Footprint must have silkscreen lines");
    }
}

// ============================================================================
// TEST 6: Closed Polygon Edge.Cuts Validation
// ============================================================================

#[test]
fn test_challenger_edge_cuts_closed_loop_polygon() {
    let board = PcbBoard::new(75.5, 42.25);
    let output = export_kicad_pcb(&board);
    let ast = SExprParser::parse_string(&output).expect("Must parse valid KiCad S-expr");

    let root = ast.as_list().unwrap();
    let mut edge_cut_lines = Vec::new();

    for item in root {
        if let SExpr::List(sub) = item {
            if sub.len() >= 5 && sub[0].as_str() == Some("gr_line") {
                let mut is_edge_cut = false;
                let mut start_pt = None;
                let mut end_pt = None;

                for f in sub {
                    if let SExpr::List(field) = f {
                        if field.len() >= 2 && field[0].as_str() == Some("layer") {
                            if field[1].as_str() == Some("Edge.Cuts") {
                                is_edge_cut = true;
                            }
                        } else if field.len() >= 3 && field[0].as_str() == Some("start") {
                            let x = field[1].as_str().unwrap().parse::<f64>().unwrap();
                            let y = field[2].as_str().unwrap().parse::<f64>().unwrap();
                            start_pt = Some(Point::new(x, y));
                        } else if field.len() >= 3 && field[0].as_str() == Some("end") {
                            let x = field[1].as_str().unwrap().parse::<f64>().unwrap();
                            let y = field[2].as_str().unwrap().parse::<f64>().unwrap();
                            end_pt = Some(Point::new(x, y));
                        }
                    }
                }

                if is_edge_cut {
                    edge_cut_lines.push((start_pt.unwrap(), end_pt.unwrap()));
                }
            }
        }
    }

    assert_eq!(
        edge_cut_lines.len(),
        4,
        "Rectangular board must emit exactly 4 Edge.Cuts segments"
    );

    // Verify segments form a strictly continuous closed loop
    for i in 0..edge_cut_lines.len() {
        let current_end = edge_cut_lines[i].1;
        let next_start = edge_cut_lines[(i + 1) % edge_cut_lines.len()].0;
        assert!(
            current_end.distance_to(next_start) < 1e-4,
            "Edge.Cuts segment {} end {:?} must touch segment {} start {:?}",
            i,
            current_end,
            (i + 1) % edge_cut_lines.len(),
            next_start
        );
    }
}

// ============================================================================
// TEST 7: Adversarial Special Characters, Quotes, and Symbols
// ============================================================================

#[test]
fn test_challenger_adversarial_special_characters_and_symbols() {
    let mut board = PcbBoard::new(50.0, 50.0);

    // Net names with symbols, quotes, parentheses, slashes, mathematical operators
    board.register_net(10, "+3.3V(CORE)");
    board.register_net(11, "USB_D-");
    board.register_net(12, "NET\"SPECIAL\"");
    board.register_net(13, "BUS[0..7]");
    board.register_net(14, "CLK/16MHz");
    board.register_net(15, "PATH\\SEPARATOR");

    // Component with quotes, parens, and backslashes in reference and value
    let p1 = Pad::new_smd("1", "A(1)", Point::new(-1.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(10);
    let p2 = Pad::new_smd("2", "B\"2\"", Point::new(1.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(12);

    let fp = Footprint::new(
        "R\"1\"",
        "100uF (25V) \"Low-ESR\"",
        "Package:0805 (Metric 2012)",
        Point::new(25.0, 25.0),
        0.0,
        vec![p1, p2],
        vec![(Point::new(-2.0, -1.0), Point::new(2.0, -1.0))],
    );
    board.add_footprint(fp);

    board.add_track(TrackSegment::new(Point::new(10.0, 10.0), Point::new(20.0, 10.0), 0.25, Layer::FCu, 13));
    board.add_via(Via::new(Point::new(20.0, 10.0), 0.8, 0.4, 14));

    let output = export_kicad_pcb(&board);

    // Must parse into AST cleanly without throwing syntax/parenthesis error
    let ast = SExprParser::parse_string(&output).expect("Must parse file with special characters");

    // Audit netlist cross referencing
    let audit = audit_kicad_netlist(&ast).expect("Audit must complete");
    assert!(
        audit.is_clean(),
        "Special characters must preserve netlist cross-referencing: {:?}",
        audit
    );

    // Verify unescaped net names match exactly in declared net table
    assert_eq!(audit.declared_nets.get(&10), Some(&"+3.3V(CORE)".to_string()));
    assert_eq!(audit.declared_nets.get(&11), Some(&"USB_D-".to_string()));
    assert_eq!(audit.declared_nets.get(&12), Some(&"NET\"SPECIAL\"".to_string()));
    assert_eq!(audit.declared_nets.get(&13), Some(&"BUS[0..7]".to_string()));
    assert_eq!(audit.declared_nets.get(&14), Some(&"CLK/16MHz".to_string()));
    assert_eq!(audit.declared_nets.get(&15), Some(&"PATH\\SEPARATOR".to_string()));

    // Verify unescaped component reference and value
    let footprints = ast.find_all_sublists("footprint");
    assert_eq!(footprints.len(), 1);
    let fp_sub = footprints[0];
    let mut found_ref = None;
    let mut found_val = None;
    for elem in fp_sub {
        if let SExpr::List(sub) = elem {
            if sub.len() >= 3 && sub[0].as_str() == Some("property") {
                if sub[1].as_str() == Some("Reference") {
                    found_ref = sub[2].as_str().map(|s| s.to_string());
                } else if sub[1].as_str() == Some("Value") {
                    found_val = sub[2].as_str().map(|s| s.to_string());
                }
            }
        }
    }
    assert_eq!(found_ref, Some("R\"1\"".to_string()));
    assert_eq!(found_val, Some("100uF (25V) \"Low-ESR\"".to_string()));
}

// ============================================================================
// TEST 8: Adversarial Empty Board
// ============================================================================

#[test]
fn test_challenger_adversarial_empty_board() {
    let board = PcbBoard::new(30.0, 20.0);
    let output = export_kicad_pcb(&board);

    // Must parse as valid S-expression with 0 depth at EOF
    let ast = SExprParser::parse_string(&output).expect("Empty board must parse cleanly");
    assert_eq!(ast.tag(), Some("kicad_pcb"));

    let audit = audit_kicad_netlist(&ast).expect("Audit empty board");
    assert!(audit.is_clean());
    assert_eq!(audit.declared_nets.len(), 1);
    assert_eq!(audit.declared_nets.get(&0), Some(&"".to_string()));

    // Edge.Cuts must still be present and closed
    let edge_cuts = ast.find_all_sublists("gr_line");
    assert_eq!(edge_cuts.len(), 4);
}

// ============================================================================
// TEST 9: Adversarial Dense Multi-Net Layout (50 Nets, 30 Footprints)
// ============================================================================

#[test]
fn test_challenger_adversarial_dense_multinet_layout() {
    let mut board = PcbBoard::new(200.0, 150.0);

    // Register 50 distinct nets
    for net_id in 1..=50 {
        board.register_net(net_id, format!("NET_{:03}_SIG", net_id));
    }

    // Add 25 resistors and 5 headers
    for i in 0..25 {
        let n1 = (i * 2 % 50) + 1;
        let n2 = ((i * 2 + 1) % 50) + 1;
        let r = resistor_0805(format!("R{}", i + 1), "1k")
            .at_position(Point::new(20.0 + (i as f64 * 6.0), 30.0 + ((i % 5) as f64 * 10.0)))
            .with_rotation((i as f64 * 45.0) % 360.0)
            .with_net_assigned("1", n1)
            .with_net_assigned("2", n2);
        board.add_footprint(r);
    }

    for h in 0..5 {
        let mut hdr = header_1x15(format!("J{}", h + 1), "HEADER")
            .at_position(Point::new(30.0 + (h as f64 * 30.0), 100.0));
        for p in 1..=15 {
            let net = ((h * 15 + p) % 50 + 1) as u32;
            hdr = hdr.with_net_assigned(&p.to_string(), net);
        }
        board.add_footprint(hdr);
    }

    // Add 100 routed tracks across nets 1..=50
    for t in 0..100 {
        let net = (t % 50 + 1) as u32;
        let layer = if t % 2 == 0 { Layer::FCu } else { Layer::BCu };
        board.add_track(TrackSegment::new(
            Point::new(10.0 + (t as f64 * 1.5), 20.0),
            Point::new(15.0 + (t as f64 * 1.5), 35.0),
            0.25,
            layer,
            net,
        ));
    }

    // Add 50 vias
    for v in 0..50 {
        let net = (v + 1) as u32;
        board.add_via(Via::new(
            Point::new(15.0 + (v as f64 * 3.0), 35.0),
            0.8,
            0.4,
            net,
        ));
    }

    let start_time = std::time::Instant::now();
    let output = export_kicad_pcb(&board);
    let export_dur = start_time.elapsed();
    assert!(
        export_dur.as_millis() < 50,
        "Export must complete in < 50ms (took {}ms)",
        export_dur.as_millis()
    );

    let parse_start = std::time::Instant::now();
    let ast = SExprParser::parse_string(&output).expect("Dense board must parse into AST");
    let parse_dur = parse_start.elapsed();
    assert!(
        parse_dur.as_millis() < 100,
        "Parsing AST must complete in < 100ms (took {}ms)",
        parse_dur.as_millis()
    );

    let audit = audit_kicad_netlist(&ast).expect("Audit dense board");
    assert!(
        audit.is_clean(),
        "Dense layout must have 100% netlist consistency: {:?}",
        audit
    );

    // Verify all 50 nets + net 0 are present
    assert_eq!(audit.declared_nets.len(), 51);
}

// ============================================================================
// TEST 10: Coordinate Precision, Decimal Formatting, and Non-Neg Zero
// ============================================================================

#[test]
fn test_challenger_coordinate_precision_and_zero_normalization() {
    let mut board = PcbBoard::new(10.0, 10.0);

    // Small coordinates, negative near-zero coordinates
    board.add_track(TrackSegment::new(
        Point::new(0.00004, -0.00004),
        Point::new(1.234567, 9.876543),
        0.25,
        Layer::FCu,
        0,
    ));
    board.add_via(Via::new(Point::new(-0.00000004, 0.0), 0.6, 0.3, 0));

    let output = export_kicad_pcb(&board);

    // Reject negative zero tokens, while retaining real negative coordinates.
    assert!(!output.contains("-0 "), "Output must not contain '-0 '");
    assert!(!output.contains("-0)"), "Output must not contain '-0)'");
    assert!(!output.split_whitespace().any(|token| {
        let number = token.trim_matches(['(', ')']);
        number.starts_with('-') && number.parse::<f64>() == Ok(0.0)
    }), "Output must not contain a negative-zero coordinate");
    assert!(!output.contains("NaN"), "Output must not contain 'NaN'");
    assert!(!output.contains("inf"), "Output must not contain 'inf'");

    let ast = SExprParser::parse_string(&output).expect("Must parse coordinate precision board");
    let _root = ast.as_list().unwrap();
    let segments = ast.find_all_sublists("segment");
    assert_eq!(segments.len(), 1);

    // Native nanometre coordinates retain six decimal places.
    let seg = segments[0];
    let mut start_pt = None;
    let mut end_pt = None;
    for f in seg {
        if let SExpr::List(field) = f {
            if field[0].as_str() == Some("start") {
                let x = field[1].as_str().unwrap().parse::<f64>().unwrap();
                let y = field[2].as_str().unwrap().parse::<f64>().unwrap();
                start_pt = Some(Point::new(x, y));
            } else if field[0].as_str() == Some("end") {
                let x = field[1].as_str().unwrap().parse::<f64>().unwrap();
                let y = field[2].as_str().unwrap().parse::<f64>().unwrap();
                end_pt = Some(Point::new(x, y));
            }
        }
    }

    let p_start = start_pt.unwrap();
    let p_end = end_pt.unwrap();
    assert_eq!(p_start.x, 0.00004);
    assert_eq!(p_start.y, -0.00004);
    assert_eq!(p_end.x, 1.234567);
    assert_eq!(p_end.y, 9.876543);
    assert!(output.contains("(at 0 0)"));
}

// ============================================================================
// TEST 11: Obround (Oval) Pads and B.Cu Layer SMD Modeling
// ============================================================================

#[test]
fn test_challenger_obround_pads_and_bcu_layers() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "VCC");

    let p_h_obround = Pad::new_smd(
        "1",
        "HORIZ_OVAL",
        Point::new(-5.0, 0.0),
        PadShape::Obround { width: 3.0, height: 1.5 },
        Layer::FCu,
    ).with_net(1);

    let p_v_obround_bcu = Pad::new_smd(
        "2",
        "VERT_OVAL_BCU",
        Point::new(5.0, 0.0),
        PadShape::Obround { width: 1.5, height: 3.0 },
        Layer::BCu,
    ).with_net(1);

    let p_tht_obround = Pad::new_tht(
        "3",
        "THT_OVAL",
        Point::new(0.0, 5.0),
        PadShape::Obround { width: 3.2, height: 2.0 },
        1.0,
        2.0,
    ).with_net(1);

    let p_bcu_circ = Pad::new_smd(
        "4",
        "CIRC_BCU",
        Point::new(0.0, -5.0),
        PadShape::Circle { diameter: 2.0 },
        Layer::BCu,
    ).with_net(1);

    let fp = Footprint::new(
        "MOD1",
        "CUSTOM_MODULE",
        "MODULE_OVAL",
        Point::new(25.0, 25.0),
        0.0,
        vec![p_h_obround, p_v_obround_bcu, p_tht_obround, p_bcu_circ],
        Vec::new(),
    );
    board.add_footprint(fp);

    let output = export_kicad_pcb(&board);
    let ast = SExprParser::parse_string(&output).expect("Must parse oval and B.Cu pads");

    let pads = ast.find_all_recursive("pad");
    assert_eq!(pads.len(), 4);

    // Pad 1: SMD oval on F.Cu
    let pad1 = pads.iter().find(|p| p[1].as_str() == Some("1")).unwrap();
    assert_eq!(pad1[2].as_str(), Some("smd"));
    assert_eq!(pad1[3].as_str(), Some("oval"));
    let p1_layers = pad1.iter().find(|f| f.tag() == Some("layers")).unwrap();
    assert_eq!(p1_layers.as_list().unwrap()[1].as_str(), Some("F.Cu"));

    // Pad 2: SMD oval on B.Cu
    let pad2 = pads.iter().find(|p| p[1].as_str() == Some("2")).unwrap();
    assert_eq!(pad2[2].as_str(), Some("smd"));
    assert_eq!(pad2[3].as_str(), Some("oval"));
    let p2_layers = pad2.iter().find(|f| f.tag() == Some("layers")).unwrap();
    assert_eq!(p2_layers.as_list().unwrap()[1].as_str(), Some("B.Cu"));

    // Pad 3: THT oval
    let pad3 = pads.iter().find(|p| p[1].as_str() == Some("3")).unwrap();
    assert_eq!(pad3[2].as_str(), Some("thru_hole"));
    assert_eq!(pad3[3].as_str(), Some("oval"));
    let p3_drill = pad3.iter().find(|f| f.tag() == Some("drill")).unwrap();
    assert_eq!(p3_drill.as_list().unwrap()[1].as_str(), Some("1"));

    // Pad 4: SMD circle on B.Cu
    let pad4 = pads.iter().find(|p| p[1].as_str() == Some("4")).unwrap();
    assert_eq!(pad4[2].as_str(), Some("smd"));
    assert_eq!(pad4[3].as_str(), Some("circle"));
    let p4_layers = pad4.iter().find(|f| f.tag() == Some("layers")).unwrap();
    assert_eq!(p4_layers.as_list().unwrap()[1].as_str(), Some("B.Cu"));
}

// ============================================================================
// TEST 12: Unconnected Pads Omit Net Declaration
// ============================================================================

#[test]
fn test_challenger_unconnected_pads_omit_net() {
    let mut board = PcbBoard::new(40.0, 40.0);
    board.register_net(1, "NET_ACTIVE");

    // Pad 1: None
    let p1 = Pad::new_smd("1", "NC1", Point::new(-2.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu);
    // Pad 2: Some(0) (UNCONNECTED_NET)
    let p2 = Pad::new_smd("2", "NC2", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(0);
    // Pad 3: Some(1)
    let p3 = Pad::new_smd("3", "ACTIVE", Point::new(2.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);

    let fp = Footprint::new("U1", "IC", "SOIC-8", Point::new(20.0, 20.0), 0.0, vec![p1, p2, p3], Vec::new());
    board.add_footprint(fp);

    let output = export_kicad_pcb(&board);
    let ast = SExprParser::parse_string(&output).expect("Must parse cleanly");

    let pads = ast.find_all_recursive("pad");
    assert_eq!(pads.len(), 3);

    for pad_sub in pads {
        let pin = pad_sub[1].as_str().unwrap();
        let has_net = pad_sub.iter().any(|f| f.tag() == Some("net"));
        if pin == "1" || pin == "2" {
            assert!(
                !has_net,
                "Unconnected pad pin {} must NOT have a (net ...) tag in KiCad",
                pin
            );
        } else if pin == "3" {
            assert!(has_net, "Connected pad pin 3 must have a (net ...) tag");
        }
    }
}

// ============================================================================
// TEST 13: Deterministic Ascending Net Sorting in Net Table
// ============================================================================

#[test]
fn test_challenger_deterministic_net_sorting() {
    let mut board = PcbBoard::new(50.0, 50.0);

    // Register nets in completely scrambled order
    let scrambled_ids = [99, 42, 7, 105, 1, 15, 3, 50, 2];
    for &id in &scrambled_ids {
        board.register_net(id, format!("NET_{}", id));
    }

    let output = export_kicad_pcb(&board);
    let ast = SExprParser::parse_string(&output).expect("Must parse cleanly");

    let root = ast.as_list().unwrap();
    let mut net_table_ids = Vec::new();

    for item in root {
        if let SExpr::List(sub) = item {
            if sub.len() >= 3 && sub[0].as_str() == Some("net") {
                let id = sub[1].as_str().unwrap().parse::<u32>().unwrap();
                net_table_ids.push(id);
            }
        }
    }

    // Expected order: 0, 1, 2, 3, 7, 15, 42, 50, 99, 105
    let mut expected_sorted = vec![0];
    let mut sorted_scrambled = scrambled_ids.to_vec();
    sorted_scrambled.sort();
    expected_sorted.extend(sorted_scrambled);

    assert_eq!(
        net_table_ids, expected_sorted,
        "Net table must be strictly sorted in ascending order of NetId"
    );
}

// ============================================================================
// TEST 14: Extreme Scale Stress Test (100 Nets, 60 Footprints, 1200 Pads, 600 Tracks)
// ============================================================================

#[test]
fn test_challenger_extreme_density_scale_100_nets() {
    let mut board = PcbBoard::new(300.0, 200.0);

    // 100 distinct nets
    for id in 1..=100 {
        board.register_net(id, format!("HIGH_DENSITY_NET_{:04}", id));
    }

    // 60 footprints: 50 resistors + 10 1x15 headers
    for i in 0..50 {
        let n1 = (i * 2 % 100) + 1;
        let n2 = ((i * 2 + 1) % 100) + 1;
        let r = resistor_0805(format!("R{}", i + 1), "10k")
            .at_position(Point::new(20.0 + (i as f64 * 5.0), 30.0 + ((i % 10) as f64 * 12.0)))
            .with_rotation((i as f64 * 30.0) % 360.0)
            .with_net_assigned("1", n1)
            .with_net_assigned("2", n2);
        board.add_footprint(r);
    }

    for h in 0..10 {
        let mut hdr = header_1x15(format!("J{}", h + 1), "CONN")
            .at_position(Point::new(30.0 + (h as f64 * 25.0), 160.0));
        for p in 1..=15 {
            let net = ((h * 15 + p) % 100 + 1) as u32;
            hdr = hdr.with_net_assigned(&p.to_string(), net);
        }
        board.add_footprint(hdr);
    }

    // 600 tracks across F.Cu and B.Cu
    for t in 0..600 {
        let net = (t % 100 + 1) as u32;
        let layer = if t % 2 == 0 { Layer::FCu } else { Layer::BCu };
        board.add_track(TrackSegment::new(
            Point::new(10.0 + (t as f64 * 0.45), 20.0 + (t as f64 * 0.2)),
            Point::new(15.0 + (t as f64 * 0.45), 40.0 + (t as f64 * 0.2)),
            0.20,
            layer,
            net,
        ));
    }

    // 250 vias
    for v in 0..250 {
        let net = (v % 100 + 1) as u32;
        board.add_via(Via::new(
            Point::new(20.0 + (v as f64 * 1.0), 50.0 + ((v % 20) as f64 * 5.0)),
            0.7,
            0.35,
            net,
        ));
    }

    let t0 = std::time::Instant::now();
    let output = export_kicad_pcb(&board);
    let export_ms = t0.elapsed().as_millis();

    let t1 = std::time::Instant::now();
    let ast = SExprParser::parse_string(&output).expect("Must parse extreme board");
    let parse_ms = t1.elapsed().as_millis();

    let t2 = std::time::Instant::now();
    let audit = audit_kicad_netlist(&ast).expect("Audit must pass");
    let audit_ms = t2.elapsed().as_millis();

    assert!(audit.is_clean(), "Extreme scale board must have clean netlist audit: {:?}", audit);
    assert_eq!(audit.declared_nets.len(), 101); // 100 nets + net 0

    println!(
        "Extreme scale stress test results: export={}ms, parse={}ms, audit={}ms, total_chars={}",
        export_ms, parse_ms, audit_ms, output.len()
    );
    assert!(export_ms < 150, "Export took {}ms, expected < 150ms", export_ms);
    assert!(parse_ms < 250, "Parse took {}ms, expected < 250ms", parse_ms);
}
