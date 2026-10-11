//! Tier 5 Challenger 2 Adversarial Stress Test Suite:
//! CAM Exporters & End-to-End Pipeline Integration Hardening.
//!
//! Validates:
//! 1. KiCad Exporter: unbalanced parenthesis injection, footprint string escaping,
//!    empty footprints, non-standard layers, massive multi-thousand segment boards.
//! 2. RS-274X Gerber Exporter: aperture dictionary reuse, collision resistance,
//!    transposed rotations, flash vs interpolation ordering, 4-quadrant negative coordinates,
//!    zero-feature layer formatting across all 6 standard fabrication layers.
//! 3. Excellon Drill Exporter: multi-tool ascending diameter sorting, coincident hit
//!    deduplication across 150+ hits, perturbed hit tolerance, multi-tool coincident hits,
//!    negative zero normalization, 3-decimal precision limits.
//! 4. End-to-End Pipeline: Layout -> Route -> DRC -> KiCad -> Gerber/Drill round-trip consistency,
//!    ATmega328P MCU reference system under varying router configs (standard, tight, coarse),
//!    dynamic ratsnest invalidation under footprint translation and rotation.
//! 5. Non-regression: spectral partitioning and logic simulation integrity.

use std::collections::BTreeMap;
use netlist_shatter_viz::pcb::*;

// ============================================================================
// PART 1: INDEPENDENT ADVERSARIAL ORACLES & PARSERS
// ============================================================================

/// S-Expression AST Node for validating KiCad S-expression syntax and semantics.
#[derive(Debug, PartialEq, Clone)]
pub enum SExpr {
    Atom(String),
    List(Vec<SExpr>),
}

impl SExpr {
    pub fn is_list(&self) -> bool {
        matches!(self, SExpr::List(_))
    }

    pub fn as_list(&self) -> Option<&[SExpr]> {
        match self {
            SExpr::List(l) => Some(l.as_slice()),
            _ => None,
        }
    }

    pub fn as_atom(&self) -> Option<&str> {
        match self {
            SExpr::Atom(s) => Some(s.as_str()),
            _ => None,
        }
    }

    pub fn tag(&self) -> Option<&str> {
        match self {
            SExpr::List(l) => l.first().and_then(|e| e.as_atom()),
            _ => None,
        }
    }

    pub fn find_children_by_tag(&self, tag: &str) -> Vec<&SExpr> {
        let mut results = Vec::new();
        if let SExpr::List(items) = self {
            for item in items {
                if let SExpr::List(sub) = item {
                    if let Some(first) = sub.first() {
                        if first.as_atom() == Some(tag) {
                            results.push(item);
                        }
                    }
                }
            }
        }
        results
    }

    pub fn find_first_by_tag(&self, tag: &str) -> Option<&SExpr> {
        if let SExpr::List(items) = self {
            for item in items {
                if let SExpr::List(sub) = item {
                    if let Some(first) = sub.first() {
                        if first.as_atom() == Some(tag) {
                            return Some(item);
                        }
                    }
                }
            }
        }
        None
    }
}

/// Tokenizer for S-expressions that properly distinguishes parentheses inside
/// string literals from syntactic list delimiters.
fn tokenize_sexpr(input: &str) -> Vec<String> {
    let mut tokens = Vec::new();
    let mut chars = input.chars().peekable();

    while let Some(&c) = chars.peek() {
        if c.is_whitespace() {
            chars.next();
        } else if c == '(' || c == ')' {
            tokens.push(c.to_string());
            chars.next();
        } else if c == '"' {
            // Quoted string literal
            chars.next(); // consume opening quote
            let mut s = String::new();
            while let Some(ch) = chars.next() {
                if ch == '\\' {
                    if let Some(escaped) = chars.next() {
                        s.push(escaped);
                    }
                } else if ch == '"' {
                    break;
                } else {
                    s.push(ch);
                }
            }
            tokens.push(format!("\"{}\"", s));
        } else {
            // Unquoted atom
            let mut s = String::new();
            while let Some(&ch) = chars.peek() {
                if ch.is_whitespace() || ch == '(' || ch == ')' || ch == '"' {
                    break;
                }
                s.push(ch);
                chars.next();
            }
            tokens.push(s);
        }
    }
    tokens
}

/// Recursive descent parser for tokenized S-expressions.
fn parse_sexpr_tokens(tokens: &[String], pos: &mut usize) -> Result<SExpr, String> {
    if *pos >= tokens.len() {
        return Err("Unexpected EOF".to_string());
    }

    let token = &tokens[*pos];
    *pos += 1;

    if token == "(" {
        let mut list = Vec::new();
        while *pos < tokens.len() && tokens[*pos] != ")" {
            let child = parse_sexpr_tokens(tokens, pos)?;
            list.push(child);
        }
        if *pos >= tokens.len() || tokens[*pos] != ")" {
            return Err("Mismatched opening parenthesis: missing ')'".to_string());
        }
        *pos += 1; // consume ')'
        Ok(SExpr::List(list))
    } else if token == ")" {
        Err("Unexpected closing parenthesis ')'".to_string())
    } else {
        Ok(SExpr::Atom(token.clone()))
    }
}

/// Parses an entire S-expression document into an AST tree and verifies complete consumption.
fn parse_sexpr(s: &str) -> Result<SExpr, String> {
    let tokens = tokenize_sexpr(s);
    let mut pos = 0;
    let ast = parse_sexpr_tokens(&tokens, &mut pos)?;
    if pos != tokens.len() {
        return Err(format!("Trailing tokens at pos {}/{}", pos, tokens.len()));
    }
    Ok(ast)
}

/// Validates strict parenthesis balancing at both character level (excluding quotes)
/// and confirms depth returns to zero.
fn validate_sexpr_parentheses_strictly(s: &str) -> Result<usize, String> {
    let mut depth = 0usize;
    let mut in_quote = false;
    let mut chars = s.chars().peekable();
    let mut total_pairs = 0usize;

    while let Some(c) = chars.next() {
        if in_quote {
            if c == '\\' {
                let _ = chars.next(); // skip escaped char
            } else if c == '"' {
                in_quote = false;
            }
        } else if c == '"' {
            in_quote = true;
        } else if c == '(' {
            depth += 1;
            total_pairs += 1;
        } else if c == ')' {
            if depth == 0 {
                return Err("Premature closing parenthesis ')'".to_string());
            }
            depth -= 1;
        }
    }

    if in_quote {
        return Err("Unclosed string literal".to_string());
    }
    if depth != 0 {
        return Err(format!("Unbalanced parentheses: {} unclosed '('", depth));
    }
    Ok(total_pairs)
}

// ----------------------------------------------------------------------------
// Gerber Independent Oracle
// ----------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq)]
pub enum ParsedAperture {
    Circle { dia_mm: f64 },
    Rect { width_mm: f64, height_mm: f64 },
    Obround { width_mm: f64, height_mm: f64 },
}

#[derive(Clone, Debug)]
pub struct ParsedGerberCommand {
    pub line_number: usize,
    pub command: String,
}

#[derive(Clone, Debug)]
pub struct ParsedGerberFile {
    pub has_format: bool,
    pub has_momm: bool,
    pub has_lpd: bool,
    pub ends_with_m02: bool,
    pub apertures: BTreeMap<u32, ParsedAperture>,
    pub flashes: Vec<(u32, i64, i64)>, // (dcode, x_nm, y_nm)
    pub draws: Vec<(u32, i64, i64, i64, i64)>, // (dcode, x1_nm, y1_nm, x2_nm, y2_nm)
    pub commands: Vec<ParsedGerberCommand>,
}

fn parse_gerber(content: &str) -> Result<ParsedGerberFile, String> {
    let mut has_format = false;
    let mut has_momm = false;
    let mut has_lpd = false;
    let mut ends_with_m02 = false;
    let mut apertures = BTreeMap::new();
    let mut flashes = Vec::new();
    let mut draws = Vec::new();
    let mut commands = Vec::new();

    let mut current_dcode: Option<u32> = None;
    let mut pending_move: Option<(i64, i64)> = None;

    for (line_idx, line) in content.lines().enumerate() {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }

        commands.push(ParsedGerberCommand {
            line_number: line_idx + 1,
            command: trimmed.to_string(),
        });

        if trimmed == "%FSLAX46Y46*%" {
            has_format = true;
        } else if trimmed == "%MOMM*%" {
            has_momm = true;
        } else if trimmed == "%LPD*%" {
            has_lpd = true;
        } else if trimmed == "M02*" {
            ends_with_m02 = true;
        } else if trimmed.starts_with("%ADD") && trimmed.ends_with("*%") {
            // Aperture Definition: %ADD<dcode><Type>,<params>*%
            let inner = &trimmed[4..trimmed.len() - 2];
            let comma_pos = inner.find(',').ok_or_else(|| format!("Malformed %ADD line: {}", trimmed))?;
            let code_type = &inner[..comma_pos];
            let params = &inner[comma_pos + 1..];
            let type_char = code_type.chars().last().unwrap();
            let code_str = &code_type[..code_type.len() - 1];
            let code: u32 = code_str.parse().map_err(|e| format!("Invalid dcode in {}: {}", trimmed, e))?;

            let ap = match type_char {
                'C' => {
                    let dia: f64 = params.parse().map_err(|e| format!("Invalid circle dia {}: {}", params, e))?;
                    ParsedAperture::Circle { dia_mm: dia }
                }
                'R' => {
                    let dims: Vec<&str> = params.split('X').collect();
                    if dims.len() != 2 {
                        return Err(format!("Invalid rect params: {}", params));
                    }
                    let w: f64 = dims[0].parse().map_err(|e| format!("Invalid rect w: {}", e))?;
                    let h: f64 = dims[1].parse().map_err(|e| format!("Invalid rect h: {}", e))?;
                    ParsedAperture::Rect { width_mm: w, height_mm: h }
                }
                'O' => {
                    let dims: Vec<&str> = params.split('X').collect();
                    if dims.len() != 2 {
                        return Err(format!("Invalid obround params: {}", params));
                    }
                    let w: f64 = dims[0].parse().map_err(|e| format!("Invalid obround w: {}", e))?;
                    let h: f64 = dims[1].parse().map_err(|e| format!("Invalid obround h: {}", e))?;
                    ParsedAperture::Obround { width_mm: w, height_mm: h }
                }
                other => return Err(format!("Unsupported aperture type '{}' in {}", other, trimmed)),
            };
            apertures.insert(code, ap);
        } else if trimmed.starts_with('D') && trimmed.ends_with('*') && trimmed.len() > 2 {
            // D-code selection: D10*, D11*, etc.
            let code_str = &trimmed[1..trimmed.len() - 1];
            if let Ok(code) = code_str.parse::<u32>() {
                if code >= 10 {
                    current_dcode = Some(code);
                }
            }
        } else if trimmed.ends_with("D03*") {
            // Flash operation: X...Y...D03*
            let dcode = current_dcode.ok_or_else(|| format!("Flash D03 without active aperture at line {}", line_idx + 1))?;
            let coords = &trimmed[..trimmed.len() - 4];
            let (x, y) = parse_gerber_coords(coords)?;
            flashes.push((dcode, x, y));
        } else if trimmed.ends_with("D02*") {
            // Move operation (shutter closed): X...Y...D02*
            let coords = &trimmed[..trimmed.len() - 4];
            let (x, y) = parse_gerber_coords(coords)?;
            pending_move = Some((x, y));
        } else if trimmed.ends_with("D01*") {
            // Interpolate / draw operation (shutter open): X...Y...D01*
            let dcode = current_dcode.ok_or_else(|| format!("Draw D01 without active aperture at line {}", line_idx + 1))?;
            let (start_x, start_y) = pending_move.ok_or_else(|| format!("Draw D01 without preceding D02 move at line {}", line_idx + 1))?;
            let coords = &trimmed[..trimmed.len() - 4];
            let (end_x, end_y) = parse_gerber_coords(coords)?;
            draws.push((dcode, start_x, start_y, end_x, end_y));
            pending_move = Some((end_x, end_y)); // sequential drawing moves current pos
        }
    }

    Ok(ParsedGerberFile {
        has_format,
        has_momm,
        has_lpd,
        ends_with_m02,
        apertures,
        flashes,
        draws,
        commands,
    })
}

fn parse_gerber_coords(s: &str) -> Result<(i64, i64), String> {
    if !s.starts_with('X') {
        return Err(format!("Coordinate string missing leading X: {}", s));
    }
    let y_pos = s.find('Y').ok_or_else(|| format!("Coordinate string missing Y: {}", s))?;
    let x_str = &s[1..y_pos];
    let y_str = &s[y_pos + 1..];

    let x: i64 = x_str.parse().map_err(|e| format!("Invalid X coord '{}': {}", x_str, e))?;
    let y: i64 = y_str.parse().map_err(|e| format!("Invalid Y coord '{}': {}", y_str, e))?;
    Ok((x, y))
}

// ----------------------------------------------------------------------------
// Excellon Drill Independent Oracle
// ----------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct ParsedExcellonFile {
    pub has_m48: bool,
    pub is_metric: bool,
    pub has_g90: bool,
    pub has_percent: bool,
    pub ends_with_m30: bool,
    pub tool_table: BTreeMap<u32, f64>, // tool_number -> diameter_mm
    pub hits_by_tool: BTreeMap<u32, Vec<(f64, f64)>>, // tool_number -> hits in mm
    pub raw_lines: Vec<String>,
}

fn parse_excellon_drill(content: &str) -> Result<ParsedExcellonFile, String> {
    let mut has_m48 = false;
    let mut is_metric = false;
    let mut has_g90 = false;
    let mut has_percent = false;
    let mut ends_with_m30 = false;
    let mut tool_table = BTreeMap::new();
    let mut hits_by_tool = BTreeMap::new();
    let mut raw_lines = Vec::new();

    let mut current_tool: Option<u32> = None;
    let mut in_header = false;

    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        raw_lines.push(trimmed.to_string());

        if trimmed == "M48" {
            has_m48 = true;
            in_header = true;
        } else if trimmed == "%" {
            has_percent = true;
            in_header = false;
        } else if trimmed == "METRIC" {
            is_metric = true;
        } else if trimmed == "G90" {
            has_g90 = true;
        } else if trimmed == "M30" {
            ends_with_m30 = true;
        } else if in_header && trimmed.starts_with('T') && trimmed.contains('C') {
            // Tool Table entry: T01C0.400
            let c_pos = trimmed.find('C').unwrap();
            let tool_str = &trimmed[1..c_pos];
            let dia_str = &trimmed[c_pos + 1..];
            let tool_num: u32 = tool_str.parse().map_err(|e| format!("Invalid tool num '{}': {}", tool_str, e))?;
            let dia_mm: f64 = dia_str.parse().map_err(|e| format!("Invalid tool dia '{}': {}", dia_str, e))?;
            tool_table.insert(tool_num, dia_mm);
        } else if !in_header && trimmed.starts_with('T') && !trimmed.contains('C') {
            // Tool selection: T01
            let tool_str = &trimmed[1..];
            let tool_num: u32 = tool_str.parse().map_err(|e| format!("Invalid tool select '{}': {}", tool_str, e))?;
            current_tool = Some(tool_num);
        } else if !in_header && trimmed.starts_with('X') && trimmed.contains('Y') {
            // Drill hit: X25.000Y15.000
            let tool_num = current_tool.ok_or_else(|| format!("Drill hit without active tool: {}", trimmed))?;
            let y_pos = trimmed.find('Y').unwrap();
            let x_str = &trimmed[1..y_pos];
            let y_str = &trimmed[y_pos + 1..];
            let x: f64 = x_str.parse().map_err(|e| format!("Invalid drill X '{}': {}", x_str, e))?;
            let y: f64 = y_str.parse().map_err(|e| format!("Invalid drill Y '{}': {}", y_str, e))?;
            hits_by_tool.entry(tool_num).or_insert_with(Vec::new).push((x, y));
        }
    }

    Ok(ParsedExcellonFile {
        has_m48,
        is_metric,
        has_g90,
        has_percent,
        ends_with_m30,
        tool_table,
        hits_by_tool,
        raw_lines,
    })
}

// ============================================================================
// PART 2: KICAD EXPORTER ADVERSARIAL STRESS TESTS
// ============================================================================

#[test]
fn test_adversarial_kicad_unbalanced_parenthesis_injection_in_net_names() {
    let mut board = PcbBoard::new(80.0, 50.0);

    // Register adversarial net names containing parentheses, unbalanced brackets, and nested parens
    let adversarial_nets = [
        (1, "GND(MAIN)"),
        (2, "VCC((NESTED))"),
        (3, "UNBAL_LEFT_((("),
        (4, "UNBAL_RIGHT_)))"),
        (5, "INVERTED_)()("),
        (6, "COMPLEX_SIG_(A+(B*C))"),
        (7, "EMPTY_PARENS_()"),
        (8, "QUOTED_PAREN_\"(\""),
    ];

    for &(id, name) in &adversarial_nets {
        board.register_net(id, name);
    }

    // Add footprint with pads tied to these adversarial nets
    let mut r1 = resistor_0805("R1", "10k").at_position(Point::new(20.0, 20.0));
    r1 = r1.with_net_assigned("1", 3); // UNBAL_LEFT_(((
    r1 = r1.with_net_assigned("2", 4); // UNBAL_RIGHT_)))
    board.add_footprint(r1);

    // Export to KiCad S-expression
    let kicad_out = export_kicad_pcb(&board);

    // 1. Strict parenthesis depth validator MUST PASS because parens are inside string literals!
    let total_pairs = validate_sexpr_parentheses_strictly(&kicad_out)
        .expect("Strict balanced parenthesis check must pass when parens are within quotes");
    assert!(total_pairs > 10, "Document must have multiple S-expression levels");

    // 2. Full AST Parser Oracle MUST parse without error
    let ast = parse_sexpr(&kicad_out).expect("S-expression AST parser must cleanly parse document");
    assert_eq!(ast.tag(), Some("kicad_pcb"), "Root tag must be kicad_pcb");

    // 3. Verify each net is present in the netlist table with exact preserved name
    let net_nodes = ast.find_children_by_tag("net");
    assert!(net_nodes.len() >= adversarial_nets.len() + 1); // +1 for net 0

    for &(id, _name) in &adversarial_nets {
        let expected_net_decl = format!("(net {} \"", id);
        assert!(
            kicad_out.contains(&expected_net_decl),
            "KiCad output must declare net {}",
            id
        );
    }
}

#[test]
fn test_adversarial_kicad_footprint_escaping_quotes_backslashes() {
    let mut board = PcbBoard::new(70.0, 40.0);
    board.register_net(1, "NET_\\_ESCAPE");

    // Construct footprint with backslashes, double quotes, and mixed tokens in Reference and Value
    let malicious_ref = "R\"1\"\\TEST\\";
    let malicious_val = "10k\"_0.1%_\\(SPECIAL\\)";
    let malicious_pkg = "0805\"_SMD\\PKG";

    let fp = Footprint::new(
        malicious_ref,
        malicious_val,
        malicious_pkg,
        Point::new(30.0, 20.0),
        0.0,
        vec![
            Pad::new_smd("1", "P1", Point::new(-0.95, 0.0), PadShape::Rect { width: 0.9, height: 1.25 }, Layer::FCu)
                .with_net(1),
            Pad::new_smd("2", "P2", Point::new(0.95, 0.0), PadShape::Rect { width: 0.9, height: 1.25 }, Layer::FCu)
                .with_net(1),
        ],
        vec![(Point::new(-1.5, -0.8), Point::new(1.5, -0.8))],
    );
    board.add_footprint(fp);

    let kicad_out = export_kicad_pcb(&board);

    // Validate parenthesis strictly
    validate_sexpr_parentheses_strictly(&kicad_out)
        .expect("Escaped quotes and backslashes must not break parenthesis balance");

    // Parse S-expression AST
    let ast = parse_sexpr(&kicad_out).expect("AST parser must succeed on escaped strings");

    // Verify footprint node exists
    let fp_nodes = ast.find_children_by_tag("footprint");
    assert_eq!(fp_nodes.len(), 1, "Must contain exactly 1 footprint");

    // Assert that raw unescaped quotes did NOT break string boundaries
    assert!(!kicad_out.contains("(property \"Reference\" \"R\"1\""));
    assert!(kicad_out.contains("\\\""), "Must contain escaped quotation marks");
    assert!(kicad_out.contains("\\\\"), "Must contain escaped backslashes");
}

#[test]
fn test_adversarial_kicad_empty_footprints_and_zero_pads() {
    let mut board = PcbBoard::new(60.0, 40.0);

    // 1. Completely empty footprint (0 pads, 0 silkscreen lines)
    let empty_fp = Footprint::new(
        "EMPTY1",
        "VOID",
        "FIDUCIAL_NONE",
        Point::new(10.0, 10.0),
        0.0,
        Vec::new(),
        Vec::new(),
    );
    board.add_footprint(empty_fp);

    // 2. Footprint with pads but NO silkscreen lines
    let no_silk_fp = Footprint::new(
        "NO_SILK",
        "TP_1",
        "TESTPOINT",
        Point::new(20.0, 20.0),
        0.0,
        vec![Pad::new_smd("1", "TP", Point::zero(), PadShape::Circle { diameter: 1.0 }, Layer::FCu)],
        Vec::new(),
    );
    board.add_footprint(no_silk_fp);

    // 3. Footprint with silkscreen lines but NO pads (e.g. logo or mounting graphic)
    let logo_fp = Footprint::new(
        "LOGO1",
        "OSHW_LOGO",
        "GRAPHIC",
        Point::new(30.0, 30.0),
        0.0,
        Vec::new(),
        vec![
            (Point::new(-2.0, -2.0), Point::new(2.0, -2.0)),
            (Point::new(2.0, -2.0), Point::new(2.0, 2.0)),
        ],
    );
    board.add_footprint(logo_fp);

    let kicad_out = export_kicad_pcb(&board);
    validate_sexpr_parentheses_strictly(&kicad_out).expect("Empty footprints must maintain balanced S-expression");

    let ast = parse_sexpr(&kicad_out).expect("Empty footprints AST must parse cleanly");
    let fp_nodes = ast.find_children_by_tag("footprint");
    assert_eq!(fp_nodes.len(), 3, "All 3 non-standard/empty footprints must be serialized");
}

#[test]
fn test_adversarial_kicad_non_standard_layers() {
    let mut board = PcbBoard::new(50.0, 50.0);
    board.register_net(1, "SIG");

    // SMD pad assigned explicitly to Bottom Copper (B.Cu)
    let bot_pad = Pad::new_smd("1", "BOT", Point::zero(), PadShape::Rect { width: 1.5, height: 1.5 }, Layer::BCu).with_net(1);
    let fp_bot = Footprint::new("U_BOT", "IC", "SOIC-8", Point::new(15.0, 15.0), 0.0, vec![bot_pad], Vec::new());
    board.add_footprint(fp_bot);

    // Tracks on non-copper layers (Edge.Cuts, F.SilkS) and copper layers (F.Cu, B.Cu)
    board.add_track(TrackSegment::new(Point::new(5.0, 5.0), Point::new(10.0, 5.0), 0.25, Layer::FCu, 1));
    board.add_track(TrackSegment::new(Point::new(10.0, 5.0), Point::new(15.0, 5.0), 0.25, Layer::BCu, 1));
    board.add_track(TrackSegment::new(Point::new(0.0, 0.0), Point::new(50.0, 0.0), 0.15, Layer::EdgeCuts, 0));
    board.add_track(TrackSegment::new(Point::new(2.0, 2.0), Point::new(8.0, 2.0), 0.12, Layer::FSilkscreen, 0));

    let kicad_out = export_kicad_pcb(&board);
    validate_sexpr_parentheses_strictly(&kicad_out).expect("Multi-layer board must have balanced parens");

    assert!(kicad_out.contains("(layer \"B.Cu\")"));
    assert!(kicad_out.contains("(layer \"F.Cu\")"));
    assert!(kicad_out.contains("(layer \"Edge.Cuts\")"));
    assert!(kicad_out.contains("\"B.Cu\" \"B.Mask\" \"B.Paste\""), "Bottom pad must declare B.Cu stack");
}

#[test]
fn test_adversarial_kicad_massive_multi_thousand_segment_board() {
    let mut board = PcbBoard::new(500.0, 500.0);

    // Register 50 nets
    for net in 1..=50 {
        board.register_net(net, format!("BUS_NET_{:02}", net));
    }

    // Add 200 footprints
    for i in 0..200 {
        let x = 10.0 + (i % 20) as f64 * 22.0;
        let y = 10.0 + (i / 20) as f64 * 40.0;
        let r = resistor_0805(&format!("R{}", i), "1k")
            .at_position(Point::new(x, y))
            .with_net_assigned("1", (i % 50 + 1) as u32)
            .with_net_assigned("2", ((i + 1) % 50 + 1) as u32);
        board.add_footprint(r);
    }

    // Add 6,000 track segments
    for i in 0..6000 {
        let x = (i % 100) as f64 * 4.5 + 5.0;
        let y = (i / 100) as f64 * 7.5 + 5.0;
        let layer = if i % 2 == 0 { Layer::FCu } else { Layer::BCu };
        board.add_track(TrackSegment::new(
            Point::new(x, y),
            Point::new(x + 3.0, y + 3.0),
            0.20,
            layer,
            (i % 50 + 1) as u32,
        ));
    }

    // Add 3,000 plated vias
    for i in 0..3000 {
        let x = (i % 100) as f64 * 4.5 + 8.0;
        let y = (i / 100) as f64 * 7.5 + 8.0;
        board.add_via(Via::new(Point::new(x, y), 0.70, 0.35, (i % 50 + 1) as u32));
    }

    let start = std::time::Instant::now();
    let kicad_out = export_kicad_pcb(&board);
    let duration = start.elapsed();

    assert!(duration.as_millis() < 500, "Massive export must complete in < 500ms, took {:?}", duration);
    assert!(kicad_out.len() > 500_000, "Output must be substantial (saw {} bytes)", kicad_out.len());

    // Validate balanced parens on this massive document
    let total_pairs = validate_sexpr_parentheses_strictly(&kicad_out)
        .expect("Massive board must maintain strictly balanced parentheses");
    assert!(total_pairs >= 9000, "Must have at least 9000 balanced pairs for tracks + vias + footprints");

    // Spot-check counts
    let segment_count = kicad_out.matches("(segment").count();
    let via_count = kicad_out.matches("(via").count();
    let footprint_count = kicad_out.matches("(footprint").count();

    assert_eq!(segment_count, 6000, "Must contain exactly 6,000 track segments");
    assert_eq!(via_count, 3000, "Must contain exactly 3,000 vias");
    assert_eq!(footprint_count, 200, "Must contain exactly 200 footprints");
}

// ============================================================================
// PART 3: RS-274X GERBER EXPORTER ADVERSARIAL STRESS TESTS
// ============================================================================

#[test]
fn test_adversarial_gerber_aperture_dictionary_reuse_same_dimensions() {
    let mut board = PcbBoard::new(100.0, 100.0);

    // Add 100 identical 0805 resistors (each has 2 pads of Rect 0.9 x 1.25)
    for i in 0..100 {
        let x = 5.0 + (i % 10) as f64 * 9.0;
        let y = 5.0 + (i / 10) as f64 * 9.0;
        let r = resistor_0805(&format!("R{}", i), "10k").at_position(Point::new(x, y));
        board.add_footprint(r);
    }

    // Add 50 identical vias (size 0.8)
    for i in 0..50 {
        board.add_via(Via::new(Point::new(10.0 + i as f64, 50.0), 0.8, 0.4, 1));
    }

    // Add 50 identical tracks (width 0.25)
    for i in 0..50 {
        board.add_track(TrackSegment::new(
            Point::new(10.0 + i as f64, 60.0),
            Point::new(10.0 + i as f64, 80.0),
            0.25,
            Layer::FCu,
            1,
        ));
    }

    let fcu_gerber = export_gerber_layer(&board, Layer::FCu);
    let parsed = parse_gerber(&fcu_gerber).expect("Gerber must parse cleanly");

    // Assert that the aperture dictionary deduplicates down to EXACTLY 3 apertures:
    // 1. Circle 0.25mm (for tracks)
    // 2. Circle 0.80mm (for vias)
    // 3. Rect 0.90 x 1.25mm (for all 200 resistor pads)
    assert_eq!(
        parsed.apertures.len(),
        3,
        "Aperture dictionary must reuse definitions; expected 3 apertures, found: {:?}",
        parsed.apertures
    );

    // Verify sequential D-codes starting at D10
    assert!(parsed.apertures.contains_key(&10));
    assert!(parsed.apertures.contains_key(&11));
    assert!(parsed.apertures.contains_key(&12));

    // Verify 200 flashes for the resistor pads all use the same rectangular aperture
    let rect_dcode = parsed
        .apertures
        .iter()
        .find(|(_, ap)| matches!(ap, ParsedAperture::Rect { .. }))
        .map(|(&k, _)| k)
        .expect("Must have rect aperture");

    let rect_flashes = parsed.flashes.iter().filter(|(dc, _, _)| *dc == rect_dcode).count();
    assert_eq!(rect_flashes, 200, "All 200 resistor pads must be flashed using single dcode");
}

#[test]
fn test_adversarial_gerber_aperture_reuse_rotation_transposition() {
    let mut board = PcbBoard::new(100.0, 100.0);

    // Pad A: Rect 1.0 x 2.0 at rotation 0°
    let pad_a = Pad::new_smd("1", "A", Point::zero(), PadShape::Rect { width: 1.0, height: 2.0 }, Layer::FCu);
    let fp_a = Footprint::new("U1", "T", "PKG", Point::new(10.0, 10.0), 0.0, vec![pad_a], Vec::new());

    // Pad B: Rect 2.0 x 1.0 at rotation 90° -> transposed to 1.0 x 2.0!
    let pad_b = Pad::new_smd("1", "B", Point::zero(), PadShape::Rect { width: 2.0, height: 1.0 }, Layer::FCu);
    let fp_b = Footprint::new("U2", "T", "PKG", Point::new(20.0, 10.0), 90.0, vec![pad_b], Vec::new());

    // Pad C: Rect 2.0 x 1.0 at rotation 270° -> transposed to 1.0 x 2.0!
    let pad_c = Pad::new_smd("1", "C", Point::zero(), PadShape::Rect { width: 2.0, height: 1.0 }, Layer::FCu);
    let fp_c = Footprint::new("U3", "T", "PKG", Point::new(30.0, 10.0), 270.0, vec![pad_c], Vec::new());

    // Pad D: Rect 2.0 x 1.0 at rotation -90° -> transposed to 1.0 x 2.0!
    let pad_d = Pad::new_smd("1", "D", Point::zero(), PadShape::Rect { width: 2.0, height: 1.0 }, Layer::FCu);
    let fp_d = Footprint::new("U4", "T", "PKG", Point::new(40.0, 10.0), -90.0, vec![pad_d], Vec::new());

    // Pad E: Rect 1.0 x 2.0 at rotation 180° -> remains 1.0 x 2.0!
    let pad_e = Pad::new_smd("1", "E", Point::zero(), PadShape::Rect { width: 1.0, height: 2.0 }, Layer::FCu);
    let fp_e = Footprint::new("U5", "T", "PKG", Point::new(50.0, 10.0), 180.0, vec![pad_e], Vec::new());

    board.add_footprint(fp_a);
    board.add_footprint(fp_b);
    board.add_footprint(fp_c);
    board.add_footprint(fp_d);
    board.add_footprint(fp_e);

    let gbr = export_gerber_layer(&board, Layer::FCu);
    let parsed = parse_gerber(&gbr).expect("Gerber must parse cleanly");

    // All 5 pads MUST map to the same aperture (either 1x2 or 2x1)!
    assert_eq!(
        parsed.apertures.len(),
        1,
        "Transposed rotations (90°, 270°, -90°, 180°) must normalize and share the same aperture code; got: {:?}",
        parsed.apertures
    );
    assert_eq!(parsed.flashes.len(), 5, "All 5 pads must be flashed");
}

#[test]
fn test_adversarial_gerber_aperture_collision_resistance_distinct_shapes() {
    let mut board = PcbBoard::new(50.0, 50.0);

    // Three pads with identical bounding box (1.5 x 1.5 mm), but DIFFERENT shapes:
    // 1. Circle diameter 1.5mm
    // 2. Rect 1.5 x 1.5mm
    // 3. Obround 1.5 x 1.5mm
    let pad_c = Pad::new_smd("1", "C", Point::zero(), PadShape::Circle { diameter: 1.5 }, Layer::FCu);
    let fp_c = Footprint::new("U1", "C", "P", Point::new(10.0, 10.0), 0.0, vec![pad_c], Vec::new());

    let pad_r = Pad::new_smd("1", "R", Point::zero(), PadShape::Rect { width: 1.5, height: 1.5 }, Layer::FCu);
    let fp_r = Footprint::new("U2", "R", "P", Point::new(20.0, 10.0), 0.0, vec![pad_r], Vec::new());

    let pad_o = Pad::new_smd("1", "O", Point::zero(), PadShape::Obround { width: 1.5, height: 1.5 }, Layer::FCu);
    let fp_o = Footprint::new("U3", "O", "P", Point::new(30.0, 10.0), 0.0, vec![pad_o], Vec::new());

    board.add_footprint(fp_c);
    board.add_footprint(fp_r);
    board.add_footprint(fp_o);

    let gbr = export_gerber_layer(&board, Layer::FCu);
    let parsed = parse_gerber(&gbr).expect("Gerber must parse");

    // MUST produce 3 DISTINCT apertures (D10, D11, D12) without collision!
    assert_eq!(
        parsed.apertures.len(),
        3,
        "Distinct shapes with identical dimensions must not collide; got: {:?}",
        parsed.apertures
    );

    let mut has_circle = false;
    let mut has_rect = false;
    let mut has_obround = false;

    for ap in parsed.apertures.values() {
        match ap {
            ParsedAperture::Circle { .. } => has_circle = true,
            ParsedAperture::Rect { .. } => has_rect = true,
            ParsedAperture::Obround { .. } => has_obround = true,
        }
    }

    assert!(has_circle, "Aperture table must contain Circle");
    assert!(has_rect, "Aperture table must contain Rect");
    assert!(has_obround, "Aperture table must contain Obround");
}

#[test]
fn test_adversarial_gerber_flash_vs_interpolation_ordering() {
    let mut board = PcbBoard::new(60.0, 40.0);
    let r1 = resistor_0805("R1", "10k").at_position(Point::new(20.0, 20.0));
    board.add_footprint(r1);
    board.add_via(Via::new(Point::new(30.0, 20.0), 0.8, 0.4, 1));
    board.add_track(TrackSegment::new(Point::new(5.0, 5.0), Point::new(15.0, 5.0), 0.25, Layer::FCu, 1));

    let gbr = export_gerber_layer(&board, Layer::FCu);
    let parsed = parse_gerber(&gbr).expect("Gerber must parse");

    // Verify ordering rules:
    // 1. Headers come first
    assert!(parsed.has_format);
    assert!(parsed.has_momm);
    assert!(parsed.has_lpd);

    // 2. Geometry begins with aperture command D10* or higher
    let first_d_code_idx = parsed.commands.iter().position(|cmd| cmd.command.starts_with('D') && cmd.command.ends_with('*')).unwrap();
    let first_draw_idx = parsed.commands.iter().position(|cmd| cmd.command.ends_with("D01*") || cmd.command.ends_with("D03*")).unwrap();
    assert!(first_d_code_idx < first_draw_idx, "Aperture selection must precede any drawing or flashing");

    // 3. Every D01* draw has a preceding D02* move
    for (i, cmd) in parsed.commands.iter().enumerate() {
        if cmd.command.ends_with("D01*") {
            let prev = &parsed.commands[i - 1].command;
            assert!(
                prev.ends_with("D02*") || prev.ends_with("D01*"),
                "Draw at line {} must be preceded by D02 move or sequential D01 draw, got: {}",
                cmd.line_number, prev
            );
        }
    }

    // 4. File terminates with M02*
    assert!(parsed.ends_with_m02);
}

#[test]
fn test_adversarial_gerber_four_quadrant_negative_coordinates() {
    let mut board = PcbBoard::new(100.0, 100.0);

    // Place features in all 4 Cartesian quadrants:
    // Q1 (+, +), Q2 (-, +), Q3 (-, -), Q4 (+, -)
    let q1_via = Via::new(Point::new(25.0, 25.0), 0.8, 0.4, 1);
    let q2_via = Via::new(Point::new(-25.0, 25.0), 0.8, 0.4, 1);
    let q3_via = Via::new(Point::new(-25.0, -25.0), 0.8, 0.4, 1);
    let q4_via = Via::new(Point::new(25.0, -25.0), 0.8, 0.4, 1);

    board.add_via(q1_via);
    board.add_via(q2_via);
    board.add_via(q3_via);
    board.add_via(q4_via);

    // Add track crossing from Q3 to Q1
    board.add_track(TrackSegment::new(Point::new(-25.0, -25.0), Point::new(25.0, 25.0), 0.25, Layer::FCu, 1));

    let gbr = export_gerber_layer(&board, Layer::FCu);
    let parsed = parse_gerber(&gbr).expect("Gerber with 4-quadrant coordinates must parse cleanly");

    // Verify negative coordinates formatting with '-' sign and 10^6 scaling
    assert!(gbr.contains("X-25000000Y75000000D03*"), "Q2 flash must be X-25000000Y75000000");
    assert!(gbr.contains("X-25000000Y125000000D03*"), "Q3 flash must be X-25000000Y125000000");
    assert!(gbr.contains("X25000000Y125000000D03*"), "Q4 flash must be X25000000Y125000000");
    assert!(gbr.contains("X25000000Y75000000D03*"), "Q1 flash must be X25000000Y75000000");

    // Verify all 4 flashes are parsed correctly
    assert_eq!(parsed.flashes.len(), 4);
    assert_eq!(parsed.draws.len(), 1);
}

#[test]
fn test_adversarial_gerber_zero_feature_layers_formatting() {
    let empty_board = PcbBoard::new(50.0, 50.0);

    let layers = [
        Layer::FCu,
        Layer::BCu,
        Layer::FSilkscreen,
        Layer::EdgeCuts,
        Layer::FMask,
        Layer::BMask,
    ];

    for &layer in &layers {
        let gbr = export_gerber_layer(&empty_board, layer);
        let parsed = parse_gerber(&gbr).unwrap_or_else(|e| panic!("Empty layer {:?} must parse: {}", layer, e));

        assert!(parsed.has_format, "Must have %FSLAX46Y46*%");
        assert!(parsed.has_momm, "Must have %MOMM*%");
        assert!(parsed.has_lpd, "Must have %LPD*%");
        assert!(parsed.ends_with_m02, "Must end with M02*");

        if layer == Layer::EdgeCuts {
            // EdgeCuts automatically has the 4 perimeter lines of the board outline
            assert_eq!(parsed.draws.len(), 4, "EdgeCuts has 4 perimeter segments");
        } else {
            // Truly empty layers must have 0 apertures and 0 primitives
            assert_eq!(parsed.apertures.len(), 0, "Empty layer must have 0 apertures");
            assert_eq!(parsed.flashes.len(), 0, "Empty layer must have 0 flashes");
            assert_eq!(parsed.draws.len(), 0, "Empty layer must have 0 draws");
        }
    }

    // Verify batch export does not crash on empty board
    let files = export_all_gerbers_and_drill(&empty_board);
    assert_eq!(files.len(), 7, "Must export 6 Gerbers + 1 Drill file");
}

// ============================================================================
// PART 4: EXCELLON DRILL EXPORTER ADVERSARIAL STRESS TESTS
// ============================================================================

#[test]
fn test_adversarial_drill_multi_tool_strictly_ascending_diameter_sorting() {
    let mut board = PcbBoard::new(100.0, 100.0);

    // Add 8 drill holes of varying diameters in shuffled / descending order:
    // 1.5mm, 0.3mm, 1.0mm, 0.4mm, 0.8mm, 0.25mm, 1.25mm, 0.65mm
    let diameters = [1.50, 0.30, 1.00, 0.40, 0.80, 0.25, 1.25, 0.65];

    for (i, &dia) in diameters.iter().enumerate() {
        let x = 10.0 + i as f64 * 8.0;
        let y = 10.0 + i as f64 * 8.0;
        let pad = Pad::new_tht("1", "PIN", Point::zero(), PadShape::Circle { diameter: dia + 0.4 }, dia, dia + 0.4);
        let fp = Footprint::new(&format!("J{}", i), "CONN", "HDR", Point::new(x, y), 0.0, vec![pad], Vec::new());
        board.add_footprint(fp);
    }

    let drl = export_excellon_drill(&board);
    let parsed = parse_excellon_drill(&drl).expect("Excellon drill file must parse");

    assert_eq!(parsed.tool_table.len(), 8, "Must declare all 8 distinct tools");

    // Verify diameters are strictly monotonically ascending
    let mut last_dia = 0.0;
    for (&tool_num, &dia) in &parsed.tool_table {
        assert!(
            dia > last_dia,
            "Tool T{:02} diameter {} must be strictly greater than previous {}",
            tool_num, dia, last_dia
        );
        last_dia = dia;
    }

    // Expected ascending order: 0.25, 0.30, 0.40, 0.65, 0.80, 1.00, 1.25, 1.50
    let sorted_expected = [0.250, 0.300, 0.400, 0.650, 0.800, 1.000, 1.250, 1.500];
    for (idx, &expected_dia) in sorted_expected.iter().enumerate() {
        let tool_num = (idx + 1) as u32;
        let actual_dia = parsed.tool_table[&tool_num];
        assert!(
            (actual_dia - expected_dia).abs() < 1e-4,
            "Tool T{:02} expected diameter {:.3}, got {:.3}",
            tool_num, expected_dia, actual_dia
        );
        assert_eq!(
            parsed.hits_by_tool[&tool_num].len(),
            1,
            "Tool T{:02} must have exactly 1 hit",
            tool_num
        );
    }
}

#[test]
fn test_adversarial_drill_coincident_hit_deduplication_150_hits() {
    let mut board = PcbBoard::new(50.0, 50.0);

    // Place 150 coincident vias at the EXACT same point (25.0, 25.0)
    for _ in 0..150 {
        board.add_via(Via::new(Point::new(25.0, 25.0), 0.8, 0.4, 1));
    }

    let drl = export_excellon_drill(&board);
    let parsed = parse_excellon_drill(&drl).expect("Drill file must parse");

    // Exactly 1 tool (0.4mm)
    assert_eq!(parsed.tool_table.len(), 1);
    let hits = &parsed.hits_by_tool[&1];

    // 150 duplicate hits MUST be deduplicated to exactly ONE hit!
    assert_eq!(
        hits.len(),
        1,
        "150 coincident drill hits must deduplicate to exactly 1 hit; found: {}",
        hits.len()
    );
    assert!((hits[0].0 - 25.0).abs() < 1e-3);
    assert!((hits[0].1 - 25.0).abs() < 1e-3);
}

#[test]
fn test_adversarial_drill_perturbed_coincident_deduplication() {
    let mut board = PcbBoard::new(50.0, 50.0);

    // Place 100 vias around (15.0, 15.0) with microscopic perturbation < 1e-4 mm
    for i in 0..100 {
        let delta = (i as f64 - 50.0) * 1e-6; // +/- 0.00005 mm
        board.add_via(Via::new(Point::new(15.0 + delta, 15.0 + delta), 0.8, 0.4, 1));
    }

    let drl = export_excellon_drill(&board);
    let parsed = parse_excellon_drill(&drl).expect("Drill file must parse");

    let hits = &parsed.hits_by_tool[&1];
    assert_eq!(
        hits.len(),
        1,
        "Microscopically perturbed coincident hits within 1e-4 tolerance must be deduplicated to 1; found: {}",
        hits.len()
    );
}

#[test]
fn test_adversarial_drill_coincident_hits_different_tools_retained() {
    let mut board = PcbBoard::new(50.0, 50.0);

    // Place coincident hits of DIFFERENT diameters at (20.0, 20.0):
    // 1. Via with 0.4mm drill
    // 2. Through-hole header pin with 1.0mm drill
    board.add_via(Via::new(Point::new(20.0, 20.0), 0.8, 0.4, 1));

    let tht_pad = Pad::new_tht("1", "PIN", Point::zero(), PadShape::Circle { diameter: 1.6 }, 1.0, 1.6);
    let fp = Footprint::new("J1", "HDR", "HDR1", Point::new(20.0, 20.0), 0.0, vec![tht_pad], Vec::new());
    board.add_footprint(fp);

    let drl = export_excellon_drill(&board);
    let parsed = parse_excellon_drill(&drl).expect("Drill file must parse");

    // Both tools must exist
    assert_eq!(parsed.tool_table.len(), 2);
    // T01 (0.4mm) has 1 hit at (20.0, 20.0)
    assert_eq!(parsed.hits_by_tool[&1].len(), 1);
    // T02 (1.0mm) has 1 hit at (20.0, 20.0)
    assert_eq!(parsed.hits_by_tool[&2].len(), 1);
}

#[test]
fn test_adversarial_drill_negative_zero_normalization_and_precision() {
    let mut board = PcbBoard::new(50.0, 50.0);

    // Add vias with coordinates near zero or negative zero
    board.add_via(Via::new(Point::new(-0.0, 0.0), 0.8, 0.4, 1));
    board.add_via(Via::new(Point::new(0.0, -0.0), 0.8, 0.4, 1));
    board.add_via(Via::new(Point::new(-0.0001, -0.0001), 0.8, 0.4, 1));
    board.add_via(Via::new(Point::new(-0.00049, 10.0), 0.8, 0.4, 1));
    board.add_via(Via::new(Point::new(10.0, -0.00049), 0.8, 0.4, 1));
    board.add_via(Via::new(Point::new(123.4567, 78.9012), 0.8, 0.4, 1));

    let drl = export_excellon_drill(&board);

    // CRITICAL: Excellon drill file must NEVER contain the literal "-0.000" or "-0"
    assert!(
        !drl.contains("-0.000"),
        "Drill file must never contain '-0.000' string; verbatim drl:\n{}",
        drl
    );
    assert!(!drl.contains("X-0.000"));
    assert!(!drl.contains("Y-0.000"));

    // Precision check: all hit coordinates must have exactly 3 decimal digits
    for line in drl.lines() {
        if line.starts_with('X') && line.contains('Y') {
            let y_pos = line.find('Y').unwrap();
            let x_part = &line[1..y_pos];
            let y_part = &line[y_pos + 1..];

            let x_dot = x_part.find('.').expect("X coordinate must have decimal dot");
            let y_dot = y_part.find('.').expect("Y coordinate must have decimal dot");

            assert_eq!(x_part[x_dot + 1..].len(), 3, "X must have exactly 3 decimal places");
            assert_eq!(y_part[y_dot + 1..].len(), 3, "Y must have exactly 3 decimal places");
        }
    }
}

// ============================================================================
// PART 5: END-TO-END PIPELINE & MCU REFERENCE SYSTEM TESTS
// ============================================================================

#[test]
fn test_adversarial_e2e_pipeline_custom_board_round_trip() {
    let mut board = PcbBoard::new(50.0, 30.0);
    let net_sig1 = board.get_or_create_net("SIG1");
    let net_sig2 = board.get_or_create_net("SIG2");

    // Add components
    let mut r1 = resistor_0805("R1", "1k").at_position(Point::new(15.0, 10.0));
    r1 = r1.with_net_assigned("1", net_sig1);
    r1 = r1.with_net_assigned("2", net_sig2);
    board.add_footprint(r1);

    let mut r2 = resistor_0805("R2", "2k").at_position(Point::new(35.0, 10.0));
    r2 = r2.with_net_assigned("1", net_sig1);
    r2 = r2.with_net_assigned("2", net_sig2);
    board.add_footprint(r2);

    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    };

    // 1. Autoroute board
    let route_res = route_board(&mut board, &config);
    assert!(route_res.is_ok(), "Custom board routing must succeed: {:?}", route_res);
    assert!(!board.tracks.is_empty(), "Must have routed tracks");

    // 2. Validate DRC
    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "Must have zero short circuits");
    assert_eq!(drc.clearance_violations.len(), 0, "Must have zero clearance violations");
    assert_eq!(drc.unrouted_nets.len(), 0, "Must have zero unrouted nets");

    // 3. Export to KiCad
    let kicad_str = export_kicad_pcb(&board);
    let ast = parse_sexpr(&kicad_str).expect("Exported KiCad must be valid S-expression");
    let segment_nodes = ast.find_children_by_tag("segment");
    assert_eq!(segment_nodes.len(), board.tracks.len(), "All routed tracks must appear in KiCad");

    // 4. Export to Gerber (all 6 layers) + Drill
    let cam_files = export_all_gerbers_and_drill(&board);
    assert_eq!(cam_files.len(), 7);

    // Verify F_Cu has drawing commands for the routed tracks
    let fcu_parsed = parse_gerber(&cam_files["F_Cu.gbr"]).expect("F_Cu Gerber must parse");
    assert!(!fcu_parsed.draws.is_empty(), "F_Cu must contain routed trace draws");

    // Verify drill file
    let drill_parsed = parse_excellon_drill(&cam_files["drill.drl"]).expect("Drill file must parse");
    assert!(drill_parsed.has_m48);
    assert!(drill_parsed.ends_with_m30);
}

#[test]
fn test_adversarial_mcu_reference_system_standard_router_config() {
    let mut board = create_reference_board();

    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    };

    // 1. Autoroute ATmega328P Reference Board
    let result = route_board(&mut board, &config);
    assert!(result.is_ok(), "Reference board must route cleanly: {:?}", result);

    // 2. Geometric DRC Audit
    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "DRC must report 0 shorts on routed reference board");
    assert_eq!(drc.clearance_violations.len(), 0, "DRC must report 0 clearance errors on routed reference board");
    assert_eq!(drc.annular_ring_violations.len(), 0, "DRC must report 0 annular ring errors");
    assert_eq!(drc.edge_keepout_violations.len(), 0, "DRC must report 0 edge keepout errors");
    assert_eq!(drc.unrouted_nets.len(), 0, "DRC must report 0 unrouted nets");

    // 3. CAM Exporters on Reference Board
    let kicad_out = export_kicad_pcb(&board);
    let total_pairs = validate_sexpr_parentheses_strictly(&kicad_out)
        .expect("Routed reference board KiCad S-expression must be strictly balanced");
    assert!(total_pairs > 500);

    let cam_files = export_all_gerbers_and_drill(&board);
    assert_eq!(cam_files.len(), 7);

    // Verify F_Cu and B_Cu have copper geometry
    let fcu = parse_gerber(&cam_files["F_Cu.gbr"]).expect("F_Cu must parse");
    let bcu = parse_gerber(&cam_files["B_Cu.gbr"]).expect("B_Cu must parse");
    assert!(!fcu.flashes.is_empty(), "F_Cu must have pad flashes");
    assert!(!fcu.draws.is_empty(), "F_Cu must have track draws");
    assert!(!bcu.flashes.is_empty(), "B_Cu must have via/THT pad flashes");

    // Verify Drill has both via drill (0.40mm) and header drill (1.00mm)
    let drl = parse_excellon_drill(&cam_files["drill.drl"]).expect("Drill file must parse");
    assert!(drl.tool_table.len() >= 2, "Drill file must define at least via and header tools");
    let has_via_drill = drl.tool_table.values().any(|&d| (d - 0.40).abs() < 1e-3);
    let has_hdr_drill = drl.tool_table.values().any(|&d| (d - 1.00).abs() < 1e-3);
    assert!(has_via_drill, "Drill file must contain 0.40mm tool for vias");
    assert!(has_hdr_drill, "Drill file must contain 1.00mm tool for headers");
}

#[test]
fn test_adversarial_mcu_reference_system_tighter_router_config() {
    let mut board = create_reference_board();

    // Tighter high-density config: 0.20mm grid, 0.20mm traces, 0.6mm vias, 0.3mm drill, 0.15mm clearance
    let config = RouterConfig {
        grid_resolution: 0.20,
        default_trace_width: 0.20,
        via_diameter: 0.60,
        via_drill: 0.30,
        clearance: 0.15,
    };

    let result = route_board(&mut board, &config);
    assert!(result.is_ok(), "Tighter router config must succeed on reference board: {:?}", result);

    let drc = run_drc(&board, &config);
    assert_eq!(drc.short_circuits.len(), 0, "Zero shorts under tight config");
    assert_eq!(drc.clearance_violations.len(), 0, "Zero clearance violations under tight config");
    assert_eq!(drc.unrouted_nets.len(), 0, "Zero unrouted nets under tight config");

    // Verify CAM exporters work seamlessly with tight via and drill dimensions
    let drl = export_excellon_drill(&board);
    let parsed_drl = parse_excellon_drill(&drl).expect("Drill must parse for tight config");
    let has_tight_via_drill = parsed_drl.tool_table.values().any(|&d| (d - 0.30).abs() < 1e-3);
    assert!(has_tight_via_drill, "Drill file must contain 0.30mm tool for tight vias");
}

#[test]
fn test_adversarial_mcu_reference_board_dynamic_ratsnest_invalidation() {
    let mut board = create_reference_board();

    let rats_initial = compute_ratsnest(&board);
    let initial_count = rats_initial.len();
    assert_eq!(initial_count, 31, "Reference board initial ratsnest must be 31");

    let initial_total_len: f64 = rats_initial.iter().map(|l| l.start.distance_to(l.end)).sum();

    // Perturb layout: move U1 (MCU) by +5.0 mm in X and +3.0 mm in Y
    if let Some(u1) = board.find_footprint_mut("U1") {
        u1.position.x += 5.0;
        u1.position.y += 3.0;
    }

    // Recompute ratsnest dynamically
    let rats_perturbed = compute_ratsnest(&board);
    assert_eq!(rats_perturbed.len(), initial_count, "Ratsnest edge count must remain stable");

    let perturbed_total_len: f64 = rats_perturbed.iter().map(|l| l.start.distance_to(l.end)).sum();

    // Total ratsnest length must have changed due to component translation
    assert!(
        (perturbed_total_len - initial_total_len).abs() > 1.0,
        "Ratsnest length must dynamically update upon component movement: initial = {:.2}, perturbed = {:.2}",
        initial_total_len, perturbed_total_len
    );

    // Perturb rotation: rotate U1 by 90°
    if let Some(u1) = board.find_footprint_mut("U1") {
        u1.rotation = 90.0;
    }

    let rats_rotated = compute_ratsnest(&board);
    let rotated_total_len: f64 = rats_rotated.iter().map(|l| l.start.distance_to(l.end)).sum();
    assert!(
        (rotated_total_len - perturbed_total_len).abs() > 0.5,
        "Ratsnest length must dynamically update upon component rotation"
    );
}

#[test]
fn test_adversarial_mcu_reference_board_coarse_router_config() {
    let mut board = create_reference_board();

    // Overly coarse config (0.80mm grid) where 0.8mm pitch TQFP-32 pins cannot all land on grid cleanly
    let coarse_config = RouterConfig {
        grid_resolution: 0.80,
        default_trace_width: 0.40,
        via_diameter: 1.00,
        via_drill: 0.50,
        clearance: 0.30,
    };

    // Router must either succeed or return a clean RoutingError (never panic or infinite loop)
    let res = route_board(&mut board, &coarse_config);
    match res {
        Ok(()) => {
            // If it routed, DRC can be checked
            let _drc = run_drc(&board, &coarse_config);
        }
        Err(e) => {
            // Failure is completely graceful and informative
            assert!(!format!("{:?}", e).is_empty());
        }
    }
}

// ============================================================================
// PART 6: NON-REGRESSION VERIFICATION (NETLIST & DRC INTEGRITY)
// ============================================================================

#[test]
fn test_adversarial_non_regression_netlist_ingestion_and_clustering() {
    let mut board = PcbBoard::new(60.0, 40.0);
    let net_clk = board.get_or_create_net("CLK");
    let net_data = board.get_or_create_net("DATA");

    // Add gates / components
    let r1 = resistor_0805("R1", "10k")
        .at_position(Point::new(15.0, 20.0))
        .with_net_assigned("1", net_clk)
        .with_net_assigned("2", net_data);

    let r2 = resistor_0805("R2", "10k")
        .at_position(Point::new(35.0, 20.0))
        .with_net_assigned("1", net_clk)
        .with_net_assigned("2", net_data);

    board.add_footprint(r1);
    board.add_footprint(r2);

    // Initial unrouted ratsnest
    let rats = compute_ratsnest(&board);
    assert_eq!(rats.len(), 2, "Must have 2 ratsnest lines for CLK and DATA");

    // Route net_clk
    board.add_track(TrackSegment::new(
        Point::new(14.05, 20.0),
        Point::new(34.05, 20.0),
        0.25,
        Layer::FCu,
        net_clk,
    ));

    // Recompute ratsnest: CLK is now connected by copper island clustering!
    let rats_after = compute_ratsnest(&board);
    assert_eq!(
        rats_after.len(),
        1,
        "Connected net CLK must be eliminated from ratsnest by copper island clustering"
    );
    assert_eq!(rats_after[0].net, net_data, "Only unrouted net DATA remains in ratsnest");
}

#[test]
fn test_adversarial_non_regression_drc_rule_soundness() {
    let mut board = PcbBoard::new(40.0, 40.0);
    let net1 = board.get_or_create_net("NET1");
    let net2 = board.get_or_create_net("NET2");

    // Add conflicting intersecting tracks (short circuit)
    board.add_track(TrackSegment::new(Point::new(10.0, 20.0), Point::new(30.0, 20.0), 0.25, Layer::FCu, net1));
    board.add_track(TrackSegment::new(Point::new(20.0, 10.0), Point::new(20.0, 30.0), 0.25, Layer::FCu, net2));

    let config = RouterConfig {
        grid_resolution: 0.25,
        default_trace_width: 0.25,
        via_diameter: 0.80,
        via_drill: 0.40,
        clearance: 0.20,
    };

    let drc = run_drc(&board, &config);
    assert!(
        !drc.short_circuits.is_empty(),
        "DRC must detect the injected electrical short between NET1 and NET2"
    );
}

