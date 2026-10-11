//! Universal Specctra DSN & SES Engine for KiCad, Altium, and Eagle.
//!
//! Provides:
//! - Full S-expression lexer and AST parser with quoted string and comment support
//! - Robust parser for Specctra Design (`.dsn`) files exported by KiCad, Altium, and Eagle
//! - Multi-unit support (`um`, `mm`, `mil`, `inch`, `cm`) and arbitrary resolution scaling
//! - Physical structure extraction: board boundary (`rect`, `path`, `polygon`), layers, keepouts
//! - Footprint library and placement extraction: SMD/THT pads (`rect`, `circle`, `oval`, `path`)
//! - Netlist ingestion: logical nets, multi-pin connections (`U1-1`, `"U1"-"1"`, etc.)
//! - Design rule extraction: trace width, clearance constraints
//! - Automated routing via internal multi-layer grid A* with Rip-Up & Reroute (RRR)
//! - Specctra Session (`.ses`) export and transactional external route import
//! - Bidirectional roundtrip `.dsn` exporter from internal `PcbBoard`

use crate::pcb::core::{
    Footprint, Layer, NetId, Pad, PadShape, PadType, PcbBoard, Point, Rect,
    UNCONNECTED_NET,
};
use crate::pcb::router::{route_board_with_report, RouterConfig, RoutingReport};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::fmt;
use std::fs;
use std::path::Path;

// ============================================================================
// Errors & Status
// ============================================================================

/// Taxonomy of errors encountered during Specctra DSN parsing, routing, or SES export.
#[derive(Clone, Debug, PartialEq)]
pub enum SpecctraError {
    LexError {
        line: usize,
        col: usize,
        details: String,
    },
    ParseError {
        line: usize,
        col: usize,
        details: String,
    },
    MissingSection {
        section: String,
    },
    InvalidUnit {
        unit: String,
    },
    InvalidLayer {
        layer: String,
    },
    RoutingFailed {
        details: String,
    },
    IoError {
        path: String,
        details: String,
    },
    DrcViolations {
        details: String,
    },
}

impl fmt::Display for SpecctraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::LexError { line, col, details } => {
                write!(f, "Specctra lexer error at line {}:{}: {}", line, col, details)
            }
            Self::ParseError { line, col, details } => {
                write!(f, "Specctra parser error at line {}:{}: {}", line, col, details)
            }
            Self::MissingSection { section } => {
                write!(f, "Required Specctra section missing: '{}'", section)
            }
            Self::InvalidUnit { unit } => {
                write!(f, "Invalid Specctra unit specifier: '{}'", unit)
            }
            Self::InvalidLayer { layer } => {
                write!(f, "Invalid Specctra copper layer: '{}'", layer)
            }
            Self::RoutingFailed { details } => {
                write!(f, "Specctra routing failed: {}", details)
            }
            Self::IoError { path, details } => {
                write!(f, "Specctra I/O error at '{}': {}", path, details)
            }
            Self::DrcViolations { details } => {
                write!(f, "Specctra DRC violations detected: {}", details)
            }
        }
    }
}

impl std::error::Error for SpecctraError {}

// ============================================================================
// S-Expression AST & Parser
// ============================================================================

/// S-expression node: either an atomic string/number or a parenthesized list.
#[derive(Clone, Debug, PartialEq)]
pub enum SExpr {
    Atom(String),
    List(Vec<SExpr>),
}

impl SExpr {
    #[inline]
    pub fn as_atom(&self) -> Option<&str> {
        match self {
            SExpr::Atom(s) => Some(s.as_str()),
            SExpr::List(_) => None,
        }
    }

    #[inline]
    pub fn as_list(&self) -> Option<&[SExpr]> {
        match self {
            SExpr::List(list) => Some(list.as_slice()),
            SExpr::Atom(_) => None,
        }
    }

    /// If this node is a list, returns the name of the tag (first atom).
    /// If an atom, returns the atom itself.
    pub fn name(&self) -> Option<&str> {
        match self {
            SExpr::Atom(s) => Some(s.as_str()),
            SExpr::List(list) => list.first().and_then(|item| item.as_atom()),
        }
    }

    /// Case-insensitive tag matching on the first atom of a list.
    pub fn matches_tag(&self, tag: &str) -> bool {
        self.name().map_or(false, |n| n.eq_ignore_ascii_case(tag))
    }

    /// Finds the first child list whose tag matches `tag` (case-insensitive).
    pub fn child(&self, tag: &str) -> Option<&SExpr> {
        match self {
            SExpr::List(list) => list.iter().find(|item| item.matches_tag(tag)),
            SExpr::Atom(_) => None,
        }
    }

    /// Iterator over all child lists whose tag matches `tag` (case-insensitive).
    pub fn children<'a>(&'a self, tag: &'a str) -> impl Iterator<Item = &'a SExpr> {
        let items: &[SExpr] = match self {
            SExpr::List(list) => list.as_slice(),
            SExpr::Atom(_) => &[],
        };
        items.iter().filter(move |item| item.matches_tag(tag))
    }

    /// Gets the n-th element of a list.
    #[inline]
    pub fn nth(&self, n: usize) -> Option<&SExpr> {
        match self {
            SExpr::List(list) => list.get(n),
            SExpr::Atom(_) => None,
        }
    }

    /// Gets the n-th element as an atom string.
    #[inline]
    pub fn nth_atom(&self, n: usize) -> Option<&str> {
        self.nth(n).and_then(|e| e.as_atom())
    }

    /// Gets the n-th element parsed as a floating-point number.
    #[inline]
    pub fn nth_f64(&self, n: usize) -> Option<f64> {
        self.nth_atom(n).and_then(|s| s.parse::<f64>().ok())
    }

    /// Formats this S-expression back to clean text.
    pub fn to_string_pretty(&self, indent: usize) -> String {
        let mut out = String::new();
        self.format_indent(&mut out, indent);
        out
    }

    fn format_indent(&self, out: &mut String, indent: usize) {
        match self {
            SExpr::Atom(s) => {
                if s.contains(|c: char| c.is_whitespace() || c == '(' || c == ')') || s.is_empty() {
                    out.push('"');
                    out.push_str(&s.replace('\\', "\\\\").replace('"', "\\\""));
                    out.push('"');
                } else {
                    out.push_str(s);
                }
            }
            SExpr::List(list) => {
                out.push('(');
                let is_short = list.len() <= 6 && list.iter().all(|e| matches!(e, SExpr::Atom(_)));
                for (i, item) in list.iter().enumerate() {
                    if i > 0 {
                        out.push(' ');
                    }
                    if !is_short && i > 0 && matches!(item, SExpr::List(_)) {
                        out.push('\n');
                        out.push_str(&"  ".repeat(indent + 1));
                        item.format_indent(out, indent + 1);
                    } else {
                        item.format_indent(out, indent + 1);
                    }
                }
                out.push(')');
            }
        }
    }
}

/// Token generated by the S-expression lexer.
#[derive(Clone, Debug, PartialEq)]
enum Token {
    LParen(usize, usize),
    RParen(usize, usize),
    Atom(String, usize, usize),
}

/// Tokenizes Specctra S-expressions with support for quoted strings and comments.
fn tokenize_sexpr(input: &str) -> Result<Vec<Token>, SpecctraError> {
    let mut tokens = Vec::with_capacity(input.len() / 8);
    let chars: Vec<char> = input.chars().collect();
    let n = chars.len();
    let mut i = 0;
    let mut line = 1;
    let mut col = 1;

    while i < n {
        let ch = chars[i];

        // Track line and column numbers
        if ch == '\n' {
            line += 1;
            col = 1;
            i += 1;
            continue;
        }

        // Whitespace
        if ch.is_whitespace() {
            col += 1;
            i += 1;
            continue;
        }

        // Comments: ';' or '#' up to newline
        if ch == ';' || ch == '#' {
            while i < n && chars[i] != '\n' {
                i += 1;
            }
            continue;
        }

        // Left Parenthesis
        if ch == '(' {
            tokens.push(Token::LParen(line, col));
            col += 1;
            i += 1;
            continue;
        }

        // Right Parenthesis
        if ch == ')' {
            tokens.push(Token::RParen(line, col));
            col += 1;
            i += 1;
            continue;
        }

        // Quoted string (double or single quotes)
        if ch == '"' || ch == '\'' {
            // Check if this is the single character delimiter definition in `(string_quote ")` or `(string_quote ')`
            let is_string_quote_def = tokens.last().map_or(false, |tok| match tok {
                Token::Atom(name, _, _) => name.eq_ignore_ascii_case("string_quote"),
                _ => false,
            });

            if is_string_quote_def {
                tokens.push(Token::Atom(ch.to_string(), line, col));
                i += 1;
                col += 1;
                continue;
            }

            let quote_char = ch;
            let start_col = col;
            let start_line = line;
            let mut s = String::new();
            i += 1;
            col += 1;

            while i < n {
                let c = chars[i];
                if c == '\n' {
                    line += 1;
                    col = 1;
                } else {
                    col += 1;
                }

                if c == quote_char {
                    // Check for doubled quote escape ("" or '')
                    if i + 1 < n && chars[i + 1] == quote_char {
                        s.push(quote_char);
                        i += 2;
                        col += 1;
                        continue;
                    }
                    i += 1;
                    break;
                } else if c == '\\' && i + 1 < n {
                    let next = chars[i + 1];
                    match next {
                        'n' => s.push('\n'),
                        't' => s.push('\t'),
                        'r' => s.push('\r'),
                        '\\' => s.push('\\'),
                        '"' => s.push('"'),
                        '\'' => s.push('\''),
                        _ => {
                            s.push('\\');
                            s.push(next);
                        }
                    }
                    i += 2;
                    col += 1;
                } else {
                    s.push(c);
                    i += 1;
                }
            }

            tokens.push(Token::Atom(s, start_line, start_col));
            continue;
        }

        // Unquoted atom / symbol / number
        let start_col = col;
        let start_line = line;
        let mut s = String::new();

        while i < n {
            let c = chars[i];
            if c.is_whitespace() || c == '(' || c == ')' || c == ';' || c == '#' || c == '"' || c == '\'' {
                break;
            }
            s.push(c);
            i += 1;
            col += 1;
        }

        if !s.is_empty() {
            tokens.push(Token::Atom(s, start_line, start_col));
        }
    }

    Ok(tokens)
}

/// Parses a stream of tokens into a vector of top-level S-expressions.
pub fn parse_sexpr(input: &str) -> Result<Vec<SExpr>, SpecctraError> {
    let tokens = tokenize_sexpr(input)?;
    let mut stack: Vec<Vec<SExpr>> = Vec::new();
    let mut root: Vec<SExpr> = Vec::new();

    for token in tokens {
        match token {
            Token::LParen(_, _) => {
                stack.push(Vec::new());
            }
            Token::RParen(line, col) => {
                if let Some(list) = stack.pop() {
                    let expr = SExpr::List(list);
                    if let Some(parent) = stack.last_mut() {
                        parent.push(expr);
                    } else {
                        root.push(expr);
                    }
                } else {
                    return Err(SpecctraError::ParseError {
                        line,
                        col,
                        details: "Unmatched closing parenthesis ')'".to_string(),
                    });
                }
            }
            Token::Atom(val, _, _) => {
                let atom = SExpr::Atom(val);
                if let Some(current) = stack.last_mut() {
                    current.push(atom);
                } else {
                    root.push(atom);
                }
            }
        }
    }

    if !stack.is_empty() {
        return Err(SpecctraError::ParseError {
            line: 0,
            col: 0,
            details: format!("Unclosed parenthesis: {} open lists remain", stack.len()),
        });
    }

    Ok(root)
}

// ============================================================================
// Specctra Unit & Resolution Engine
// ============================================================================

/// Engineering units supported by Specctra DSN & SES standards.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpecctraUnit {
    Mm,
    Um,
    Mil,
    Inch,
    Cm,
}

impl SpecctraUnit {
    /// Distance multiplier: 1 unit = `factor` millimeters (mm).
    pub fn to_mm_factor(&self) -> f64 {
        match self {
            Self::Mm => 1.0,
            Self::Um => 0.001,
            Self::Mil => 0.0254,
            Self::Inch => 25.4,
            Self::Cm => 10.0,
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Mm => "mm",
            Self::Um => "um",
            Self::Mil => "mil",
            Self::Inch => "inch",
            Self::Cm => "cm",
        }
    }

    pub fn from_str(s: &str) -> Option<Self> {
        let s_lower = s.to_ascii_lowercase();
        match s_lower.as_str() {
            "mm" | "millimeter" | "millimeters" => Some(Self::Mm),
            "um" | "micron" | "microns" | "micrometer" | "micrometers" => Some(Self::Um),
            "mil" | "mils" | "thou" => Some(Self::Mil),
            "inch" | "inches" | "in" => Some(Self::Inch),
            "cm" | "centimeter" | "centimeters" => Some(Self::Cm),
            _ => None,
        }
    }
}

/// Unit resolution descriptor: specifies base unit and optional divisor/multiplier.
///
/// In Specctra: `(resolution <unit> <factor>)` indicates that `<factor>` file units = 1 `<unit>`.
/// If no resolution factor is specified (or factor is 1), 1 file unit = 1 `<unit>`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpecctraResolution {
    pub unit: SpecctraUnit,
    pub factor: f64,
}

impl Default for SpecctraResolution {
    fn default() -> Self {
        Self {
            unit: SpecctraUnit::Mm,
            factor: 1.0,
        }
    }
}

impl SpecctraResolution {
    pub fn new(unit: SpecctraUnit, factor: f64) -> Self {
        let f = if factor > 0.0 { factor } else { 1.0 };
        Self { unit, factor: f }
    }

    /// DSN coordinates use the declared length unit; resolution controls precision.
    pub fn to_mm(&self, val: f64) -> f64 { val * self.unit.to_mm_factor() }
    pub fn from_mm(&self, mm: f64) -> f64 { mm / self.unit.to_mm_factor() }
    /// SES coordinates, unlike DSN, are integer resolution units.
    pub fn ses_resolution_factor(&self) -> f64 { self.factor }
    pub fn ses_from_mm(&self, mm: f64) -> f64 { self.from_mm(mm) * self.factor }

}

// ============================================================================
// Specctra Intermediate Representation (IR)
// ============================================================================

/// Copper layer definition in DSN structure.
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraLayer {
    pub name: String,
    pub is_signal: bool,
    pub index: usize,
}

/// Geometric shape definition within a padstack.
#[derive(Clone, Debug, PartialEq)]
pub enum SpecctraPadShape {
    Rect { layer: String, width: f64, height: f64 },
    Circle { layer: String, diameter: f64 },
    Oval { layer: String, width: f64, height: f64 },
    Path { layer: String, aperture: f64, p1: Point, p2: Point },
}

/// Padstack definition in the library.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SpecctraPadstack {
    pub id: String,
    pub shapes: Vec<SpecctraPadShape>,
    pub drill: Option<f64>, // Drill hole diameter in mm, if parsed
}

// Integer nanometre names survive Freerouting's decimal-suffix removal.
fn via_padstack_id(diameter: f64, drill: f64) -> String {
    format!("Via[0-1]_{:.0}:{:.0}_nm", (diameter * 1_000_000.0).ceil(), (drill * 1_000_000.0).ceil())
}

fn via_nm_dimensions(id: &str) -> Option<(f64, f64)> {
    let dimensions = id.strip_prefix("Via[0-1]_")?.strip_suffix("_nm")?;
    let (diameter, drill) = dimensions.split_once(':')?;
    if ![diameter, drill].iter().all(|s| !s.is_empty() && s.bytes().all(|c| c.is_ascii_digit())) { return None; }
    let diameter = diameter.parse::<u64>().ok()? as f64 / 1_000_000.0;
    let drill = drill.parse::<u64>().ok()? as f64 / 1_000_000.0;
    if drill > 0.0 && diameter > drill { Some((diameter, drill)) } else { None }
}

impl SpecctraPadstack {
    /// Determines whether this padstack represents a through-hole or SMD pad,
    /// and calculates primary outer dimensions in mm.
    pub fn to_pad_model(&self, top_layer_name: &str, bot_layer_name: &str) -> (PadShape, PadType) {
        let mut max_w = 0.0f64;
        let mut max_h = 0.0f64;
        let mut is_circle = false;
        let mut is_oval = false;
        let mut touches_top = false;
        let mut touches_bot = false;
        let mut has_all_layers = false;

        for shape in &self.shapes {
            match shape {
                SpecctraPadShape::Circle { layer, diameter } => {
                    max_w = max_w.max(*diameter);
                    max_h = max_h.max(*diameter);
                    is_circle = true;
                    if layer.eq_ignore_ascii_case("all") || layer.eq_ignore_ascii_case("signal") {
                        has_all_layers = true;
                    } else if layer.eq_ignore_ascii_case(top_layer_name) {
                        touches_top = true;
                    } else if layer.eq_ignore_ascii_case(bot_layer_name) {
                        touches_bot = true;
                    }
                }
                SpecctraPadShape::Rect { layer, width, height } => {
                    max_w = max_w.max(*width);
                    max_h = max_h.max(*height);
                    if layer.eq_ignore_ascii_case("all") || layer.eq_ignore_ascii_case("signal") {
                        has_all_layers = true;
                    } else if layer.eq_ignore_ascii_case(top_layer_name) {
                        touches_top = true;
                    } else if layer.eq_ignore_ascii_case(bot_layer_name) {
                        touches_bot = true;
                    }
                }
                SpecctraPadShape::Oval { layer, width, height } => {
                    max_w = max_w.max(*width);
                    max_h = max_h.max(*height);
                    is_oval = true;
                    if layer.eq_ignore_ascii_case("all") || layer.eq_ignore_ascii_case("signal") {
                        has_all_layers = true;
                    } else if layer.eq_ignore_ascii_case(top_layer_name) {
                        touches_top = true;
                    } else if layer.eq_ignore_ascii_case(bot_layer_name) {
                        touches_bot = true;
                    }
                }
                SpecctraPadShape::Path { layer, aperture, p1, p2 } => {
                    let len = p1.distance_to(*p2);
                    let w = aperture.max(len);
                    let h = *aperture;
                    max_w = max_w.max(w);
                    max_h = max_h.max(h);
                    is_oval = true;
                    if layer.eq_ignore_ascii_case("all") || layer.eq_ignore_ascii_case("signal") {
                        has_all_layers = true;
                    } else if layer.eq_ignore_ascii_case(top_layer_name) {
                        touches_top = true;
                    } else if layer.eq_ignore_ascii_case(bot_layer_name) {
                        touches_bot = true;
                    }
                }
            }
        }

        // Clamp minimum pad dimensions to 0.1 mm to avoid zero-size degenerate pads
        if max_w < 0.1 {
            max_w = 0.8;
        }
        if max_h < 0.1 {
            max_h = 0.8;
        }

        let pad_shape = if is_circle || (max_w - max_h).abs() < 1e-6 {
            PadShape::Circle { diameter: max_w }
        } else if is_oval {
            PadShape::Obround { width: max_w, height: max_h }
        } else {
            PadShape::Rect { width: max_w, height: max_h }
        };

        // Determine if through-hole or SMD
        let is_via = self.id.to_ascii_lowercase().starts_with("via");
        // Through-hole detection: shapes on all/both layers, explicit drill, or via naming.
        // NOTE: Do NOT use `contains('_')` — it matches virtually every KiCad padstack name
        // (e.g. "Rect_Pads_1.70x1.30mm") and would falsely classify SMD pads as THT.
        // The colon check catches drill-encoding patterns like "Pad_100:60" that aren't vias.
        let is_tht = has_all_layers || (touches_top && touches_bot) || self.drill.is_some() || is_via;

        let pad_type = if is_tht {
            let drill = self.drill.unwrap_or_else(|| {
                // If drill is encoded in name (e.g. Via[0-1]_800:400_um, Via_24_12_mil, Via:0.8_0.4mm, or Pad_100:60)
                let extract_drill = || -> Option<f64> {
                    if self.id.to_ascii_lowercase().contains("_nm") { return via_nm_dimensions(&self.id).map(|(_, drill)| drill); }
                    // Try colon separator
                    if let Some(colon_idx) = self.id.find(':') {
                        let sub = &self.id[colon_idx + 1..];
                        let end_idx = sub.find(|c: char| !c.is_numeric() && c != '.').unwrap_or(sub.len());
                        if let Ok(val) = sub[..end_idx].parse::<f64>() {
                            let id_lower = self.id.to_ascii_lowercase();
                            return Some(if id_lower.contains("um") {
                                val * 0.001
                            } else if id_lower.contains("mil") {
                                val * 0.0254
                            } else {
                                val
                            });
                        }
                    }
                    // Try underscore separator e.g. Via_24_12_mil
                    let parts: Vec<&str> = self.id.split('_').collect();
                    if parts.len() >= 3 {
                        if let Ok(drill_val) = parts[2].parse::<f64>() {
                            let is_mil = self.id.to_ascii_lowercase().contains("mil");
                            let is_um = self.id.to_ascii_lowercase().contains("um");
                            return Some(if is_um {
                                drill_val * 0.001
                            } else if is_mil {
                                drill_val * 0.0254
                            } else {
                                drill_val
                            });
                        }
                    }
                    None
                };
                extract_drill().unwrap_or(max_w * 0.5)
            });
            let max_drill = (max_w * 0.85).max(0.05);
            let min_drill = 0.05f64.min(max_drill * 0.5);
            let drill_clamped = drill.clamp(min_drill, max_drill);
            PadType::ThroughHole {
                drill: drill_clamped,
                outer_diameter: max_w,
            }
        } else {
            let layer = if touches_bot && !touches_top {
                Layer::BCu
            } else {
                Layer::FCu
            };
            PadType::Smd { layer }
        };

        (pad_shape, pad_type)
    }
}

/// Pin declaration inside an Image footprint.
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraPin {
    pub padstack_id: String,
    pub pin_id: String,
    pub relative_pos: Point, // in millimeters (mm)
    pub rotation: f64,       // in degrees
}

/// Image (footprint definition) in the library.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SpecctraImage {
    pub id: String,
    pub pins: Vec<SpecctraPin>,
    pub silkscreen_lines: Vec<(Point, Point)>, // in mm
}

/// Placed component instance on the board.
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraPlacedComponent {
    pub image_id: String,
    pub component_id: String,
    pub position: Point, // in mm
    pub rotation: f64,   // degrees
    pub side: String,    // "front" or "back"
    pub part_number: Option<String>,
}

/// Logical net connectivity declaration.
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraNet {
    pub name: String,
    pub pins: Vec<(String, String)>, // (component_id, pin_id)
    pub width: Option<f64>,          // in mm
    pub clearance: Option<f64>,      // in mm
}

/// Keepout area (obstacle).
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraKeepout {
    pub id: String,
    pub shape: KeepoutShape,
}

#[derive(Clone, Debug, PartialEq)]
pub enum KeepoutShape {
    Rect(Rect),
    Polygon(Vec<Point>),
}

/// Complete parsed Specctra DSN document model.
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraDsn {
    pub pcb_name: String,
    pub resolution: SpecctraResolution,
    pub layers: Vec<SpecctraLayer>,
    pub boundary_points: Vec<Point>, // in mm
    pub bounds: Rect,                // in mm
    pub origin_offset: Point,        // in mm (DSN coord system offset to board [0,0])
    pub keepouts: Vec<SpecctraKeepout>,
    pub padstacks: HashMap<String, SpecctraPadstack>,
    pub images: HashMap<String, SpecctraImage>,
    pub components: Vec<SpecctraPlacedComponent>,
    pub nets: Vec<SpecctraNet>,
    pub default_width: Option<f64>,     // mm
    pub default_clearance: Option<f64>, // mm
    pub via_padstack_ids: Vec<String>,
    /// True if the DSN contained a `(wiring ...)` section with pre-routed traces.
    /// These are not currently imported — the router starts from scratch.
    pub has_prerouted_wiring: bool,
}

impl Default for SpecctraDsn {
    fn default() -> Self {
        Self {
            pcb_name: "board".to_string(),
            resolution: SpecctraResolution::default(),
            layers: Vec::new(),
            boundary_points: Vec::new(),
            bounds: Rect::new(Point::zero(), Point::new(50.0, 50.0)),
            origin_offset: Point::zero(),
            keepouts: Vec::new(),
            padstacks: HashMap::new(),
            images: HashMap::new(),
            components: Vec::new(),
            nets: Vec::new(),
            default_width: None,
            default_clearance: None,
            via_padstack_ids: Vec::new(),
            has_prerouted_wiring: false,
        }
    }
}

// ============================================================================
// Specctra DSN Parser Implementation
// ============================================================================

/// Parses raw Specctra DSN text into the structured `SpecctraDsn` model.
pub fn parse_specctra_dsn(content: &str) -> Result<SpecctraDsn, SpecctraError> {
    let trees = parse_sexpr(content)?;

    let root = trees
        .iter()
        .find(|t| t.matches_tag("pcb"))
        .ok_or_else(|| SpecctraError::MissingSection {
            section: "pcb".to_string(),
        })?;

    let pcb_name = root.nth_atom(1).unwrap_or("board").to_string();

    // 1. Resolution & Unit
    let mut resolution = SpecctraResolution::default();

    if let Some(unit_node) = root.child("unit") {
        if let Some(u_str) = unit_node.nth_atom(1) {
            if let Some(u) = SpecctraUnit::from_str(u_str) {
                resolution.unit = u;
            }
        }
    }

    if let Some(res_node) = root.child("resolution") {
        if let Some(u_str) = res_node.nth_atom(1) {
            if let Some(u) = SpecctraUnit::from_str(u_str) {
                resolution.unit = u;
            }
        }
        if let Some(val) = res_node.nth_f64(2) {
            resolution.factor = val;
        }
    }

    let mut dsn = SpecctraDsn {
        pcb_name,
        resolution,
        ..Default::default()
    };

    // 2. Structure (Layers, Boundary, Keepouts, Rules)
    if let Some(struct_node) = root.child("structure") {
        parse_structure(&mut dsn, struct_node)?;
    }

    // 3. Library (Padstacks & Images)
    if let Some(lib_node) = root.child("library") {
        parse_library(&mut dsn, lib_node)?;
    }

    // 4. Placement (Components)
    if let Some(place_node) = root.child("placement") {
        parse_placement(&mut dsn, place_node)?;
    }

    // 5. Network (Nets & Pins)
    if let Some(net_node) = root.child("network") {
        parse_network(&mut dsn, net_node)?;
    }

    // 6. Detect pre-routed wiring (not currently imported — full re-route)
    if root.child("wiring").is_some() {
        dsn.has_prerouted_wiring = true;
    }

    // Calculate Board Bounds and Normalization Origin Offset
    if !dsn.boundary_points.is_empty() {
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;

        for p in &dsn.boundary_points {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }

        dsn.bounds = Rect::new(Point::new(min_x, min_y), Point::new(max_x, max_y));
        dsn.origin_offset = Point::new(min_x, min_y);
    } else {
        // Fallback: Compute bounding box from placed components
        let mut min_x: f64 = 0.0;
        let mut max_x: f64 = 50.0;
        let mut min_y: f64 = 0.0;
        let mut max_y: f64 = 50.0;

        for comp in &dsn.components {
            min_x = min_x.min(comp.position.x - 5.0);
            max_x = max_x.max(comp.position.x + 5.0);
            min_y = min_y.min(comp.position.y - 5.0);
            max_y = max_y.max(comp.position.y + 5.0);
        }

        dsn.bounds = Rect::new(Point::new(min_x, min_y), Point::new(max_x, max_y));
        dsn.origin_offset = Point::new(min_x, min_y);
    }

    for id in dsn.via_padstack_ids.iter().chain(dsn.padstacks.keys()) {
        if id.starts_with("Via[") && id.to_ascii_lowercase().contains("_nm") && via_nm_dimensions(id).is_none() {
            return Err(SpecctraError::ParseError { line: 0, col: 0, details: "Invalid integer nanometre via padstack ID".into() });
        }
    }
    Ok(dsn)
}

/// Helper parsing the `(structure ...)` block.
fn parse_structure(dsn: &mut SpecctraDsn, node: &SExpr) -> Result<(), SpecctraError> {
    let res = dsn.resolution;

    // Unit / resolution inside structure if overridden
    if let Some(unit_node) = node.child("unit") {
        if let Some(u_str) = unit_node.nth_atom(1) {
            if let Some(u) = SpecctraUnit::from_str(u_str) {
                dsn.resolution.unit = u;
            }
        }
    }
    if let Some(res_node) = node.child("resolution") {
        if let Some(u_str) = res_node.nth_atom(1) {
            if let Some(u) = SpecctraUnit::from_str(u_str) {
                dsn.resolution.unit = u;
            }
        }
        if let Some(val) = res_node.nth_f64(2) {
            dsn.resolution.factor = val;
        }
    }

    // Layers
    let mut layer_idx = 0;
    for layer_node in node.children("layer") {
        if let Some(name) = layer_node.nth_atom(1) {
            let is_signal = layer_node.child("type").map_or(true, |t| {
                t.nth_atom(1).map_or(true, |typ| typ.eq_ignore_ascii_case("signal") || typ.eq_ignore_ascii_case("mixed"))
            });
            dsn.layers.push(SpecctraLayer {
                name: name.to_string(),
                is_signal,
                index: layer_idx,
            });
            layer_idx += 1;
        }
    }

    // Boundary
    if let Some(boundary_node) = node.child("boundary") {
        // Can be (rect pcb x1 y1 x2 y2) or (path pcb width x1 y1 x2 y2 ...)
        for rect_node in boundary_node.children("rect") {
            // (rect pcb x1 y1 x2 y2)
            if let (Some(x1), Some(y1), Some(x2), Some(y2)) = (
                rect_node.nth_f64(2),
                rect_node.nth_f64(3),
                rect_node.nth_f64(4),
                rect_node.nth_f64(5),
            ) {
                let p1 = Point::new(res.to_mm(x1), res.to_mm(y1));
                let p2 = Point::new(res.to_mm(x2), res.to_mm(y2));
                dsn.boundary_points.push(Point::new(p1.x.min(p2.x), p1.y.min(p2.y)));
                dsn.boundary_points.push(Point::new(p1.x.max(p2.x), p1.y.min(p2.y)));
                dsn.boundary_points.push(Point::new(p1.x.max(p2.x), p1.y.max(p2.y)));
                dsn.boundary_points.push(Point::new(p1.x.min(p2.x), p1.y.max(p2.y)));
            }
        }
        for path_node in boundary_node.children("path") {
            // (path pcb width x1 y1 x2 y2 ...)
            if let Some(list) = path_node.as_list() {
                let mut coords = Vec::new();
                for elem in list.iter().skip(3) {
                    if let Some(val) = elem.as_atom().and_then(|s| s.parse::<f64>().ok()) {
                        coords.push(res.to_mm(val));
                    }
                }
                for chunk in coords.chunks_exact(2) {
                    dsn.boundary_points.push(Point::new(chunk[0], chunk[1]));
                }
            }
        }
        for poly_node in boundary_node.children("polygon") {
            if let Some(list) = poly_node.as_list() {
                let skip_count = if list.len() >= 4 && (list.len() - 2) % 2 == 0 { 2 } else { 3 };
                let mut coords = Vec::new();
                for elem in list.iter().skip(skip_count) {
                    if let Some(val) = elem.as_atom().and_then(|s| s.parse::<f64>().ok()) {
                        coords.push(res.to_mm(val));
                    }
                }
                for chunk in coords.chunks_exact(2) {
                    dsn.boundary_points.push(Point::new(chunk[0], chunk[1]));
                }
            }
        }
    }

    // Keepouts (keepout, place_keepout, via_keepout, wire_keepout)
    let ko_tags = ["keepout", "place_keepout", "via_keepout", "wire_keepout"];
    for tag in &ko_tags {
        for ko_node in node.children(tag) {
            let name = ko_node.nth_atom(1).unwrap_or("keepout").to_string();
            for rect_node in ko_node.children("rect") {
                if let (Some(x1), Some(y1), Some(x2), Some(y2)) = (
                    rect_node.nth_f64(2),
                    rect_node.nth_f64(3),
                    rect_node.nth_f64(4),
                    rect_node.nth_f64(5),
                ) {
                    let r = Rect::new(
                        Point::new(res.to_mm(x1.min(x2)), res.to_mm(y1.min(y2))),
                        Point::new(res.to_mm(x1.max(x2)), res.to_mm(y1.max(y2))),
                    );
                    dsn.keepouts.push(SpecctraKeepout {
                        id: name.clone(),
                        shape: KeepoutShape::Rect(r),
                    });
                }
            }
            for poly_node in ko_node.children("polygon") {
                if let Some(list) = poly_node.as_list() {
                    let skip_count = if list.len() >= 4 && (list.len() - 2) % 2 == 0 { 2 } else { 3 };
                    let mut pts = Vec::new();
                    let mut coords = Vec::new();
                    for elem in list.iter().skip(skip_count) {
                        if let Some(val) = elem.as_atom().and_then(|s| s.parse::<f64>().ok()) {
                            coords.push(res.to_mm(val));
                        }
                    }
                    for chunk in coords.chunks_exact(2) {
                        pts.push(Point::new(chunk[0], chunk[1]));
                    }
                    if !pts.is_empty() {
                        dsn.keepouts.push(SpecctraKeepout {
                            id: name.clone(),
                            shape: KeepoutShape::Polygon(pts),
                        });
                    }
                }
            }
            for path_node in ko_node.children("path") {
                if let Some(list) = path_node.as_list() {
                    let mut pts = Vec::new();
                    let mut coords = Vec::new();
                    for elem in list.iter().skip(3) {
                        if let Some(val) = elem.as_atom().and_then(|s| s.parse::<f64>().ok()) {
                            coords.push(res.to_mm(val));
                        }
                    }
                    for chunk in coords.chunks_exact(2) {
                        pts.push(Point::new(chunk[0], chunk[1]));
                    }
                    if !pts.is_empty() {
                        dsn.keepouts.push(SpecctraKeepout {
                            id: name.clone(),
                            shape: KeepoutShape::Polygon(pts),
                        });
                    }
                }
            }
            for circle_node in ko_node.children("circle") {
                let diam = circle_node.nth_f64(2).or_else(|| circle_node.nth_f64(1)).unwrap_or(1.0);
                let cx = circle_node.nth_f64(3).unwrap_or(0.0);
                let cy = circle_node.nth_f64(4).unwrap_or(0.0);
                let diam_mm = res.to_mm(diam);
                let r = Rect::from_center_size(Point::new(res.to_mm(cx), res.to_mm(cy)), diam_mm, diam_mm);
                dsn.keepouts.push(SpecctraKeepout {
                    id: name.clone(),
                    shape: KeepoutShape::Rect(r),
                });
            }
        }
    }

    // Rules
    for rule_node in node.children("rule") {
        if let Some(width_node) = rule_node.child("width") {
            if let Some(w) = width_node.nth_f64(1) {
                dsn.default_width = Some(res.to_mm(w));
            }
        } else if rule_node.nth_atom(1).map_or(false, |a| a.eq_ignore_ascii_case("width")) {
            if let Some(w) = rule_node.nth_f64(2) {
                dsn.default_width = Some(res.to_mm(w));
            }
        }

        if let Some(clear_node) = rule_node.child("clearance") {
            if let Some(c) = clear_node.nth_f64(1) {
                dsn.default_clearance = Some(res.to_mm(c));
            }
        } else if rule_node.nth_atom(1).map_or(false, |a| a.eq_ignore_ascii_case("clearance")) {
            if let Some(c) = rule_node.nth_f64(2) {
                dsn.default_clearance = Some(res.to_mm(c));
            }
        }
    }

    // Allowed Vias
    for via_node in node.children("via") {
        if let Some(list) = via_node.as_list() {
            for item in list.iter().skip(1) {
                if let Some(id) = item.as_atom() {
                    dsn.via_padstack_ids.push(id.to_string());
                }
            }
        }
    }

    Ok(())
}

/// Helper parsing the `(library ...)` block.
fn parse_library(dsn: &mut SpecctraDsn, node: &SExpr) -> Result<(), SpecctraError> {
    let res = dsn.resolution;

    // 1. Padstacks
    for ps_node in node.children("padstack") {
        if let Some(id) = ps_node.nth_atom(1) {
            let mut padstack = SpecctraPadstack {
                id: id.to_string(),
                ..Default::default()
            };

            // Parse drill / hole diameter if specified
            if let Some(drill_node) = ps_node.child("drill").or_else(|| ps_node.child("hole")) {
                if let Some(d) = drill_node.nth_f64(1) {
                    padstack.drill = Some(res.to_mm(d));
                }
            }

            for shape_node in ps_node.children("shape") {
                if let Some(shape_list) = shape_node.as_list() {
                    for item in shape_list.iter().skip(1) {
                        if let Some(shape_item) = item.as_list() {
                            let shape_tag = shape_item.first().and_then(|e| e.as_atom()).unwrap_or("");
                            let (layer, dim_offset) = if let Some(atom1) = shape_item.get(1).and_then(|e| e.as_atom()) {
                                if atom1.parse::<f64>().is_ok() {
                                    ("all".to_string(), 1)
                                } else {
                                    (atom1.to_string(), 2)
                                }
                            } else {
                                ("all".to_string(), 2)
                            };

                            if shape_tag.eq_ignore_ascii_case("circle") || shape_tag.eq_ignore_ascii_case("circ") {
                                let diam = shape_item.get(dim_offset).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.8);
                                padstack.shapes.push(SpecctraPadShape::Circle {
                                    layer,
                                    diameter: res.to_mm(diam),
                                });
                            } else if shape_tag.eq_ignore_ascii_case("rect") {
                                if shape_item.len() >= dim_offset + 4 {
                                    let x1 = shape_item.get(dim_offset).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0);
                                    let y1 = shape_item.get(dim_offset + 1).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0);
                                    let x2 = shape_item.get(dim_offset + 2).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(1.0);
                                    let y2 = shape_item.get(dim_offset + 3).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(1.0);
                                    padstack.shapes.push(SpecctraPadShape::Rect {
                                        layer,
                                        width: res.to_mm((x2 - x1).abs()),
                                        height: res.to_mm((y2 - y1).abs()),
                                    });
                                } else {
                                    let w = shape_item.get(dim_offset).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(1.0);
                                    let h = shape_item.get(dim_offset + 1).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(1.0);
                                    padstack.shapes.push(SpecctraPadShape::Rect {
                                        layer,
                                        width: res.to_mm(w),
                                        height: res.to_mm(h),
                                    });
                                }
                            } else if shape_tag.eq_ignore_ascii_case("oval") {
                                let w = shape_item.get(dim_offset).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(1.0);
                                let h = shape_item.get(dim_offset + 1).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(1.0);
                                padstack.shapes.push(SpecctraPadShape::Oval {
                                    layer,
                                    width: res.to_mm(w),
                                    height: res.to_mm(h),
                                });
                            } else if shape_tag.eq_ignore_ascii_case("path") {
                                let ap = shape_item.get(dim_offset).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.5);
                                let x1 = shape_item.get(dim_offset + 1).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0);
                                let y1 = shape_item.get(dim_offset + 2).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0);
                                let x2 = shape_item.get(dim_offset + 3).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0);
                                let y2 = shape_item.get(dim_offset + 4).and_then(|e| e.as_atom()).and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0);
                                padstack.shapes.push(SpecctraPadShape::Path {
                                    layer,
                                    aperture: res.to_mm(ap),
                                    p1: Point::new(res.to_mm(x1), res.to_mm(y1)),
                                    p2: Point::new(res.to_mm(x2), res.to_mm(y2)),
                                });
                            }
                        }
                    }
                }
            }

            dsn.padstacks.insert(padstack.id.clone(), padstack);
        }
    }

    // 2. Images (Footprint definitions)
    for img_node in node.children("image") {
        if let Some(id) = img_node.nth_atom(1) {
            let mut image = SpecctraImage {
                id: id.to_string(),
                ..Default::default()
            };

            for pin_node in img_node.children("pin") {
                // Pin syntax variations:
                // (pin <padstack> <pin_id> <x> <y> [(rotate <rot>)])
                // (pin <padstack> (rotate <rot>) <pin_id> <x> <y>)
                if let Some(list) = pin_node.as_list() {
                    if list.len() >= 5 {
                        let padstack_id = list[1].as_atom().unwrap_or("").to_string();
                        let mut pin_id = String::new();
                        let mut x = 0.0;
                        let mut y = 0.0;
                        let mut rot = 0.0;
                        let mut coords_read = 0;

                        let mut idx = 2;
                        while idx < list.len() {
                            if list[idx].matches_tag("rotate") {
                                rot = list[idx].nth_f64(1).unwrap_or(0.0);
                                idx += 1;
                            } else if pin_id.is_empty() && coords_read == 0 && list[idx].as_atom().is_some() && (list[idx].as_atom().unwrap().parse::<f64>().is_err() || idx == 2) {
                                pin_id = list[idx].as_atom().unwrap().to_string();
                                idx += 1;
                            } else if let Some(val) = list[idx].as_atom().and_then(|s| s.parse::<f64>().ok()) {
                                match coords_read {
                                    0 => x = val,
                                    1 => y = val,
                                    2 => rot = val,
                                    _ => {}
                                }
                                coords_read += 1;
                                idx += 1;
                            } else {
                                idx += 1;
                            }
                        }

                        if !pin_id.is_empty() {
                            image.pins.push(SpecctraPin {
                                padstack_id,
                                pin_id,
                                relative_pos: Point::new(res.to_mm(x), res.to_mm(y)),
                                rotation: rot,
                            });
                        }
                    }
                }
            }

            // Silkscreen outline
            for outline_node in img_node.children("outline") {
                if let Some(path_node) = outline_node.child("path") {
                    if let Some(list) = path_node.as_list() {
                        let mut coords = Vec::new();
                        for elem in list.iter().skip(3) {
                            if let Some(val) = elem.as_atom().and_then(|s| s.parse::<f64>().ok()) {
                                coords.push(res.to_mm(val));
                            }
                        }
                        for i in 0..(coords.len() / 2).saturating_sub(1) {
                            let p1 = Point::new(coords[i * 2], coords[i * 2 + 1]);
                            let p2 = Point::new(coords[(i + 1) * 2], coords[(i + 1) * 2 + 1]);
                            image.silkscreen_lines.push((p1, p2));
                        }
                    }
                }
            }

            dsn.images.insert(image.id.clone(), image);
        }
    }

    Ok(())
}

/// Helper parsing the `(placement ...)` block.
fn parse_placement(dsn: &mut SpecctraDsn, node: &SExpr) -> Result<(), SpecctraError> {
    let res = dsn.resolution;

    for comp_node in node.children("component") {
        if let Some(image_id) = comp_node.nth_atom(1) {
            for place_node in comp_node.children("place") {
                if let Some(comp_id) = place_node.nth_atom(1) {
                    let x = place_node.nth_f64(2).unwrap_or(0.0);
                    let y = place_node.nth_f64(3).unwrap_or(0.0);

                    // Check if side was omitted (e.g. index 4 is a numeric rotation)
                    let (side, rot) = if let Some(rot_val) = place_node.nth_f64(4) {
                        ("front".to_string(), rot_val)
                    } else {
                        let s = place_node.nth_atom(4).unwrap_or("front").to_string();
                        let r = place_node.nth_f64(5).unwrap_or(0.0);
                        (s, r)
                    };

                    // Extract optional part number or value (e.g. (pn "10k") or (value "10k") or (part "R0805"))
                    let part_number = place_node
                        .child("pn")
                        .and_then(|n| n.nth_atom(1))
                        .or_else(|| place_node.child("value").and_then(|n| n.nth_atom(1)))
                        .or_else(|| place_node.child("part").and_then(|n| n.nth_atom(1)))
                        .map(|s| s.to_string());

                    dsn.components.push(SpecctraPlacedComponent {
                        image_id: image_id.to_string(),
                        component_id: comp_id.to_string(),
                        position: Point::new(res.to_mm(x), res.to_mm(y)),
                        rotation: rot,
                        side,
                        part_number,
                    });
                }
            }
        }
    }

    Ok(())
}

/// Helper parsing the `(network ...)` block.
fn parse_network(dsn: &mut SpecctraDsn, node: &SExpr) -> Result<(), SpecctraError> {
    let res = dsn.resolution;

    // Track net-class rule overrides: net_name -> (Option<width_mm>, Option<clearance_mm>)
    let mut class_rules: HashMap<String, (Option<f64>, Option<f64>)> = HashMap::new();
    for class_node in node.children("class") {
        let mut class_w = None;
        let mut class_c = None;

        for rule_node in class_node.children("rule") {
            if let Some(w_node) = rule_node.child("width") {
                if let Some(w) = w_node.nth_f64(1) {
                    class_w = Some(res.to_mm(w));
                }
            } else if rule_node.nth_atom(1) == Some("width") {
                if let Some(w) = rule_node.nth_f64(2) {
                    class_w = Some(res.to_mm(w));
                }
            }
            if let Some(c_node) = rule_node.child("clearance") {
                if let Some(c) = c_node.nth_f64(1) {
                    class_c = Some(res.to_mm(c));
                }
            } else if rule_node.nth_atom(1) == Some("clearance") {
                if let Some(c) = rule_node.nth_f64(2) {
                    class_c = Some(res.to_mm(c));
                }
            }
        }

        if let Some(class_list) = class_node.as_list() {
            for item in &class_list[2..] {
                if let Some(net_name) = item.as_atom() {
                    class_rules.insert(net_name.to_string(), (class_w, class_c));
                }
            }
        }
        for pins_node in class_node.children("pins") {
            if let Some(plist) = pins_node.as_list() {
                for item in &plist[1..] {
                    if let Some(net_name) = item.as_atom() {
                        class_rules.insert(net_name.to_string(), (class_w, class_c));
                    }
                }
            }
        }
    }

    for net_node in node.children("net") {
        if let Some(net_name) = net_node.nth_atom(1) {
            let mut net = SpecctraNet {
                name: net_name.to_string(),
                pins: Vec::new(),
                width: None,
                clearance: None,
            };

            // Rule override on net
            for rule_node in net_node.children("rule") {
                if let Some(w_node) = rule_node.child("width") {
                    if let Some(w) = w_node.nth_f64(1) {
                        net.width = Some(res.to_mm(w));
                    }
                } else if rule_node.nth_atom(1) == Some("width") {
                    if let Some(w) = rule_node.nth_f64(2) {
                        net.width = Some(res.to_mm(w));
                    }
                }
                if let Some(c_node) = rule_node.child("clearance") {
                    if let Some(c) = c_node.nth_f64(1) {
                        net.clearance = Some(res.to_mm(c));
                    }
                } else if rule_node.nth_atom(1) == Some("clearance") {
                    if let Some(c) = rule_node.nth_f64(2) {
                        net.clearance = Some(res.to_mm(c));
                    }
                }
            }

            // Inherit from net class if not overridden on net
            if let Some(&(cw, cc)) = class_rules.get(net_name) {
                if net.width.is_none() {
                    net.width = cw;
                }
                if net.clearance.is_none() {
                    net.clearance = cc;
                }
            }

            // Pins: iterate over all (pins ...) and (order ...) blocks
            for container in net_node.children("pins").chain(net_node.children("order")) {
                if let Some(list) = container.as_list() {
                    let mut i = 1;
                    while i < list.len() {
                        if let Some(atom) = list[i].as_atom() {
                            if let Some(dash_idx) = atom.find('-') {
                                if dash_idx > 0 && dash_idx < atom.len() - 1 {
                                    // "U1-1"
                                    let comp = &atom[..dash_idx];
                                    let pin = &atom[dash_idx + 1..];
                                    net.pins.push((comp.to_string(), pin.to_string()));
                                    i += 1;
                                    continue;
                                } else if dash_idx == atom.len() - 1 && i + 1 < list.len() {
                                    // "U1-" "1"
                                    let comp = &atom[..dash_idx];
                                    if let Some(pin) = list[i + 1].as_atom() {
                                        net.pins.push((comp.to_string(), pin.to_string()));
                                        i += 2;
                                        continue;
                                    }
                                }
                            }
                            if i + 2 < list.len() && list[i + 1].as_atom() == Some("-") {
                                // "U1" "-" "1"
                                let comp = atom;
                                if let Some(pin) = list[i + 2].as_atom() {
                                    net.pins.push((comp.to_string(), pin.to_string()));
                                }
                                i += 3;
                                continue;
                            }
                            if i + 1 < list.len() {
                                if let Some(next_atom) = list[i + 1].as_atom() {
                                    if next_atom.starts_with('-') && next_atom.len() > 1 {
                                        // "U1" "-1"
                                        let comp = atom;
                                        let pin = &next_atom[1..];
                                        net.pins.push((comp.to_string(), pin.to_string()));
                                        i += 2;
                                        continue;
                                    } else if !next_atom.starts_with('-') {
                                        // Space-separated "U1" "1" — non-standard fallback.
                                        // Only accept if first atom looks like a component ref (contains letters).
                                        // This prevents mispairing bare numbers or other unexpected atoms.
                                        if atom.chars().any(|c| c.is_ascii_alphabetic()) {
                                            let comp = atom;
                                            let pin = next_atom;
                                            net.pins.push((comp.to_string(), pin.to_string()));
                                            i += 2;
                                            continue;
                                        }
                                    }
                                }
                            }
                            i += 1;
                        } else {
                            i += 1;
                        }
                    }
                }
            }

            dsn.nets.push(net);
        }
    }

    Ok(())
}


// ============================================================================
// Board Ingestion & Conversion to PcbBoard
// ============================================================================

impl SpecctraDsn {
    /// Ingests this parsed Specctra DSN model into an internal `PcbBoard` and generates
    /// a matching `RouterConfig` ready for automated multi-layer routing.
    pub fn to_board(&self) -> Result<(PcbBoard, RouterConfig), SpecctraError> {
        let board_w = (self.bounds.width()).max(10.0);
        let board_h = (self.bounds.height()).max(10.0);

        let mut board = PcbBoard::new(board_w, board_h);

        // Identify top and bottom copper layer names from DSN
        let (top_name, bot_name) = self.get_copper_layer_names();

        // 1. Register all Nets
        let mut net_name_to_id: HashMap<String, NetId> = HashMap::new();
        for net in &self.nets {
            let id = board.get_or_create_net(&net.name);
            net_name_to_id.insert(net.name.clone(), id);
        }

        // Map pin refs: (comp_ref, pin_id) -> NetId
        let mut pin_to_net: HashMap<(String, String), NetId> = HashMap::new();
        for net in &self.nets {
            if let Some(&net_id) = net_name_to_id.get(&net.name) {
                for (comp_ref, pin_id) in &net.pins {
                    pin_to_net.insert((comp_ref.clone(), pin_id.clone()), net_id);
                }
            }
        }

        // 2. Instantiate and Place Footprints
        for comp in &self.components {
            if let Some(image) = self.images.get(&comp.image_id) {
                let mut pads = Vec::with_capacity(image.pins.len());

                for pin in &image.pins {
                    let padstack = self.padstacks.get(&pin.padstack_id);
                    let (mut pad_shape, mut pad_type) = if let Some(ps) = padstack {
                        ps.to_pad_model(&top_name, &bot_name)
                    } else {
                        (
                            PadShape::Circle { diameter: 0.8 },
                            PadType::ThroughHole {
                                drill: 0.4,
                                outer_diameter: 0.8,
                            },
                        )
                    };

                    // If pin has 90 or 270 degree rotation, swap rect/obround width and height
                    let rot_mod = ((pin.rotation % 180.0).abs() - 90.0).abs();
                    if rot_mod < 1.0 {
                        pad_shape = match pad_shape {
                            PadShape::Rect { width, height } => PadShape::Rect {
                                width: height,
                                height: width,
                            },
                            PadShape::Obround { width, height } => PadShape::Obround {
                                width: height,
                                height: width,
                            },
                            other => other,
                        };
                    }

                    // If component is placed on back, flip SMD pads to BCu
                    if comp.side.eq_ignore_ascii_case("back") {
                        if let PadType::Smd { .. } = pad_type {
                            pad_type = PadType::Smd { layer: Layer::BCu };
                        }
                    }

                    let net = pin_to_net
                        .get(&(comp.component_id.clone(), pin.pin_id.clone()))
                        .copied();

                    let pad = Pad {
                        pin_number: pin.pin_id.clone(),
                        pin_name: pin.pin_id.clone(),
                        relative_pos: pin.relative_pos,
                        shape: pad_shape,
                        pad_type,
                        net,
                    };
                    pads.push(pad);
                }

                // Transform position to normalized [0, width] x [0, height] board space
                let normalized_pos = Point::new(
                    comp.position.x - self.origin_offset.x,
                    comp.position.y - self.origin_offset.y,
                );

                let footprint = Footprint::new(
                    comp.component_id.clone(),
                    comp.part_number.clone().unwrap_or_else(|| comp.image_id.clone()),
                    comp.image_id.clone(),
                    normalized_pos,
                    comp.rotation,
                    pads,
                    image.silkscreen_lines.clone(),
                );

                board.add_footprint(footprint);
            }
        }

        // 3. Keepouts as Static Obstacle Footprints
        for (i, ko) in self.keepouts.iter().enumerate() {
            let (center, pad_shape) = match &ko.shape {
                KeepoutShape::Rect(r) => {
                    let c = Point::new(
                        r.center().x - self.origin_offset.x,
                        r.center().y - self.origin_offset.y,
                    );
                    (c, PadShape::Rect { width: r.width().max(0.5), height: r.height().max(0.5) })
                }
                KeepoutShape::Polygon(pts) => {
                    if pts.is_empty() {
                        continue;
                    }
                    let mut sum_x = 0.0;
                    let mut sum_y = 0.0;
                    let mut min_x = f64::INFINITY;
                    let mut max_x = f64::NEG_INFINITY;
                    let mut min_y = f64::INFINITY;
                    let mut max_y = f64::NEG_INFINITY;
                    for p in pts {
                        sum_x += p.x;
                        sum_y += p.y;
                        min_x = min_x.min(p.x);
                        max_x = max_x.max(p.x);
                        min_y = min_y.min(p.y);
                        max_y = max_y.max(p.y);
                    }
                    let c = Point::new(
                        (sum_x / pts.len() as f64) - self.origin_offset.x,
                        (sum_y / pts.len() as f64) - self.origin_offset.y,
                    );
                    let w = (max_x - min_x).max(0.5);
                    let h = (max_y - min_y).max(0.5);
                    (c, PadShape::Rect { width: w, height: h })
                }
            };

            let ko_pad = Pad {
                pin_number: "KO".to_string(),
                pin_name: "KO".to_string(),
                relative_pos: Point::zero(),
                shape: pad_shape,
                pad_type: PadType::ThroughHole {
                    drill: 0.1,
                    outer_diameter: 0.2,
                },
                net: Some(UNCONNECTED_NET),
            };

            let ko_fp = Footprint::new(
                format!("__KEEPOUT_{}", i),
                "KEEPOUT",
                "KEEPOUT",
                center,
                0.0,
                vec![ko_pad],
                Vec::new(),
            );
            board.add_footprint(ko_fp);
        }

        // 4. Derive Router Configuration
        let default_width = self.default_width.unwrap_or(0.25).max(0.1);
        let default_clearance = self.default_clearance.unwrap_or(0.20).max(0.1);

        // Derive via sizes from via padstack in DSN if available
        let (via_diam, via_drill) = self.get_via_dimensions(default_width);

        let mut min_pitch = f64::INFINITY;
        for fp in &board.footprints {
            if fp.reference.starts_with("__KEEPOUT") { continue; }
            for i in 0..fp.pads.len() {
                for j in (i + 1)..fp.pads.len() {
                    let dist = fp.pads[i].relative_pos.distance_to(fp.pads[j].relative_pos);
                    if dist > 0.01 && dist < min_pitch {
                        min_pitch = dist;
                    }
                }
            }
        }

        let mut grid_res: f64 = if default_width <= 0.15 || default_clearance <= 0.15 {
            0.10
        } else if default_width <= 0.20 || default_clearance <= 0.20 {
            0.20
        } else {
            0.25
        };

        if min_pitch < 0.7 {
            grid_res = grid_res.min(0.05);
        } else if min_pitch < 1.1 {
            grid_res = grid_res.min(0.10);
        } else if (min_pitch - 1.27).abs() < 0.05 || (min_pitch - 2.54).abs() < 0.05 {
            grid_res = grid_res.min(0.127);
        }

        let config = RouterConfig {
            grid_resolution: grid_res,
            default_trace_width: default_width,
            via_diameter: via_diam,
            via_drill: via_drill,
            clearance: default_clearance,
        };

        Ok((board, config))
    }

    /// Identifies the names of the top and bottom copper layers.
    pub fn get_copper_layer_names(&self) -> (String, String) {
        let signal_layers: Vec<&SpecctraLayer> = self.layers.iter().filter(|l| l.is_signal).collect();
        if signal_layers.len() >= 2 {
            (signal_layers[0].name.clone(), signal_layers.last().unwrap().name.clone())
        } else if signal_layers.len() == 1 {
            (signal_layers[0].name.clone(), "B.Cu".to_string())
        } else {
            ("F.Cu".to_string(), "B.Cu".to_string())
        }
    }

    /// Derives via diameter and drill from allowed via padstacks or default rules.
    pub fn get_via_dimensions(&self, default_trace_width: f64) -> (f64, f64) {
        let (top_name, bot_name) = self.get_copper_layer_names();
        for via_id in &self.via_padstack_ids {
            if let Some(ps) = self.padstacks.get(via_id) {
                let (_, pad_type) = ps.to_pad_model(&top_name, &bot_name);
                if let PadType::ThroughHole { drill, outer_diameter } = pad_type {
                    return (outer_diameter, drill);
                }
            }
            if let Some(dimensions) = via_nm_dimensions(via_id) { return dimensions; }
            // Parse dimensions from via name directly: e.g. Via[0-1]_600:300_um or Via_24_12_mil
            if via_id.starts_with("Via[") && via_id.contains(':') && via_id.ends_with("_um") {
                if let Some(under) = via_id.find('_') {
                    let rest = &via_id[under + 1..via_id.len() - 3];
                    if let Some(colon) = rest.find(':') {
                        if let (Ok(diam_um), Ok(drill_um)) = (
                            rest[..colon].parse::<f64>(),
                            rest[colon + 1..].parse::<f64>(),
                        ) {
                            return (diam_um / 1000.0, drill_um / 1000.0);
                        }
                    }
                }
            }
        }
        for (id, ps) in &self.padstacks {
            if id.to_ascii_lowercase().starts_with("via") {
                let (_, pad_type) = ps.to_pad_model(&top_name, &bot_name);
                if let PadType::ThroughHole { drill, outer_diameter } = pad_type {
                    return (outer_diameter, drill);
                }
            }
        }
        // Fallback standard sizes
        let diam = (default_trace_width * 3.2).clamp(0.6, 0.8);
        let drill = (diam * 0.5).clamp(0.3, 0.4);
        (diam, drill)
    }
}

// ============================================================================
// Specctra Session Exporter (`.ses`)
// ============================================================================

/// Formats a coordinate value cleanly for S-expressions without trailing zeros.
#[inline]
fn format_ses_coord(val: f64) -> String {
    if val.abs() < 1e-9 {
        return "0".to_string();
    }
    let rounded = (val * 100000.0).round() / 100000.0;
    let s = format!("{:.5}", rounded);
    let trimmed = s.trim_end_matches('0');
    let res = trimmed.trim_end_matches('.');
    if res.is_empty() || res == "-0" {
        "0".to_string()
    } else {
        res.to_string()
    }
}

/// Exports routed tracks and vias from `board` into a standards-compliant Specctra
/// Session (`.ses`) S-expression document, ready for KiCad's "Import Specctra Session".
pub fn export_specctra_ses(board: &PcbBoard, dsn: &SpecctraDsn) -> String {
    let (top_layer_name, bot_layer_name) = dsn.get_copper_layer_names();
    let preferred_via = dsn.via_padstack_ids.first().map(|s| s.as_str());
    export_specctra_ses_custom(
        board,
        &dsn.pcb_name,
        &dsn.pcb_name,
        &dsn.resolution,
        dsn.origin_offset,
        &top_layer_name,
        &bot_layer_name,
        preferred_via,
    )
}

/// Lower-level customizable SES exporter allowing precise control of names, resolution,
/// coordinate offsets, and copper layer identifiers.
#[allow(clippy::too_many_arguments)]
pub fn export_specctra_ses_custom(
    board: &PcbBoard,
    session_name: &str,
    base_design: &str,
    resolution: &SpecctraResolution,
    origin_offset: Point,
    top_layer_name: &str,
    bot_layer_name: &str,
    preferred_via_id: Option<&str>,
) -> String {
    let mut out = String::with_capacity(16384);

    // 1. Session Header & Base Design Reference
    out.push_str(&format!("(session \"{}\"\n", session_name));
    out.push_str(&format!("  (base_design \"{}\")\n\n", base_design));

    // 2. Routes Block
    out.push_str("  (routes\n");
    out.push_str(&format!(
        "    (resolution {} {})\n",
        resolution.unit.as_str(),
        format_ses_coord(resolution.ses_resolution_factor())
    ));
    out.push_str("    (parser\n");
    out.push_str("      (string_quote \")\n");
    out.push_str("      (space_in_quoted_tokens on)\n");
    out.push_str("    )\n\n");

    // 3. Library Out (Via Padstack Declarations)
    // import_specctra_ses recovers the drill from the ID; circles retain copper geometry.
    let via_diam_mm = if let Some(v) = board.vias.first() {
        v.size
    } else {
        0.80
    };
    let via_drill_mm = if let Some(v) = board.vias.first() {
        v.drill
    } else {
        0.40
    };

    let via_diam_dsn = resolution.ses_from_mm(via_diam_mm);
    let kicad_padstack_id = via_padstack_id(via_diam_mm, via_drill_mm);
    let active_padstack_id = if let Some(pref) = preferred_via_id {
        if pref.starts_with("Via[") {
            pref
        } else {
            &kicad_padstack_id
        }
    } else {
        &kicad_padstack_id
    };

    out.push_str("    (library_out\n");
    out.push_str(&format!("      (padstack \"{}\"\n", active_padstack_id));
    out.push_str(&format!(
        "        (shape (circle \"{}\" {}))\n",
        top_layer_name,
        format_ses_coord(via_diam_dsn)
    ));
    out.push_str(&format!(
        "        (shape (circle \"{}\" {}))\n",
        bot_layer_name,
        format_ses_coord(via_diam_dsn)
    ));
    out.push_str("      )\n");

    // Also declare preferred_via_id or kicad_padstack_id if different
    if let Some(pref) = preferred_via_id {
        if pref != active_padstack_id {
            out.push_str(&format!("      (padstack \"{}\"\n", pref));
            out.push_str(&format!(
                "        (shape (circle \"{}\" {}))\n",
                top_layer_name,
                format_ses_coord(via_diam_dsn)
            ));
            out.push_str(&format!(
                "        (shape (circle \"{}\" {}))\n",
                bot_layer_name,
                format_ses_coord(via_diam_dsn)
            ));
            out.push_str("      )\n");
        }
    }
    out.push_str("    )\n\n");

    // 4. Network Out (Wires and Vias organized by Net)
    out.push_str("    (network_out\n");

    // Group tracks and vias by NetId
    let mut net_tracks: BTreeMap<NetId, Vec<&crate::pcb::core::TrackSegment>> = BTreeMap::new();
    let mut net_vias: BTreeMap<NetId, Vec<&crate::pcb::core::Via>> = BTreeMap::new();

    for track in &board.tracks {
        net_tracks.entry(track.net).or_default().push(track);
    }
    for via in &board.vias {
        net_vias.entry(via.net).or_default().push(via);
    }

    // Collect all active net IDs
    let mut all_nets: HashSet<NetId> = HashSet::new();
    for &net in net_tracks.keys() {
        if net != UNCONNECTED_NET {
            all_nets.insert(net);
        }
    }
    for &net in net_vias.keys() {
        if net != UNCONNECTED_NET {
            all_nets.insert(net);
        }
    }

    let mut sorted_nets: Vec<NetId> = all_nets.into_iter().collect();
    sorted_nets.sort_unstable();

    for net_id in sorted_nets {
        let net_name = board
            .get_net_name(net_id)
            .unwrap_or(&format!("NET_{}", net_id))
            .to_string();

        out.push_str(&format!("      (net \"{}\"\n", net_name));

        // Output Wires
        if let Some(tracks) = net_tracks.get(&net_id) {
            for track in tracks {
                let layer_str = if track.layer == Layer::FCu {
                    top_layer_name
                } else {
                    bot_layer_name
                };

                let width_dsn = resolution.ses_from_mm(track.width);
                let x1_dsn = resolution.ses_from_mm(track.start.x + origin_offset.x);
                let y1_dsn = resolution.ses_from_mm(track.start.y + origin_offset.y);
                let x2_dsn = resolution.ses_from_mm(track.end.x + origin_offset.x);
                let y2_dsn = resolution.ses_from_mm(track.end.y + origin_offset.y);

                out.push_str(&format!(
                    "        (wire (path \"{}\" {} {} {} {} {}) (type route))\n",
                    layer_str,
                    format_ses_coord(width_dsn),
                    format_ses_coord(x1_dsn),
                    format_ses_coord(y1_dsn),
                    format_ses_coord(x2_dsn),
                    format_ses_coord(y2_dsn)
                ));
            }
        }

        // Output Vias
        if let Some(vias) = net_vias.get(&net_id) {
            for via in vias {
                let vx_dsn = resolution.ses_from_mm(via.at.x + origin_offset.x);
                let vy_dsn = resolution.ses_from_mm(via.at.y + origin_offset.y);

                out.push_str(&format!(
                    "        (via \"{}\" {} {})\n",
                    active_padstack_id,
                    format_ses_coord(vx_dsn),
                    format_ses_coord(vy_dsn)
                ));
            }
        }

        out.push_str("      )\n");
    }

    out.push_str("    )\n"); // end network_out
    out.push_str("  )\n");   // end routes
    out.push_str(")\n");     // end session

    out
}

// ============================================================================
// Specctra DSN Exporter (Bidirectional Roundtrip)
// ============================================================================

/// Configuration for exporting a `PcbBoard` into a Specctra `.dsn` S-expression document.
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraDsnExportConfig {
    pub pcb_name: String,
    pub unit: SpecctraUnit,
    pub resolution_factor: f64,
    pub top_layer_name: String,
    pub bot_layer_name: String,
}

impl Default for SpecctraDsnExportConfig {
    fn default() -> Self {
        Self {
            pcb_name: "board".to_string(),
            unit: SpecctraUnit::Mm,
            resolution_factor: 1000.0,
            top_layer_name: "F.Cu".to_string(),
            bot_layer_name: "B.Cu".to_string(),
        }
    }
}

/// Exports any internal `PcbBoard` into a valid Specctra DSN S-expression document.
pub fn export_specctra_dsn(board: &PcbBoard, config: &SpecctraDsnExportConfig) -> String {
    let rules = RouterConfig {
        default_trace_width: board.tracks.first().map(|t|t.width).unwrap_or(0.25),
        via_diameter: board.vias.first().map(|v|v.size).unwrap_or(0.8),
        via_drill: board.vias.first().map(|v|v.drill).unwrap_or(0.4),
        ..RouterConfig::default()
    };
    export_specctra_dsn_with_rules(board, config, &rules)
}

fn dsn_padstack_id(pad: &Pad) -> String {
    let b=pad.bounding_box(Point::zero(),0.0);
    let kind=match pad.pad_type { PadType::Smd {layer} => format!("smd_{layer:?}"), PadType::ThroughHole {drill,..} => format!("tht_{drill:.6}") };
    // Freerouting strips /\.\d+/ from identifiers. Decimal names alias distinct pads.
    format!("{}_{:.6}x{:.6}_{}",pad.shape.kicad_shape_name(),b.width(),b.height(),kind).replace('.', "p")
}

pub fn export_specctra_dsn_with_rules(board: &PcbBoard, config: &SpecctraDsnExportConfig, rules: &RouterConfig) -> String {
    let res = SpecctraResolution::new(config.unit, config.resolution_factor);
    let mut out = String::with_capacity(16384);

    // 1. Root & Parser Descriptor
    out.push_str(&format!("(pcb \"{}\"\n", config.pcb_name));
    out.push_str("  (parser\n");
    out.push_str("    (string_quote \")\n");
    out.push_str("    (space_in_quoted_tokens on)\n");
    out.push_str("    (host_cad \"netlist_shatter_viz\")\n");
    out.push_str("    (host_version \"0.1.0\")\n");
    out.push_str("  )\n");
    out.push_str(&format!(
        "  (resolution {} {})\n",
        res.unit.as_str(),
        format_ses_coord(res.factor)
    ));
    out.push_str(&format!("  (unit {})\n\n", res.unit.as_str()));

    // 2. Structure Descriptor
    out.push_str("  (structure\n");
    out.push_str(&format!(
        "    (layer \"{}\" (type signal) (property (index 0)))\n",
        config.top_layer_name
    ));
    out.push_str(&format!(
        "    (layer \"{}\" (type signal) (property (index 1)))\n",
        config.bot_layer_name
    ));

    // Board Outline Boundary
    let w_dsn = res.from_mm(board.width);
    let h_dsn = res.from_mm(board.height);
    out.push_str(&format!(
        "    (boundary (rect pcb 0 0 {} {}))\n",
        format_ses_coord(w_dsn),
        format_ses_coord(h_dsn)
    ));

    // Via & Rules
    let via_diam_mm = rules.via_diameter;
    let via_drill_mm = rules.via_drill;
    let via_ps_name = via_padstack_id(via_diam_mm, via_drill_mm);
    out.push_str(&format!("    (via \"{}\")\n", via_ps_name));

    let trace_w_mm = rules.default_trace_width;
    let clear_mm = rules.clearance;
    let trace_w_dsn = res.from_mm(trace_w_mm);
    let clear_dsn = res.from_mm(clear_mm);
    out.push_str(&format!(
        "    (rule (width {}) (clearance {}))\n",
        format_ses_coord(trace_w_dsn),
        format_ses_coord(clear_dsn)
    ));

    // Keepout areas
    for fp in &board.footprints {
        if fp.reference.starts_with("__KEEPOUT") {
            if let Some(pad) = fp.pads.first() {
                let (w, h) = match pad.shape {
                    PadShape::Rect { width, height } => (width, height),
                    PadShape::Obround { width, height } => (width, height),
                    PadShape::Circle { diameter } => (diameter, diameter),
                };
                let x1 = res.from_mm(fp.position.x - w / 2.0);
                let y1 = res.from_mm(fp.position.y - h / 2.0);
                let x2 = res.from_mm(fp.position.x + w / 2.0);
                let y2 = res.from_mm(fp.position.y + h / 2.0);
                out.push_str(&format!(
                    "    (keepout \"{}\"\n      (rect pcb {} {} {} {})\n    )\n",
                    fp.reference,
                    format_ses_coord(x1),
                    format_ses_coord(y1),
                    format_ses_coord(x2),
                    format_ses_coord(y2)
                ));
            }
        }
    }
    out.push_str("  )\n\n");

    // 3. Placement Descriptor
    out.push_str("  (placement\n");
    // Group footprints by package
    let mut by_package: BTreeMap<String, Vec<&Footprint>> = BTreeMap::new();
    for fp in &board.footprints {
        if fp.reference.starts_with("__KEEPOUT") {
            continue;
        }
        by_package.entry(format!("{}__{}",fp.package,fp.reference)).or_default().push(fp);
    }

    for (pkg, fps) in &by_package {
        out.push_str(&format!("    (component \"{}\"\n", pkg));
        for fp in fps {
            let px = res.from_mm(fp.position.x);
            let py = res.from_mm(fp.position.y);
            out.push_str(&format!(
                "      (place \"{}\" {} {} front {} (pn \"{}\"))\n",
                fp.reference,
                format_ses_coord(px),
                format_ses_coord(py),
                format_ses_coord(fp.rotation),
                fp.value
            ));
        }
        out.push_str("    )\n");
    }
    out.push_str("  )\n\n");

    // 4. Library Descriptor
    out.push_str("  (library\n");
    let mut emitted_padstacks: HashSet<String> = HashSet::new();

    for (pkg, fps) in &by_package {
        if let Some(fp) = fps.first() {
            out.push_str(&format!("    (image \"{}\"\n", pkg));
            for pad in &fp.pads {
                let ps_name = dsn_padstack_id(pad);
                let rx = res.from_mm(pad.relative_pos.x);
                let ry = res.from_mm(pad.relative_pos.y);
                out.push_str(&format!(
                    "      (pin \"{}\" \"{}\" {} {})\n",
                    ps_name, pad.pin_number, format_ses_coord(rx), format_ses_coord(ry)
                ));
            }
            out.push_str("    )\n");
        }
    }

    // Padstacks
    for fps in by_package.values() {
        if let Some(fp) = fps.first() {
            for pad in &fp.pads {
                let bbox = pad.bounding_box(Point::zero(), 0.0);
                let pw_dsn = res.from_mm(bbox.width());
                let ph_dsn = res.from_mm(bbox.height());
                let ps_name = dsn_padstack_id(pad);

                if emitted_padstacks.insert(ps_name.clone()) {
                    out.push_str(&format!("    (padstack \"{}\"\n", ps_name));
                    let layers: Vec<&str> = match pad.pad_type {
                        PadType::ThroughHole { .. } => vec![&config.top_layer_name, &config.bot_layer_name],
                        PadType::Smd { layer: Layer::BCu } => vec![&config.bot_layer_name],
                        _ => vec![&config.top_layer_name],
                    };
                    for layer in layers {
                        match pad.shape {
                            PadShape::Circle { diameter } => out.push_str(&format!("      (shape (circle \"{}\" {}))\n", layer, format_ses_coord(res.from_mm(diameter)))),
                            PadShape::Rect { .. } => out.push_str(&format!("      (shape (rect \"{}\" {} {} {} {}))\n", layer, format_ses_coord(-pw_dsn/2.0), format_ses_coord(-ph_dsn/2.0), format_ses_coord(pw_dsn/2.0), format_ses_coord(ph_dsn/2.0))),
                            PadShape::Obround { .. } => {
                                let (dx,dy) = if pw_dsn > ph_dsn { ((pw_dsn-ph_dsn)/2.0,0.0) } else { (0.0,(ph_dsn-pw_dsn)/2.0) };
                                out.push_str(&format!("      (shape (path \"{}\" {} {} {} {} {}))\n",layer,format_ses_coord(pw_dsn.min(ph_dsn)),format_ses_coord(-dx),format_ses_coord(-dy),format_ses_coord(dx),format_ses_coord(dy)));
                            }
                        }
                    }
                    if let PadType::ThroughHole {drill,..}=pad.pad_type { out.push_str(&format!("      (drill {})\n",format_ses_coord(res.from_mm(drill)))); }
                    out.push_str("    )\n");
                }
            }
        }
    }

    // Via Padstack
    let via_d_dsn = res.from_mm(via_diam_mm);
    out.push_str(&format!("    (padstack \"{}\"\n", via_ps_name));
    out.push_str(&format!("      (shape (circle \"{}\" {}))\n", config.top_layer_name, format_ses_coord(via_d_dsn)));
    out.push_str(&format!("      (shape (circle \"{}\" {}))\n", config.bot_layer_name, format_ses_coord(via_d_dsn)));
    out.push_str("    )\n");
    out.push_str("  )\n\n");

    // 5. Network Descriptor
    out.push_str("  (network\n");
    let mut net_pins: BTreeMap<NetId, Vec<String>> = BTreeMap::new();
    for fp in &board.footprints {
        if fp.reference.starts_with("__KEEPOUT") {
            continue;
        }
        for pad in &fp.pads {
            if let Some(net) = pad.net {
                if net != UNCONNECTED_NET {
                    net_pins.entry(net).or_default().push(format!("{}-{}", fp.reference, pad.pin_number));
                }
            }
        }
    }

    for (net_id, pins) in net_pins {
        let net_name = board.get_net_name(net_id).unwrap_or(&format!("NET_{}", net_id)).to_string();
        out.push_str(&format!("    (net \"{}\"\n", net_name));
        out.push_str("      (pins");
        for p in pins {
            out.push(' ');
            out.push_str(&p);
        }
        out.push_str(")\n");
        out.push_str("    )\n");
    }
    out.push_str("  )\n");

    out.push_str(")\n"); // end pcb
    out
}

// ============================================================================
// High-Level Pipeline: Route DSN to SES
// ============================================================================

/// End-to-end autorouting execution: parses a Specctra `.dsn` document, ingests it into
/// `PcbBoard`, runs multi-layer maze routing with Rip-Up & Reroute, and generates a `.ses` file.
pub fn route_dsn_to_ses(
    dsn_content: &str,
    config_override: Option<&RouterConfig>,
) -> Result<(String, RoutingReport), SpecctraError> {
    let dsn = parse_specctra_dsn(dsn_content)?;
    let (mut board, default_config) = dsn.to_board()?;

    let active_config = config_override.unwrap_or(&default_config);

    let report = route_board_with_report(&mut board, active_config)
        .map_err(|e| SpecctraError::RoutingFailed {
            details: e.to_string(),
        })?;

    let ses = export_specctra_ses(&board, &dsn);
    Ok((ses, report))
}

/// File-based autorouting execution: reads an input `.dsn` file from disk, routes all nets,
/// and writes the resulting `.ses` session file to disk.
pub fn route_dsn_file_to_ses_file(
    input_dsn_path: impl AsRef<Path>,
    output_ses_path: impl AsRef<Path>,
    config_override: Option<&RouterConfig>,
) -> Result<RoutingReport, SpecctraError> {
    let in_path = input_dsn_path.as_ref();
    let out_path = output_ses_path.as_ref();

    let content = fs::read_to_string(in_path).map_err(|e| SpecctraError::IoError {
        path: in_path.display().to_string(),
        details: e.to_string(),
    })?;

    let (ses_content, report) = route_dsn_to_ses(&content, config_override)?;

    if let Some(parent) = out_path.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent).map_err(|e| SpecctraError::IoError {
                path: parent.display().to_string(),
                details: e.to_string(),
            })?;
        }
    }

    fs::write(out_path, ses_content).map_err(|e| SpecctraError::IoError {
        path: out_path.display().to_string(),
        details: e.to_string(),
    })?;

    Ok(report)
}

// ============================================================================
// Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sexpr_tokenizer_basic() {
        let input = "(pcb \"test_board\" (resolution mm 1000) (structure))";
        let ast = parse_sexpr(input).expect("parse sexpr");
        assert_eq!(ast.len(), 1);
        let root = &ast[0];
        assert_eq!(root.name(), Some("pcb"));
        assert_eq!(root.nth_atom(1), Some("test_board"));
        assert!(root.child("resolution").is_some());
        assert!(root.child("structure").is_some());
    }

    #[test]
    fn test_sexpr_comments_and_escapes() {
        let input = r#"
            ; Full line comment
            (pcb "board_with_spaces and \"escapes\"" # another comment
                (unit mm) ; inline comment
                (pins "U1"-"1" "U2"-"2")
            )
        "#;
        let ast = parse_sexpr(input).expect("parse complex sexpr");
        let root = &ast[0];
        assert_eq!(root.nth_atom(1), Some("board_with_spaces and \"escapes\""));
        let pins = root.child("pins").expect("pins");
        assert_eq!(pins.nth_atom(1), Some("U1"));
        assert_eq!(pins.nth_atom(2), Some("-"));
        assert_eq!(pins.nth_atom(3), Some("1"));
    }

    #[test]
    fn test_resolution_conversions() {
        // DSN mm coordinates remain in mm at any resolution
        let res_mm1000 = SpecctraResolution::new(SpecctraUnit::Mm, 1000.0);
        assert_eq!(res_mm1000.to_mm(1.0), 1.0);
        assert_eq!(res_mm1000.from_mm(1.0), 1.0);

        // (resolution um 10): in KiCad coordinates are in micrometers (1000 um = 1 mm)
        let res_um10 = SpecctraResolution::new(SpecctraUnit::Um, 10.0);
        assert_eq!(res_um10.to_mm(1000.0), 1.0);
        assert_eq!(res_um10.from_mm(1.0), 1000.0);

        // (resolution mil 10): in Altium coordinates are in mils (100 mil = 2.54 mm)
        let res_mil10 = SpecctraResolution::new(SpecctraUnit::Mil, 10.0);
        assert!((res_mil10.to_mm(100.0) - 2.54).abs() < 1e-9);
        assert!((res_mil10.from_mm(2.54) - 100.0).abs() < 1e-9);

        // DSN inch coordinates remain in inches at any resolution
        let res_inch = SpecctraResolution::new(SpecctraUnit::Inch, 10000.0);
        assert!((res_inch.to_mm(1.0) - 25.4).abs() < 1e-9);
        assert!((res_inch.from_mm(25.4) - 1.0).abs() < 1e-9);
    }
}

/// Import a router session into an existing board. Unknown nets, layers, via
/// padstacks and malformed coordinates are errors. The board is unchanged on error.
/// SES coordinates are integer resolution units (unlike DSN coordinates).
pub fn import_specctra_ses(board: &mut PcbBoard, content: &str) -> Result<(), SpecctraError> {
    use super::{TrackSegment,Via};
    let error = |details: &str| SpecctraError::ParseError {line:0,col:0,details:details.into()};
    let roots = parse_sexpr(content)?;
    let root = roots.first().filter(|x|x.matches_tag("session")).ok_or_else(||error("Expected session"))?;
    let routes=root.child("routes").ok_or_else(||error("Missing routes"))?;
    let res=routes.child("resolution").ok_or_else(||error("Missing session resolution"))?;
    let unit=res.nth_atom(1).and_then(SpecctraUnit::from_str).ok_or_else(||error("Unknown session unit"))?;
    let factor=res.nth_f64(2).filter(|n|n.is_finite() && *n>0.0).ok_or_else(||error("Invalid resolution"))?;
    let scale=unit.to_mm_factor()/factor;
    let number=|node:&SExpr,i:usize| -> Result<f64,SpecctraError> { node.nth_f64(i).filter(|n|n.is_finite()).map(|n|n*scale).ok_or_else(||error("Invalid session coordinate")) };
    if let Some(placement)=root.child("placement") {
        let pres=placement.child("resolution").ok_or_else(||error("Missing placement resolution"))?;
        let punit=pres.nth_atom(1).and_then(SpecctraUnit::from_str).ok_or_else(||error("Unknown placement unit"))?;
        let pf=pres.nth_f64(2).filter(|v|v.is_finite() && *v>0.0).ok_or_else(||error("Invalid placement resolution"))?;
        let pscale=punit.to_mm_factor()/pf;
        let mut seen=HashSet::new();
        for component in placement.children("component") { for place in component.children("place") {
            let reference=place.nth_atom(1).ok_or_else(||error("Missing placement reference"))?;
            let fp=board.footprints.iter().find(|f|f.reference==reference).ok_or_else(||error("Session contains an unknown component"))?;
            let x=place.nth_f64(2).ok_or_else(||error("Invalid placement"))?*pscale;
            let y=place.nth_f64(3).ok_or_else(||error("Invalid placement"))?*pscale;
            let rot=place.nth_f64(5).ok_or_else(||error("Missing placement rotation"))?;
            if ![x,y,rot].iter().all(|v|v.is_finite()) || (x-fp.position.x).abs()>0.001 || (y-fp.position.y).abs()>0.001 || (rot-fp.rotation).rem_euclid(360.0).min((fp.rotation-rot).rem_euclid(360.0))>0.001 || place.nth_atom(4)!=Some("front") || !seen.insert(reference) {
                return Err(error("Session placement does not match this board"));
            }
        }}
        if seen.len()!=board.footprints.len() {return Err(error("Session is missing components"));}
    }
    let mut via_sizes=HashMap::new();
    if let Some(library)=routes.child("library_out") {
        for ps in library.children("padstack") {
            let id=ps.nth_atom(1).ok_or_else(||error("Unnamed via"))?;
            let circle=ps.child("shape").and_then(|s|s.child("circle")).ok_or_else(||error("Non-circular via unsupported"))?;
            let diameter=number(circle,2)?;
            let drill = if id.to_ascii_lowercase().contains("_nm") {
                via_nm_dimensions(id).map(|(_, drill)| drill).ok_or_else(||error("Invalid integer nanometre via padstack ID"))?
            } else {
                id.split(':').nth(1).and_then(|s|s.split('_').next()).and_then(|s|s.parse::<f64>().ok()).map(|x|x/1000.0).ok_or_else(||error("Via drill diameter not encoded in padstack ID"))?
            };
            if diameter<=drill || drill<=0.0 {return Err(error("Invalid via dimensions"));}
            via_sizes.insert(id.to_string(),(diameter,drill));
        }
    }
    let network=routes.child("network_out").ok_or_else(||error("Missing network_out"))?;
    let mut tracks=Vec::new();let mut vias=Vec::new();
    for net in network.children("net") {
        let name=net.nth_atom(1).ok_or_else(||error("Missing net name"))?;
        let id=board.net_names.iter().find(|(_,n)|n.as_str()==name).map(|(&id,_)|id).ok_or_else(||error(&format!("Unknown net {name}")))?;
        for wire in net.children("wire") {
            let path=wire.child("path").ok_or_else(||error("Only line paths supported"))?;
            let layer=match path.nth_atom(1) {Some("F.Cu")=>Layer::FCu,Some("B.Cu")=>Layer::BCu,_=>return Err(error("Unknown session copper layer"))};
            let width=number(path,2)?;if width<=0.0 {return Err(error("Nonpositive trace width"));}
            let len=path.as_list().unwrap().len();if len<5 || (len-3)%2!=0 {return Err(error("Invalid path coordinate count"));}
            // A one-point path is a round copper dot. Preserve it; cleanup may remove
            // it only when another trace completely covers it after width repair.
            if len==5 {let p=Point::new(number(path,3)?,number(path,4)?);tracks.push(TrackSegment::new(p,p,width,layer,id));continue;}
            for i in (3..len-2).step_by(2) {
                tracks.push(TrackSegment::new(Point::new(number(path,i)?,number(path,i+1)?),Point::new(number(path,i+2)?,number(path,i+3)?),width,layer,id));
            }
        }
        for via in net.children("via") {
            let ps=via.nth_atom(1).and_then(|p|via_sizes.get(p)).ok_or_else(||error("Unknown via padstack"))?;
            vias.push(Via::new(Point::new(number(via,2)?,number(via,3)?),ps.0,ps.1,id));
        }
    }
    board.tracks=tracks;board.vias=vias;
    Ok(())
}
