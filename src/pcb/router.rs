//! Automated 2-Layer Grid A* Maze Autorouter with Rip-Up & Reroute (RRR)
//!
//! Features:
//! - Discretized 2D/3D grid pathfinder (configurable resolution, e.g. 0.25mm) mapping F.Cu (layer 0) and B.Cu (layer 1)
//! - 45° and 90° octilinear direction steps with corner-pinching avoidance
//! - Anisotropic layer biasing (horizontal preference on F.Cu, vertical preference on B.Cu)
//! - Automated via insertion on layer transitions with via cost penalty and clearance checking
//! - Clearance inflation around obstacles (pads, existing tracks, vias, board edges)
//! - Congestion-negotiated Rip-Up and Reroute (RRR) for resolving crossing/blocking nets
//! - Collinear track segment merging (simplifying paths to long, clean octilinear segments)
//! - Elimination of degenerate zero-length segments
//! - Explicit failure when the bounded search cannot complete a route

use crate::pcb::core::*;
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

/// Configuration parameters for the PCB autorouter and DRC engine.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RouterConfig {
    pub grid_resolution: f64,     // mm, e.g. 0.25mm
    pub default_trace_width: f64, // mm, e.g. 0.25mm
    pub via_diameter: f64,        // mm, e.g. 0.80mm
    pub via_drill: f64,           // mm, e.g. 0.40mm
    pub clearance: f64,           // mm, e.g. 0.20mm
}

impl RouterConfig {
    pub fn validate(&self) -> Result<(), RoutingError> {
        if ![self.grid_resolution, self.default_trace_width, self.via_diameter, self.via_drill]
            .iter().all(|v| v.is_finite() && *v > 0.0)
            || !self.clearance.is_finite() || self.clearance < 0.0
            || self.via_drill >= self.via_diameter
        {
            return Err(RoutingError::InvalidConfiguration {
                details: "Rule dimensions must be finite and positive, drill smaller than via, and clearance nonnegative".into(),
            });
        }
        Ok(())
    }
}

impl Default for RouterConfig {
    fn default() -> Self {
        Self {
            grid_resolution: 0.25,
            default_trace_width: 0.25,
            via_diameter: 0.80,
            via_drill: 0.40,
            clearance: 0.20,
        }
    }
}

/// Summary report returned upon successful completion of routing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RoutingReport {
    pub routed_nets: usize,
    pub unrouted_nets: usize,
    pub total_tracks: usize,
    pub total_vias: usize,
    pub rrr_iterations: usize,
}

/// Errors encountered during automated routing.
#[derive(Clone, Debug, PartialEq)]
pub enum RoutingError {
    UnroutableNet {
        net_id: NetId,
        details: String,
    },
    NoPathFound {
        net_id: NetId,
        start: Point,
        end: Point,
    },
    BoardTooDense {
        details: String,
    },
    InvalidConfiguration {
        details: String,
    },
}

impl std::fmt::Display for RoutingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnroutableNet { net_id, details } => {
                write!(f, "Net {} is unroutable: {}", net_id, details)
            }
            Self::NoPathFound { net_id, start, end } => {
                write!(
                    f,
                    "No routing path found for net {} between ({:.2},{:.2}) and ({:.2},{:.2})",
                    net_id, start.x, start.y, end.x, end.y
                )
            }
            Self::BoardTooDense { details } => {
                write!(f, "Board routing density exceeded: {}", details)
            }
            Self::InvalidConfiguration { details } => {
                write!(f, "Invalid router configuration: {}", details)
            }
        }
    }
}

impl std::error::Error for RoutingError {}

// ============================================================================
// Internal 3D Grid & A* Pathfinder
// ============================================================================

const LAYER_FCU: usize = 0;
const LAYER_BCU: usize = 1;
const NUM_LAYERS: usize = 2;

const MIN_EDGE_KEEPOUT: f64 = 0.40;

/// State of an individual grid cell.
#[derive(Clone, Copy, Debug, PartialEq)]
enum CellState {
    Free,
    EdgeKeepout,
    Pad { net: NetId },
    Track { net: NetId },
    Via { net: NetId },
}

/// Flat 3D occupancy grid for fast cache-friendly obstacle queries.
struct RoutingGrid {
    width: f64,
    height: f64,
    resolution: f64,
    nx: usize,
    ny: usize,
    // Layer 0 = FCu, Layer 1 = BCu
    cells: [Vec<CellState>; NUM_LAYERS],
    pad_dist: [Vec<f32>; NUM_LAYERS],
}

impl RoutingGrid {
    fn new(width: f64, height: f64, resolution: f64) -> Self {
        let nx = ((width / resolution).ceil() as usize) + 1;
        let ny = ((height / resolution).ceil() as usize) + 1;
        let total_cells = nx * ny;
        Self {
            width,
            height,
            resolution,
            nx,
            ny,
            cells: [
                vec![CellState::Free; total_cells],
                vec![CellState::Free; total_cells],
            ],
            pad_dist: [
                vec![f32::INFINITY; total_cells],
                vec![f32::INFINITY; total_cells],
            ],
        }
    }

    #[inline]
    fn idx(&self, gx: usize, gy: usize) -> usize {
        gy * self.nx + gx
    }

    #[inline]
    fn in_bounds(&self, gx: isize, gy: isize) -> bool {
        gx >= 0 && (gx as usize) < self.nx && gy >= 0 && (gy as usize) < self.ny
    }

    #[inline]
    fn world_to_grid(&self, p: Point) -> (isize, isize) {
        let gx = (p.x / self.resolution).round() as isize;
        let gy = (p.y / self.resolution).round() as isize;
        (gx, gy)
    }

    #[inline]
    fn grid_to_world(&self, gx: usize, gy: usize) -> Point {
        Point::new(
            gx as f64 * self.resolution,
            gy as f64 * self.resolution,
        )
    }

    #[inline]
    fn get_cell(&self, layer: usize, gx: usize, gy: usize) -> CellState {
        let idx = self.idx(gx, gy);
        self.cells[layer][idx]
    }

    #[inline]
    fn set_cell(&mut self, layer: usize, gx: usize, gy: usize, state: CellState) {
        let idx = self.idx(gx, gy);
        self.cells[layer][idx] = state;
    }

    /// Reset grid and mark static obstacles (board edges and footprint pads).
    fn reset_and_mark_static_obstacles(
        &mut self,
        footprints: &[Footprint],
        config: &RouterConfig,
    ) {
        // Clear all cells and reset pad distance buffers
        for layer in 0..NUM_LAYERS {
            self.cells[layer].fill(CellState::Free);
            self.pad_dist[layer].fill(f32::INFINITY);
        }

        // 1. Mark Board Edge Keepout (0.40mm)
        let half_w = config.default_trace_width / 2.0;
        let edge_margin = MIN_EDGE_KEEPOUT + half_w;

        let (margin_gx, margin_gy) = self.world_to_grid(Point::new(edge_margin, edge_margin));
        for gy in 0..self.ny {
            for gx in 0..self.nx {
                let igx = gx as isize;
                let igy = gy as isize;
                if igx < margin_gx
                    || igx >= (self.nx as isize - margin_gx)
                    || igy < margin_gy
                    || igy >= (self.ny as isize - margin_gy)
                {
                    self.set_cell(LAYER_FCU, gx, gy, CellState::EdgeKeepout);
                    self.set_cell(LAYER_BCU, gx, gy, CellState::EdgeKeepout);
                }
            }
        }

        // 2. Mark Pad Obstacles using Voronoi nearest-pad partitioning
        for fp in footprints {
            for pad in &fp.pads {
                let pos = pad.global_position(fp.position, fp.rotation);
                let pad_net = pad.net.unwrap_or(0);
                let inflation = config.default_trace_width / 2.0 + config.clearance;

                match pad.shape {
                    PadShape::Circle { diameter } => {
                        let rad = diameter / 2.0;
                        let min_p = Point::new(pos.x - rad, pos.y - rad);
                        let max_p = Point::new(pos.x + rad, pos.y + rad);
                        if pad.pad_type.is_through_hole() {
                            self.mark_inflated_box(LAYER_FCU, min_p, max_p, inflation, pos, CellState::Pad { net: pad_net });
                            self.mark_inflated_box(LAYER_BCU, min_p, max_p, inflation, pos, CellState::Pad { net: pad_net });
                        } else if let PadType::Smd { layer } = pad.pad_type {
                            let l = if layer == Layer::FCu { LAYER_FCU } else { LAYER_BCU };
                            self.mark_inflated_box(l, min_p, max_p, inflation, pos, CellState::Pad { net: pad_net });
                        }
                    }
                    PadShape::Rect { .. } | PadShape::Obround { .. } => {
                        let bbox = pad.bounding_box(fp.position, fp.rotation);
                        if pad.pad_type.is_through_hole() {
                            self.mark_inflated_box(LAYER_FCU, bbox.min, bbox.max, inflation, pos, CellState::Pad { net: pad_net });
                            self.mark_inflated_box(LAYER_BCU, bbox.min, bbox.max, inflation, pos, CellState::Pad { net: pad_net });
                        } else if let PadType::Smd { layer } = pad.pad_type {
                            let l = if layer == Layer::FCu { LAYER_FCU } else { LAYER_BCU };
                            self.mark_inflated_box(l, bbox.min, bbox.max, inflation, pos, CellState::Pad { net: pad_net });
                        }
                    }
                }
            }
        }

        // Pass 2: Strictly restore uninflated pad interiors to their respective nets
        for fp in footprints {
            for pad in &fp.pads {
                let pad_net = pad.net.unwrap_or(0);
                let bbox = pad.bounding_box(fp.position, fp.rotation);

                let (gx0, gy0) = self.world_to_grid(bbox.min);
                let (gx1, gy1) = self.world_to_grid(bbox.max);

                let gx0 = gx0.clamp(0, self.nx as isize - 1) as usize;
                let gx1 = gx1.clamp(0, self.nx as isize - 1) as usize;
                let gy0 = gy0.clamp(0, self.ny as isize - 1) as usize;
                let gy1 = gy1.clamp(0, self.ny as isize - 1) as usize;

                let mark_layers: &[usize] = if pad.pad_type.is_through_hole() {
                    &[LAYER_FCU, LAYER_BCU]
                } else if let PadType::Smd { layer } = pad.pad_type {
                    if layer == Layer::FCu { &[LAYER_FCU] } else { &[LAYER_BCU] }
                } else {
                    &[]
                };

                let pad_center = pad.global_position(fp.position, fp.rotation);
                let (closest_gx, closest_gy) = self.world_to_grid(pad_center);

                for &l in mark_layers {
                    for gy in gy0..=gy1 {
                        for gx in gx0..=gx1 {
                            let cell_p = self.grid_to_world(gx, gy);
                            let is_closest = gx as isize == closest_gx && gy as isize == closest_gy;
                            let is_inside = cell_p.x >= bbox.min.x - 1e-5
                                && cell_p.x <= bbox.max.x + 1e-5
                                && cell_p.y >= bbox.min.y - 1e-5
                                && cell_p.y <= bbox.max.y + 1e-5;

                            if is_inside || is_closest {
                                let idx = self.idx(gx, gy);
                                if is_closest || self.cells[l][idx] != (CellState::Pad { net: 0 }) {
                                    self.pad_dist[l][idx] = 0.0;
                                    self.cells[l][idx] = CellState::Pad { net: pad_net };
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// Mark an inflated bounding box as occupied using distance-based priority.
    fn mark_inflated_box(
        &mut self,
        layer: usize,
        min_p: Point,
        max_p: Point,
        inflation: f64,
        center: Point,
        state: CellState,
    ) {
        let min_x = (min_p.x - inflation).max(0.0);
        let max_x = (max_p.x + inflation).min(self.width);
        let min_y = (min_p.y - inflation).max(0.0);
        let max_y = (max_p.y + inflation).min(self.height);

        let (gx0, gy0) = self.world_to_grid(Point::new(min_x, min_y));
        let (gx1, gy1) = self.world_to_grid(Point::new(max_x, max_y));

        let gx0 = gx0.clamp(0, self.nx as isize - 1) as usize;
        let gx1 = gx1.clamp(0, self.nx as isize - 1) as usize;
        let gy0 = gy0.clamp(0, self.ny as isize - 1) as usize;
        let gy1 = gy1.clamp(0, self.ny as isize - 1) as usize;

        for gy in gy0..=gy1 {
            for gx in gx0..=gx1 {
                let cell_p = self.grid_to_world(gx, gy);
                let dx = (min_p.x - cell_p.x).max(0.0).max(cell_p.x - max_p.x);
                let dy = (min_p.y - cell_p.y).max(0.0).max(cell_p.y - max_p.y);
                let dist_to_pad = (dx * dx + dy * dy).sqrt();
                if dist_to_pad > inflation + 1e-5 {
                    continue;
                }
                let d = cell_p.distance_to(center) as f32;
                let idx = self.idx(gx, gy);
                match self.cells[layer][idx] {
                    CellState::Free => {
                        self.pad_dist[layer][idx] = d;
                        self.cells[layer][idx] = state;
                    }
                    CellState::Pad { net: existing_net } => {
                        if let CellState::Pad { net: new_net } = state {
                            if existing_net != new_net {
                                // Cell is within clearance inflation zone of different nets:
                                // strictly impassable to all nets to avoid clearance/short violations
                                self.cells[layer][idx] = CellState::Pad { net: 0 };
                                self.pad_dist[layer][idx] = 0.0;
                            } else if d < self.pad_dist[layer][idx] {
                                self.pad_dist[layer][idx] = d;
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    /// Mark an inflated line segment (track) as occupied.
    fn mark_inflated_segment(
        &mut self,
        layer: usize,
        p1: Point,
        p2: Point,
        width: f64,
        clearance: f64,
        net: NetId,
    ) {
        let inflation = width + clearance;
        let seg = TrackSegment::new(p1, p2, width, Layer::FCu, net);
        let bbox = seg.bounding_box();

        let min_x = (bbox.min.x - clearance - width / 2.0).max(0.0);
        let max_x = (bbox.max.x + clearance + width / 2.0).min(self.width);
        let min_y = (bbox.min.y - clearance - width / 2.0).max(0.0);
        let max_y = (bbox.max.y + clearance + width / 2.0).min(self.height);

        let (gx0, gy0) = self.world_to_grid(Point::new(min_x, min_y));
        let (gx1, gy1) = self.world_to_grid(Point::new(max_x, max_y));

        let gx0 = gx0.clamp(0, self.nx as isize - 1) as usize;
        let gx1 = gx1.clamp(0, self.nx as isize - 1) as usize;
        let gy0 = gy0.clamp(0, self.ny as isize - 1) as usize;
        let gy1 = gy1.clamp(0, self.ny as isize - 1) as usize;

        for gy in gy0..=gy1 {
            for gx in gx0..=gx1 {
                let cell_p = self.grid_to_world(gx, gy);
                if seg.distance_to_point(cell_p) <= inflation {
                    self.set_cell(layer, gx, gy, CellState::Track { net });
                }
            }
        }
    }

    /// Mark an inflated via as occupied on both copper layers.
    fn mark_inflated_via(&mut self, at: Point, size: f64, clearance: f64, trace_width: f64, net: NetId) {
        let inflation = size / 2.0 + clearance + trace_width / 2.0;
        let (gx0, gy0) = self.world_to_grid(Point::new(at.x - inflation, at.y - inflation));
        let (gx1, gy1) = self.world_to_grid(Point::new(at.x + inflation, at.y + inflation));

        let gx0 = gx0.clamp(0, self.nx as isize - 1) as usize;
        let gx1 = gx1.clamp(0, self.nx as isize - 1) as usize;
        let gy0 = gy0.clamp(0, self.ny as isize - 1) as usize;
        let gy1 = gy1.clamp(0, self.ny as isize - 1) as usize;

        for gy in gy0..=gy1 {
            for gx in gx0..=gx1 {
                let cell_p = self.grid_to_world(gx, gy);
                if at.distance_to(cell_p) <= inflation {
                    self.set_cell(LAYER_FCU, gx, gy, CellState::Via { net });
                    self.set_cell(LAYER_BCU, gx, gy, CellState::Via { net });
                }
            }
        }
    }

    /// Verify if a via can be placed at (gx, gy) for current_net.
    fn can_place_via(
        &self,
        gx: isize,
        gy: isize,
        via_radius: f64,
        clearance: f64,
        trace_width: f64,
        current_net: NetId,
        negotiated: bool,
    ) -> (bool, HashSet<NetId>) {
        let center = Point::new(
            gx as f64 * self.resolution,
            gy as f64 * self.resolution,
        );

        let safe_radius = via_radius + clearance + trace_width / 2.0;
        let mut conflicts = HashSet::new();

        // Board edge keepout
        if center.x - safe_radius < MIN_EDGE_KEEPOUT
            || center.x + safe_radius > self.width - MIN_EDGE_KEEPOUT
            || center.y - safe_radius < MIN_EDGE_KEEPOUT
            || center.y + safe_radius > self.height - MIN_EDGE_KEEPOUT
        {
            return (false, conflicts);
        }

        let radius_cells = (safe_radius / self.resolution).ceil() as isize;
        for dy in -radius_cells..=radius_cells {
            for dx in -radius_cells..=radius_cells {
                let check_x = gx + dx;
                let check_y = gy + dy;
                if !self.in_bounds(check_x, check_y) {
                    return (false, conflicts);
                }
                let pt = self.grid_to_world(check_x as usize, check_y as usize);
                if center.distance_to(pt) <= safe_radius {
                    for layer in 0..NUM_LAYERS {
                        match self.get_cell(layer, check_x as usize, check_y as usize) {
                            CellState::Free => {}
                            CellState::EdgeKeepout => return (false, HashSet::new()),
                            CellState::Pad { net } => {
                                if net != current_net {
                                    return (false, HashSet::new()); // Never place via on foreign pad
                                }
                            }
                            CellState::Track { net } | CellState::Via { net } => {
                                if net != current_net {
                                    if !negotiated {
                                        return (false, HashSet::new());
                                    }
                                    conflicts.insert(net);
                                }
                            }
                        }
                    }
                }
            }
        }
        (true, conflicts)
    }

    /// Evaluate cell passability and conflict detection for A*.
    #[inline]
    fn evaluate_cell(
        &self,
        layer: usize,
        gx: isize,
        gy: isize,
        current_net: NetId,
        negotiated: bool,
    ) -> Option<(f64, Option<NetId>)> {
        if !self.in_bounds(gx, gy) {
            return None;
        }
        let ux = gx as usize;
        let uy = gy as usize;
        match self.get_cell(layer, ux, uy) {
            CellState::Free => Some((0.0, None)),
            CellState::EdgeKeepout => None,
            CellState::Pad { net } => {
                if net == current_net {
                    Some((0.0, None))
                } else {
                    None // Foreign pads are strictly impassable
                }
            }
            CellState::Track { net } | CellState::Via { net } => {
                if net == current_net {
                    Some((0.0, None))
                } else if negotiated {
                    Some((50.0, Some(net))) // Traversable in negotiated mode with conflict penalty
                } else {
                    None
                }
            }
        }
    }
}

// ============================================================================
// A* Pathfinding Data Structures
// ============================================================================

#[derive(Clone, Copy, PartialEq)]
struct AStarState {
    cost: f64,
    estimated_total: f64,
    gx: isize,
    gy: isize,
    layer: usize,
    prev_dir: Option<(isize, isize)>,
}

impl Eq for AStarState {}

impl Ord for AStarState {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap
        other
            .estimated_total
            .partial_cmp(&self.estimated_total)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for AStarState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct GridNode {
    gx: isize,
    gy: isize,
    layer: usize,
}

// 8 Octilinear Planar Steps
const OCTILINEAR_STEPS: [(isize, isize); 8] = [
    (1, 0),   // Horizontal Right
    (-1, 0),  // Horizontal Left
    (0, 1),   // Vertical Up
    (0, -1),  // Vertical Down
    (1, 1),   // 45° Up-Right
    (1, -1),  // 45° Down-Right
    (-1, 1),  // 45° Up-Left
    (-1, -1), // 45° Down-Left
];

/// Computes admissible octile distance heuristic with via cost.
#[inline]
fn octile_heuristic(
    p1: Point,
    p2: Point,
    l1: usize,
    l2: usize,
    via_cost: f64,
) -> f64 {
    let dx = (p1.x - p2.x).abs();
    let dy = (p1.y - p2.y).abs();
    let min_d = dx.min(dy);
    let max_d = dx.max(dy);
    let planar = (max_d - min_d) + std::f64::consts::SQRT_2 * min_d;
    let via = if l1 != l2 { via_cost } else { 0.0 };
    planar + via
}

// ============================================================================
// Core Autorouter Implementation
// ============================================================================

#[derive(Clone)]
struct PadEndpoint {
    pos: Point,
    layer: usize, // 0 for FCu, 1 for BCu
    is_tht: bool,
}

struct ActiveRoute {
    tracks: Vec<TrackSegment>,
    vias: Vec<Via>,
}

/// Automated 2-layer router connecting netlist pins across F.Cu and B.Cu.
pub fn route_board(board: &mut PcbBoard, config: &RouterConfig) -> Result<(), RoutingError> {
    route_board_with_report(board, config).map(|_| ())
}

/// Automated 2-layer router returning detailed routing report.
pub fn route_board_with_report(
    board: &mut PcbBoard,
    config: &RouterConfig,
) -> Result<RoutingReport, RoutingError> {
    let resolution = config.grid_resolution;
    config.validate()?;
    if ![board.width, board.height].iter().all(|x| x.is_finite() && *x > 0.0)
        || (board.width/resolution+1.0)*(board.height/resolution+1.0)>4_000_000.0 {
        return Err(RoutingError::InvalidConfiguration {
            details: "Dimensions must be finite and positive, drill smaller than via, clearance nonnegative, and grid below four million cells".to_string(),
        });
    }

    // 1. Initialize Grid and Mark Static Obstacles
    let mut grid = RoutingGrid::new(board.width, board.height, resolution);
    grid.reset_and_mark_static_obstacles(&board.footprints, config);

    // Preserve the previous routing until a complete replacement succeeds.

    // 2. Collect Nets to Route
    let mut net_pins: HashMap<NetId, Vec<PadEndpoint>> = HashMap::new();
    for fp in &board.footprints {
        for pad in &fp.pads {
            if let Some(net) = pad.net {
                if net != UNCONNECTED_NET {
                    let pos = pad.global_position(fp.position, fp.rotation);
                    let is_tht = pad.pad_type.is_through_hole();
                    let layer = match pad.pad_type {
                        PadType::Smd { layer } => {
                            if layer == Layer::BCu { LAYER_BCU } else { LAYER_FCU }
                        }
                        PadType::ThroughHole { .. } => LAYER_FCU,
                    };
                    net_pins.entry(net).or_default().push(PadEndpoint {
                        pos,
                        layer,
                        is_tht,
                    });
                }
            }
        }
    }

    let mut routable_net_ids: Vec<NetId> = net_pins
        .iter()
        .filter(|(_, pins)| pins.len() >= 2)
        .map(|(&net_id, _)| net_id)
        .collect();

    // Sort nets: route complex (many-pin) nets first — they have fewer alternative
    // paths and are harder to rip-up and reroute. Short simple nets are easy to
    // squeeze in later. This matches the strategy of commercial autorouters.
    routable_net_ids.sort_by(|&a, &b| {
        let pa = &net_pins[&a];
        let pb = &net_pins[&b];
        // Primary: more pins first (harder to reroute)
        pb.len().cmp(&pa.len()).then_with(|| {
            // Secondary: longer bounding span first (claims more grid space)
            let bbox_span = |pins: &[PadEndpoint]| -> f64 {
                let (mut min_x, mut max_x) = (f64::INFINITY, f64::NEG_INFINITY);
                let (mut min_y, mut max_y) = (f64::INFINITY, f64::NEG_INFINITY);
                for p in pins {
                    min_x = min_x.min(p.pos.x);
                    max_x = max_x.max(p.pos.x);
                    min_y = min_y.min(p.pos.y);
                    max_y = max_y.max(p.pos.y);
                }
                (max_x - min_x) + (max_y - min_y)
            };
            bbox_span(pb).partial_cmp(&bbox_span(pa)).unwrap_or(Ordering::Equal).then_with(|| a.cmp(&b))
        })
    });

    let total_routable = routable_net_ids.len();
    let mut active_routes: std::collections::BTreeMap<NetId, ActiveRoute> = std::collections::BTreeMap::new();
    let mut unrouted_queue: Vec<NetId> = routable_net_ids;
    let mut history_cost: HashMap<NetId, f64> = HashMap::new();

    const MAX_RRR_ITERATIONS: usize = 10;
    let mut rrr_iteration = 0;

    // 3. Rip-Up and Reroute (RRR) Execution Loop
    while !unrouted_queue.is_empty() && rrr_iteration < MAX_RRR_ITERATIONS {
        let current_round = std::mem::take(&mut unrouted_queue);

        for net_id in current_round {
            let pins = &net_pins[&net_id];

            // Try standard A* routing (no conflict allowed)
            let standard_attempt = route_multi_pin_net(
                &grid,
                pins,
                net_id,
                config,
                false,
                &history_cost,
            );

            match standard_attempt {
                Ok((tracks, vias, _)) => {
                    // Mark grid with new route
                    for t in &tracks {
                        let l = if t.layer == Layer::FCu { LAYER_FCU } else { LAYER_BCU };
                        grid.mark_inflated_segment(l, t.start, t.end, t.width, config.clearance, net_id);
                    }
                    for v in &vias {
                        grid.mark_inflated_via(v.at, v.size, config.clearance, config.default_trace_width, net_id);
                    }
                    active_routes.insert(net_id, ActiveRoute { tracks, vias });
                }
                Err(_) => {
                    // Standard routing blocked -> attempt negotiated A* with RRR
                    let negotiated_attempt = route_multi_pin_net(
                        &grid,
                        pins,
                        net_id,
                        config,
                        true,
                        &history_cost,
                    );

                    match negotiated_attempt {
                        Ok((tracks, vias, conflicts)) => {
                            // Rip up conflicting nets
                            for ripped_net in conflicts {
                                if ripped_net != net_id && active_routes.contains_key(&ripped_net) {
                                    active_routes.remove(&ripped_net);
                                    if !unrouted_queue.contains(&ripped_net) {
                                        unrouted_queue.push(ripped_net);
                                    }
                                    *history_cost.entry(ripped_net).or_insert(0.0) += 1.0;
                                }
                            }

                            // Commit this net
                            active_routes.insert(net_id, ActiveRoute { tracks, vias });

                            // Rebuild grid with static obstacles + currently active routes
                            grid.reset_and_mark_static_obstacles(&board.footprints, config);
                            for (&active_net, route) in &active_routes {
                                for t in &route.tracks {
                                    let l = if t.layer == Layer::FCu { LAYER_FCU } else { LAYER_BCU };
                                    grid.mark_inflated_segment(l, t.start, t.end, t.width, config.clearance, active_net);
                                }
                                for v in &route.vias {
                                    grid.mark_inflated_via(v.at, v.size, config.clearance, config.default_trace_width, active_net);
                                }
                            }
                        }
                        Err(err) => {
                            return Err(err);
                        }
                    }
                }
            }
        }

        rrr_iteration += 1;
    }

    if !unrouted_queue.is_empty() {
        return Err(RoutingError::BoardTooDense {
            details: format!(
                "Failed to route nets {:?} after {} RRR iterations",
                unrouted_queue, MAX_RRR_ITERATIONS
            ),
        });
    }

    // 4. Commit all routed tracks and vias to board
    board.tracks.clear();
    board.vias.clear();
    let mut total_tracks = 0;

    for (_, route) in active_routes {
        total_tracks += route.tracks.len();

        for t in route.tracks {
            board.add_track(t);
        }
        for v in route.vias {
            board.add_via(v);
        }
    }

    board.vias.sort_by(|a,b| a.net.cmp(&b.net).then(a.at.x.total_cmp(&b.at.x)).then(a.at.y.total_cmp(&b.at.y)));
    board.vias.dedup_by(|a,b| a.net == b.net && a.at.distance_to(b.at) < 1e-6);
    let total_vias = board.vias.len();
    Ok(RoutingReport {
        routed_nets: total_routable,
        unrouted_nets: 0,
        total_tracks,
        total_vias,
        rrr_iterations: rrr_iteration,
    })
}

/// Route all pins of a multi-pin net sequentially.
fn route_multi_pin_net(
    grid: &RoutingGrid,
    pins: &[PadEndpoint],
    net_id: NetId,
    config: &RouterConfig,
    negotiated: bool,
    history_cost: &HashMap<NetId, f64>,
) -> Result<(Vec<TrackSegment>, Vec<Via>, HashSet<NetId>), RoutingError> {
    let mut all_tracks = Vec::new();
    let mut all_vias = Vec::new();
    let mut all_conflicts = HashSet::new();

    let mut connected_pins = vec![0];
    let mut remaining_pins: Vec<usize> = (1..pins.len()).collect();

    while !remaining_pins.is_empty() {
        let mut best_c = connected_pins[0];
        let mut best_r_idx = 0;
        let mut min_d = f64::INFINITY;

        for &c in &connected_pins {
            for (r_idx, &r) in remaining_pins.iter().enumerate() {
                let d = pins[c].pos.distance_to(pins[r].pos);
                if d < min_d {
                    min_d = d;
                    best_c = c;
                    best_r_idx = r_idx;
                }
            }
        }

        let start_pin = &pins[best_c];
        let target_pin = &pins[remaining_pins[best_r_idx]];
        let target_idx = remaining_pins.remove(best_r_idx);
        connected_pins.push(target_idx);

        let (path, conflicts) = route_a_star(
            grid,
            start_pin.pos,
            start_pin.layer,
            start_pin.is_tht,
            target_pin.pos,
            target_pin.layer,
            target_pin.is_tht,
            net_id,
            config,
            negotiated,
            history_cost,
        )?;

        let (tracks, vias) = build_tracks_and_vias(&path, net_id, config, grid.resolution);
        all_tracks.extend(tracks);
        all_vias.extend(vias);
        all_conflicts.extend(conflicts);
    }

    Ok((all_tracks, all_vias, all_conflicts))
}

/// 3D A* Maze Pathfinder supporting 45°/90° octilinear tracks, layer bias, via insertion, and RRR.
fn route_a_star(
    grid: &RoutingGrid,
    start: Point,
    start_layer: usize,
    start_is_tht: bool,
    goal: Point,
    goal_layer: usize,
    goal_is_tht: bool,
    net_id: NetId,
    config: &RouterConfig,
    negotiated: bool,
    history_cost: &HashMap<NetId, f64>,
) -> Result<(Vec<GridNode>, HashSet<NetId>), RoutingError> {
    let res = grid.resolution;
    let (sx, sy) = grid.world_to_grid(start);
    let (gx, gy) = grid.world_to_grid(goal);

    let via_radius = config.via_diameter / 2.0;
    let via_cost = 4.0 * res; // Cost penalty for layer transition

    let mut open_set = BinaryHeap::new();
    let mut g_scores: HashMap<GridNode, f64> = HashMap::new();
    let mut came_from: HashMap<GridNode, GridNode> = HashMap::new();
    let mut node_conflicts: HashMap<GridNode, HashSet<NetId>> = HashMap::new();

    // Start nodes: if through-hole, start can begin on either layer
    let initial_layers = if start_is_tht {
        vec![LAYER_FCU, LAYER_BCU]
    } else {
        vec![start_layer]
    };

    for &layer in &initial_layers {
        let node = GridNode { gx: sx, gy: sy, layer };
        let h = octile_heuristic(
            grid.grid_to_world(sx as usize, sy as usize),
            goal,
            layer,
            goal_layer,
            via_cost,
        );
        g_scores.insert(node, 0.0);
        node_conflicts.insert(node, HashSet::new());
        open_set.push(AStarState {
            cost: 0.0,
            estimated_total: h,
            gx: sx,
            gy: sy,
            layer,
            prev_dir: None,
        });
    }

    let mut target_reached: Option<GridNode> = None;

    while let Some(current) = open_set.pop() {
        let current_node = GridNode {
            gx: current.gx,
            gy: current.gy,
            layer: current.layer,
        };

        // Goal check
        let is_at_goal = current.gx == gx && current.gy == gy;
        let layer_matches = current.layer == goal_layer || goal_is_tht;

        if is_at_goal && layer_matches {
            target_reached = Some(current_node);
            break;
        }

        let current_g = match g_scores.get(&current_node) {
            Some(&score) => score,
            None => continue,
        };

        if current.cost > current_g + 1e-6 {
            continue; // Suboptimal node in queue
        }

        let current_conflicts = node_conflicts
            .get(&current_node)
            .cloned()
            .unwrap_or_default();

        // 1. Planar Octilinear Steps on Current Layer
        for &(dx, dy) in &OCTILINEAR_STEPS {
            let nx = current.gx + dx;
            let ny = current.gy + dy;

            let (penalty, conflict_net) = match grid.evaluate_cell(current.layer, nx, ny, net_id, negotiated) {
                Some(res) => res,
                None => continue,
            };

            // Corner-Pinching Avoidance:
            // For diagonal steps, both adjacent orthogonal cells must be traversable
            if dx != 0 && dy != 0 {
                let ortho1 = grid.evaluate_cell(current.layer, current.gx + dx, current.gy, net_id, negotiated);
                let ortho2 = grid.evaluate_cell(current.layer, current.gx, current.gy + dy, net_id, negotiated);
                if ortho1.is_none() || ortho2.is_none() {
                    continue; // Skip corner pinching!
                }
            }

            // Anisotropic Layer Biasing:
            // F.Cu (Layer 0) prefers horizontal (dx != 0, dy == 0)
            // B.Cu (Layer 1) prefers vertical (dx == 0, dy != 0)
            let base_dist = if dx != 0 && dy != 0 {
                std::f64::consts::SQRT_2 * res
            } else {
                res
            };

            let bias_multiplier = match current.layer {
                LAYER_FCU => {
                    if dy == 0 {
                        1.0 // Horizontal is preferred on F.Cu
                    } else if dx == 0 {
                        2.5 // Vertical penalty on F.Cu
                    } else {
                        1.75 // 45° diagonal intermediate
                    }
                }
                LAYER_BCU => {
                    if dx == 0 {
                        1.0 // Vertical is preferred on B.Cu
                    } else if dy == 0 {
                        2.5 // Horizontal penalty on B.Cu
                    } else {
                        1.75 // 45° diagonal intermediate
                    }
                }
                _ => 1.0,
            };

            // Bend penalty to encourage long straight segments
            let turn_penalty = match current.prev_dir {
                Some(prev) if prev != (dx, dy) => 0.1 * res,
                _ => 0.0,
            };

            // History cost multiplier for negotiated routing
            let history_mul = match conflict_net {
                Some(c_net) => 1.0 + history_cost.get(&c_net).copied().unwrap_or(0.0),
                None => 1.0,
            };

            let step_cost = base_dist * bias_multiplier + turn_penalty + penalty * history_mul;
            let tentative_g = current_g + step_cost;

            let neighbor_node = GridNode {
                gx: nx,
                gy: ny,
                layer: current.layer,
            };

            if tentative_g < *g_scores.get(&neighbor_node).unwrap_or(&f64::INFINITY) {
                g_scores.insert(neighbor_node, tentative_g);
                came_from.insert(neighbor_node, current_node);

                let mut next_conflicts = current_conflicts.clone();
                if let Some(c_net) = conflict_net {
                    next_conflicts.insert(c_net);
                }
                node_conflicts.insert(neighbor_node, next_conflicts);

                let cell_world = grid.grid_to_world(nx as usize, ny as usize);
                let h = octile_heuristic(
                    cell_world,
                    goal,
                    current.layer,
                    goal_layer,
                    via_cost,
                );

                open_set.push(AStarState {
                    cost: tentative_g,
                    estimated_total: tentative_g + h,
                    gx: nx,
                    gy: ny,
                    layer: current.layer,
                    prev_dir: Some((dx, dy)),
                });
            }
        }

        // 2. Layer Transition via Automated Via Insertion
        let other_layer = 1 - current.layer;
        let (can_via, via_conflicts) = grid.can_place_via(
            current.gx,
            current.gy,
            via_radius,
            config.clearance,
            config.default_trace_width,
            net_id,
            negotiated,
        );

        if can_via {
            let via_node = GridNode {
                gx: current.gx,
                gy: current.gy,
                layer: other_layer,
            };

            let mut via_penalty = 0.0;
            for &c_net in &via_conflicts {
                let h_cost = history_cost.get(&c_net).copied().unwrap_or(0.0);
                via_penalty += 50.0 * (1.0 + h_cost);
            }

            let tentative_g = current_g + via_cost + via_penalty;

            if tentative_g < *g_scores.get(&via_node).unwrap_or(&f64::INFINITY) {
                g_scores.insert(via_node, tentative_g);
                came_from.insert(via_node, current_node);

                let mut next_conflicts = current_conflicts.clone();
                next_conflicts.extend(via_conflicts);
                node_conflicts.insert(via_node, next_conflicts);

                let cell_world = grid.grid_to_world(current.gx as usize, current.gy as usize);
                let h = octile_heuristic(
                    cell_world,
                    goal,
                    other_layer,
                    goal_layer,
                    via_cost,
                );

                open_set.push(AStarState {
                    cost: tentative_g,
                    estimated_total: tentative_g + h,
                    gx: current.gx,
                    gy: current.gy,
                    layer: other_layer,
                    prev_dir: None, // Reset direction after via
                });
            }
        }
    }

    let end_node = match target_reached {
        Some(node) => node,
        None => {
            return Err(RoutingError::NoPathFound {
                net_id,
                start,
                end: goal,
            });
        }
    };

    let total_conflicts = node_conflicts.remove(&end_node).unwrap_or_default();

    // Reconstruct Path
    let mut path = Vec::new();
    let mut curr = end_node;
    path.push(curr);
    while let Some(&prev) = came_from.get(&curr) {
        path.push(prev);
        curr = prev;
    }
    path.reverse();

    Ok((path, total_conflicts))
}

/// Consolidates raw grid steps into long collinear track segments and vias.
fn build_tracks_and_vias(
    path: &[GridNode],
    net_id: NetId,
    config: &RouterConfig,
    resolution: f64,
) -> (Vec<TrackSegment>, Vec<Via>) {
    let mut tracks = Vec::new();
    let mut vias = Vec::new();

    if path.is_empty() {
        return (tracks, vias);
    }

    let mut seg_start = Point::new(
        path[0].gx as f64 * resolution,
        path[0].gy as f64 * resolution,
    );
    let mut seg_layer = if path[0].layer == LAYER_FCU {
        Layer::FCu
    } else {
        Layer::BCu
    };
    let mut current_dir: Option<(isize, isize)> = None;

    for i in 1..path.len() {
        let prev = path[i - 1];
        let curr = path[i];

        // Via insertion check
        if prev.layer != curr.layer {
            // Finalize any ongoing track segment before via
            let via_pos = Point::new(
                prev.gx as f64 * resolution,
                prev.gy as f64 * resolution,
            );
            if seg_start.distance_to(via_pos) > 1e-5 {
                tracks.push(TrackSegment::new(
                    seg_start,
                    via_pos,
                    config.default_trace_width,
                    seg_layer,
                    net_id,
                ));
            }

            // Insert Via
            vias.push(Via::new(
                via_pos,
                config.via_diameter,
                config.via_drill,
                net_id,
            ));

            // Reset segment start after via
            seg_start = Point::new(
                curr.gx as f64 * resolution,
                curr.gy as f64 * resolution,
            );
            seg_layer = if curr.layer == LAYER_FCU {
                Layer::FCu
            } else {
                Layer::BCu
            };
            current_dir = None;
            continue;
        }

        // Planar step
        let dir = (curr.gx - prev.gx, curr.gy - prev.gy);
        match current_dir {
            None => {
                current_dir = Some(dir);
            }
            Some(prev_dir) if prev_dir == dir => {
                // Same direction: continue extending current segment
            }
            Some(_) => {
                // Direction changed: emit previous segment and start new one
                let seg_end = Point::new(
                    prev.gx as f64 * resolution,
                    prev.gy as f64 * resolution,
                );
                if seg_start.distance_to(seg_end) > 1e-5 {
                    tracks.push(TrackSegment::new(
                        seg_start,
                        seg_end,
                        config.default_trace_width,
                        seg_layer,
                        net_id,
                    ));
                }
                seg_start = seg_end;
                current_dir = Some(dir);
            }
        }
    }

    // Emit final segment
    let final_pos = Point::new(
        path.last().unwrap().gx as f64 * resolution,
        path.last().unwrap().gy as f64 * resolution,
    );
    if seg_start.distance_to(final_pos) > 1e-5 {
        tracks.push(TrackSegment::new(
            seg_start,
            final_pos,
            config.default_trace_width,
            seg_layer,
            net_id,
        ));
    }

    (tracks, vias)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_router_config_validation_rejects_non_positive_values() {
        let mut board = PcbBoard::new(50.0, 50.0);
        let config_zero_res = RouterConfig {
            grid_resolution: 0.0,
            ..Default::default()
        };
        let err = route_board(&mut board, &config_zero_res).unwrap_err();
        assert!(matches!(err, RoutingError::InvalidConfiguration { .. }));

        let config_neg_width = RouterConfig {
            default_trace_width: -0.25,
            ..Default::default()
        };
        let err2 = route_board(&mut board, &config_neg_width).unwrap_err();
        assert!(matches!(err2, RoutingError::InvalidConfiguration { .. }));
    }

    #[test]
    fn test_route_board_with_report_returns_accurate_metrics() {
        let mut board = PcbBoard::new(50.0, 50.0);
        board.register_net(1, "NET1");
        let p1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
        let p2 = Pad::new_smd("2", "B", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(1);
        board.add_footprint(Footprint::new("U1", "P", "P", Point::new(10.0, 20.0), 0.0, vec![p1], vec![]));
        board.add_footprint(Footprint::new("U2", "P", "P", Point::new(30.0, 20.0), 0.0, vec![p2], vec![]));

        let config = RouterConfig::default();
        let report = route_board_with_report(&mut board, &config).expect("Routing should succeed");
        assert_eq!(report.routed_nets, 1);
        assert_eq!(report.unrouted_nets, 0);
        assert!(report.total_tracks >= 1);
        assert_eq!(report.total_vias, 0);
        assert_eq!(report.rrr_iterations, 1);
    }

    #[test]
    fn test_single_pin_net_is_ignored_by_router() {
        let mut board = PcbBoard::new(50.0, 50.0);
        board.register_net(10, "SOLO_PIN");
        let p1 = Pad::new_smd("1", "A", Point::new(0.0, 0.0), PadShape::Rect { width: 1.0, height: 1.0 }, Layer::FCu).with_net(10);
        board.add_footprint(Footprint::new("U1", "P", "P", Point::new(10.0, 20.0), 0.0, vec![p1], vec![]));

        let config = RouterConfig::default();
        let report = route_board_with_report(&mut board, &config).expect("Single-pin net should succeed with 0 routes");
        assert_eq!(report.routed_nets, 0);
        assert_eq!(board.tracks.len(), 0);
    }
}
