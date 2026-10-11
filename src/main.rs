use macroquad::prelude::*;
use spectral_pruner::{TauSpectralPruner, Topology};
use std::collections::{HashSet, HashMap};
use std::time::Instant;
use netlist_shatter_viz::pcb::*;

// ==========================================================================
// 1. TOPOLOGY CORES & TYPES
// ==========================================================================

#[derive(Clone, Copy, PartialEq, Debug, Eq, Hash)]
enum Partition {
    Unassigned,
    Index(usize),
}

#[derive(Clone, Copy, Debug)]
enum CircuitType {
    AluGrid,
    MeshInterconnect,
    StarClockTree,
    RippleCarryAdder8,
    ArrayMultiplier4,
    PipelineCore,
    RiscVCore,
}

#[derive(Clone, Copy, PartialEq, Debug)]
#[allow(dead_code)]
enum GateType {
    And,
    Or,
    Xor,
    Not,
    Nand,
    Nor,
    Xnor,
    Dff,
    Mux,
    Latch,
    Buffer,
    Oscillator,
}

struct VisualGate {
    id: usize,
    pos: Vec2,
    vel: Vec2,
    partition: Partition,
    gate_type: GateType,
    label: String,
}

struct Particle {
    pos: Vec2,
    vel: Vec2,
    color: Color,
    lifetime: f32,
    max_lifetime: f32,
    size: f32,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ViewMode {
    GraphSimulation,
    MatrixHeatmap,
    PcbLayout,
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum PlacementMode {
    Organic,
    SiliconRow,
}
struct Telemetry {
    gate_count: usize,
    edge_count: usize,
    execution_time_us: u128,
    cross_wire_count: usize,
    is_partitioned: bool,
    current_circuit: CircuitType,
    placement_mode: PlacementMode,
    community_count: usize,
}


// Custom Graph structure acting as an adjacency-list cache and builder for spectral-pruner's Topology
#[derive(Clone)]
struct Graph {
    neighbors: Vec<HashSet<usize>>,
    edges: Vec<(usize, usize)>,
}

impl Graph {
    fn new() -> Self {
        Self {
            neighbors: Vec::new(),
            edges: Vec::new(),
        }
    }

    fn ensure_node(&mut self, id: usize) {
        if id >= self.neighbors.len() {
            self.neighbors.resize(id + 1, HashSet::new());
        }
    }

    fn add_edge(&mut self, u: usize, v: usize) {
        self.ensure_node(u);
        self.ensure_node(v);
        self.neighbors[u].insert(v);
        self.neighbors[v].insert(u);
        self.edges.push((u, v));
    }

    fn get_neighbors(&self, id: usize) -> Option<&HashSet<usize>> {
        self.neighbors.get(id)
    }

    #[allow(dead_code)]
    fn to_topology(&self) -> Topology {
        let num_nodes = self.neighbors.len();
        let mut topo = Topology::new(num_nodes);
        for &(u, v) in &self.edges {
            topo.add_edge(u, v);
        }
        topo
    }
}

// ==========================================================================
// 2. SYNTHETIC & STRUCTURAL HARDWARE GENERATORS
// ==========================================================================

fn generate_circuit(circuit_type: CircuitType) -> (Graph, Vec<VisualGate>) {
    let mut graph = Graph::new();
    let mut gates = Vec::new();
    let center = vec2(screen_width() / 2.0, screen_height() / 2.0);

    match circuit_type {
        CircuitType::AluGrid => {
            // Generates a matrix structure resembling a hardware processing array
            let rows = 12;
            let cols = 12;
            for r in 0..rows {
                for c in 0..cols {
                    let id = r * cols + c;
                    let gate_type = match id % 5 {
                        0 => GateType::And,
                        1 => GateType::Or,
                        2 => GateType::Xor,
                        3 => GateType::Nand,
                        _ => GateType::Nor,
                    };
                    let label = format!("U{}:{}", id, match gate_type {
                        GateType::And => "AND2",
                        GateType::Or => "OR2",
                        GateType::Xor => "XOR2",
                        GateType::Nand => "NAND2",
                        _ => "NOR2",
                    });

                    gates.push(VisualGate {
                        id,
                        pos: center + vec2((c as f32 - 6.0) * 30.0 + rand::gen_range(-10.0, 10.0), (r as f32 - 6.0) * 30.0 + rand::gen_range(-10.0, 10.0)),
                        vel: Vec2::ZERO,
                        partition: Partition::Unassigned,
                        gate_type,
                        label,
                    });

                    // Local grid routing (horizontal and vertical data lines)
                    if c < cols - 1 { graph.add_edge(id, id + 1); }
                    if r < rows - 1 { graph.add_edge(id, id + cols); }
                }
            }
        }
        CircuitType::MeshInterconnect => {
            // Highly clustered multi-core layout bridged by narrow pipelines
            let core_size = 50;
            // Core 0 (Left Block)
            for i in 0..core_size {
                let gate_type = if i % 10 == 0 {
                    GateType::Dff
                } else if i % 5 == 1 {
                    GateType::Mux
                } else {
                    match i % 3 {
                        0 => GateType::And,
                        1 => GateType::Or,
                        _ => GateType::Not,
                    }
                };
                let label = format!("C0_U{}:{}", i, match gate_type {
                    GateType::Dff => "DFF",
                    GateType::Mux => "MUX",
                    GateType::And => "AND2",
                    GateType::Or => "OR2",
                    _ => "INV",
                });

                gates.push(VisualGate {
                    id: i,
                    pos: center + vec2(-150.0 + rand::gen_range(-50.0, 50.0), rand::gen_range(-50.0, 50.0)),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type,
                    label,
                });
                if i > 0 { graph.add_edge(i, i - 1); }
                if i % 5 == 0 && i >= 5 { graph.add_edge(i, i - 5); }
            }
            // Core 1 (Right Block)
            for i in 0..core_size {
                let id = i + core_size;
                let gate_type = if i % 10 == 0 {
                    GateType::Dff
                } else if i % 5 == 1 {
                    GateType::Mux
                } else {
                    match i % 3 {
                        0 => GateType::And,
                        1 => GateType::Or,
                        _ => GateType::Not,
                    }
                };
                let label = format!("C1_U{}:{}", id, match gate_type {
                    GateType::Dff => "DFF",
                    GateType::Mux => "MUX",
                    GateType::And => "AND2",
                    GateType::Or => "OR2",
                    _ => "INV",
                });

                gates.push(VisualGate {
                    id,
                    pos: center + vec2(150.0 + rand::gen_range(-50.0, 50.0), rand::gen_range(-50.0, 50.0)),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type,
                    label,
                });
                if i > 0 { graph.add_edge(id, id - 1); }
                if i % 5 == 0 && i >= 5 { graph.add_edge(id, id - 5); }
            }
            // Minimal Bottleneck Pipeline Wires crossing the physical chip boundaries
            graph.add_edge(0, core_size);
            graph.add_edge(core_size / 2, core_size + (core_size / 2));
        }
        CircuitType::StarClockTree => {
            // Clock tree distribution network with Oscillator, Buffers, and DFF Leaves
            let root_id = 0;
            gates.push(VisualGate {
                id: root_id,
                pos: center + rand::gen_range(-10.0, 10.0),
                vel: Vec2::ZERO,
                partition: Partition::Unassigned,
                gate_type: GateType::Oscillator,
                label: "OSC0".to_string(),
            });

            let branches = 6;
            let leaves_per_branch = 20;
            let mut current_id = 1;

            for b in 0..branches {
                let branch_root = current_id;
                gates.push(VisualGate {
                    id: branch_root,
                    pos: center + vec2(((b as f32 * 60.0).to_radians().cos()) * 100.0, ((b as f32 * 60.0).to_radians().sin()) * 100.0),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::Buffer,
                    label: format!("CLK_BUF{}", b),
                });
                graph.add_edge(root_id, branch_root);
                current_id += 1;

                for leaf in 0..leaves_per_branch {
                    gates.push(VisualGate {
                        id: current_id,
                        pos: gates[branch_root].pos + vec2(rand::gen_range(-40.0, 40.0), rand::gen_range(-40.0, 40.0)),
                        vel: Vec2::ZERO,
                        partition: Partition::Unassigned,
                        gate_type: GateType::Dff,
                        label: format!("FF_{}_{}", b, leaf),
                    });
                    graph.add_edge(branch_root, current_id);
                    current_id += 1;
                }
            }
        }
        CircuitType::RippleCarryAdder8 => {
            // 8-Bit Ripple-Carry Adder Structurally Chained (5 Gates per Full Adder = 40 Gates total)
            for i in 0..8 {
                let stage_x = center.x + (i as f32 - 3.5) * 90.0;
                
                let xor1_id = i * 5 + 0;
                let sum_id = i * 5 + 1;
                let and1_id = i * 5 + 2;
                let and2_id = i * 5 + 3;
                let carry_id = i * 5 + 4;

                // Position the 5 gates of this Full Adder stage in a localized, clear structural cluster
                gates.push(VisualGate {
                    id: xor1_id,
                    pos: vec2(stage_x - 15.0, center.y - 15.0 + rand::gen_range(-5.0, 5.0)),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::Xor,
                    label: format!("FA{}_XOR1", i),
                });
                gates.push(VisualGate {
                    id: sum_id,
                    pos: vec2(stage_x + 15.0, center.y - 15.0 + rand::gen_range(-5.0, 5.0)),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::Xor,
                    label: format!("FA{}_SUM", i),
                });
                gates.push(VisualGate {
                    id: and1_id,
                    pos: vec2(stage_x - 20.0, center.y + 15.0 + rand::gen_range(-5.0, 5.0)),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::And,
                    label: format!("FA{}_AND1", i),
                });
                gates.push(VisualGate {
                    id: and2_id,
                    pos: vec2(stage_x, center.y + 15.0 + rand::gen_range(-5.0, 5.0)),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::And,
                    label: format!("FA{}_AND2", i),
                });
                gates.push(VisualGate {
                    id: carry_id,
                    pos: vec2(stage_x + 20.0, center.y + 15.0 + rand::gen_range(-5.0, 5.0)),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::Or,
                    label: format!("FA{}_OR_COUT", i),
                });

                // Internal connections of a standard Full Adder cell
                graph.add_edge(xor1_id, sum_id);
                graph.add_edge(xor1_id, and2_id);
                graph.add_edge(and1_id, carry_id);
                graph.add_edge(and2_id, carry_id);

                // Chained Carry propagation to the next stage
                if i < 7 {
                    let next_sum_id = (i + 1) * 5 + 1;
                    let next_and2_id = (i + 1) * 5 + 3;
                    graph.add_edge(carry_id, next_sum_id);
                    graph.add_edge(carry_id, next_and2_id);
                }
            }
        }
        CircuitType::ArrayMultiplier4 => {
            // 4x4 Array Multiplier Grid (16 partial products and hierarchical adders = 48 logic cells)
            for r in 0..4 {
                for c in 0..4 {
                    let cell_x = center.x + (c as f32 - 1.5) * 110.0;
                    let cell_y = center.y + (r as f32 - 1.5) * 110.0;

                    let base_id = (r * 4 + c) * 3;
                    let and_id = base_id + 0;
                    let sum_id = base_id + 1;
                    let carry_id = base_id + 2;

                    // Position product and adder nodes clustered in a 2D mesh
                    gates.push(VisualGate {
                        id: and_id,
                        pos: vec2(cell_x - 15.0, cell_y - 15.0 + rand::gen_range(-5.0, 5.0)),
                        vel: Vec2::ZERO,
                        partition: Partition::Unassigned,
                        gate_type: GateType::And,
                        label: format!("P_{}_{}", r, c),
                    });
                    gates.push(VisualGate {
                        id: sum_id,
                        pos: vec2(cell_x + 15.0, cell_y - 15.0 + rand::gen_range(-5.0, 5.0)),
                        vel: Vec2::ZERO,
                        partition: Partition::Unassigned,
                        gate_type: GateType::Xor,
                        label: format!("S_{}_{}", r, c),
                    });
                    gates.push(VisualGate {
                        id: carry_id,
                        pos: vec2(cell_x, cell_y + 15.0 + rand::gen_range(-5.0, 5.0)),
                        vel: Vec2::ZERO,
                        partition: Partition::Unassigned,
                        gate_type: GateType::Or,
                        label: format!("C_{}_{}", r, c),
                    });

                    // Internal cell linkages
                    graph.add_edge(and_id, sum_id);
                    graph.add_edge(and_id, carry_id);

                    // Horizontal Carry-chain links
                    if c < 3 {
                        let right_sum_id = (r * 4 + c + 1) * 3 + 1;
                        graph.add_edge(carry_id, right_sum_id);
                    }
                    // Vertical Sum accumulation links (mesh down-flow)
                    if r < 3 {
                        let bottom_sum_id = ((r + 1) * 4 + c) * 3 + 1;
                        graph.add_edge(sum_id, bottom_sum_id);
                    }
                }
            }
        }
        CircuitType::PipelineCore => {
            // 4-Stage RISC Processor Pipeline Datapath (IF / ID / EX / WB = 56 logic cells)
            // Stage 1: Instruction Fetch (IF) - Nodes 0..16
            let if_center_x = center.x - 240.0;
            // PC register ring (0..4)
            for i in 0..4 {
                gates.push(VisualGate {
                    id: i,
                    pos: vec2(if_center_x + ((i as f32 * 90.0).to_radians().cos()) * 25.0, center.y - 60.0 + ((i as f32 * 90.0).to_radians().sin()) * 25.0),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::Dff,
                    label: format!("PC_REG{}", i),
                });
            }
            graph.add_edge(0, 1);
            graph.add_edge(1, 2);
            graph.add_edge(2, 3);
            graph.add_edge(3, 0);

            // Instruction Memory matrix (4..16)
            for r in 0..3 {
                for c in 0..4 {
                    let id = 4 + r * 4 + c;
                    gates.push(VisualGate {
                        id,
                        pos: vec2(if_center_x - 45.0 + c as f32 * 30.0, center.y + 10.0 + r as f32 * 25.0),
                        vel: Vec2::ZERO,
                        partition: Partition::Unassigned,
                        gate_type: GateType::Latch,
                        label: format!("IMEM_C{}_{}", r, c),
                    });
                    // Grid connections
                    if c < 3 { graph.add_edge(id, id + 1); }
                    if r < 2 { graph.add_edge(id, id + 4); }
                }
            }
            // PC connects to first row of IM
            for i in 0..4 {
                graph.add_edge(i, 4 + i);
            }

            // Stage 2: Instruction Decode (ID) - Nodes 16..32
            let id_center_x = center.x - 80.0;
            // Instruction registers (16..20)
            for i in 0..4 {
                let id = 16 + i;
                gates.push(VisualGate {
                    id,
                    pos: vec2(id_center_x - 30.0, center.y - 70.0 + i as f32 * 25.0),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::Dff,
                    label: format!("IR_REG{}", i),
                });
                // Connect from IM outputs to Instruction Register
                graph.add_edge(12 + i, id);
            }
            // Control Decoder gates (20..24)
            for i in 0..4 {
                let id = 20 + i;
                gates.push(VisualGate {
                    id,
                    pos: vec2(id_center_x, center.y - 70.0 + i as f32 * 25.0),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::And,
                    label: format!("DEC_GATE{}", i),
                });
                graph.add_edge(16 + i, id);
            }
            // Register File memory cells (24..30). 3 cross-coupled latch loops
            for i in 0..3 {
                let node_a = 24 + i * 2;
                let node_b = 25 + i * 2;
                gates.push(VisualGate {
                    id: node_a,
                    pos: vec2(id_center_x + 30.0, center.y + 10.0 + i as f32 * 30.0),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::Latch,
                    label: format!("REG_L{}_A", i),
                });
                gates.push(VisualGate {
                    id: node_b,
                    pos: vec2(id_center_x + 50.0, center.y + 10.0 + i as f32 * 30.0),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::Latch,
                    label: format!("REG_L{}_B", i),
                });
                // Cross-coupled loops representing bi-stable latches
                graph.add_edge(node_a, node_b);
            }
            // Read port selectors / muxes (30..32)
            for i in 0..2 {
                let id = 30 + i;
                gates.push(VisualGate {
                    id,
                    pos: vec2(id_center_x + 40.0, center.y - 50.0 + i as f32 * 30.0),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::Mux,
                    label: format!("REG_MUX{}", i),
                });
                // Read ports connect to register latches and control decoder signals
                graph.add_edge(24 + i, id);
                graph.add_edge(20 + i, id);
            }

            // Stage 3: Execute / ALU (EX) - Nodes 32..48
            let ex_center_x = center.x + 80.0;
            // Operand registers (32..36)
            for i in 0..4 {
                let id = 32 + i;
                gates.push(VisualGate {
                    id,
                    pos: vec2(ex_center_x - 40.0, center.y - 60.0 + i as f32 * 25.0),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::Dff,
                    label: format!("OP_REG{}", i),
                });
                // Connect from ID Stage read muxes & decoder
                graph.add_edge(30 + (i % 2), id);
                graph.add_edge(22 + (i % 2), id);
            }
            // ALU arithmetic core gates (36..44)
            for i in 0..8 {
                let id = 36 + i;
                let gate_type = match i % 3 {
                    0 => GateType::Xor,
                    1 => GateType::And,
                    _ => GateType::Or,
                };
                gates.push(VisualGate {
                    id,
                    pos: vec2(ex_center_x + (i as f32 - 3.5) * 12.0, center.y + 10.0 + rand::gen_range(-10.0, 10.0)),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type,
                    label: format!("ALU_{}", match gate_type {
                        GateType::Xor => "XOR",
                        GateType::And => "AND",
                        _ => "OR",
                    }),
                });
                // Inputs from Operand registers
                graph.add_edge(32 + (i % 4), id);
                if i > 0 { graph.add_edge(id, id - 1); } // Chained ripple/carry logic inside the ALU
            }
            // ALU Status flag gates (44..48)
            for i in 0..4 {
                let id = 44 + i;
                gates.push(VisualGate {
                    id,
                    pos: vec2(ex_center_x + 30.0, center.y - 60.0 + i as f32 * 25.0),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::Latch,
                    label: format!("FLAG_L{}", i),
                });
                graph.add_edge(36 + i * 2, id);
            }

            // Stage 4: Writeback (WB) - Nodes 48..56
            let wb_center_x = center.x + 240.0;
            // WB pipeline registers (48..52)
            for i in 0..4 {
                let id = 48 + i;
                gates.push(VisualGate {
                    id,
                    pos: vec2(wb_center_x - 30.0, center.y - 45.0 + i as f32 * 25.0),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::Dff,
                    label: format!("WB_REG{}", i),
                });
                // Connect from EX outputs
                graph.add_edge(40 + i, id);
                graph.add_edge(44 + (i % 2), id);
            }
            // Writeback logic cells (52..56)
            for i in 0..4 {
                let id = 52 + i;
                gates.push(VisualGate {
                    id,
                    pos: vec2(wb_center_x + 20.0, center.y - 45.0 + i as f32 * 25.0),
                    vel: Vec2::ZERO,
                    partition: Partition::Unassigned,
                    gate_type: GateType::Buffer,
                    label: format!("WB_BYP{}", i),
                });
                graph.add_edge(48 + i, id);

                // CRITICAL STRUCTURAL PIPELINE FEEDBACK LOOP:
                // Writeback data loops all the way back into the Register File memory cells in ID stage!
                if i < 3 {
                    let reg_latch_id = 24 + i;
                    graph.add_edge(id, reg_latch_id);
                }
            }
        }
        CircuitType::RiscVCore => {
            let json_path = "src/riscv_core.json";
            if let Ok(json_content) = std::fs::read_to_string(json_path) {
                if let Ok(parsed_json) = serde_json::from_str::<serde_json::Value>(&json_content) {
                    if let Some(modules) = parsed_json.get("modules") {
                        if let Some(golden_mod) = modules.get("RiscVCore") {
                            if let Some(cells) = golden_mod.get("cells") {
                                let mut cell_to_node = HashMap::new();
                                for (cell_name, cell_data) in cells.as_object().unwrap() {
                                    let id = gates.len();
                                    cell_to_node.insert(cell_name.clone(), id);
                                    
                                    let type_str = cell_data.get("type").and_then(|t| t.as_str()).unwrap_or("");
                                    let gate_type = match type_str {
                                        "$_AND_" | "$_ANDNOT_" => GateType::And,
                                        "$_OR_" | "$_ORNOT_" => GateType::Or,
                                        "$_XOR_" | "$_XNOR_" => GateType::Xor,
                                        "$_NOT_" => GateType::Not,
                                        "$_NAND_" => GateType::Nand,
                                        "$_NOR_" => GateType::Nor,
                                        "$_DFF_PP0_" | "$_DFFE_PP_" => GateType::Dff,
                                        "$_MUX_" => GateType::Mux,
                                        _ => GateType::Buffer,
                                    };
                                    
                                    let short_name = cell_name.split('$').last().unwrap_or(cell_name).replace("slice", "U");
                                    let label = format!("{}:{}", short_name, match gate_type {
                                        GateType::And => "AND",
                                        GateType::Or => "OR",
                                        GateType::Xor => "XOR",
                                        GateType::Not => "INV",
                                        GateType::Nand => "NAND",
                                        GateType::Nor => "NOR",
                                        GateType::Dff => "DFF",
                                        GateType::Mux => "MUX",
                                        _ => "BUF",
                                    });

                                    let angle = rand::gen_range(0.0, 360.0_f32).to_radians();
                                    let radius = rand::gen_range(50.0, 280.0_f32);
                                    let pos = center + vec2(angle.cos() * radius, angle.sin() * radius);

                                    gates.push(VisualGate {
                                        id,
                                        pos,
                                        vel: Vec2::ZERO,
                                        partition: Partition::Unassigned,
                                        gate_type,
                                        label,
                                    });
                                }

                                let mut net_to_drivers = HashMap::new();
                                for (cell_name, cell_data) in cells.as_object().unwrap() {
                                    if let Some(&id) = cell_to_node.get(cell_name) {
                                        if let Some(connections) = cell_data.get("connections") {
                                            for (_port, nets) in connections.as_object().unwrap() {
                                                if let Some(net_array) = nets.as_array() {
                                                    for net in net_array {
                                                        if let Some(net_num) = net.as_u64() {
                                                            net_to_drivers.entry(net_num).or_insert(Vec::new()).push(id);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

                                for (_net, cell_ids) in net_to_drivers {
                                    if cell_ids.len() > 1 {
                                        for idx in 0..cell_ids.len() - 1 {
                                            graph.add_edge(cell_ids[idx], cell_ids[idx + 1]);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if gates.is_empty() {
                let size = 120;
                for i in 0..size {
                    let gate_type = if i % 8 == 0 {
                        GateType::Dff
                    } else if i % 6 == 1 {
                        GateType::Mux
                    } else {
                        match i % 4 {
                            0 => GateType::And,
                            1 => GateType::Or,
                            2 => GateType::Xor,
                            _ => GateType::Not,
                        }
                    };
                    let label = format!("FALLBACK_RV_U{}:{}", i, match gate_type {
                        GateType::Dff => "DFF",
                        GateType::Mux => "MUX",
                        GateType::And => "AND2",
                        GateType::Or => "OR2",
                        GateType::Xor => "XOR2",
                        _ => "INV",
                    });

                    let angle = rand::gen_range(0.0, 360.0_f32).to_radians();
                    let radius = rand::gen_range(50.0, 200.0_f32);
                    gates.push(VisualGate {
                        id: i,
                        pos: center + vec2(angle.cos() * radius, angle.sin() * radius),
                        vel: Vec2::ZERO,
                        partition: Partition::Unassigned,
                        gate_type,
                        label,
                    });

                    if i > 0 { graph.add_edge(i, i - 1); }
                    if i % 6 == 0 && i >= 6 { graph.add_edge(i, i - 6); }
                }
            }
        }
    }

    (graph, gates)
}

// ==========================================================================
// 3. MATHEMATICAL PIPELINE LINK
// ==========================================================================

fn partition_subgroup(
    graph: &Graph,
    nodes: &[usize],
) -> Result<(Vec<usize>, Vec<usize>, f64), &'static str> {
    if nodes.len() <= 1 {
        return Ok((nodes.to_vec(), Vec::new(), 0.0));
    }
    
    // Map nodes to contiguous indices
    let mut node_to_idx = HashMap::new();
    for (idx, &node) in nodes.iter().enumerate() {
        node_to_idx.insert(node, idx);
    }
    
    let mut sub_topo = Topology::new(nodes.len());
    
    // Add edges that are strictly within the subgroup
    for &u in nodes {
        if let Some(neighbors) = graph.get_neighbors(u) {
            for &v in neighbors {
                if v > u && nodes.contains(&v) {
                    if let (Some(&u_mapped), Some(&v_mapped)) = (node_to_idx.get(&u), node_to_idx.get(&v)) {
                        sub_topo.add_edge(u_mapped, v_mapped);
                    }
                }
            }
        }
    }
    
    let pruner = TauSpectralPruner::builder().tau(0.0).build();
    let result = pruner.prune(&sub_topo, 0);
    
    match result {
        Ok(resolution) => {
            let left_nodes = resolution.mainland_nodes.iter().map(|&idx| nodes[idx]).collect();
            let right_nodes = resolution.island_nodes.iter().map(|&idx| nodes[idx]).collect();
            let score = resolution.connectivity_score;
            Ok((left_nodes, right_nodes, score))
        }
        Err(_) => {
            // Fallback: split in half
            let mid = nodes.len() / 2;
            Ok((nodes[0..mid].to_vec(), nodes[mid..].to_vec(), 0.0))
        }
    }
}

fn calculate_subgroup_metrics(graph: &Graph, subgroup: &[usize]) -> (usize, usize) {
    let mut e_internal = 0;
    let mut e_external = 0;
    
    let subgroup_set: HashSet<usize> = subgroup.iter().copied().collect();
    
    for &u in subgroup {
        if let Some(neighbors) = graph.get_neighbors(u) {
            for &v in neighbors {
                if subgroup_set.contains(&v) {
                    if u < v {
                        e_internal += 1;
                    }
                } else {
                    e_external += 1;
                }
            }
        }
    }
    (e_internal, e_external)
}

#[derive(Clone)]
struct SplitNode {
    nodes: Vec<usize>,
    left: Option<Box<SplitNode>>,
    right: Option<Box<SplitNode>>,
    fiedler_score: f64,
    partition_id: Option<usize>,
}

fn build_partition_tree(
    graph: &Graph,
    nodes: Vec<usize>,
    depth: usize,
    max_depth: usize,
    next_partition_id: &mut usize,
) -> SplitNode {
    let size = nodes.len();
    
    if size <= 2 || depth >= max_depth {
        let leaf_id = *next_partition_id;
        *next_partition_id += 1;
        return SplitNode {
            nodes,
            left: None,
            right: None,
            fiedler_score: 0.0,
            partition_id: Some(leaf_id),
        };
    }
    
    match partition_subgroup(graph, &nodes) {
        Ok((left_group, right_group, fiedler_score)) => {
            if left_group.is_empty() || right_group.is_empty() {
                let leaf_id = *next_partition_id;
                *next_partition_id += 1;
                return SplitNode {
                    nodes,
                    left: None,
                    right: None,
                    fiedler_score,
                    partition_id: Some(leaf_id),
                };
            }
            
            let (e_internal, e_external) = calculate_subgroup_metrics(graph, &nodes);
            
            let size_override = size > 60;
            let connectivity_halt = !size_override && depth > 0 && fiedler_score > 1.5;
            let density_ratio = (e_internal as f64) / (e_external as f64 + 1.0);
            let density_halt = !size_override && depth > 0 && density_ratio > 5.0;
            
            let max_possible_edges = (size * (size - 1)) as f64 / 2.0;
            let clique_density = if size > 1 { (e_internal as f64) / max_possible_edges } else { 1.0 };
            let clique_halt = !size_override && depth > 0 && clique_density > 0.85;
            
            if connectivity_halt || density_halt || clique_halt {
                let leaf_id = *next_partition_id;
                *next_partition_id += 1;
                return SplitNode {
                    nodes,
                    left: None,
                    right: None,
                    fiedler_score,
                    partition_id: Some(leaf_id),
                };
            }
            
            let left_child = build_partition_tree(graph, left_group, depth + 1, max_depth, next_partition_id);
            let right_child = build_partition_tree(graph, right_group, depth + 1, max_depth, next_partition_id);
            
            SplitNode {
                nodes,
                left: Some(Box::new(left_child)),
                right: Some(Box::new(right_child)),
                fiedler_score,
                partition_id: None,
            }
        }
        Err(_) => {
            let leaf_id = *next_partition_id;
            *next_partition_id += 1;
            SplitNode {
                nodes,
                left: None,
                right: None,
                fiedler_score: 0.0,
                partition_id: Some(leaf_id),
            }
        }
    }
}

fn collect_leaves(
    node: &SplitNode,
    partitions: &mut HashMap<usize, usize>,
    sorted_order: &mut Vec<usize>,
    communities: &mut Vec<(usize, Vec<usize>)>,
) {
    if let Some(p_id) = node.partition_id {
        communities.push((p_id, node.nodes.clone()));
        for &gate_id in &node.nodes {
            partitions.insert(gate_id, p_id);
            sorted_order.push(gate_id);
        }
    } else {
        if let Some(ref left) = node.left {
            collect_leaves(left, partitions, sorted_order, communities);
        }
        if let Some(ref right) = node.right {
            collect_leaves(right, partitions, sorted_order, communities);
        }
    }
}

fn execute_spectral_partition(
    graph: &Graph,
    gates: &mut [VisualGate],
    telemetry: &mut Telemetry,
    sorted_gate_ids: &mut Vec<usize>,
    community_blocks: &mut Vec<(usize, Vec<usize>)>,
) -> SplitNode {
    let start = Instant::now();
    
    let all_nodes: Vec<usize> = gates.iter().map(|g| g.id).collect();
    
    let mut next_partition_id = 0;
    let root_node = build_partition_tree(graph, all_nodes, 0, 5, &mut next_partition_id);
    
    let mut partitions = HashMap::new();
    sorted_gate_ids.clear();
    community_blocks.clear();
    
    collect_leaves(&root_node, &mut partitions, sorted_gate_ids, community_blocks);
    
    for gate in gates.iter_mut() {
        if let Some(&p_id) = partitions.get(&gate.id) {
            gate.partition = Partition::Index(p_id);
        } else {
            gate.partition = Partition::Unassigned;
        }
    }
    
    telemetry.is_partitioned = true;
    telemetry.community_count = next_partition_id;
    
    let duration = start.elapsed();
    telemetry.execution_time_us = duration.as_micros();
    
    telemetry.cross_wire_count = calculate_edge_cuts(graph, gates);
    root_node
}


fn calculate_edge_cuts(graph: &Graph, gates: &[VisualGate]) -> usize {
    let mut cuts = 0;
    let mut evaluated = HashSet::new();

    for gate in gates {
        if let Some(neighbors) = graph.get_neighbors(gate.id) {
            for &neighbor_id in neighbors {
                let edge_key = if gate.id < neighbor_id { (gate.id, neighbor_id) } else { (neighbor_id, gate.id) };
                if !evaluated.contains(&edge_key) {
                    evaluated.insert(edge_key);
                    if let Some(neighbor_gate) = gates.iter().find(|g| g.id == neighbor_id) {
                        if gate.partition != Partition::Unassigned 
                           && neighbor_gate.partition != Partition::Unassigned 
                           && gate.partition != neighbor_gate.partition 
                        {
                            cuts += 1;
                        }
                    }
                }
            }
        }
    }
    cuts
}

// ==========================================================================
// 4. PHYSICS LAYOUT ENGINE (2D FORCE-DIRECTED INTERPRETER)
// ==========================================================================

fn update_physics_layout(graph: &Graph, gates: &mut [VisualGate], telemetry: &Telemetry, camera_zoom: f32) {
    let dampening = 0.85;
    let center_x = screen_width() / 2.0;
    let center_y = screen_height() / 2.0;

    // Dynamic scale based on zoom to expand layout organically when zooming out
    let expansion_scale = (1.0 / camera_zoom).clamp(1.0, 10.0);
    let cell_zoom_scale = (1.0 / camera_zoom).sqrt().clamp(1.0, 4.0);

    // Phase A: Node-to-Node Coulombic Repulsion (ONLY in Organic mode)
    if telemetry.placement_mode == PlacementMode::Organic {
        let k_rep = 65.0 * cell_zoom_scale; // Scale repulsion with cell size zoom
        for i in 0..gates.len() {
            let mut force = Vec2::ZERO;
            for j in 0..gates.len() {
                if i == j { continue; }
                let dir = gates[i].pos - gates[j].pos;
                let dist = dir.length().max(1.0);
                if dist < 350.0 * expansion_scale { // Scale repulsion radius with layout expansion
                    force += (dir / dist) * ((k_rep * k_rep) / dist);
                }
            }
            gates[i].vel += force * 0.14;
        }
    }

    // Phase B: Hooke's Law Attraction Along Electrical Nets (Edges) (ONLY in Organic mode)
    if telemetry.placement_mode == PlacementMode::Organic {
        let k_att = 8.0;
        let rest_len = 90.0 * cell_zoom_scale;
        for i in 0..gates.len() {
            if let Some(neighbors) = graph.get_neighbors(gates[i].id) {
                let mut force = Vec2::ZERO;
                for &neighbor_id in neighbors {
                    if let Some(neighbor_gate) = gates.iter().find(|g| g.id == neighbor_id) {
                        let dir = neighbor_gate.pos - gates[i].pos;
                        let dist = dir.length().max(1.0);
                        if dist > rest_len {
                            force += (dir / dist) * ((dist - rest_len) / k_att);
                        }
                    }
                }
                gates[i].vel += force * 0.04;
            }
        }
    }

    // Phase C: Spatial separation / snapping targets
    let padding = screen_width() * 0.08;
    let drawable_width = screen_width() - padding * 2.0;
    let row_height = (screen_height() - 200.0) / 5.0;

    let mut community_gates_map: HashMap<Partition, Vec<usize>> = HashMap::new();
    if telemetry.placement_mode == PlacementMode::SiliconRow && telemetry.is_partitioned {
        for g in gates.iter() {
            community_gates_map.entry(g.partition).or_default().push(g.id);
        }
        for list in community_gates_map.values_mut() {
            list.sort();
        }
    }

    for i in 0..gates.len() {
        let gate = &mut gates[i];
        if telemetry.placement_mode == PlacementMode::SiliconRow {
            let (target_x, target_y) = if telemetry.is_partitioned {
                // Snapping partitioned targets: separate by community columns
                let idx = match gate.partition {
                    Partition::Index(idx_val) => idx_val,
                    Partition::Unassigned => 0,
                };
                
                let c_count = telemetry.community_count.max(1);
                let block_width = drawable_width / c_count as f32;
                let col_center_x = padding + block_width * (idx as f32 + 0.5);

                let empty_vec = Vec::new();
                let comm_gates = community_gates_map.get(&gate.partition).unwrap_or(&empty_vec);
                let k = comm_gates.iter().position(|&id| id == gate.id).unwrap_or(0);
                
                let row = k % 5;
                let col = k / 5;

                // Spacing within the community block column
                let num_cols = ((comm_gates.len() as f32) / 5.0).ceil().max(1.0);
                let max_width_allowed = block_width - 30.0; // Margin padding
                let col_spacing = (max_width_allowed / num_cols).clamp(25.0, 75.0);
                let col_offset = (col as f32 - (num_cols - 1.0) * 0.5) * col_spacing;

                let target_x = col_center_x + col_offset;
                let target_y = 155.0 + row as f32 * row_height;
                (target_x, target_y)
            } else {
                // Original unpartitioned snapping target calculations
                match telemetry.current_circuit {
                    CircuitType::PipelineCore => {
                        let id = gate.id;
                        let block_width = drawable_width / 4.0;
                        if id < 16 {
                            // Stage 0: IF
                            let block_center_x = padding + block_width * 0.5;
                            if id < 4 {
                                let col_idx = id as f32 - 1.5;
                                (block_center_x + col_idx * 55.0, 155.0 + 0.0 * row_height)
                            } else {
                                let row = (id - 4) / 4;
                                let col = (id - 4) % 4;
                                let col_idx = col as f32 - 1.5;
                                (block_center_x + col_idx * 55.0, 155.0 + (1 + row) as f32 * row_height)
                            }
                        } else if id < 32 {
                            // Stage 1: ID
                            let block_center_x = padding + block_width * 1.5;
                            if id < 20 {
                                let col_idx = (id - 16) as f32 - 1.5;
                                (block_center_x + col_idx * 55.0, 155.0 + 0.0 * row_height)
                            } else if id < 24 {
                                let col_idx = (id - 20) as f32 - 1.5;
                                (block_center_x + col_idx * 55.0, 155.0 + 1.0 * row_height)
                            } else if id < 30 {
                                let pair = (id - 24) / 2;
                                let side = (id - 24) % 2;
                                let col_idx = pair as f32 - 1.0;
                                let row = if side == 0 { 2 } else { 3 };
                                (block_center_x + col_idx * 60.0, 155.0 + row as f32 * row_height)
                            } else {
                                let col_idx = (id - 30) as f32 - 0.5;
                                (block_center_x + col_idx * 65.0, 155.0 + 4.0 * row_height)
                            }
                        } else if id < 48 {
                            // Stage 2: EX
                            let block_center_x = padding + block_width * 2.5;
                            if id < 36 {
                                let col_idx = (id - 32) as f32 - 1.5;
                                (block_center_x + col_idx * 55.0, 155.0 + 0.0 * row_height)
                            } else if id < 44 {
                                let row = (id - 36) / 3;
                                let col = (id - 36) % 3;
                                let col_idx = col as f32 - 1.0;
                                (block_center_x + col_idx * 60.0, 155.0 + (1 + row) as f32 * row_height)
                            } else {
                                let col_idx = (id - 44) as f32 - 1.5;
                                (block_center_x + col_idx * 55.0, 155.0 + 4.0 * row_height)
                            }
                        } else {
                            // Stage 3: WB
                            let block_center_x = padding + block_width * 3.5;
                            if id < 52 {
                                let col_idx = (id - 48) as f32 - 1.5;
                                (block_center_x + col_idx * 55.0, 155.0 + 1.0 * row_height)
                            } else {
                                let col_idx = (id - 52) as f32 - 1.5;
                                (block_center_x + col_idx * 55.0, 155.0 + 3.0 * row_height)
                            }
                        }
                    }
                    CircuitType::RippleCarryAdder8 => {
                        let id = gate.id;
                        let slice_width = drawable_width / 8.0;
                        let stage_idx = id / 5;
                        let row_idx = id % 5;
                        let target_x = padding + slice_width * (stage_idx as f32 + 0.5);
                        let target_y = 155.0 + row_idx as f32 * row_height;
                        (target_x, target_y)
                    }
                    CircuitType::ArrayMultiplier4 => {
                        let id = gate.id;
                        let r = (id / 3) / 4;
                        let c = (id / 3) % 4;
                        let gate_offset = id % 3;
                        let target_x = center_x + (c as f32 - 1.5) * 130.0 + (r as f32 - 1.5) * 20.0;
                        let target_y = 175.0 + r as f32 * 100.0 + gate_offset as f32 * 32.0;
                        (target_x, target_y)
                    }
                    CircuitType::StarClockTree => {
                        let id = gate.id;
                        if id == 0 {
                            // Root oscillator in center
                            (center_x, center_y + 40.0)
                        } else if id == 1 || id == 22 || id == 43 || id == 64 || id == 85 || id == 106 {
                            // Clock Buffers in a symmetric perimeter ring
                            let b = (id - 1) / 21;
                            let angle = b as f32 * 60.0_f32.to_radians();
                            (center_x + angle.cos() * 120.0, center_y + 40.0 + angle.sin() * 120.0)
                        } else {
                            // Leaves distributed uniformly in standard cell rows
                            let leaf_idx = (id - 2) % 21;
                            let b = (id - 2) / 21;
                            let row = leaf_idx % 5;
                            let col = b * 4 + (leaf_idx / 5);
                            let target_x = padding + col as f32 * (drawable_width / 23.0);
                            let target_y = 155.0 + row as f32 * row_height;
                            (target_x, target_y)
                        }
                    }
                    CircuitType::MeshInterconnect => {
                        let id = gate.id;
                        if id < 50 {
                            // Core A
                            let r = id / 10;
                            let c = id % 10;
                            let target_x = padding + c as f32 * ((drawable_width * 0.42) / 9.0);
                            let target_y = 155.0 + r as f32 * row_height;
                            (target_x, target_y)
                        } else {
                            // Core B
                            let i = id - 50;
                            let r = i / 10;
                            let c = i % 10;
                            let target_x = padding + drawable_width * 0.58 + c as f32 * ((drawable_width * 0.42) / 9.0);
                            let target_y = 155.0 + r as f32 * row_height;
                            (target_x, target_y)
                        }
                    }
                    CircuitType::AluGrid => {
                        let id = gate.id;
                        let r = id / 12;
                        let c = id % 12;
                        let target_x = padding + c as f32 * (drawable_width / 11.0);
                        let target_y = 155.0 + (r % 5) as f32 * row_height + (r / 5) as f32 * 12.0;
                        (target_x, target_y)
                    }
                    CircuitType::RiscVCore => {
                        let id = gate.id;
                        let block_width = drawable_width / 4.0;
                        let stage = match gate.gate_type {
                            GateType::Buffer | GateType::Oscillator | GateType::Not => 0,
                            GateType::Dff | GateType::Latch => 1,
                            GateType::And | GateType::Or | GateType::Xor | GateType::Nand | GateType::Nor | GateType::Xnor => 2,
                            GateType::Mux => 3,
                        };
                        let block_center_x = padding + block_width * (stage as f32 + 0.5);
                        let row = (id % 5) as f32;
                        let col_idx = ((id / 5) % 20) as f32 - 9.5;
                        let target_x = block_center_x + col_idx * 12.5;
                        let target_y = 155.0 + row * row_height;
                        (target_x, target_y)
                    }
                }
            };

            // Scale snap targets relative to center_x, center_y + 40.0
            let center_y_ref = center_y + 40.0;
            let target_x = center_x + (target_x - center_x) * expansion_scale;
            let target_y = center_y_ref + (target_y - center_y_ref) * expansion_scale;

            // Restorative spring forces (increased for extremely snappy snaps)
            gate.vel.x += (target_x - gate.pos.x) * 0.28;
            gate.vel.y += (target_y - gate.pos.y) * 0.45; // Strong vertical snap to cell rows
            
            // Dampen vertical velocity to prevent oscillations around cell rows
            gate.vel.y *= 0.4;
        } else if telemetry.is_partitioned {
            // Organic hierarchical separation into beautiful 2D community grid clouds
            if let Partition::Index(idx) = gate.partition {
                let c_count = telemetry.community_count.max(1);
                let padding_x = 120.0;
                let padding_y = 180.0;
                let drawable_w = screen_width() - padding_x * 2.0;
                let drawable_h = screen_height() - padding_y - 80.0;

                let grid_cols = (c_count as f32).sqrt().ceil() as usize;
                let grid_rows = (c_count + grid_cols - 1) / grid_cols;

                let col = idx % grid_cols;
                let row = idx / grid_cols;

                let target_x_base = if grid_cols > 1 {
                    padding_x + col as f32 * (drawable_w / (grid_cols - 1) as f32)
                } else {
                    center_x
                };

                let target_y_base = if grid_rows > 1 {
                    padding_y + row as f32 * (drawable_h / (grid_rows - 1) as f32)
                } else {
                    center_y + 40.0
                };

                // Expand targets outwards from center based on expansion_scale to occupy more space when zooming out
                let target_x = center_x + (target_x_base - center_x) * expansion_scale;
                let target_y = (center_y + 40.0) + (target_y_base - (center_y + 40.0)) * expansion_scale;

                // Gentle community guidance force
                gate.vel.x += (target_x - gate.pos.x) * 0.015;
                gate.vel.y += (target_y - gate.pos.y) * 0.015;
            }
        }

        // Apply velocities, screen bounds checking, and dampening constraints
        gate.pos += gate.vel * 0.4;
        gate.vel *= dampening;
        
        if telemetry.placement_mode == PlacementMode::Organic {
            let min_x = center_x - (center_x - 35.0) * expansion_scale;
            let max_x = center_x + (screen_width() - 35.0 - center_x) * expansion_scale;
            let min_y = (center_y + 40.0) - ((center_y + 40.0) - 130.0) * expansion_scale;
            let max_y = (center_y + 40.0) + (screen_height() - 40.0 - (center_y + 40.0)) * expansion_scale;
            gate.pos.x = gate.pos.x.clamp(min_x, max_x);
            gate.pos.y = gate.pos.y.clamp(min_y, max_y);
        } else {
            let min_x = center_x - (center_x - 35.0) * expansion_scale;
            let max_x = center_x + (screen_width() - 35.0 - center_x) * expansion_scale;
            let min_y = (center_y + 40.0) - ((center_y + 40.0) - 130.0) * expansion_scale;
            let max_y = (center_y + 40.0) + (screen_height() - 40.0 - (center_y + 40.0)) * expansion_scale;
            gate.pos.x = gate.pos.x.clamp(min_x, max_x);
            gate.pos.y = gate.pos.y.clamp(min_y, max_y);
        }
    }

    // Phase D: Collision Avoidance (gentle push apart pass to prevent any standard cells from overlapping)
    for _ in 0..2 { // Run two integration iterations for high stability packing
        for i in 0..gates.len() {
            for j in (i + 1)..gates.len() {
                let dx = gates[i].pos.x - gates[j].pos.x;
                let dy = gates[i].pos.y - gates[j].pos.y;
                let width_i = gates[i].gate_type.width() * cell_zoom_scale;
                let height_i = gates[i].gate_type.height() * cell_zoom_scale;
                let width_j = gates[j].gate_type.width() * cell_zoom_scale;
                let height_j = gates[j].gate_type.height() * cell_zoom_scale;
                
                // Allow a tiny margin of 10% spacing
                let min_dist_x = (width_i + width_j) * 0.55;
                let min_dist_y = (height_i + height_j) * 0.55;
                
                if dx.abs() < min_dist_x && dy.abs() < min_dist_y {
                    let overlap_x = min_dist_x - dx.abs();
                    let overlap_y = min_dist_y - dy.abs();
                    
                    // Push in the direction that minimizes displacement
                    let push_x = if dx >= 0.0 { overlap_x * 0.3 } else { -overlap_x * 0.3 };
                    let push_y = if dy >= 0.0 { overlap_y * 0.3 } else { -overlap_y * 0.3 };
                    
                    if telemetry.placement_mode == PlacementMode::SiliconRow {
                        // In Snapping Mode, only push horizontally to keep perfect row alignment!
                        gates[i].pos.x += push_x * 0.5;
                        gates[j].pos.x -= push_x * 0.5;
                    } else {
                        // In Organic Mode, push in 2D
                        gates[i].pos.x += push_x * 0.5;
                        gates[i].pos.y += push_y * 0.5;
                        gates[j].pos.x -= push_x * 0.5;
                        gates[j].pos.y -= push_y * 0.5;
                    }
                }
            }
        }
    }
}

impl GateType {
    fn width(&self) -> f32 {
        match self {
            GateType::Dff => 65.0,
            GateType::Latch => 60.0,
            GateType::Mux => 60.0,
            GateType::Oscillator => 60.0,
            GateType::And | GateType::Or | GateType::Xor | GateType::Nand | GateType::Nor | GateType::Xnor => 52.0,
            GateType::Not | GateType::Buffer => 38.0,
        }
    }

    fn height(&self) -> f32 {
        match self {
            GateType::Oscillator => 50.0,
            _ => 32.0,
        }
    }
}

fn get_neon_color(idx: usize) -> Color {
    let colors = [
        Color::from_rgba(6, 182, 212, 255),   // Cyberpunk Cyan
        Color::from_rgba(34, 197, 94, 255),   // Neon Green
        Color::from_rgba(249, 115, 22, 255),  // Neon Orange
        Color::from_rgba(168, 85, 247, 255),  // Vibrant Purple
        Color::from_rgba(236, 72, 153, 255),  // Neon Pink
        Color::from_rgba(234, 179, 8, 255),   // Neon Yellow
        Color::from_rgba(239, 68, 68, 255),   // Neon Red
        Color::from_rgba(59, 130, 246, 255),  // Neon Blue
        Color::from_rgba(20, 184, 166, 255),  // Teal
        Color::from_rgba(244, 63, 94, 255),   // Rose
    ];
    colors[idx % colors.len()]
}

fn draw_standard_cell(
    _gate_id: usize,
    pos: Vec2,
    gate_type: GateType,
    label: &str,
    partition: Partition,
    is_hovered: bool,
    camera_zoom: f32,
    is_probed: bool,
    is_fan_in: bool,
    is_fan_out: bool,
    is_high: bool,
    crit_delay: Option<f32>,
) {
    let zoom_scale = (1.0 / camera_zoom).sqrt().clamp(1.0, 4.0);
    let width = gate_type.width() * zoom_scale;
    let height = gate_type.height() * zoom_scale;

    let rx = pos.x - width / 2.0;
    let ry = pos.y - height / 2.0;

    // 0. Render subtle glowing outer halo for High logic states
    if is_high {
        let pulse = (get_time() * 6.0).sin() as f32 * 0.15 + 0.85;
        let halo_w = width + 8.0 * zoom_scale;
        let halo_h = height + 8.0 * zoom_scale;
        draw_rectangle(
            pos.x - halo_w / 2.0,
            pos.y - halo_h / 2.0,
            halo_w,
            halo_h,
            Color::from_rgba(34, 197, 94, (35.0 * pulse) as u8),
        );
    }

    // Get color accents based on gate functionality class
    let (bg_color, mut border_color, accent_color) = match gate_type {
        GateType::Dff | GateType::Latch => (
            Color::from_rgba(15, 23, 42, 230),   // Slate dark
            Color::from_rgba(6, 182, 212, 180),  // Cyan border
            Color::from_rgba(6, 182, 212, 255),  // Cyan accent
        ),
        GateType::And | GateType::Or | GateType::Xor | GateType::Nand | GateType::Nor | GateType::Xnor => (
            Color::from_rgba(15, 23, 42, 230),
            Color::from_rgba(34, 197, 94, 180),  // Green border
            Color::from_rgba(34, 197, 94, 255),  // Green accent
        ),
        GateType::Mux => (
            Color::from_rgba(15, 23, 42, 230),
            Color::from_rgba(249, 115, 22, 180), // Orange border
            Color::from_rgba(249, 115, 22, 255), // Orange accent
        ),
        GateType::Not | GateType::Buffer => (
            Color::from_rgba(15, 23, 42, 230),
            Color::from_rgba(168, 85, 247, 180), // Purple border
            Color::from_rgba(168, 85, 247, 255), // Purple accent
        ),
        GateType::Oscillator => (
            Color::from_rgba(15, 23, 42, 230),
            Color::from_rgba(239, 68, 68, 180),  // Red border
            Color::from_rgba(239, 68, 68, 255),  // Red accent
        ),
    };

    // Override borders if partitioned
    border_color = match partition {
        Partition::Unassigned => border_color,
        Partition::Index(idx) => get_neon_color(idx),
    };

    // Override borders based on active probe logic cones
    if is_probed {
        border_color = Color::from_rgba(217, 70, 239, 255); // Neon Magenta
    } else if is_fan_in {
        border_color = Color::from_rgba(217, 70, 239, 255); // Magenta
    } else if is_fan_out {
        border_color = Color::from_rgba(139, 92, 246, 255); // Deep Purple
    } else if crit_delay.is_some() {
        border_color = Color::from_rgba(234, 179, 8, 255); // Gold for Critical Path nodes
    }

    // Draw component main body
    draw_rectangle(rx, ry, width, height, bg_color);
    let border_thickness = if is_hovered || is_probed { 2.5 } else { 1.5 } * zoom_scale;
    draw_rectangle_lines(rx, ry, width, height, border_thickness, border_color);

    // Draw a small left/right accent bar
    draw_rectangle(rx, ry, 4.0 * zoom_scale, height, accent_color);

    // Draw small logic function symbol in the center
    let sym = match gate_type {
        GateType::And => "&",
        GateType::Or => ">=1",
        GateType::Xor => "=1",
        GateType::Not => "1",
        GateType::Nand => "!&",
        GateType::Nor => "!>=1",
        GateType::Xnor => "XNOR",
        GateType::Dff => "DFF",
        GateType::Latch => "LAT",
        GateType::Mux => "MUX",
        GateType::Buffer => "BUF",
        GateType::Oscillator => "OSC",
    };
    
    // Perfectly center the text symbol inside the standard cell
    let sym_font_size = 16.0 * zoom_scale;
    let sym_dim = measure_text(sym, None, sym_font_size as u16, 1.0);
    draw_text(
        sym,
        rx + (width - sym_dim.width) / 2.0 + 1.0 * zoom_scale,
        ry + (height + sym_dim.height) / 2.0 - 2.0 * zoom_scale,
        sym_font_size,
        accent_color,
    );

    // Draw label centered above the cell
    let label_font_size = 15.0 * zoom_scale;
    let label_dim = measure_text(label, None, label_font_size as u16, 1.0);
    draw_text(
        label,
        rx + (width - label_dim.width) / 2.0,
        ry - 5.0 * zoom_scale,
        label_font_size,
        Color::from_rgba(226, 232, 240, 255), // Bright slate-200
    );

    // Draw timing badge if on critical path
    if let Some(delay) = crit_delay {
        let badge_text = format!("+{:.0}ps", delay);
        let badge_font_size = 12.5 * zoom_scale;
        let badge_dim = measure_text(&badge_text, None, badge_font_size as u16, 1.0);
        let badge_w = badge_dim.width + 6.0 * zoom_scale;
        let badge_h = badge_dim.height + 4.0 * zoom_scale;
        let bx = rx + (width - badge_w) / 2.0;
        let by = ry - (18.0 * zoom_scale + badge_h);
        
        // Draw glassmorphic gold badge background
        draw_rectangle(bx, by, badge_w, badge_h, Color::from_rgba(234, 179, 8, 45));
        draw_rectangle_lines(bx, by, badge_w, badge_h, 1.0 * zoom_scale, Color::from_rgba(234, 179, 8, 220));
        
        // Draw badge text
        draw_text(
            &badge_text,
            bx + 3.0 * zoom_scale,
            by + badge_h - 2.0 * zoom_scale,
            badge_font_size,
            Color::from_rgba(254, 240, 138, 255), // Bright Gold text
        );
    }

    // Draw gorgeous target corner brackets around probed gate
    if is_probed {
        let pulse = (get_time() * 8.0).sin() as f32 * 0.2 + 0.8;
        let bracket_color = Color::from_rgba(217, 70, 239, (255.0 * pulse) as u8); // Neon Magenta
        let thick = 3.0 * zoom_scale;
        let d = 5.0 * zoom_scale;
        
        // Top-left corner
        draw_line(rx - d, ry - d, rx - d + 12.0 * zoom_scale, ry - d, thick, bracket_color);
        draw_line(rx - d, ry - d, rx - d, ry - d + 12.0 * zoom_scale, thick, bracket_color);
        // Top-right corner
        draw_line(rx + width + d, ry - d, rx + width + d - 12.0 * zoom_scale, ry - d, thick, bracket_color);
        draw_line(rx + width + d, ry - d, rx + width + d, ry - d + 12.0 * zoom_scale, thick, bracket_color);
        // Bottom-left corner
        draw_line(rx - d, ry + height + d, rx - d + 12.0 * zoom_scale, ry + height + d, thick, bracket_color);
        draw_line(rx - d, ry + height + d, rx - d, ry + height + d - 12.0 * zoom_scale, thick, bracket_color);
        // Bottom-right corner
        draw_line(rx + width + d, ry + height + d, rx + width + d - 12.0 * zoom_scale, ry + height + d, thick, bracket_color);
        draw_line(rx + width + d, ry + height + d, rx + width + d, ry + height + d - 12.0 * zoom_scale, thick, bracket_color);
    }

    // Draw miniature pin terminals (scaled slightly larger)
    let input_pins = match gate_type {
        GateType::Not | GateType::Buffer | GateType::Oscillator => 1,
        _ => 2,
    };
    let pin_radius = 2.2 * zoom_scale;
    if input_pins == 1 {
        draw_circle(rx, ry + height / 2.0, pin_radius, accent_color);
    } else {
        draw_circle(rx, ry + height * 0.3, pin_radius, accent_color);
        draw_circle(rx, ry + height * 0.7, pin_radius, accent_color);
    }
    draw_circle(rx + width, ry + height / 2.0, pin_radius, accent_color);
}

fn get_output_pin(gate: &VisualGate, camera_zoom: f32) -> Vec2 {
    let zoom_scale = (1.0 / camera_zoom).sqrt().clamp(1.0, 4.0);
    let w = gate.gate_type.width() * zoom_scale;
    vec2(gate.pos.x + w / 2.0, gate.pos.y)
}

fn get_input_pin(gate: &VisualGate, from_pos: Vec2, camera_zoom: f32) -> Vec2 {
    let zoom_scale = (1.0 / camera_zoom).sqrt().clamp(1.0, 4.0);
    let w = gate.gate_type.width() * zoom_scale;
    let h = gate.gate_type.height() * zoom_scale;
    let input_pins = match gate.gate_type {
        GateType::Not | GateType::Buffer | GateType::Oscillator => 1,
        _ => 2,
    };
    if input_pins == 1 {
        vec2(gate.pos.x - w / 2.0, gate.pos.y)
    } else {
        let pin_top = vec2(gate.pos.x - w / 2.0, gate.pos.y - h * 0.2);
        let pin_bot = vec2(gate.pos.x - w / 2.0, gate.pos.y + h * 0.2);
        if from_pos.distance_squared(pin_top) < from_pos.distance_squared(pin_bot) {
            pin_top
        } else {
            pin_bot
        }
    }
}

// ==========================================================================
// 4. MASTER CAD SUITE HELPERS (SIMULATION, TIMING & DENDROGRAM)
// ==========================================================================

#[derive(Clone)]
#[allow(dead_code)]
struct DendrogramItem {
    label: String,
    depth: usize,
    nodes: Vec<usize>,
    is_leaf: bool,
    partition_id: Option<usize>,
    score: f64,
}

fn flatten_tree(
    node: &SplitNode,
    depth: usize,
    items: &mut Vec<DendrogramItem>,
) {
    let label = if let Some(pid) = node.partition_id {
        format!("Leaf Community {}", pid)
    } else {
        format!("Split (Fiedler: {:.3})", node.fiedler_score)
    };
    items.push(DendrogramItem {
        label,
        depth,
        nodes: node.nodes.clone(),
        is_leaf: node.partition_id.is_some(),
        partition_id: node.partition_id,
        score: node.fiedler_score,
    });

    if let Some(ref left) = node.left {
        flatten_tree(left, depth + 1, items);
    }
    if let Some(ref right) = node.right {
        flatten_tree(right, depth + 1, items);
    }
}

fn focus_camera_on_nodes(
    nodes: &[usize],
    gates: &[VisualGate],
    camera_offset: &mut Vec2,
    camera_zoom: &mut f32,
) {
    if nodes.is_empty() { return; }
    let mut min_x = f32::MAX;
    let mut max_x = f32::MIN;
    let mut min_y = f32::MAX;
    let mut max_y = f32::MIN;

    for &id in nodes {
        if let Some(gate) = gates.iter().find(|g| g.id == id) {
            min_x = min_x.min(gate.pos.x);
            max_x = max_x.max(gate.pos.x);
            min_y = min_y.min(gate.pos.y);
            max_y = max_y.max(gate.pos.y);
        }
    }

    if min_x < max_x {
        let center_x = (min_x + max_x) / 2.0;
        let center_y = (min_y + max_y) / 2.0;
        *camera_offset = vec2(center_x, center_y);

        let width_world = (max_x - min_x).max(100.0) + 250.0;
        let height_world = (max_y - min_y).max(100.0) + 250.0;

        let zoom_w = screen_width() / width_world;
        let zoom_h = screen_height() / height_world;
        *camera_zoom = zoom_w.min(zoom_h).clamp(0.25, 4.0);
    }
}

fn get_gate_delay_ps(gate_type: GateType) -> f32 {
    match gate_type {
        GateType::And | GateType::Nor => 45.0,
        GateType::Or => 50.0,
        GateType::Xor | GateType::Xnor => 80.0,
        GateType::Not => 20.0,
        GateType::Nand => 40.0,
        GateType::Buffer => 25.0,
        GateType::Mux => 60.0,
        GateType::Latch => 90.0,
        GateType::Dff => 120.0,
        GateType::Oscillator => 0.0,
    }
}

fn evaluate_logic_simulation(
    graph: &Graph,
    gates: &[VisualGate],
    logic_states: &mut HashMap<usize, bool>,
    user_overrides: &HashMap<usize, bool>,
) {
    let mut next_states = HashMap::new();
    let time_val = get_time();
    let osc_value = (time_val * 1.5 * 2.0 * std::f64::consts::PI).sin() > 0.0;

    let id_to_x: HashMap<usize, f32> = gates.iter().map(|g| (g.id, g.pos.x)).collect();

    for gate in gates {
        if let Some(&forced) = user_overrides.get(&gate.id) {
            next_states.insert(gate.id, forced);
            continue;
        }

        let mut inputs = Vec::new();
        if let Some(neighbors) = graph.get_neighbors(gate.id) {
            for &u in neighbors {
                if let Some(&u_x) = id_to_x.get(&u) {
                    let is_driver = if u_x != gate.pos.x {
                        u_x < gate.pos.x
                    } else {
                        u < gate.id
                    };
                    if is_driver {
                        inputs.push(*logic_states.get(&u).unwrap_or(&false));
                    }
                }
            }
        }

        let val = match gate.gate_type {
            GateType::Oscillator => osc_value,
            GateType::And => {
                if inputs.is_empty() { false } else { inputs.iter().all(|&v| v) }
            }
            GateType::Or => {
                if inputs.is_empty() { false } else { inputs.iter().any(|&v| v) }
            }
            GateType::Xor => {
                if inputs.is_empty() { false } else { inputs.iter().filter(|&&v| v).count() % 2 == 1 }
            }
            GateType::Not => {
                if inputs.is_empty() { true } else { !inputs[0] }
            }
            GateType::Nand => {
                if inputs.is_empty() { true } else { !inputs.iter().all(|&v| v) }
            }
            GateType::Nor => {
                if inputs.is_empty() { true } else { !inputs.iter().any(|&v| v) }
            }
            GateType::Xnor => {
                if inputs.is_empty() { true } else { inputs.iter().filter(|&&v| v).count() % 2 == 0 }
            }
            GateType::Buffer | GateType::Dff | GateType::Latch => {
                if inputs.is_empty() { false } else { inputs[0] }
            }
            GateType::Mux => {
                if inputs.is_empty() {
                    false
                } else if inputs.len() == 1 {
                    inputs[0]
                } else {
                    let sel = inputs[0];
                    if sel {
                        *inputs.get(1).unwrap_or(&false)
                    } else {
                        *inputs.get(2).unwrap_or(&false)
                    }
                }
            }
        };

        next_states.insert(gate.id, val);
    }

    *logic_states = next_states;
}

fn get_fan_in_cone(
    graph: &Graph,
    gate_id: usize,
    id_to_gate: &HashMap<usize, &VisualGate>,
    cone: &mut HashSet<usize>,
) {
    if !cone.insert(gate_id) { return; }
    let gate = match id_to_gate.get(&gate_id) {
        Some(g) => g,
        None => return,
    };
    if let Some(neighbors) = graph.get_neighbors(gate_id) {
        for &u in neighbors {
            if let Some(u_gate) = id_to_gate.get(&u) {
                let is_driver = if u_gate.pos.x != gate.pos.x {
                    u_gate.pos.x < gate.pos.x
                } else {
                    u < gate_id
                };
                if is_driver {
                    get_fan_in_cone(graph, u, id_to_gate, cone);
                }
            }
        }
    }
}

fn get_fan_out_cone(
    graph: &Graph,
    gate_id: usize,
    id_to_gate: &HashMap<usize, &VisualGate>,
    cone: &mut HashSet<usize>,
) {
    if !cone.insert(gate_id) { return; }
    let gate = match id_to_gate.get(&gate_id) {
        Some(g) => g,
        None => return,
    };
    if let Some(neighbors) = graph.get_neighbors(gate_id) {
        for &u in neighbors {
            if let Some(u_gate) = id_to_gate.get(&u) {
                let is_driver = if u_gate.pos.x != gate.pos.x {
                    u_gate.pos.x < gate.pos.x
                } else {
                    u < gate_id
                };
                if !is_driver {
                    get_fan_out_cone(graph, u, id_to_gate, cone);
                }
            }
        }
    }
}

fn compute_critical_path(
    graph: &Graph,
    gates: &[VisualGate],
) -> (Vec<usize>, f32) {
    let mut id_to_gate = HashMap::new();
    for gate in gates {
        id_to_gate.insert(gate.id, gate);
    }

    let mut memo: HashMap<usize, (f32, Option<usize>)> = HashMap::new();

    fn get_max_delay(
        node_id: usize,
        graph: &Graph,
        id_to_gate: &HashMap<usize, &VisualGate>,
        memo: &mut HashMap<usize, (f32, Option<usize>)>,
    ) -> f32 {
        if let Some(&(delay, _)) = memo.get(&node_id) {
            return delay;
        }

        let gate = match id_to_gate.get(&node_id) {
            Some(&g) => g,
            None => return 0.0,
        };

        let gate_delay = get_gate_delay_ps(gate.gate_type);
        let mut max_downstream_delay = 0.0;
        let mut best_next_node = None;

        if let Some(neighbors) = graph.get_neighbors(node_id) {
            for &u in neighbors {
                if let Some(u_gate) = id_to_gate.get(&u) {
                    let is_driven = if u_gate.pos.x != gate.pos.x {
                        u_gate.pos.x > gate.pos.x
                    } else {
                        u > node_id
                    };
                    if is_driven {
                        let d = get_max_delay(u, graph, id_to_gate, memo);
                        if d > max_downstream_delay {
                            max_downstream_delay = d;
                            best_next_node = Some(u);
                        }
                    }
                }
            }
        }

        let total_delay = gate_delay + max_downstream_delay;
        memo.insert(node_id, (total_delay, best_next_node));
        total_delay
    }

    let mut absolute_max_delay = 0.0;
    let mut best_start_node = None;

    for gate in gates {
        let mut has_incoming_driver = false;
        if let Some(neighbors) = graph.get_neighbors(gate.id) {
            for &u in neighbors {
                if let Some(u_gate) = id_to_gate.get(&u) {
                    let is_driver = if u_gate.pos.x != gate.pos.x {
                        u_gate.pos.x < gate.pos.x
                    } else {
                        u < gate.id
                    };
                    if is_driver {
                        has_incoming_driver = true;
                        break;
                    }
                }
            }
        }

        if !has_incoming_driver || gate.gate_type == GateType::Oscillator {
            let d = get_max_delay(gate.id, graph, &id_to_gate, &mut memo);
            if d > absolute_max_delay {
                absolute_max_delay = d;
                best_start_node = Some(gate.id);
            }
        }
    }

    let mut path = Vec::new();
    let mut curr = best_start_node;
    while let Some(curr_id) = curr {
        path.push(curr_id);
        curr = memo.get(&curr_id).and_then(|&(_, next)| next);
    }

    (path, absolute_max_delay)
}

fn draw_hierarchy_tree_sidebar(
    root_node: Option<&SplitNode>,
    sidebar_width: f32,
    sidebar_scroll: &mut f32,
    sidebar_hovered_node: &mut Option<Vec<usize>>,
    gates: &[VisualGate],
    camera_offset: &mut Vec2,
    camera_zoom: &mut f32,
) {
    if sidebar_width <= 5.0 {
        *sidebar_hovered_node = None;
        return;
    }

    let screen_w = screen_width();
    let screen_h = screen_height();
    let x_pos = screen_w - sidebar_width;

    // Draw sliding glassmorphic sidebar panel background
    draw_rectangle(x_pos, 125.0, sidebar_width, screen_h - 125.0, Color::from_rgba(15, 23, 42, 230));
    draw_line(x_pos, 125.0, x_pos, screen_h, 2.0, Color::from_rgba(6, 182, 212, 180)); // Cyan glowing border

    let root = match root_node {
        Some(r) => r,
        None => {
            // Draw empty state message
            let msg = "NO PARTITIONING DATA";
            let sub_msg = "PRESS SPACE TO BISECT NETLIST";
            draw_text(msg, x_pos + 40.0, 215.0, 22.0, Color::from_rgba(148, 163, 184, 180));
            draw_text(sub_msg, x_pos + 40.0, 255.0, 16.5, Color::from_rgba(148, 163, 184, 120));
            *sidebar_hovered_node = None;
            return;
        }
    };

    let mut items = Vec::new();
    flatten_tree(root, 0, &mut items);

    // Filter items to fit in scroll zone
    let item_height = 28.0;
    let list_top = 175.0;
    let list_bottom = screen_h - 30.0;
    let drawable_height = list_bottom - list_top;

    let max_scroll = (items.len() as f32 * item_height - drawable_height).max(0.0);
    *sidebar_scroll = sidebar_scroll.clamp(0.0, max_scroll);

    // Draw Title
    draw_text("HIERARCHICAL DECOMPOSITION", x_pos + 15.0, 152.0, 18.0, Color::from_rgba(6, 182, 212, 255));
    draw_line(x_pos + 15.0, 160.0, x_pos + 300.0, 160.0, 1.0, Color::from_rgba(6, 182, 212, 50));

    let mouse_pos = mouse_position();
    let mut hovered_item_idx = None;

    // Scrollable viewport clipping
    for (idx, item) in items.iter().enumerate() {
        let item_y = list_top + idx as f32 * item_height - *sidebar_scroll;
        if item_y + item_height < list_top || item_y > list_bottom {
            continue;
        }

        let is_hovered = mouse_pos.0 >= x_pos
            && mouse_pos.0 <= screen_w
            && mouse_pos.1 >= item_y
            && mouse_pos.1 < item_y + item_height;

        if is_hovered {
            hovered_item_idx = Some(idx);
        }

        // Draw selection / hover background
        if is_hovered {
            draw_rectangle(x_pos + 5.0, item_y, sidebar_width - 10.0, item_height - 2.0, Color::from_rgba(6, 182, 212, 40));
        }

        // Format hierarchical indicators
        let mut prefix = String::new();
        for _ in 0..item.depth {
            prefix.push_str("│ ");
        }
        let display_label = if item.is_leaf {
            format!("{}└── {}", prefix, item.label)
        } else {
            format!("{}├── {}", prefix, item.label)
        };

        let text_color = if is_hovered {
            Color::from_rgba(255, 255, 255, 255)
        } else if item.is_leaf {
            if let Some(pid) = item.partition_id {
                get_neon_color(pid)
            } else {
                Color::from_rgba(226, 232, 240, 200)
            }
        } else {
            Color::from_rgba(226, 232, 240, 160)
        };

        draw_text(&display_label, x_pos + 15.0, item_y + 19.0, 14.5, text_color);
        
        // Show community size / gate count badge on the right
        let size_text = format!("{}g", item.nodes.len());
        let size_color = Color::from_rgba(148, 163, 184, 150);
        draw_text(&size_text, screen_w - 60.0, item_y + 19.0, 13.5, size_color);

        // Click interaction
        if is_hovered && is_mouse_button_pressed(MouseButton::Left) {
            focus_camera_on_nodes(&item.nodes, gates, camera_offset, camera_zoom);
        }
    }

    // Update hovered node set for 2D boundary drawing
    if let Some(idx) = hovered_item_idx {
        *sidebar_hovered_node = Some(items[idx].nodes.clone());
    } else {
        *sidebar_hovered_node = None;
    }
}

fn draw_adjacency_matrix_view(
    graph: &Graph,
    gates: &[VisualGate],
    sorted_gate_ids: &[usize],
    community_blocks: &[(usize, Vec<usize>)],
    telemetry: &Telemetry,
) {
    let top_y = 130.0;
    let bottom_y = screen_height() - 50.0;
    let left_x = 50.0;
    let right_x = screen_width() - 50.0;
    
    let height = bottom_y - top_y;
    let width = right_x - left_x;
    
    let matrix_size = height.min(width).min(650.0);
    let mx_start = left_x + (width - matrix_size) / 2.0;
    let my_start = top_y + (height - matrix_size) / 2.0;
    
    // Draw background and frame
    draw_rectangle(mx_start, my_start, matrix_size, matrix_size, Color::from_rgba(20, 24, 30, 240));
    draw_rectangle_lines(mx_start, my_start, matrix_size, matrix_size, 2.0, Color::from_rgba(255, 255, 255, 30));
    
    draw_text(
        "FRACTAL ADJACENCY MATRIX // RECURSIVE DECOMPOSITION PATTERN",
        mx_start,
        my_start - 20.0,
        18.0,
        Color::from_rgba(160, 170, 190, 255),
    );
    
    let n = sorted_gate_ids.len();
    if n == 0 {
        return;
    }
    
    let cell_size = matrix_size / n as f32;
    
    // Map gate ID to sorted order index
    let mut gate_id_to_sorted_idx = HashMap::new();
    for (idx, &gate_id) in sorted_gate_ids.iter().enumerate() {
        gate_id_to_sorted_idx.insert(gate_id, idx);
    }
    
    // Draw beautiful shaded community blocks along diagonal FIRST (underneath the connections)
    let mut current_offset = 0;
    for &(part_id, ref gate_ids) in community_blocks {
        let size = gate_ids.len();
        if size == 0 { continue; }
        
        let start_pos_x = mx_start + current_offset as f32 * cell_size;
        let start_pos_y = my_start + current_offset as f32 * cell_size;
        let block_w = size as f32 * cell_size;
        
        let color = get_neon_color(part_id);
        let fill_color = Color::from_rgba(
            (color.r * 255.0) as u8,
            (color.g * 255.0) as u8,
            (color.b * 255.0) as u8,
            20, // subtle neon glow fill
        );
        let border_color = Color::from_rgba(
            (color.r * 255.0) as u8,
            (color.g * 255.0) as u8,
            (color.b * 255.0) as u8,
            120, // clean border
        );
        
        draw_rectangle(start_pos_x, start_pos_y, block_w, block_w, fill_color);
        draw_rectangle_lines(start_pos_x, start_pos_y, block_w, block_w, 1.5, border_color);
        
        current_offset += size;
    }
    
    // Draw edges in O(E)
    for &(u, v) in &graph.edges {
        if let (Some(&idx_u), Some(&idx_v)) = (gate_id_to_sorted_idx.get(&u), gate_id_to_sorted_idx.get(&v)) {
            let u_gate = &gates[u];
            let v_gate = &gates[v];
            
            let cell_color = if telemetry.is_partitioned {
                match (u_gate.partition, v_gate.partition) {
                    (Partition::Index(id_u), Partition::Index(id_v)) if id_u == id_v => {
                        let base_color = get_neon_color(id_u);
                        Color::from_rgba(
                            (base_color.r * 255.0) as u8,
                            (base_color.g * 255.0) as u8,
                            (base_color.b * 255.0) as u8,
                            220,
                        )
                    }
                    _ => Color::from_rgba(239, 68, 68, 120), // Threat Red for cut nets crossing partitions
                }
            } else {
                Color::from_rgba(6, 182, 212, 200) // Cyan for unpartitioned state
            };
            
            // Draw row=idx_u, col=idx_v
            let size = cell_size.max(1.0);
            draw_rectangle(
                mx_start + idx_v as f32 * cell_size,
                my_start + idx_u as f32 * cell_size,
                size,
                size,
                cell_color,
            );
            // Symmetric drawing
            draw_rectangle(
                mx_start + idx_u as f32 * cell_size,
                my_start + idx_v as f32 * cell_size,
                size,
                size,
                cell_color,
            );
        }
    }
    
    // Interactive mouse hovered tooltip
    let mouse_pos = mouse_position();
    let mouse_vec = vec2(mouse_pos.0, mouse_pos.1);
    if mouse_vec.x >= mx_start && mouse_vec.x <= mx_start + matrix_size
       && mouse_vec.y >= my_start && mouse_vec.y <= my_start + matrix_size
    {
        let col_idx = ((mouse_vec.x - mx_start) / cell_size) as usize;
        let row_idx = ((mouse_vec.y - my_start) / cell_size) as usize;
        if col_idx < n && row_idx < n {
            let gate_col_id = sorted_gate_ids[col_idx];
            let gate_row_id = sorted_gate_ids[row_idx];
            
            if let (Some(u_gate), Some(v_gate)) = (gates.iter().find(|g| g.id == gate_col_id), gates.iter().find(|g| g.id == gate_row_id)) {
                let has_edge = graph.get_neighbors(gate_col_id).map(|n| n.contains(&gate_row_id)).unwrap_or(false);
                let edge_status = if has_edge { "CONNECTED NET" } else { "NO CONNECTION" };
                let status_color = if has_edge { Color::from_rgba(34, 197, 94, 255) } else { Color::from_rgba(148, 163, 184, 255) };
                
                let tooltip_y = mouse_vec.y - 75.0;
                let tooltip_x = (mouse_vec.x + 15.0).min(screen_width() - 295.0);
                draw_rectangle(tooltip_x, tooltip_y, 280.0, 65.0, Color::from_rgba(15, 23, 42, 245));
                draw_rectangle_lines(tooltip_x, tooltip_y, 280.0, 65.0, 1.2, Color::from_rgba(6, 182, 212, 120));
                
                draw_text(&format!("Row Gate: {}", v_gate.label), tooltip_x + 12.0, tooltip_y + 20.0, 14.0, WHITE);
                draw_text(&format!("Col Gate: {}", u_gate.label), tooltip_x + 12.0, tooltip_y + 38.0, 14.0, WHITE);
                draw_text(edge_status, tooltip_x + 12.0, tooltip_y + 54.0, 13.0, status_color);
            }
        }
    }
}

// ==========================================================================
// 5. PCB INTERACTIVE VISUALIZER & MULTI-LAYER ENGINE (MILESTONE 2)
// ==========================================================================

const PCB_PIXELS_PER_MM: f64 = 16.0;

#[inline]
fn mm_to_world(p: Point, origin_x: f32, origin_y: f32) -> Vec2 {
    vec2(
        origin_x + (p.x * PCB_PIXELS_PER_MM) as f32,
        origin_y + (p.y * PCB_PIXELS_PER_MM) as f32,
    )
}

#[inline]
fn world_to_mm(w: Vec2, origin_x: f32, origin_y: f32) -> Point {
    Point::new(
        (w.x - origin_x) as f64 / PCB_PIXELS_PER_MM,
        (w.y - origin_y) as f64 / PCB_PIXELS_PER_MM,
    )
}

pub struct PcbViewState {
    pub board: PcbBoard,
    pub loaded_board: PcbBoard,
    pub save_path: String,
    pub selected_footprint: Option<String>,
    pub dragging_footprint: Option<(String, Point)>,
    pub hovered_footprint: Option<String>,
    pub hovered_pad: Option<(String, String, Option<NetId>)>,
    pub show_f_cu: bool,
    pub show_b_cu: bool,
    pub show_f_silk: bool,
    pub show_edge_cuts: bool,
    pub show_ratsnest: bool,
    pub grid_snap_mm: f64,
    pub camera_zoom: f32,
    pub camera_offset: Vec2,
    pub drag_pan_start: Option<Vec2>,
    pub cached_ratsnest: Vec<RatsnestLine>,
    pub ratsnest_dirty: bool,
    pub status_message: String,
    pub status_timer: f32,
}

impl PcbViewState {
    pub fn new() -> Self {
        let board = create_demo_dev_board();
        let ratsnest = compute_ratsnest(&board);
        Self {
            loaded_board: board.clone(),
            save_path: "build/edited-board.json".into(),
            board,
            selected_footprint: None,
            dragging_footprint: None,
            hovered_footprint: None,
            hovered_pad: None,
            show_f_cu: true,
            show_b_cu: true,
            show_f_silk: true,
            show_edge_cuts: true,
            show_ratsnest: true,
            grid_snap_mm: 0.5,
            camera_zoom: 1.0,
            camera_offset: Vec2::ZERO,
            drag_pan_start: None,
            cached_ratsnest: ratsnest,
            ratsnest_dirty: false,
            status_message: "PCB Layout: Click to select, Drag to move, 'R' to rotate, 1-5 toggles layers".to_string(),
            status_timer: 5.0,
        }
    }

    pub fn fit_board(&mut self) {
        self.camera_zoom=((screen_width()-60.0)/(self.board.width as f32*PCB_PIXELS_PER_MM as f32))
            .min((screen_height()-240.0)/(self.board.height as f32*PCB_PIXELS_PER_MM as f32)).clamp(0.1,2.0);
        self.camera_offset=vec2(0.0,-45.0/self.camera_zoom);
    }

    pub fn set_status(&mut self, msg: impl Into<String>) {
        self.status_message = msg.into();
        self.status_timer = 3.5;
    }
}

pub fn create_demo_dev_board() -> PcbBoard {
    let mut board = PcbBoard::new(65.0, 32.0);

    board.register_net(1, "GND");
    board.register_net(2, "VBUS_5V");
    board.register_net(3, "+3V3");
    board.register_net(4, "RESET");
    board.register_net(5, "XTAL1");
    board.register_net(6, "XTAL2");
    board.register_net(7, "USB_DP");
    board.register_net(8, "USB_DN");
    board.register_net(9, "SCK");
    board.register_net(10, "MISO");
    board.register_net(11, "MOSI");
    board.register_net(12, "LED_STAT");
    board.register_net(13, "TXD");
    board.register_net(14, "RXD");
    board.register_net(20, "LED_ANODE");

    // U1: ATmega328P MCU (TQFP-32)
    let mut u1 = tqfp32("U1", "ATmega328P").at_position(Point::new(32.5, 16.0));
    u1 = u1.with_net_assigned("3", 1)  // GND
           .with_net_assigned("5", 1)  // GND
           .with_net_assigned("21", 1) // GND
           .with_net_assigned("4", 3)  // VCC
           .with_net_assigned("6", 3)  // VCC
           .with_net_assigned("18", 3) // AVCC
           .with_net_assigned("7", 5)  // XTAL1
           .with_net_assigned("8", 6)  // XTAL2
           .with_net_assigned("29", 4) // RESET
           .with_net_assigned("15", 11)// MOSI
           .with_net_assigned("16", 10)// MISO
           .with_net_assigned("17", 9) // SCK
           .with_net_assigned("13", 12)// LED_STAT
           .with_net_assigned("30", 14)// RXD
           .with_net_assigned("31", 13);// TXD
    board.add_footprint(u1);

    // J1: USB-C Receptacle
    let mut j1 = usb_c_16pin("J1", "USB-C").at_position(Point::new(6.0, 16.0));
    j1 = j1.with_net_assigned("A1", 1)
           .with_net_assigned("B1", 1)
           .with_net_assigned("A12", 1)
           .with_net_assigned("B12", 1)
           .with_net_assigned("S1", 1)
           .with_net_assigned("S2", 1)
           .with_net_assigned("S3", 1)
           .with_net_assigned("S4", 1)
           .with_net_assigned("A4", 2)
           .with_net_assigned("B4", 2)
           .with_net_assigned("A9", 2)
           .with_net_assigned("B9", 2)
           .with_net_assigned("A6", 7)
           .with_net_assigned("B6", 7)
           .with_net_assigned("A7", 8)
           .with_net_assigned("B7", 8);
    board.add_footprint(j1);

    // U2: AMS1117-3.3 LDO Regulator (SOT-223)
    let mut u2 = sot223("U2", "AMS1117-3.3").at_position(Point::new(16.0, 24.0));
    u2 = u2.with_net_assigned("1", 1)  // GND
           .with_net_assigned("2", 3)  // VOUT (+3V3)
           .with_net_assigned("3", 2)  // VIN (VBUS)
           .with_net_assigned("4", 3); // TAB (VOUT)
    board.add_footprint(u2);

    // Y1: 16MHz Crystal (SMD-3225)
    let mut y1 = crystal_smd_3225("Y1", "16MHz").at_position(Point::new(45.0, 16.0));
    y1 = y1.with_net_assigned("1", 5)  // XTAL1
           .with_net_assigned("3", 6)  // XTAL2
           .with_net_assigned("2", 1)  // GND
           .with_net_assigned("4", 1); // GND
    board.add_footprint(y1);

    // SW1: Tactile Reset Switch (3x4mm SMD)
    let mut sw1 = tactile_switch_3x4("SW1", "RESET").at_position(Point::new(22.0, 7.0));
    sw1 = sw1.with_net_assigned("1", 4) // RESET
             .with_net_assigned("2", 1); // GND
    board.add_footprint(sw1);

    // Passives (0805 packages)
    let c1 = capacitor_0805("C1", "100nF").at_position(Point::new(26.0, 18.0)).with_net_assigned("1", 3).with_net_assigned("2", 1);
    let c2 = capacitor_0805("C2", "100nF").at_position(Point::new(39.0, 18.0)).with_net_assigned("1", 3).with_net_assigned("2", 1);
    let c3 = capacitor_0805("C3", "22pF").at_position(Point::new(49.0, 12.0)).with_net_assigned("1", 5).with_net_assigned("2", 1);
    let c4 = capacitor_0805("C4", "22pF").at_position(Point::new(49.0, 20.0)).with_net_assigned("1", 6).with_net_assigned("2", 1);
    let c5 = capacitor_0805("C5", "10uF").at_position(Point::new(11.0, 24.0)).with_net_assigned("1", 2).with_net_assigned("2", 1);
    let c6 = capacitor_0805("C6", "10uF").at_position(Point::new(21.0, 24.0)).with_net_assigned("1", 3).with_net_assigned("2", 1);
    let r1 = resistor_0805("R1", "10k").at_position(Point::new(27.0, 7.0)).with_net_assigned("1", 4).with_net_assigned("2", 3);
    let r2 = resistor_0805("R2", "1k").at_position(Point::new(32.5, 7.0)).with_net_assigned("1", 12).with_net_assigned("2", 20);
    let d1 = led_0805("D1", "GREEN").at_position(Point::new(37.0, 7.0)).with_net_assigned("1", 20).with_net_assigned("2", 1);
    let d2 = led_0805("D2", "RED").at_position(Point::new(16.0, 7.0)).with_net_assigned("1", 3).with_net_assigned("2", 1);

    board.add_footprint(c1);
    board.add_footprint(c2);
    board.add_footprint(c3);
    board.add_footprint(c4);
    board.add_footprint(c5);
    board.add_footprint(c6);
    board.add_footprint(r1);
    board.add_footprint(r2);
    board.add_footprint(d1);
    board.add_footprint(d2);

    // Headers (1x15 through-hole)
    let mut j2 = header_1x15("J2", "CONN_TOP").at_position(Point::new(14.72, 2.54));
    j2.rotation = 270.0;
    j2 = j2.with_net_assigned("1", 2)  // VBUS
           .with_net_assigned("2", 3)  // 3V3
           .with_net_assigned("3", 4)  // RESET
           .with_net_assigned("4", 1)  // GND
           .with_net_assigned("14", 14)// RXD
           .with_net_assigned("15", 13);// TXD
    board.add_footprint(j2);

    let mut j3 = header_1x15("J3", "CONN_BOT").at_position(Point::new(14.72, 29.46));
    j3.rotation = 270.0;
    j3 = j3.with_net_assigned("1", 1)  // GND
           .with_net_assigned("2", 1)  // GND
           .with_net_assigned("11", 11)// MOSI
           .with_net_assigned("12", 10)// MISO
           .with_net_assigned("13", 9);// SCK
    board.add_footprint(j3);

    // Add routed tracks and vias demonstrating live copper rendering and island clustering
    board.add_track(TrackSegment::new(Point::new(33.45, 7.0), Point::new(36.05, 7.0), 0.25, Layer::FCu, 20));
    board.add_track(TrackSegment::new(Point::new(18.3, 24.0), Point::new(20.05, 24.0), 0.35, Layer::FCu, 3));
    let v1 = Via::new(Point::new(26.0, 14.0), 0.8, 0.4, 1);
    let v2 = Via::new(Point::new(39.0, 14.0), 0.8, 0.4, 1);
    board.add_via(v1);
    board.add_via(v2);
    board.add_track(TrackSegment::new(Point::new(26.0, 14.0), Point::new(39.0, 14.0), 0.3, Layer::BCu, 1));

    board
}

fn draw_rotated_rect(center: Vec2, width_mm: f64, height_mm: f64, rot_deg: f64, color: Color) {
    let hw = width_mm / 2.0;
    let hh = height_mm / 2.0;
    let corners = [
        Point::new(-hw, -hh).rotate(rot_deg),
        Point::new( hw, -hh).rotate(rot_deg),
        Point::new( hw,  hh).rotate(rot_deg),
        Point::new(-hw,  hh).rotate(rot_deg),
    ];
    let p0 = center + vec2((corners[0].x * PCB_PIXELS_PER_MM) as f32, (corners[0].y * PCB_PIXELS_PER_MM) as f32);
    let p1 = center + vec2((corners[1].x * PCB_PIXELS_PER_MM) as f32, (corners[1].y * PCB_PIXELS_PER_MM) as f32);
    let p2 = center + vec2((corners[2].x * PCB_PIXELS_PER_MM) as f32, (corners[2].y * PCB_PIXELS_PER_MM) as f32);
    let p3 = center + vec2((corners[3].x * PCB_PIXELS_PER_MM) as f32, (corners[3].y * PCB_PIXELS_PER_MM) as f32);

    draw_triangle(p0, p1, p2, color);
    draw_triangle(p0, p2, p3, color);
}

fn draw_obround_pad(center: Vec2, width_mm: f64, height_mm: f64, rot_deg: f64, color: Color) {
    if width_mm >= height_mm {
        let radius_mm = height_mm / 2.0;
        let rect_w = width_mm - height_mm;
        let dist = rect_w / 2.0;
        draw_rotated_rect(center, rect_w, height_mm, rot_deg, color);
        let cap1_offset = Point::new(-dist, 0.0).rotate(rot_deg);
        let cap2_offset = Point::new(dist, 0.0).rotate(rot_deg);
        let cap1 = center + vec2((cap1_offset.x * PCB_PIXELS_PER_MM) as f32, (cap1_offset.y * PCB_PIXELS_PER_MM) as f32);
        let cap2 = center + vec2((cap2_offset.x * PCB_PIXELS_PER_MM) as f32, (cap2_offset.y * PCB_PIXELS_PER_MM) as f32);
        draw_circle(cap1.x, cap1.y, (radius_mm * PCB_PIXELS_PER_MM) as f32, color);
        draw_circle(cap2.x, cap2.y, (radius_mm * PCB_PIXELS_PER_MM) as f32, color);
    } else {
        let radius_mm = width_mm / 2.0;
        let rect_h = height_mm - width_mm;
        let dist = rect_h / 2.0;
        draw_rotated_rect(center, width_mm, rect_h, rot_deg, color);
        let cap1_offset = Point::new(0.0, -dist).rotate(rot_deg);
        let cap2_offset = Point::new(0.0, dist).rotate(rot_deg);
        let cap1 = center + vec2((cap1_offset.x * PCB_PIXELS_PER_MM) as f32, (cap1_offset.y * PCB_PIXELS_PER_MM) as f32);
        let cap2 = center + vec2((cap2_offset.x * PCB_PIXELS_PER_MM) as f32, (cap2_offset.y * PCB_PIXELS_PER_MM) as f32);
        draw_circle(cap1.x, cap1.y, (radius_mm * PCB_PIXELS_PER_MM) as f32, color);
        draw_circle(cap2.x, cap2.y, (radius_mm * PCB_PIXELS_PER_MM) as f32, color);
    }
}

fn draw_pcb_pad(
    pad: &Pad,
    pad_world_pos: Vec2,
    footprint_rot: f64,
    show_f_cu: bool,
    show_b_cu: bool,
    show_pin_labels: bool,
) {
    match &pad.pad_type {
        PadType::Smd { layer } => {
            let (visible, pad_color) = match layer {
                Layer::FCu => (show_f_cu, Color::from_rgba(239, 68, 68, 230)),
                Layer::BCu => (show_b_cu, Color::from_rgba(6, 182, 212, 230)),
                _ => (false, WHITE),
            };
            if !visible {
                return;
            }
            match pad.shape {
                PadShape::Rect { width, height } => {
                    draw_rotated_rect(pad_world_pos, width, height, footprint_rot, pad_color);
                }
                PadShape::Circle { diameter } => {
                    let r = (diameter * PCB_PIXELS_PER_MM / 2.0) as f32;
                    draw_circle(pad_world_pos.x, pad_world_pos.y, r, pad_color);
                }
                PadShape::Obround { width, height } => {
                    draw_obround_pad(pad_world_pos, width, height, footprint_rot, pad_color);
                }
            }
        }
        PadType::ThroughHole { drill, outer_diameter } => {
            let (visible, copper_color) = if show_f_cu && show_b_cu {
                (true, Color::from_rgba(245, 158, 11, 240))
            } else if show_f_cu {
                (true, Color::from_rgba(239, 68, 68, 230))
            } else if show_b_cu {
                (true, Color::from_rgba(6, 182, 212, 230))
            } else {
                (false, WHITE)
            };

            if visible {
                match pad.shape {
                    PadShape::Rect { width, height } => {
                        draw_rotated_rect(pad_world_pos, width, height, footprint_rot, copper_color);
                    }
                    PadShape::Circle { diameter } => {
                        let r = (diameter.max(*outer_diameter) * PCB_PIXELS_PER_MM / 2.0) as f32;
                        draw_circle(pad_world_pos.x, pad_world_pos.y, r, copper_color);
                    }
                    PadShape::Obround { width, height } => {
                        draw_obround_pad(pad_world_pos, width, height, footprint_rot, copper_color);
                    }
                }
                let drill_r = (drill * PCB_PIXELS_PER_MM / 2.0) as f32;
                draw_circle(pad_world_pos.x, pad_world_pos.y, drill_r, Color::from_rgba(15, 17, 21, 255));
                draw_circle_lines(pad_world_pos.x, pad_world_pos.y, drill_r, 1.0, Color::from_rgba(255, 255, 255, 60));
            }
        }
    }

    if show_pin_labels && !pad.pin_number.is_empty() {
        let label = &pad.pin_number;
        let font_size = 9.0;
        let text_dims = measure_text(label, None, font_size as u16, 1.0);
        let text_x = pad_world_pos.x - text_dims.width / 2.0;
        let text_y = pad_world_pos.y + text_dims.height / 2.0;
        draw_text(label, text_x, text_y, font_size, Color::from_rgba(255, 255, 255, 210));
    }
}

fn draw_pcb_footprint(
    fp: &Footprint,
    origin_x: f32,
    origin_y: f32,
    show_f_cu: bool,
    show_b_cu: bool,
    show_f_silk: bool,
    is_selected: bool,
    is_hovered: bool,
) {
    let fp_world_center = mm_to_world(fp.position, origin_x, origin_y);

    if show_f_silk {
        let silk_color = Color::from_rgba(254, 243, 199, 220);
        for (p1_mm, p2_mm) in fp.silkscreen_world_lines() {
            let p1_world = mm_to_world(p1_mm, origin_x, origin_y);
            let p2_world = mm_to_world(p2_mm, origin_x, origin_y);
            draw_line(p1_world.x, p1_world.y, p2_world.x, p2_world.y, 1.3, silk_color);
        }

        let ref_label = &fp.reference;
        let val_label = &fp.value;
        let text_dims = measure_text(ref_label, None, 11, 1.0);
        let text_pos = fp_world_center - vec2(text_dims.width / 2.0, 12.0);
        draw_text(ref_label, text_pos.x, text_pos.y, 11.0, Color::from_rgba(255, 255, 255, 240));

        let val_dims = measure_text(val_label, None, 9, 1.0);
        let val_pos = fp_world_center + vec2(-val_dims.width / 2.0, 16.0);
        draw_text(val_label, val_pos.x, val_pos.y, 9.0, Color::from_rgba(180, 190, 205, 180));
    }

    for pad in &fp.pads {
        let pad_global_mm = pad.global_position(fp.position, fp.rotation);
        let pad_world = mm_to_world(pad_global_mm, origin_x, origin_y);
        draw_pcb_pad(pad, pad_world, fp.rotation, show_f_cu, show_b_cu, show_f_silk);
    }

    let bbox = fp.world_bounding_box();
    let min_world = mm_to_world(bbox.min, origin_x, origin_y);
    let max_world = mm_to_world(bbox.max, origin_x, origin_y);
    let box_w = max_world.x - min_world.x;
    let box_h = max_world.y - min_world.y;

    if is_selected {
        let anim_pulse = ((get_time() * 4.0).sin() * 0.2 + 0.8) as f32;
        let sel_color = Color::from_rgba(249, 115, 22, (230.0 * anim_pulse) as u8);
        draw_rectangle_lines(min_world.x - 3.0, min_world.y - 3.0, box_w + 6.0, box_h + 6.0, 2.0, sel_color);

        let info = format!("{} ({}) @ ({:.2}, {:.2}) rot={:.0}°", fp.reference, fp.package, fp.position.x, fp.position.y, fp.rotation);
        let info_dims = measure_text(&info, None, 10, 1.0);
        draw_rectangle(min_world.x - 3.0, min_world.y - 18.0, info_dims.width + 10.0, 14.0, Color::from_rgba(20, 24, 30, 220));
        draw_text(&info, min_world.x + 2.0, min_world.y - 7.0, 10.0, Color::from_rgba(249, 115, 22, 255));
    } else if is_hovered {
        let hov_color = Color::from_rgba(6, 182, 212, 140);
        draw_rectangle_lines(min_world.x - 2.0, min_world.y - 2.0, box_w + 4.0, box_h + 4.0, 1.5, hov_color);
    }
}

fn draw_pcb_layout_view(pcb_state: &mut PcbViewState) {
    let board = &pcb_state.board;
    let origin_x = screen_width() / 2.0 - (board.width as f32 * PCB_PIXELS_PER_MM as f32) / 2.0;
    let origin_y = screen_height() / 2.0 + 30.0 - (board.height as f32 * PCB_PIXELS_PER_MM as f32) / 2.0;

    let pcb_camera = Camera2D {
        zoom: vec2(
            pcb_state.camera_zoom * 2.0 / screen_width(),
            pcb_state.camera_zoom * 2.0 / screen_height(),
        ),
        target: vec2(screen_width() / 2.0, screen_height() / 2.0) + pcb_state.camera_offset,
        offset: vec2(0.0, 0.0),
        ..Default::default()
    };
    set_camera(&pcb_camera);

    let board_w_px = (board.width * PCB_PIXELS_PER_MM) as f32;
    let board_h_px = (board.height * PCB_PIXELS_PER_MM) as f32;

    // Layer 1: FR4 Substrate Body
    draw_rectangle(origin_x, origin_y, board_w_px, board_h_px, Color::from_rgba(18, 48, 32, 255));
    draw_rectangle_lines(origin_x, origin_y, board_w_px, board_h_px, 1.0, Color::from_rgba(10, 30, 20, 255));

    // Layer 2: Metric Calibration Grid
    let minor_step_mm = 1.0;
    let n_x_minor = (board.width / minor_step_mm).floor() as usize;
    let n_y_minor = (board.height / minor_step_mm).floor() as usize;
    for ix in 0..=n_x_minor {
        let x_mm = ix as f64 * minor_step_mm;
        let wx = origin_x + (x_mm * PCB_PIXELS_PER_MM) as f32;
        let is_major = (ix % 5) == 0;
        if is_major {
            draw_line(wx, origin_y, wx, origin_y + board_h_px, 1.0, Color::from_rgba(255, 255, 255, 12));
        } else if pcb_state.camera_zoom >= 1.2 {
            for iy in 0..=n_y_minor {
                if (iy % 5) != 0 {
                    let y_mm = iy as f64 * minor_step_mm;
                    let wy = origin_y + (y_mm * PCB_PIXELS_PER_MM) as f32;
                    draw_circle(wx, wy, 0.8, Color::from_rgba(255, 255, 255, 18));
                }
            }
        }
    }
    for iy in 0..=n_y_minor {
        let y_mm = iy as f64 * minor_step_mm;
        let wy = origin_y + (y_mm * PCB_PIXELS_PER_MM) as f32;
        if (iy % 5) == 0 {
            draw_line(origin_x, wy, origin_x + board_w_px, wy, 1.0, Color::from_rgba(255, 255, 255, 12));
        }
    }

    // Layer 3: Board Boundary (Edge.Cuts)
    if pcb_state.show_edge_cuts {
        let edge_cuts_color = Color::from_rgba(234, 179, 8, 230);
        draw_rectangle_lines(origin_x, origin_y, board_w_px, board_h_px, 2.0, edge_cuts_color);
        let dim_text_w = format!("{:.1} mm", board.width);
        let dim_text_h = format!("{:.1} mm", board.height);
        draw_text(&dim_text_w, origin_x + board_w_px / 2.0 - 20.0, origin_y - 6.0, 10.0, edge_cuts_color);
        draw_text(&dim_text_h, origin_x - 45.0, origin_y + board_h_px / 2.0, 10.0, edge_cuts_color);
    }

    // Layer 4: Bottom Copper (B.Cu Tracks)
    if pcb_state.show_b_cu {
        let b_cu_color = Color::from_rgba(6, 182, 212, 220);
        for track in &board.tracks {
            if track.layer == Layer::BCu {
                let p1 = mm_to_world(track.start, origin_x, origin_y);
                let p2 = mm_to_world(track.end, origin_x, origin_y);
                let thickness = (track.width * PCB_PIXELS_PER_MM).max(1.5) as f32;
                draw_line(p1.x, p1.y, p2.x, p2.y, thickness, b_cu_color);
                draw_circle(p1.x, p1.y, thickness / 2.0, b_cu_color);
                draw_circle(p2.x, p2.y, thickness / 2.0, b_cu_color);
            }
        }
    }

    // Layer 5: Top Copper (F.Cu Tracks)
    if pcb_state.show_f_cu {
        let f_cu_color = Color::from_rgba(239, 68, 68, 230);
        for track in &board.tracks {
            if track.layer == Layer::FCu {
                let p1 = mm_to_world(track.start, origin_x, origin_y);
                let p2 = mm_to_world(track.end, origin_x, origin_y);
                let thickness = (track.width * PCB_PIXELS_PER_MM).max(1.5) as f32;
                draw_line(p1.x, p1.y, p2.x, p2.y, thickness, f_cu_color);
                draw_circle(p1.x, p1.y, thickness / 2.0, f_cu_color);
                draw_circle(p2.x, p2.y, thickness / 2.0, f_cu_color);
            }
        }
    }

    // Layer 6: Vias
    for via in &board.vias {
        let via_pos = mm_to_world(via.at, origin_x, origin_y);
        let outer_r = (via.size * PCB_PIXELS_PER_MM / 2.0) as f32;
        let drill_r = (via.drill * PCB_PIXELS_PER_MM / 2.0) as f32;
        if pcb_state.show_f_cu || pcb_state.show_b_cu {
            draw_circle(via_pos.x, via_pos.y, outer_r, Color::from_rgba(217, 119, 6, 240));
            draw_circle(via_pos.x, via_pos.y, drill_r, Color::from_rgba(15, 17, 21, 255));
            draw_circle_lines(via_pos.x, via_pos.y, drill_r, 1.0, Color::from_rgba(255, 255, 255, 70));
        }
    }

    // Layer 7: Component Footprints
    for fp in &board.footprints {
        let is_sel = pcb_state.selected_footprint.as_deref() == Some(&fp.reference);
        let is_hov = pcb_state.hovered_footprint.as_deref() == Some(&fp.reference);
        draw_pcb_footprint(
            fp,
            origin_x,
            origin_y,
            pcb_state.show_f_cu,
            pcb_state.show_b_cu,
            pcb_state.show_f_silk,
            is_sel,
            is_hov,
        );
    }

    // Layer 8: Dynamic Ratsnest
    if pcb_state.show_ratsnest {
        if pcb_state.ratsnest_dirty || pcb_state.dragging_footprint.is_some() {
            pcb_state.cached_ratsnest = compute_ratsnest(&pcb_state.board);
            pcb_state.ratsnest_dirty = false;
        }

        let ratsnest_color = Color::from_rgba(250, 204, 21, 220);
        let endpoint_color = Color::from_rgba(253, 224, 71, 240);

        for line in &pcb_state.cached_ratsnest {
            let p1 = mm_to_world(line.start, origin_x, origin_y);
            let p2 = mm_to_world(line.end, origin_x, origin_y);

            draw_line(p1.x, p1.y, p2.x, p2.y, 1.2, ratsnest_color);
            draw_circle(p1.x, p1.y, 2.0, endpoint_color);
            draw_circle(p2.x, p2.y, 2.0, endpoint_color);
        }
    }

    set_default_camera();
}

fn update_pcb_interactions(pcb_state: &mut PcbViewState) {
    let board_w = pcb_state.board.width;
    let board_h = pcb_state.board.height;
    let origin_x = screen_width() / 2.0 - (board_w as f32 * PCB_PIXELS_PER_MM as f32) / 2.0;
    let origin_y = screen_height() / 2.0 + 30.0 - (board_h as f32 * PCB_PIXELS_PER_MM as f32) / 2.0;

    let pcb_camera = Camera2D {
        zoom: vec2(
            pcb_state.camera_zoom * 2.0 / screen_width(),
            pcb_state.camera_zoom * 2.0 / screen_height(),
        ),
        target: vec2(screen_width() / 2.0, screen_height() / 2.0) + pcb_state.camera_offset,
        offset: vec2(0.0, 0.0),
        ..Default::default()
    };

    let mouse_pos = mouse_position();
    let mouse_world = pcb_camera.screen_to_world(vec2(mouse_pos.0, mouse_pos.1));
    let mouse_mm = world_to_mm(mouse_world, origin_x, origin_y);

    let mouse_on_hud = mouse_pos.1 <= 125.0;

    // 1. Camera Zoom
    let mouse_wheel_y = mouse_wheel().1;
    if mouse_wheel_y != 0.0 && !mouse_on_hud {
        let zoom_factor = if mouse_wheel_y > 0.0 { 1.15 } else { 0.85 };
        pcb_state.camera_zoom = (pcb_state.camera_zoom * zoom_factor).clamp(0.2, 8.0);
    }
    if is_key_down(KeyCode::Equal) || is_key_down(KeyCode::KpAdd) {
        pcb_state.camera_zoom = (pcb_state.camera_zoom * 1.02).clamp(0.2, 8.0);
    }
    if is_key_down(KeyCode::Minus) || is_key_down(KeyCode::KpSubtract) {
        pcb_state.camera_zoom = (pcb_state.camera_zoom * 0.98).clamp(0.2, 8.0);
    }

    // 2. Camera Pan
    if is_mouse_button_pressed(MouseButton::Right) || is_mouse_button_pressed(MouseButton::Middle) {
        pcb_state.drag_pan_start = Some(vec2(mouse_pos.0, mouse_pos.1));
    }
    if is_mouse_button_down(MouseButton::Right) || is_mouse_button_down(MouseButton::Middle) {
        if let Some(start) = pcb_state.drag_pan_start {
            let current = vec2(mouse_pos.0, mouse_pos.1);
            let delta = current - start;
            pcb_state.camera_offset -= delta / pcb_state.camera_zoom;
            pcb_state.drag_pan_start = Some(current);
        }
    } else {
        pcb_state.drag_pan_start = None;
    }

    if is_key_pressed(KeyCode::C) {
        pcb_state.fit_board();
        pcb_state.set_status("Board fitted to view");
    }

    // 3. Layer Visibility Hotkeys (Keys 1-5)
    if is_key_pressed(KeyCode::Key1) {
        pcb_state.show_f_cu = !pcb_state.show_f_cu;
        pcb_state.set_status(format!("Top Copper (F.Cu): {}", if pcb_state.show_f_cu { "ON" } else { "OFF" }));
    }
    if is_key_pressed(KeyCode::Key2) {
        pcb_state.show_b_cu = !pcb_state.show_b_cu;
        pcb_state.set_status(format!("Bottom Copper (B.Cu): {}", if pcb_state.show_b_cu { "ON" } else { "OFF" }));
    }
    if is_key_pressed(KeyCode::Key3) {
        pcb_state.show_f_silk = !pcb_state.show_f_silk;
        pcb_state.set_status(format!("Silkscreen (F.SilkS): {}", if pcb_state.show_f_silk { "ON" } else { "OFF" }));
    }
    if is_key_pressed(KeyCode::Key4) {
        pcb_state.show_edge_cuts = !pcb_state.show_edge_cuts;
        pcb_state.set_status(format!("Board Boundary (Edge.Cuts): {}", if pcb_state.show_edge_cuts { "ON" } else { "OFF" }));
    }
    if is_key_pressed(KeyCode::Key5) {
        pcb_state.show_ratsnest = !pcb_state.show_ratsnest;
        pcb_state.set_status(format!("Dynamic Ratsnest (EMST): {}", if pcb_state.show_ratsnest { "ON" } else { "OFF" }));
    }

    // Grid snap toggle (Key G)
    if is_key_pressed(KeyCode::G) {
        pcb_state.grid_snap_mm = match pcb_state.grid_snap_mm {
            0.0 => 0.5,
            0.5 => 1.0,
            1.0 => 2.54,
            _ => 0.0,
        };
        let snap_label = if pcb_state.grid_snap_mm == 0.0 {
            "OFF".to_string()
        } else {
            format!("{:.2} mm", pcb_state.grid_snap_mm)
        };
        pcb_state.set_status(format!("Grid Snap: {}", snap_label));
    }

    // Reset layout (Key Backspace)
    if is_key_pressed(KeyCode::Backspace) {
        pcb_state.board = pcb_state.loaded_board.clone();
        pcb_state.selected_footprint = None;
        pcb_state.dragging_footprint = None;
        pcb_state.ratsnest_dirty = true;
        pcb_state.set_status("Board layout reset to reference defaults");
    }

    if is_key_pressed(KeyCode::S) {
        let path=std::path::Path::new(&pcb_state.save_path);
        let result=(|| -> Result<(),Box<dyn std::error::Error>> {
            if let Some(parent)=path.parent() {std::fs::create_dir_all(parent)?;}
            std::fs::write(path,serde_json::to_string_pretty(&pcb_state.board)?)?;
            Ok(())
        })();
        pcb_state.set_status(match result {Ok(())=>format!("Saved {}",pcb_state.save_path),Err(e)=>format!("Save failed: {e}")});
    }
    if is_key_pressed(KeyCode::D) {
        let report=netlist_shatter_viz::pcb::validate_board(&pcb_state.board,&netlist_shatter_viz::pcb::prototype_rules());
        pcb_state.set_status(format!("DRC: {} violations; electrical: {}; geometry: {}",report.drc.total_violations(),report.electrical_issues.len(),report.geometry_issues.len()));
    }

    // 4. Hover Detection
    let mut hovered_fp = None;
    let mut hovered_pad_info = None;

    if !mouse_on_hud {
        for fp in &pcb_state.board.footprints {
            for pad in &fp.pads {
                let pad_box = pad.bounding_box(fp.position, fp.rotation);
                if pad_box.contains_point(mouse_mm) {
                    hovered_pad_info = Some((fp.reference.clone(), pad.pin_number.clone(), pad.net));
                    break;
                }
            }

            let bbox = fp.world_bounding_box().inflate(0.8);
            if bbox.contains_point(mouse_mm) {
                hovered_fp = Some(fp.reference.clone());
            }
        }
    }
    pcb_state.hovered_footprint = hovered_fp;
    pcb_state.hovered_pad = hovered_pad_info;

    // 5. Click Selection & Drag Initiation
    if is_mouse_button_pressed(MouseButton::Left) && !mouse_on_hud {
        if let Some(ref_des) = &pcb_state.hovered_footprint {
            pcb_state.selected_footprint = Some(ref_des.clone());
            if let Some(fp) = pcb_state.board.find_footprint(ref_des) {
                let offset = mouse_mm - fp.position;
                pcb_state.dragging_footprint = Some((ref_des.clone(), offset));
                pcb_state.set_status(format!("Selected {}", ref_des));
            }
        } else {
            pcb_state.selected_footprint = None;
            pcb_state.dragging_footprint = None;
        }
    }

    // 6. Active Mouse Dragging
    if is_mouse_button_down(MouseButton::Left) {
        if let Some((ref_des, offset)) = pcb_state.dragging_footprint.clone() {
            if let Some(fp) = pcb_state.board.find_footprint_mut(&ref_des) {
                let mut target_pos = mouse_mm - offset;
                if pcb_state.grid_snap_mm > 0.0 {
                    let snap = pcb_state.grid_snap_mm;
                    target_pos.x = (target_pos.x / snap).round() * snap;
                    target_pos.y = (target_pos.y / snap).round() * snap;
                }
                target_pos.x = target_pos.x.clamp(0.5, board_w - 0.5);
                target_pos.y = target_pos.y.clamp(0.5, board_h - 0.5);

                if (target_pos.x - fp.position.x).abs() > 1e-4 || (target_pos.y - fp.position.y).abs() > 1e-4 {
                    fp.position = target_pos;
                    pcb_state.ratsnest_dirty = true;
                }
            }
        }
    } else {
        pcb_state.dragging_footprint = None;
    }

    // 7. Component Rotation (Key 'R')
    if is_key_pressed(KeyCode::R) {
        let target_ref = pcb_state.selected_footprint.clone().or_else(|| pcb_state.hovered_footprint.clone());
        if let Some(ref_des) = target_ref {
            let rot_opt = if let Some(fp) = pcb_state.board.find_footprint_mut(&ref_des) {
                fp.rotation = ((fp.rotation + 90.0) % 360.0).round();
                Some(fp.rotation)
            } else {
                None
            };
            if let Some(rot) = rot_opt {
                pcb_state.ratsnest_dirty = true;
                pcb_state.set_status(format!("Rotated {} to {:.0}°", ref_des, rot));
            }
        }
    }

    // Deselect on Escape
    if is_key_pressed(KeyCode::Escape) {
        pcb_state.selected_footprint = None;
        pcb_state.dragging_footprint = None;
    }

    // Update status message timer
    let dt = get_frame_time();
    if pcb_state.status_timer > 0.0 {
        pcb_state.status_timer -= dt;
    }
}

fn draw_pcb_hud(pcb_state: &mut PcbViewState, _view_mode: &mut ViewMode) {
    let mouse_pos = mouse_position();
    let board = &pcb_state.board;
    let origin_x = screen_width() / 2.0 - (board.width as f32 * PCB_PIXELS_PER_MM as f32) / 2.0;
    let origin_y = screen_height() / 2.0 + 30.0 - (board.height as f32 * PCB_PIXELS_PER_MM as f32) / 2.0;

    let pcb_camera = Camera2D {
        zoom: vec2(
            pcb_state.camera_zoom * 2.0 / screen_width(),
            pcb_state.camera_zoom * 2.0 / screen_height(),
        ),
        target: vec2(screen_width() / 2.0, screen_height() / 2.0) + pcb_state.camera_offset,
        offset: vec2(0.0, 0.0),
        ..Default::default()
    };
    let mouse_world = pcb_camera.screen_to_world(vec2(mouse_pos.0, mouse_pos.1));
    let mouse_mm = world_to_mm(mouse_world, origin_x, origin_y);

    // Pad Tooltip Overlay
    if let Some((ref_des, pin_num, net_id)) = &pcb_state.hovered_pad {
        let net_name = net_id.and_then(|id| pcb_state.board.get_net_name(id)).unwrap_or("UNCONNECTED");
        let tip_text = format!("{}.Pin {} [Net: {}]", ref_des, pin_num, net_name);
        let tip_dims = measure_text(&tip_text, None, 13, 1.0);
        let tip_x = (mouse_pos.0 + 15.0).min(screen_width() - tip_dims.width - 20.0);
        let tip_y = (mouse_pos.1 + 25.0).min(screen_height() - 30.0);
        draw_rectangle(tip_x - 6.0, tip_y - 15.0, tip_dims.width + 12.0, 22.0, Color::from_rgba(15, 23, 42, 235));
        draw_rectangle_lines(tip_x - 6.0, tip_y - 15.0, tip_dims.width + 12.0, 22.0, 1.5, Color::from_rgba(6, 182, 212, 200));
        draw_text(&tip_text, tip_x, tip_y, 13.0, Color::from_rgba(240, 249, 255, 255));
    }

    // Bottom Bar: Status & Interactive Help
    let bot_bar_h = 52.0;
    let bot_bar_y = screen_height() - bot_bar_h;
    draw_rectangle(0.0, bot_bar_y, screen_width(), bot_bar_h, Color::from_rgba(15, 17, 21, 230));
    draw_line(0.0, bot_bar_y, screen_width(), bot_bar_y, 1.0, Color::from_rgba(255, 255, 255, 20));

    let coord_str = format!("X: {:.2} mm  Y: {:.2} mm | Snap: {} | Zoom: {:.0}%",
        mouse_mm.x.clamp(0.0, board.width),
        mouse_mm.y.clamp(0.0, board.height),
        if pcb_state.grid_snap_mm == 0.0 { "OFF".to_string() } else { format!("{:.2}mm", pcb_state.grid_snap_mm) },
        pcb_state.camera_zoom * 100.0
    );
    draw_text(&coord_str, 20.0, bot_bar_y + 20.0, 14.0, Color::from_rgba(148, 163, 184, 255));

    let status_text = if pcb_state.status_timer > 0.0 {
        &pcb_state.status_message
    } else {
        "Drag: Move | R: Rotate | 1-5: Layers | G: Snap | S: Save | D: Check | Backspace: Reset | C: Center"
    };
    draw_text(status_text, 20.0, bot_bar_y + 40.0, 11.0, Color::from_rgba(250, 204, 21, 240));

    // On-screen Layer Toggles HUD (Top Right Under Master Bar)
    let layer_panel_x = screen_width() - 360.0;
    let layer_panel_y = 135.0;
    draw_rectangle(layer_panel_x - 10.0, layer_panel_y, 350.0, 36.0, Color::from_rgba(22, 26, 33, 210));
    draw_rectangle_lines(layer_panel_x - 10.0, layer_panel_y, 350.0, 36.0, 1.0, Color::from_rgba(255, 255, 255, 30));

    let layer_buttons = [
        ("1:F.Cu", pcb_state.show_f_cu, Color::from_rgba(239, 68, 68, 255)),
        ("2:B.Cu", pcb_state.show_b_cu, Color::from_rgba(6, 182, 212, 255)),
        ("3:Silk", pcb_state.show_f_silk, Color::from_rgba(254, 243, 199, 255)),
        ("4:Edge", pcb_state.show_edge_cuts, Color::from_rgba(234, 179, 8, 255)),
        ("5:Rats", pcb_state.show_ratsnest, Color::from_rgba(250, 204, 21, 255)),
    ];

    let mut bx = layer_panel_x;
    for (label, active, col) in layer_buttons {
        let btn_w = 62.0;
        let btn_h = 24.0;
        let by = layer_panel_y + 6.0;
        let is_btn_hover = mouse_pos.0 >= bx && mouse_pos.0 <= bx + btn_w && mouse_pos.1 >= by && mouse_pos.1 <= by + btn_h;

        if is_mouse_button_pressed(MouseButton::Left) && is_btn_hover {
            match label {
                "1:F.Cu" => pcb_state.show_f_cu = !pcb_state.show_f_cu,
                "2:B.Cu" => pcb_state.show_b_cu = !pcb_state.show_b_cu,
                "3:Silk" => pcb_state.show_f_silk = !pcb_state.show_f_silk,
                "4:Edge" => pcb_state.show_edge_cuts = !pcb_state.show_edge_cuts,
                "5:Rats" => pcb_state.show_ratsnest = !pcb_state.show_ratsnest,
                _ => {}
            }
        }

        let bg_col = if active {
            Color::from_rgba(col.r as u8 / 3, col.g as u8 / 3, col.b as u8 / 3, 220)
        } else {
            Color::from_rgba(30, 35, 45, 180)
        };
        draw_rectangle(bx, by, btn_w, btn_h, bg_col);
        draw_rectangle_lines(bx, by, btn_w, btn_h, 1.0, if active { col } else { Color::from_rgba(80, 90, 105, 150) });
        let text_col = if active { col } else { Color::from_rgba(120, 130, 145, 200) };
        let lbl_dims = measure_text(label, None, 11, 1.0);
        draw_text(label, bx + (btn_w - lbl_dims.width) / 2.0, by + 16.0, 11.0, text_col);

        bx += btn_w + 6.0;
    }
}

// ==========================================================================
// 6. RENDERING CORE & MAIN COROUTINE LOOP
// ==========================================================================

#[macroquad::main("netlist-shatter-viz")]
async fn main() {
    let mut current_circuit_type = CircuitType::RippleCarryAdder8;
    let (mut graph, mut gates) = generate_circuit(current_circuit_type);

    let mut telemetry = Telemetry {
        gate_count: gates.len(),
        edge_count: 0, // Assigned below
        execution_time_us: 0,
        cross_wire_count: 0,
        is_partitioned: false,
        current_circuit: current_circuit_type,
        placement_mode: PlacementMode::Organic,
        community_count: 0,
    };
    
    let mut view_mode = ViewMode::GraphSimulation;
    let mut pcb_state = PcbViewState::new();
    let mut cli:Vec<String>=std::env::args().collect();
    if cli.len()==1 {
        if let Ok(exe)=std::env::current_exe() {
            let bundled=exe.parent().unwrap().join("../Resources/board.json");
            if bundled.exists() {cli.extend(["--pcb".to_string(),bundled.display().to_string()]);}
        }
    }
    if let Some(i)=cli.iter().position(|a|a=="--pcb") {
        let path=cli.get(i+1).expect("--pcb needs a board.json path");
        let board:PcbBoard=serde_json::from_str(&std::fs::read_to_string(path).expect("Cannot read board JSON")).expect("Invalid board JSON");
        pcb_state.board=board.clone();pcb_state.loaded_board=board;
        pcb_state.save_path=std::path::Path::new(path).with_file_name("edited-board.json").display().to_string();
        if path.contains(".app/Contents/") {
            if let Some(home)=std::env::var_os("HOME") {pcb_state.save_path=std::path::PathBuf::from(home).join("Documents/Netlist Shatter/edited-board.json").display().to_string();}
        }
        pcb_state.fit_board();
        pcb_state.ratsnest_dirty=true;view_mode=ViewMode::PcbLayout;
        pcb_state.set_status("Loaded board: S saves edited-board.json; D checks design; Backspace restores original");
    }
    let mut sorted_gate_ids: Vec<usize> = (0..gates.len()).collect();
    let mut community_blocks: Vec<(usize, Vec<usize>)> = Vec::new();

    let mut camera_zoom: f32 = 1.0;
    let mut camera_offset = vec2(0.0, 0.0);
    let mut drag_start: Option<Vec2> = None;
    let mut particles: Vec<Particle> = Vec::new();

    // Master CAD Suite State Variables
    let mut logic_states: HashMap<usize, bool> = HashMap::new();
    let mut user_overrides: HashMap<usize, bool> = HashMap::new();
    let mut probed_gate: Option<usize> = None;
    let mut show_hierarchy_hud = false;
    let mut show_critical_path = true; // wow the user on boot!
    let mut sidebar_scroll: f32 = 0.0;
    let mut root_partition_node: Option<SplitNode> = None;
    let mut sidebar_hovered_node: Option<Vec<usize>> = None;
    let mut sidebar_width: f32 = 0.0;

    // Count initial unique undirected structural connections
    let mut total_edges = 0;
    for i in 0..gates.len() {
        if let Some(n) = graph.get_neighbors(i) { total_edges += n.len(); }
    }
    telemetry.edge_count = total_edges / 2;

    loop {
        clear_background(Color::from_rgba(0x0F, 0x11, 0x15, 0xFF)); // Deep Silicon Gray

        // --- Mode Switching Inputs ---
        if is_key_pressed(KeyCode::Tab) {
            view_mode = match view_mode {
                ViewMode::GraphSimulation => ViewMode::MatrixHeatmap,
                ViewMode::MatrixHeatmap => ViewMode::PcbLayout,
                ViewMode::PcbLayout => ViewMode::GraphSimulation,
            };
        }
        if is_key_pressed(KeyCode::M) {
            view_mode = match view_mode {
                ViewMode::GraphSimulation => ViewMode::MatrixHeatmap,
                ViewMode::MatrixHeatmap => ViewMode::GraphSimulation,
                ViewMode::PcbLayout => ViewMode::GraphSimulation,
            };
        }
        if is_key_pressed(KeyCode::F1) { view_mode = ViewMode::GraphSimulation; }
        if is_key_pressed(KeyCode::F2) { view_mode = ViewMode::MatrixHeatmap; }
        if is_key_pressed(KeyCode::F3) { view_mode = ViewMode::PcbLayout; }

        let mouse_pos_raw = mouse_position();
        if is_mouse_button_pressed(MouseButton::Left) && mouse_pos_raw.1 >= 44.0 && mouse_pos_raw.1 <= 70.0 {
            if mouse_pos_raw.0 >= 20.0 && mouse_pos_raw.0 <= 215.0 {
                view_mode = ViewMode::GraphSimulation;
            } else if mouse_pos_raw.0 >= 225.0 && mouse_pos_raw.0 <= 420.0 {
                view_mode = ViewMode::MatrixHeatmap;
            } else if mouse_pos_raw.0 >= 430.0 && mouse_pos_raw.0 <= 650.0 {
                view_mode = ViewMode::PcbLayout;
            }
        }

        if view_mode == ViewMode::PcbLayout {
            update_pcb_interactions(&mut pcb_state);
        } else {
            // --- Keyboard Command Handling Input ---
            let mut reset_needed = None;
            if view_mode == ViewMode::GraphSimulation {
                if is_key_pressed(KeyCode::Key1) { reset_needed = Some(CircuitType::AluGrid); }
                if is_key_pressed(KeyCode::Key2) { reset_needed = Some(CircuitType::MeshInterconnect); }
                if is_key_pressed(KeyCode::Key3) { reset_needed = Some(CircuitType::StarClockTree); }
                if is_key_pressed(KeyCode::Key4) { reset_needed = Some(CircuitType::RippleCarryAdder8); }
                if is_key_pressed(KeyCode::Key5) { reset_needed = Some(CircuitType::ArrayMultiplier4); }
                if is_key_pressed(KeyCode::Key6) { reset_needed = Some(CircuitType::PipelineCore); }
                if is_key_pressed(KeyCode::Key7) { reset_needed = Some(CircuitType::RiscVCore); }
            }

            if let Some(new_circuit) = reset_needed {
                current_circuit_type = new_circuit;
                let (g, v) = generate_circuit(current_circuit_type);
                graph = g; gates = v;
                telemetry.is_partitioned = false;
                telemetry.execution_time_us = 0;
                telemetry.cross_wire_count = 0;
                telemetry.current_circuit = current_circuit_type;
                telemetry.gate_count = gates.len();
                
                let mut total_edges = 0;
                for i in 0..gates.len() { if let Some(n) = graph.get_neighbors(i) { total_edges += n.len(); } }
                telemetry.edge_count = total_edges / 2;
                
                sorted_gate_ids = (0..gates.len()).collect();
                community_blocks.clear();

                camera_zoom = 1.0;
                camera_offset = vec2(0.0, 0.0);

                // Clear Master CAD Suite state
                logic_states.clear();
                user_overrides.clear();
                probed_gate = None;
                root_partition_node = None;
                sidebar_hovered_node = None;
            }

            if is_key_pressed(KeyCode::Space) {
                root_partition_node = Some(execute_spectral_partition(&graph, &mut gates, &mut telemetry, &mut sorted_gate_ids, &mut community_blocks));
            }

            if is_key_pressed(KeyCode::R) && view_mode == ViewMode::GraphSimulation {
                telemetry.placement_mode = match telemetry.placement_mode {
                    PlacementMode::Organic => PlacementMode::SiliconRow,
                    PlacementMode::SiliconRow => PlacementMode::Organic,
                };
            }

            // --- Master CAD Suite Inputs and Processing ---
            if is_key_pressed(KeyCode::H) {
                show_hierarchy_hud = !show_hierarchy_hud;
            }
            if is_key_pressed(KeyCode::K) {
                show_critical_path = !show_critical_path;
            }
            if is_key_pressed(KeyCode::Escape) {
                probed_gate = None;
                user_overrides.clear();
            }

            // Smoothly interpolate the sliding sidebar width
            let sidebar_target = if show_hierarchy_hud && view_mode == ViewMode::GraphSimulation { 320.0 } else { 0.0 };
            sidebar_width += (sidebar_target - sidebar_width) * 0.15;

            // Screen-to-world mouse mapping using current camera bounds
            let camera = Camera2D {
                zoom: vec2(camera_zoom * 2.0 / screen_width(), -camera_zoom * 2.0 / screen_height()),
                target: vec2(screen_width() / 2.0, screen_height() / 2.0) + camera_offset,
                offset: vec2(0.0, 0.0),
                ..Default::default()
            };
            let mouse_pos = mouse_position();
            let mouse_world = camera.screen_to_world(vec2(mouse_pos.0, mouse_pos.1));

            // Hovered cell detection
            let mut hovered_gate_id = None;
            if view_mode == ViewMode::GraphSimulation {
                for gate in &gates {
                    let zoom_scale = (1.0 / camera_zoom).sqrt().clamp(1.0, 4.0);
                    let w = gate.gate_type.width() * zoom_scale;
                    let h = gate.gate_type.height() * zoom_scale;
                    if mouse_world.x >= gate.pos.x - w / 2.0
                        && mouse_world.x <= gate.pos.x + w / 2.0
                        && mouse_world.y >= gate.pos.y - h / 2.0
                        && mouse_world.y <= gate.pos.y + h / 2.0
                    {
                        hovered_gate_id = Some(gate.id);
                        break;
                    }
                }
            }

            // Left-click probed gate selection (ignoring clicks on the sliding HUD)
            let clicked_hud = show_hierarchy_hud && mouse_pos.0 >= screen_width() - sidebar_width;
            if is_mouse_button_pressed(MouseButton::Left) && !clicked_hud && view_mode == ViewMode::GraphSimulation {
                if let Some(gid) = hovered_gate_id {
                    if probed_gate == Some(gid) {
                        probed_gate = None;
                    } else {
                        probed_gate = Some(gid);
                    }
                } else {
                    probed_gate = None;
                }
            }

            // T key cycles overrides on the hovered logic cell
            if is_key_pressed(KeyCode::T) && view_mode == ViewMode::GraphSimulation {
                if let Some(gid) = hovered_gate_id {
                    let current_override = user_overrides.get(&gid);
                    match current_override {
                        None => { user_overrides.insert(gid, true); }
                        Some(&true) => { user_overrides.insert(gid, false); }
                        Some(&false) => { user_overrides.remove(&gid); }
                    }
                }
            }

            // P key toggles probed gate selection on the hovered logic cell
            if is_key_pressed(KeyCode::P) && view_mode == ViewMode::GraphSimulation {
                if let Some(gid) = hovered_gate_id {
                    if probed_gate == Some(gid) {
                        probed_gate = None;
                    } else {
                        probed_gate = Some(gid);
                    }
                }
            }

            // Evaluate active logic simulation states
            evaluate_logic_simulation(&graph, &gates, &mut logic_states, &user_overrides);

            // --- Camera Pan and Zoom Input Handling ---
            let mouse_wheel_y = mouse_wheel().1;
            if mouse_wheel_y != 0.0 {
                if clicked_hud {
                    // If scrolling over the sliding HUD, scroll the sidebar instead of zooming the camera
                    sidebar_scroll -= mouse_wheel_y * 15.0;
                } else {
                    let zoom_factor = if mouse_wheel_y > 0.0 { 1.15f32 } else { 0.85f32 };
                    camera_zoom = (camera_zoom * zoom_factor).clamp(0.15, 8.0);
                }
            }

            if is_key_down(KeyCode::Equal) || is_key_down(KeyCode::KpAdd) {
                camera_zoom = (camera_zoom * 1.02).clamp(0.15, 8.0);
            }
            if is_key_down(KeyCode::Minus) || is_key_down(KeyCode::KpSubtract) {
                camera_zoom = (camera_zoom * 0.98).clamp(0.15, 8.0);
            }

            if is_mouse_button_pressed(MouseButton::Right) || is_mouse_button_pressed(MouseButton::Middle) {
                drag_start = Some(vec2(mouse_position().0, mouse_position().1));
            }
            if is_mouse_button_down(MouseButton::Right) || is_mouse_button_down(MouseButton::Middle) {
                if let Some(start) = drag_start {
                    let current = vec2(mouse_position().0, mouse_position().1);
                    let delta = current - start;
                    camera_offset -= delta / camera_zoom;
                    drag_start = Some(current);
                }
            } else {
                drag_start = None;
            }

            if is_key_pressed(KeyCode::C) {
                camera_zoom = 1.0;
                camera_offset = vec2(0.0, 0.0);
            }

            // --- Physics Application Thread Tick ---
            update_physics_layout(&graph, &mut gates, &telemetry, camera_zoom);

            // --- Update Particle System Dynamics ---
            let dt = get_frame_time();
            for p in &mut particles {
                p.pos += p.vel * dt * 45.0;
                p.vel *= 0.94; // subtle drag/air resistance
                p.lifetime -= dt;
            }
            particles.retain(|p| p.lifetime > 0.0);

            // Spawn trailing sparks for rapidly moving standard cells
            for gate in &gates {
                let vel_sq = gate.vel.length_squared();
                if vel_sq > 2.0 {
                    let speed = vel_sq.sqrt();
                    let spawn_chance = (speed * 0.12).min(0.85);
                    if rand::gen_range(0.0, 1.0) < spawn_chance {
                        let dir = vec2(rand::gen_range(-1.0, 1.0), rand::gen_range(-1.0, 1.0)).normalize_or_zero();
                        let offset = dir * rand::gen_range(2.0, 6.0);
                        let p_vel = vec2(rand::gen_range(-1.0, 1.0), rand::gen_range(-1.0, 1.0)) * 2.0 - gate.vel * 0.35;
                        particles.push(Particle {
                            pos: gate.pos + offset,
                            vel: p_vel,
                            color: match gate.partition {
                                Partition::Unassigned => Color::from_rgba(0x06, 0xB6, 0xD4, 220), // Cyber Cyan Spark trail
                                Partition::Index(idx) => get_neon_color(idx),
                            },
                            lifetime: rand::gen_range(0.3, 0.65),
                            max_lifetime: 0.65,
                            size: rand::gen_range(1.2, 2.8),
                        });
                    }
                }
            }
        }

        // --- Render Phase ---
        match view_mode {
            ViewMode::GraphSimulation => {
            let camera = Camera2D {
                zoom: vec2(camera_zoom * 2.0 / screen_width(), -camera_zoom * 2.0 / screen_height()),
                target: vec2(screen_width() / 2.0, screen_height() / 2.0) + camera_offset,
                offset: vec2(0.0, 0.0),
                ..Default::default()
            };
            set_camera(&camera);

            // --- Render Phase: Silicon Floorplan Bounding Boxes ---
            if telemetry.placement_mode == PlacementMode::SiliconRow {
                let zoom_scale = (1.0 / camera_zoom).clamp(1.0, 10.0);
                let center_x = screen_width() / 2.0;
                let center_y_ref = screen_height() / 2.0 + 40.0;

                // Define shadowing closures to scale coordinates relative to center_x, center_y_ref
                let draw_rectangle = |x: f32, y: f32, w: f32, h: f32, color: Color| {
                    let rx = center_x + (x - center_x) * zoom_scale;
                    let ry = center_y_ref + (y - center_y_ref) * zoom_scale;
                    let rw = w * zoom_scale;
                    let rh = h * zoom_scale;
                    ::macroquad::prelude::draw_rectangle(rx, ry, rw, rh, color);
                };

                let draw_rectangle_lines = |x: f32, y: f32, w: f32, h: f32, thickness: f32, color: Color| {
                    let rx = center_x + (x - center_x) * zoom_scale;
                    let ry = center_y_ref + (y - center_y_ref) * zoom_scale;
                    let rw = w * zoom_scale;
                    let rh = h * zoom_scale;
                    let rthickness = thickness * zoom_scale;
                    ::macroquad::prelude::draw_rectangle_lines(rx, ry, rw, rh, rthickness, color);
                };

                let draw_line = |x1: f32, y1: f32, x2: f32, y2: f32, thickness: f32, color: Color| {
                    let rx1 = center_x + (x1 - center_x) * zoom_scale;
                    let ry1 = center_y_ref + (y1 - center_y_ref) * zoom_scale;
                    let rx2 = center_x + (x2 - center_x) * zoom_scale;
                    let ry2 = center_y_ref + (y2 - center_y_ref) * zoom_scale;
                    let rthickness = thickness * zoom_scale;
                    ::macroquad::prelude::draw_line(rx1, ry1, rx2, ry2, rthickness, color);
                };

                let draw_circle = |x: f32, y: f32, r: f32, color: Color| {
                    let rx = center_x + (x - center_x) * zoom_scale;
                    let ry = center_y_ref + (y - center_y_ref) * zoom_scale;
                    let rr = r * zoom_scale;
                    ::macroquad::prelude::draw_circle(rx, ry, rr, color);
                };

                let draw_circle_lines = |x: f32, y: f32, r: f32, thickness: f32, color: Color| {
                    let rx = center_x + (x - center_x) * zoom_scale;
                    let ry = center_y_ref + (y - center_y_ref) * zoom_scale;
                    let rr = r * zoom_scale;
                    let rthickness = thickness * zoom_scale;
                    ::macroquad::prelude::draw_circle_lines(rx, ry, rr, rthickness, color);
                };

                let draw_text = |text: &str, x: f32, y: f32, font_size: f32, color: Color| {
                    let rx = center_x + (x - center_x) * zoom_scale;
                    let ry = center_y_ref + (y - center_y_ref) * zoom_scale;
                    let rfont_size = font_size * 1.3 * zoom_scale;
                    ::macroquad::prelude::draw_text(text, rx, ry, rfont_size, color);
                };

                let padding = screen_width() * 0.08;
                let drawable_width = screen_width() - padding * 2.0;
                let center_x = screen_width() / 2.0;
                let center_y = screen_height() / 2.0;
                let top_y = 135.0;
                let bottom_y = screen_height() - 40.0;
                let block_height = bottom_y - top_y;
                let row_height = (screen_height() - 200.0) / 5.0;

                if telemetry.is_partitioned {
                    // Draw partitioned community silicon blocks
                    let c_count = telemetry.community_count.max(1);
                    let block_width = drawable_width / c_count as f32;

                    for idx in 0..c_count {
                        let bx = padding + block_width * idx as f32 + 5.0;
                        let bw = block_width - 10.0;
                        let color = get_neon_color(idx);

                        let fill_color = Color::from_rgba(
                            (color.r * 255.0) as u8,
                            (color.g * 255.0) as u8,
                            (color.b * 255.0) as u8,
                            10, // Subtle silicon well fill
                        );
                        let border_color = Color::from_rgba(
                            (color.r * 255.0) as u8,
                            (color.g * 255.0) as u8,
                            (color.b * 255.0) as u8,
                            70, // Clean border
                        );

                        // Draw shaded block region
                        draw_rectangle(bx, top_y, bw, block_height, fill_color);
                        draw_rectangle_lines(bx, top_y, bw, block_height, 1.5, border_color);

                        // Render standard cell horizontal rows inside this community column
                        for r in 0..5 {
                            let ry = 155.0 + r as f32 * row_height;
                            draw_line(bx + 10.0, ry, bx + bw - 10.0, ry, 1.0, Color::from_rgba(255, 255, 255, 8));
                        }

                        // Render block label
                        let block_name = format!("COMMUNITY BLOCK {}", idx);
                        draw_text(&block_name, bx + 15.0, top_y + 20.0, 13.0, border_color);
                    }
                } else {
                    // Original unpartitioned stage drawing logic
                    match telemetry.current_circuit {
                        CircuitType::PipelineCore => {
                            let total_stages = 4;
                            let block_width = drawable_width / total_stages as f32;
                            let stage_names = [
                                "BLOCK 0: INSTRUCTION FETCH (IF)",
                                "BLOCK 1: DECODE & REGISTERS (ID)",
                                "BLOCK 2: EXECUTION & ALU (EX)",
                                "BLOCK 3: WRITEBACK BYPASS (WB)",
                            ];
                            let stage_colors = [
                                Color::from_rgba(6, 182, 212, 10),   // Cyberpunk Cyan
                                Color::from_rgba(34, 197, 94, 10),   // Neon Green
                                Color::from_rgba(249, 115, 22, 10),  // Neon Orange
                                Color::from_rgba(168, 85, 247, 10),  // Vibrant Purple
                            ];
                            let border_colors = [
                                Color::from_rgba(6, 182, 212, 60),
                                Color::from_rgba(34, 197, 94, 60),
                                Color::from_rgba(249, 115, 22, 60),
                                Color::from_rgba(168, 85, 247, 60),
                            ];

                            for stage_idx in 0..total_stages {
                                let bx = padding + block_width * stage_idx as f32 + 5.0;
                                let bw = block_width - 10.0;
                                
                                // Draw shaded block region
                                draw_rectangle(bx, top_y, bw, block_height, stage_colors[stage_idx]);
                                draw_rectangle_lines(bx, top_y, bw, block_height, 1.2, border_colors[stage_idx]);

                                // Render standard cell horizontal rows
                                for r in 0..5 {
                                    let ry = 155.0 + r as f32 * row_height;
                                    draw_line(bx + 10.0, ry, bx + bw - 10.0, ry, 1.0, Color::from_rgba(255, 255, 255, 8));
                                }

                                // Render block label
                                draw_text(stage_names[stage_idx], bx + 15.0, top_y + 20.0, 13.0, border_colors[stage_idx]);
                            }
                        }
                        CircuitType::RippleCarryAdder8 => {
                            let total_slices = 8;
                            let slice_width = drawable_width / total_slices as f32;
                            let colors = [
                                Color::from_rgba(6, 182, 212, 8),   // Cyan
                                Color::from_rgba(34, 197, 94, 8),   // Green
                                Color::from_rgba(249, 115, 22, 8),  // Orange
                                Color::from_rgba(168, 85, 247, 8),  // Purple
                            ];
                            let border_colors = [
                                Color::from_rgba(6, 182, 212, 50),
                                Color::from_rgba(34, 197, 94, 50),
                                Color::from_rgba(249, 115, 22, 50),
                                Color::from_rgba(168, 85, 247, 50),
                            ];

                            for i in 0..total_slices {
                                let bx = padding + slice_width * i as f32 + 2.0;
                                let bw = slice_width - 4.0;
                                let c_idx = i % 4;

                                draw_rectangle(bx, top_y, bw, block_height, colors[c_idx]);
                                draw_rectangle_lines(bx, top_y, bw, block_height, 1.2, border_colors[c_idx]);

                                // Render slice row tracks
                                for r in 0..5 {
                                    let ry = 155.0 + r as f32 * row_height;
                                    draw_line(bx + 5.0, ry, bx + bw - 5.0, ry, 1.0, Color::from_rgba(255, 255, 255, 6));
                                }

                                // Label
                                let slice_name = if i == 0 {
                                    "BIT 0 (LSB)".to_string()
                                } else if i == 7 {
                                    "BIT 7 (MSB)".to_string()
                                } else {
                                    format!("BIT {}", i)
                                };
                                draw_text(&slice_name, bx + 8.0, top_y + 20.0, 11.0, border_colors[c_idx]);
                            }
                        }
                        CircuitType::ArrayMultiplier4 => {
                            // Render global bounds frame
                            draw_rectangle(padding, top_y, drawable_width, block_height, Color::from_rgba(255, 255, 255, 2));
                            draw_rectangle_lines(padding, top_y, drawable_width, block_height, 1.5, Color::from_rgba(249, 115, 22, 40));
                            draw_text("ARITHMETIC MULTIPLIER CORE // 4x4 CARRY-SAVE ARRAY DESIGN", padding + 20.0, top_y + 25.0, 14.0, Color::from_rgba(249, 115, 22, 180));

                            // Render 16 arithmetic tiles
                            for r in 0..4 {
                                for c in 0..4 {
                                    let tx = center_x + (c as f32 - 1.5) * 130.0 + (r as f32 - 1.5) * 20.0;
                                    let ty = 175.0 + r as f32 * 100.0 + 1.0 * 25.0; // center offset y

                                    let tile_w = 110.0;
                                    let tile_h = 90.0;
                                    let rx = tx - tile_w / 2.0;
                                    let ry = ty - tile_h / 2.0;

                                    // Glassmorphic cell outline
                                    draw_rectangle(rx, ry, tile_w, tile_h, Color::from_rgba(15, 23, 42, 180));
                                    draw_rectangle_lines(rx, ry, tile_w, tile_h, 1.0, Color::from_rgba(249, 115, 22, 35));

                                    // Subtle grid pattern within tile
                                    draw_line(rx + 5.0, ry + 25.0, rx + tile_w - 5.0, ry + 25.0, 1.0, Color::from_rgba(255, 255, 255, 5));
                                    draw_line(rx + 5.0, ry + 50.0, rx + tile_w - 5.0, ry + 50.0, 1.0, Color::from_rgba(255, 255, 255, 5));
                                    draw_line(rx + 5.0, ry + 75.0, rx + tile_w - 5.0, ry + 75.0, 1.0, Color::from_rgba(255, 255, 255, 5));

                                    // Label
                                    let label = format!("CELL [{},{}]", r, c);
                                    draw_text(&label, rx + 8.0, ry + 15.0, 10.0, Color::from_rgba(249, 115, 22, 130));
                                }
                            }
                        }
                        CircuitType::StarClockTree => {
                            // Render global bounds frame
                            draw_rectangle(padding, top_y, drawable_width, block_height, Color::from_rgba(255, 255, 255, 2));
                            draw_rectangle_lines(padding, top_y, drawable_width, block_height, 1.5, Color::from_rgba(6, 182, 212, 40));
                            draw_text("SYNCHRONOUS CLOCK TREE SYNTHESIS (CTS) GRID", padding + 20.0, top_y + 25.0, 14.0, Color::from_rgba(6, 182, 212, 180));

                            // Render central clock generator well
                            let osc_x = center_x;
                            let osc_y = center_y + 40.0;
                            draw_circle(osc_x, osc_y, 45.0, Color::from_rgba(239, 68, 68, 12));
                            draw_circle_lines(osc_x, osc_y, 45.0, 1.5, Color::from_rgba(239, 68, 68, 40));
                            draw_circle_lines(osc_x, osc_y, 30.0, 1.0, Color::from_rgba(239, 68, 68, 60));
                            draw_text("OSC0 ROOT", osc_x - 28.0, osc_y - 8.0, 10.0, Color::from_rgba(239, 68, 68, 160));

                            // Draw distribution feeders (H-Tree Clock routes) to the 6 buffers
                            let buffers_count = 6;
                            for b in 0..buffers_count {
                                let angle = b as f32 * 60.0_f32.to_radians();
                                let buf_x = center_x + angle.cos() * 120.0;
                                let buf_y = center_y + 40.0 + angle.sin() * 120.0;

                                // Glowing distribution lines
                                draw_line(osc_x, osc_y, buf_x, buf_y, 1.5, Color::from_rgba(34, 197, 94, 60));
                                
                                // Buffer wells
                                draw_circle(buf_x, buf_y, 20.0, Color::from_rgba(34, 197, 94, 15));
                                draw_circle_lines(buf_x, buf_y, 20.0, 1.2, Color::from_rgba(34, 197, 94, 80));
                                draw_text(&format!("BUF{}", b), buf_x - 12.0, buf_y + 4.0, 9.0, Color::from_rgba(34, 197, 94, 180));
                            }

                            // Render background silicon rows for flip-flops
                            for r in 0..5 {
                                let ry = 155.0 + r as f32 * row_height;
                                draw_line(padding + 10.0, ry, screen_width() - padding - 10.0, ry, 1.0, Color::from_rgba(255, 255, 255, 8));
                            }
                        }
                        CircuitType::MeshInterconnect => {
                            // Two large isolated processor silicon cores with a central routing corridor
                            let core_w = drawable_width * 0.44;
                            
                            // Core A (Left)
                            let ax = padding;
                            draw_rectangle(ax, top_y, core_w, block_height, Color::from_rgba(6, 182, 212, 10));
                            draw_rectangle_lines(ax, top_y, core_w, block_height, 1.5, Color::from_rgba(6, 182, 212, 70));
                            draw_text("PROCESSOR CORE A (DATA PATH / CONTROLLER)", ax + 15.0, top_y + 25.0, 13.0, Color::from_rgba(6, 182, 212, 180));
                            for r in 0..5 {
                                let ry = 155.0 + r as f32 * row_height;
                                draw_line(ax + 10.0, ry, ax + core_w - 10.0, ry, 1.0, Color::from_rgba(255, 255, 255, 6));
                            }

                            // Core B (Right)
                            let bx = padding + drawable_width * 0.56;
                            draw_rectangle(bx, top_y, core_w, block_height, Color::from_rgba(168, 85, 247, 10));
                            draw_rectangle_lines(bx, top_y, core_w, block_height, 1.5, Color::from_rgba(168, 85, 247, 70));
                            draw_text("PROCESSOR CORE B (ARITHMETIC CO-PROCESSOR)", bx + 15.0, top_y + 25.0, 13.0, Color::from_rgba(168, 85, 247, 180));
                            for r in 0..5 {
                                let ry = 155.0 + r as f32 * row_height;
                                draw_line(bx + 10.0, ry, bx + core_w - 10.0, ry, 1.0, Color::from_rgba(255, 255, 255, 6));
                            }

                            // Interconnect central channel
                            let ix = padding + core_w;
                            let iw = drawable_width * 0.12;
                            draw_rectangle(ix, top_y, iw, block_height, Color::from_rgba(255, 255, 255, 2));
                            draw_line(ix, top_y, ix, bottom_y, 1.0, Color::from_rgba(255, 255, 255, 12));
                            draw_line(ix + iw, top_y, ix + iw, bottom_y, 1.0, Color::from_rgba(255, 255, 255, 12));
                            
                            // High-speed routing buses crossing the channel
                            draw_line(ix, center_y - 40.0, ix + iw, center_y - 40.0, 1.5, Color::from_rgba(249, 115, 22, 100));
                            draw_line(ix, center_y, ix + iw, center_y, 1.5, Color::from_rgba(249, 115, 22, 100));
                            draw_line(ix, center_y + 40.0, ix + iw, center_y + 40.0, 1.5, Color::from_rgba(249, 115, 22, 100));
                            draw_text("INTER-CORE BUS", ix + 5.0, center_y - 50.0, 9.0, Color::from_rgba(249, 115, 22, 150));
                        }
                        CircuitType::AluGrid => {
                            // Massive uniform computation fabric
                            let bx = padding;
                            let bw = drawable_width;
                            draw_rectangle(bx, top_y, bw, block_height, Color::from_rgba(255, 255, 255, 3));
                            draw_rectangle_lines(bx, top_y, bw, block_height, 1.5, Color::from_rgba(34, 197, 94, 60));
                            draw_text("HIGH-DENSITY COMPUTE FABRIC // 12x12 ARITHMETIC UNIT GRID", bx + 20.0, top_y + 25.0, 14.0, Color::from_rgba(34, 197, 94, 180));

                            // 12 silicon rows snapping tracks
                            for r in 0..12 {
                                let ry = 155.0 + (r % 5) as f32 * row_height + (r / 5) as f32 * 12.0;
                                draw_line(bx + 10.0, ry, bx + bw - 10.0, ry, 1.0, Color::from_rgba(255, 255, 255, 8));
                            }
                        }
                        CircuitType::RiscVCore => {
                            let total_stages = 4;
                            let block_width = drawable_width / total_stages as f32;
                            let stage_names = [
                                "BLOCK 0: INPUT & DECODE CONTROL",
                                "BLOCK 1: REGISTER FILE STATE",
                                "BLOCK 2: ARITHMETIC UNIT (ALU)",
                                "BLOCK 3: DATA ROUTING MULTIPLEXERS",
                            ];
                            let stage_colors = [
                                Color::from_rgba(168, 85, 247, 8),
                                Color::from_rgba(6, 182, 212, 8),
                                Color::from_rgba(34, 197, 94, 8),
                                Color::from_rgba(249, 115, 22, 8),
                            ];
                            let border_colors = [
                                Color::from_rgba(168, 85, 247, 50),
                                Color::from_rgba(6, 182, 212, 50),
                                Color::from_rgba(34, 197, 94, 50),
                                Color::from_rgba(249, 115, 22, 50),
                            ];

                            for stage_idx in 0..total_stages {
                                let bx = padding + block_width * stage_idx as f32 + 5.0;
                                let bw = block_width - 10.0;
                                
                                draw_rectangle(bx, top_y, bw, block_height, stage_colors[stage_idx]);
                                draw_rectangle_lines(bx, top_y, bw, block_height, 1.2, border_colors[stage_idx]);

                                for r in 0..5 {
                                    let ry = 155.0 + r as f32 * row_height;
                                    draw_line(bx + 10.0, ry, bx + bw - 10.0, ry, 1.0, Color::from_rgba(255, 255, 255, 6));
                                }

                                draw_text(stage_names[stage_idx], bx + 15.0, top_y + 20.0, 12.0, border_colors[stage_idx]);
                            }
                        }
                    }
                }
            }

            // --- Render Phase: Interconnecting Nets (Edges) ---
            let (crit_path_nodes, _crit_path_delay) = if show_critical_path {
                compute_critical_path(&graph, &gates)
            } else {
                (Vec::new(), 0.0)
            };
            let crit_path_set: HashSet<usize> = crit_path_nodes.iter().cloned().collect();
            let mut crit_edges = HashSet::new();
            for i in 0..crit_path_nodes.len().saturating_sub(1) {
                let u = crit_path_nodes[i];
                let v = crit_path_nodes[i+1];
                crit_edges.insert((u.min(v), u.max(v)));
            }
            let mut crit_path_accumulated = HashMap::new();
            let mut acc_delay = 0.0;
            for &node_id in &crit_path_nodes {
                if let Some(gate) = gates.iter().find(|g| g.id == node_id) {
                    acc_delay += get_gate_delay_ps(gate.gate_type);
                    crit_path_accumulated.insert(node_id, acc_delay);
                }
            }

            let mut fan_in_set = HashSet::new();
            let mut fan_out_set = HashSet::new();
            if let Some(probed_id) = probed_gate {
                let mut id_to_gate = HashMap::new();
                for gate in &gates {
                    id_to_gate.insert(gate.id, gate);
                }
                get_fan_in_cone(&graph, probed_id, &id_to_gate, &mut fan_in_set);
                get_fan_out_cone(&graph, probed_id, &id_to_gate, &mut fan_out_set);
            }

            let zoom_scale = (1.0 / camera_zoom).sqrt().clamp(1.0, 4.0);
            let time_val = get_time();
            let mut rendered_edges = HashSet::new();
            for gate in &gates {
                if let Some(neighbors) = graph.get_neighbors(gate.id) {
                    for &neighbor_id in neighbors {
                        let edge_key = if gate.id < neighbor_id { (gate.id, neighbor_id) } else { (neighbor_id, gate.id) };
                        if !rendered_edges.contains(&edge_key) {
                            rendered_edges.insert(edge_key);
                            if let Some(neighbor_gate) = gates.iter().find(|g| g.id == neighbor_id) {
                                let is_critical_edge = show_critical_path && crit_edges.contains(&edge_key);
                                let is_in_fan_in = fan_in_set.contains(&gate.id) && fan_in_set.contains(&neighbor_gate.id);
                                let is_in_fan_out = fan_out_set.contains(&gate.id) && fan_out_set.contains(&neighbor_gate.id);
                                
                                let driver_high = if gate.pos.x <= neighbor_gate.pos.x {
                                    *logic_states.get(&gate.id).unwrap_or(&false)
                                } else {
                                    *logic_states.get(&neighbor_gate.id).unwrap_or(&false)
                                };

                                let mut line_color = Color::from_rgba(100, 110, 130, 60); // Clean silicon trace wire
                                let mut thickness = 1.0;
                                let is_cut = telemetry.is_partitioned && gate.partition != neighbor_gate.partition;
                                
                                if is_cut {
                                    line_color = Color::from_rgba(235, 94, 85, 140); // Threat Red Boundary Wire
                                    thickness = 1.5;
                                }

                                if is_critical_edge {
                                    line_color = Color::from_rgba(234, 179, 8, 255); // Gold core
                                    thickness = 2.0;
                                } else if is_in_fan_in {
                                    line_color = Color::from_rgba(217, 70, 239, 255); // Bright magenta
                                    thickness = 2.0;
                                } else if is_in_fan_out {
                                    line_color = Color::from_rgba(139, 92, 246, 255); // Deep purple
                                    thickness = 2.0;
                                } else if driver_high {
                                    line_color = Color::from_rgba(34, 197, 94, 200); // Neon green
                                    thickness = 1.6;
                                }

                                // Pin-to-pin routing calculations
                                let (start_pt, end_pt) = if gate.pos.x <= neighbor_gate.pos.x {
                                    let start = get_output_pin(gate, camera_zoom);
                                    let end = get_input_pin(neighbor_gate, start, camera_zoom);
                                    (start, end)
                                } else {
                                    let start = get_output_pin(neighbor_gate, camera_zoom);
                                    let end = get_input_pin(gate, start, camera_zoom);
                                    (start, end)
                                };

                                // 0. Pulsating Golden Ribbon overlay (under critical timing nets)
                                if is_critical_edge {
                                    let pulse = (time_val * 5.0).sin() as f32 * 0.25 + 0.75;
                                    let ribbon_thickness = thickness * 6.0 * zoom_scale;
                                    let ribbon_color = Color::from_rgba(234, 179, 8, (110.0 * pulse) as u8);
                                    draw_line(start_pt.x, start_pt.y, end_pt.x, end_pt.y, ribbon_thickness, ribbon_color);
                                }

                                // 1. Cybernetic glowing background wire trace (subtle glow)
                                let glow_thickness = thickness * 3.5 * zoom_scale;
                                let mut glow_color = line_color;
                                glow_color.a /= 4.0; // soft neon blur
                                draw_line(start_pt.x, start_pt.y, end_pt.x, end_pt.y, glow_thickness, glow_color);

                                // 2. Sharp physical signal core trace
                                draw_line(start_pt.x, start_pt.y, end_pt.x, end_pt.y, thickness * zoom_scale, line_color);

                                // 3. Active Neon Electron Flow Pulses
                                let pulse_count = if is_critical_edge {
                                    3
                                } else if is_cut {
                                    3
                                } else {
                                    2
                                };
                                for p in 0..pulse_count {
                                    // Stagger animation based on wire IDs to make it completely dynamic and asynchronous
                                    let phase = (p as f64 / pulse_count as f64) + (edge_key.0 + edge_key.1) as f64 * 0.19;
                                    let speed_mult = if is_critical_edge {
                                        1.8f64
                                    } else if is_in_fan_in || is_in_fan_out {
                                        1.4f64
                                    } else if driver_high {
                                        2.0f64 // double speed factor
                                    } else {
                                        0.95f64
                                    };
                                    let t = ((time_val * speed_mult + phase) % 1.0) as f32;
                                    
                                    let pulse_pos = start_pt.lerp(end_pt, t);
                                    let pulse_color = if is_critical_edge {
                                        Color::from_rgba(254, 240, 138, 255) // Glowing gold pulse
                                    } else if is_in_fan_in {
                                        Color::from_rgba(244, 63, 94, 255) // Magenta pulse
                                    } else if is_in_fan_out {
                                        Color::from_rgba(168, 85, 247, 255) // Purple pulse
                                    } else if driver_high {
                                        Color::from_rgba(34, 197, 94, 255) // Neon green pulse
                                    } else if is_cut {
                                        Color::from_rgba(255, 120, 110, 240) // Warning threat pulse
                                    } else {
                                        // Match community neon style or cyan
                                        match gate.partition {
                                            Partition::Unassigned => Color::from_rgba(0x06, 0xB6, 0xD4, 230),
                                            Partition::Index(idx) => {
                                                let c = get_neon_color(idx);
                                                Color::from_rgba((c.r * 255.0) as u8, (c.g * 255.0) as u8, (c.b * 255.0) as u8, 220)
                                            }
                                        }
                                    };
                                    let pulse_radius = (if is_critical_edge { 3.2 } else if is_cut { 2.8 } else { 2.0 }) * zoom_scale;
                                    draw_circle(pulse_pos.x, pulse_pos.y, pulse_radius, pulse_color);
                                }

                                // 4. Real-time bisection threat red sparks emission at cut points
                                if is_cut && rand::gen_range(0.0, 1.0) < 0.035 {
                                    let t = rand::gen_range(0.1, 0.9_f32);
                                    let spark_pos = start_pt.lerp(end_pt, t);
                                    let dir = (end_pt - start_pt).normalize_or_zero();
                                    let ortho = vec2(-dir.y, dir.x);
                                    let p_vel = (ortho * rand::gen_range(-1.0, 1.0) + dir * rand::gen_range(-0.5, 0.5)).normalize_or_zero() * rand::gen_range(6.0, 18.0);
                                    particles.push(Particle {
                                        pos: spark_pos,
                                        vel: p_vel,
                                        color: Color::from_rgba(255, 94, 85, 240),
                                        lifetime: rand::gen_range(0.2, 0.45),
                                        max_lifetime: 0.45,
                                        size: rand::gen_range(1.5, 3.2),
                                    });
                                }
                            }
                        }
                    }
                }
            }

            // --- Render Phase: Particle System Sparks ---
            let zoom_scale = (1.0 / camera_zoom).sqrt().clamp(1.0, 4.0);
            for p in &particles {
                let alpha = (p.lifetime / p.max_lifetime).clamp(0.0, 1.0);
                let mut c = p.color;
                c.a = alpha * p.color.a;
                draw_circle(p.pos.x, p.pos.y, p.size * zoom_scale, c);
            }

            // --- Render Phase: Primitives (Nodes) ---
            let mouse_pos = mouse_position();
            let mouse_vec = camera.screen_to_world(vec2(mouse_pos.0, mouse_pos.1));
            let zoom_scale = (1.0 / camera_zoom).sqrt().clamp(1.0, 4.0);
            for gate in &gates {
                let width = gate.gate_type.width() * zoom_scale;
                let height = gate.gate_type.height() * zoom_scale;
                let is_hovered = mouse_vec.x >= gate.pos.x - width / 2.0
                    && mouse_vec.x <= gate.pos.x + width / 2.0
                    && mouse_vec.y >= gate.pos.y - height / 2.0
                    && mouse_vec.y <= gate.pos.y + height / 2.0;

                let is_probed = probed_gate == Some(gate.id);
                let is_fan_in = fan_in_set.contains(&gate.id);
                let is_fan_out = fan_out_set.contains(&gate.id);
                let is_high = *logic_states.get(&gate.id).unwrap_or(&false);
                let crit_delay = if crit_path_set.contains(&gate.id) {
                    crit_path_accumulated.get(&gate.id).cloned()
                } else {
                    None
                };

                draw_standard_cell(
                    gate.id,
                    gate.pos,
                    gate.gate_type,
                    &gate.label,
                    gate.partition,
                    is_hovered,
                    camera_zoom,
                    is_probed,
                    is_fan_in,
                    is_fan_out,
                    is_high,
                    crit_delay,
                );
            }

            // --- Render Phase: 2D Bounding Box around hovered community ---
            if let Some(ref hovered_nodes) = sidebar_hovered_node {
                if !hovered_nodes.is_empty() {
                    let mut min_x = f32::MAX;
                    let mut max_x = f32::MIN;
                    let mut min_y = f32::MAX;
                    let mut max_y = f32::MIN;
                    let mut found = false;

                    for gate in &gates {
                        if hovered_nodes.contains(&gate.id) {
                            let w = gate.gate_type.width() * zoom_scale;
                            let h = gate.gate_type.height() * zoom_scale;
                            let left = gate.pos.x - w / 2.0;
                            let right = gate.pos.x + w / 2.0;
                            let top = gate.pos.y - h / 2.0;
                            let bottom = gate.pos.y + h / 2.0;

                            if left < min_x { min_x = left; }
                            if right > max_x { max_x = right; }
                            if top < min_y { min_y = top; }
                            if bottom > max_y { max_y = bottom; }
                            found = true;
                        }
                    }

                    if found {
                        let pad = 20.0 * zoom_scale;
                        let x = min_x - pad;
                        let y = min_y - pad;
                        let w = (max_x - min_x) + pad * 2.0;
                        let h = (max_y - min_y) + pad * 2.0;

                        // Animated glowing cyan dashed bounding box
                        let pulse = (time_val * 4.0).sin() as f32 * 0.15 + 0.85;
                        let line_color = Color::from_rgba(6, 182, 212, (200.0 * pulse) as u8);
                        let bg_color = Color::from_rgba(6, 182, 212, (15.0 * pulse) as u8);
                        
                        // Draw light shaded background
                        draw_rectangle(x, y, w, h, bg_color);
                        
                        // Draw dashed border lines using segments
                        let dash_len = 10.0 * zoom_scale;
                        let gap_len = 8.0 * zoom_scale;
                        let total_len = dash_len + gap_len;
                        let offset = (time_val * 45.0 * zoom_scale as f64) as f32 % total_len;

                        // Helper to draw a dashed line from (x1, y1) to (x2, y2)
                        let draw_dashed_line = |x1: f32, y1: f32, x2: f32, y2: f32, thickness: f32, color: Color| {
                            let dir = vec2(x2 - x1, y2 - y1);
                            let len = dir.length();
                            if len > 0.0 {
                                let dir_norm = dir / len;
                                let mut dist = offset;
                                while dist < len {
                                    let next_dist = (dist + dash_len).min(len);
                                    let start = vec2(x1, y1) + dir_norm * dist;
                                    let end = vec2(x1, y1) + dir_norm * next_dist;
                                    draw_line(start.x, start.y, end.x, end.y, thickness, color);
                                    dist += total_len;
                                }
                            }
                        };

                        let border_thickness = 2.0 * zoom_scale;
                        // Top edge
                        draw_dashed_line(x, y, x + w, y, border_thickness, line_color);
                        // Right edge
                        draw_dashed_line(x + w, y, x + w, y + h, border_thickness, line_color);
                        // Bottom edge
                        draw_dashed_line(x + w, y + h, x, y + h, border_thickness, line_color);
                        // Left edge
                        draw_dashed_line(x, y + h, x, y, border_thickness, line_color);
                    }
                }
            }

            set_default_camera();

            // --- Render Phase: Hierarchy Sidebar HUD ---
            draw_hierarchy_tree_sidebar(
                root_partition_node.as_ref(),
                sidebar_width,
                &mut sidebar_scroll,
                &mut sidebar_hovered_node,
                &gates,
                &mut camera_offset,
                &mut camera_zoom,
            );
        }
        ViewMode::MatrixHeatmap => {
            draw_adjacency_matrix_view(&graph, &gates, &sorted_gate_ids, &community_blocks, &telemetry);
        }
        ViewMode::PcbLayout => {
            draw_pcb_layout_view(&mut pcb_state);
            draw_pcb_hud(&mut pcb_state, &mut view_mode);
        }
    }

        // --- Render Phase: Telemetry Interface Overlay HUD ---
        draw_rectangle(0.0, 0.0, screen_width(), 125.0, Color::from_rgba(22, 26, 33, 235));
        draw_line(0.0, 125.0, screen_width(), 125.0, 1.5, Color::from_rgba(255, 255, 255, 20));

        draw_text("NETLIST SHATTER / PCB WORKBENCH", 20.0, 30.0, 22.0, WHITE);

        // Render Mode Selection Tabs
        let mode_tabs = [
            (ViewMode::GraphSimulation, "[ 1: Logic Simulation (F1) ]", 20.0, 195.0, Color::from_rgba(6, 182, 212, 255)),
            (ViewMode::MatrixHeatmap, "[ 2: Adjacency Matrix (F2) ]", 225.0, 195.0, Color::from_rgba(168, 85, 247, 255)),
            (ViewMode::PcbLayout, "[ 3: PCB Multi-Layer CAD (F3) ]", 430.0, 220.0, Color::from_rgba(249, 115, 22, 255)),
        ];

        for (mode, label, tx, tw, accent_col) in mode_tabs {
            let is_active = view_mode == mode;
            let ty = 44.0;
            let th = 26.0;
            let bg_col = if is_active {
                Color::from_rgba(accent_col.r as u8 / 4, accent_col.g as u8 / 4, accent_col.b as u8 / 4, 220)
            } else {
                Color::from_rgba(30, 35, 45, 160)
            };
            draw_rectangle(tx, ty, tw, th, bg_col);
            draw_rectangle_lines(tx, ty, tw, th, if is_active { 1.5 } else { 1.0 }, if is_active { accent_col } else { Color::from_rgba(75, 85, 99, 150) });
            let text_col = if is_active { accent_col } else { Color::from_rgba(156, 163, 175, 220) };
            draw_text(label, tx + 10.0, ty + 18.0, 13.5, text_col);
        }

        match view_mode {
            ViewMode::GraphSimulation => {
                let circuit_name = match telemetry.current_circuit {
                    CircuitType::AluGrid => "Array Matrix (8-Bit Arithmetic Unit Layout)",
                    CircuitType::MeshInterconnect => "Multi-Core Mesh (High Clustered Pipeline Network)",
                    CircuitType::StarClockTree => "Clock Distribution Star Tree Topology",
                    CircuitType::RippleCarryAdder8 => "8-Bit Ripple-Carry Adder (Linear Structural Chain)",
                    CircuitType::ArrayMultiplier4 => "4x4 Array Multiplier (2D Arithmetic Gate Grid)",
                    CircuitType::PipelineCore => "4-Stage RISC Processor Datapath Pipeline (IF/ID/EX/WB)",
                    CircuitType::RiscVCore => "RiscVCore (Synthesizable RISC-V Single-Cycle CPU)",
                };
                let mode_name = match telemetry.placement_mode {
                    PlacementMode::Organic => "ORGANIC FORCE-DIRECTED",
                    PlacementMode::SiliconRow => "SILICON FLOORPLAN GRID SNAPPING",
                };
                draw_text(&format!("Active Target: {} | Mode: {}", circuit_name, mode_name), 20.0, 93.0, 15.5, Color::from_rgba(160, 170, 190, 255));
                draw_text("[Commands] Tab/F1-F3: View | Space: Partition | 1-7: Target | R: Snapping | Scroll/Drag: Zoom/Pan | C: Reset Cam", 20.0, 114.0, 14.5, Color::from_rgba(34, 197, 94, 255));

                let right_align = screen_width() - 380.0;
                draw_text(&format!("Total Logic Gates: {}", telemetry.gate_count), right_align, 30.0, 19.5, WHITE);
                draw_text(&format!("Total Electrical Nets: {}", telemetry.edge_count), right_align, 52.0, 19.5, WHITE);
                if telemetry.is_partitioned {
                    draw_text(&format!("Spectral Compute Cost: {} us", telemetry.execution_time_us), right_align, 78.0, 19.5, Color::from_rgba(234, 179, 8, 255));
                    draw_text(&format!("Communities: {} | Cuts: {}", telemetry.community_count, telemetry.cross_wire_count), right_align, 102.0, 19.5, Color::from_rgba(6, 182, 212, 255));
                } else {
                    draw_text("Spectral Compute Cost: STALLED", right_align, 78.0, 19.5, Color::from_rgba(100, 110, 130, 255));
                    draw_text("Detected Communities: STALLED", right_align, 102.0, 19.5, Color::from_rgba(100, 110, 130, 255));
                }
            }
            ViewMode::MatrixHeatmap => {
                draw_text("Spectral Adjacency Matrix & Fiedler Eigenvector Community Ordering", 20.0, 93.0, 15.5, Color::from_rgba(160, 170, 190, 255));
                draw_text("[Commands] Tab/F1-F3: View | Space: Re-partition | Hover cells for net connections", 20.0, 114.0, 14.5, Color::from_rgba(168, 85, 247, 255));

                let right_align = screen_width() - 380.0;
                draw_text(&format!("Total Matrix Gates: {}", telemetry.gate_count), right_align, 30.0, 19.5, WHITE);
                draw_text(&format!("Total Electrical Nets: {}", telemetry.edge_count), right_align, 52.0, 19.5, WHITE);
                if telemetry.is_partitioned {
                    draw_text(&format!("Spectral Compute Cost: {} us", telemetry.execution_time_us), right_align, 78.0, 19.5, Color::from_rgba(234, 179, 8, 255));
                    draw_text(&format!("Communities: {} | Cuts: {}", telemetry.community_count, telemetry.cross_wire_count), right_align, 102.0, 19.5, Color::from_rgba(6, 182, 212, 255));
                } else {
                    draw_text("Spectral Compute Cost: STALLED", right_align, 78.0, 19.5, Color::from_rgba(100, 110, 130, 255));
                    draw_text("Detected Communities: STALLED", right_align, 102.0, 19.5, Color::from_rgba(100, 110, 130, 255));
                }
            }
            ViewMode::PcbLayout => {
                let sel_str = match &pcb_state.selected_footprint {
                    Some(r) => format!("Selected: {}", r),
                    None => "No component selected (Click footprint)".to_string(),
                };
                draw_text(&format!("Board: {:.1} x {:.1} mm | {}", pcb_state.board.width,pcb_state.board.height,sel_str), 20.0, 93.0, 14.0, Color::from_rgba(160, 170, 190, 255));
                draw_text("Drag: Place | R: Rotate | G: Snap | S: Save | D: Check | C: Fit | Backspace: Restore", 20.0, 114.0, 14.5, Color::from_rgba(249, 115, 22, 255));

                let summary=format!("{} parts | {} tracks | {} vias | {} airwires",pcb_state.board.footprints.len(),pcb_state.board.tracks.len(),pcb_state.board.vias.len(),pcb_state.cached_ratsnest.len());
                if screen_width()>1100.0 {
                    draw_text(&summary,screen_width()-measure_text(&summary,None,15,1.0).width-20.0,30.0,15.0,WHITE);
                }

            }
        }

        next_frame().await
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_demo_dev_board_validation_and_horizontal_headers() {
        let board = create_demo_dev_board();
        let report = board.validate();
        assert!(
            report.is_valid(),
            "create_demo_dev_board must be geometrically valid with 0 errors, got: {:?}",
            report.errors
        );

        let j2 = board.find_footprint("J2").expect("J2 must exist");
        assert_eq!(j2.rotation, 270.0, "J2 must have horizontal rotation 270.0");
        assert_eq!(j2.position, Point::new(14.72, 2.54));

        let j3 = board.find_footprint("J3").expect("J3 must exist");
        assert_eq!(j3.rotation, 270.0, "J3 must have horizontal rotation 270.0");
        assert_eq!(j3.position, Point::new(14.72, 29.46));

        // Ensure every pad on the demo dev board is within the board bounds
        let board_bounds = netlist_shatter_viz::pcb::Rect::new(Point::zero(), Point::new(board.width, board.height));
        for fp in &board.footprints {
            for pad in &fp.pads {
                let bbox = pad.bounding_box(fp.position, fp.rotation);
                assert!(
                    bbox.min.x >= board_bounds.min.x && bbox.max.x <= board_bounds.max.x,
                    "Pad {} of {} X bounds {:?} outside board bounds {:?}",
                    pad.pin_number, fp.reference, bbox, board_bounds
                );
                assert!(
                    bbox.min.y >= board_bounds.min.y && bbox.max.y <= board_bounds.max.y,
                    "Pad {} of {} Y bounds {:?} outside board bounds {:?}",
                    pad.pin_number, fp.reference, bbox, board_bounds
                );
            }
        }

        // Verify pins 1..15 of J2 span horizontally along +X axis
        let j2_pad1 = &j2.pads[0];
        let j2_pad15 = &j2.pads[14];
        let j2_p1_pos = j2_pad1.global_position(j2.position, j2.rotation);
        let j2_p15_pos = j2_pad15.global_position(j2.position, j2.rotation);
        assert!((j2_p1_pos.x - 14.72).abs() < 1e-4);
        assert!((j2_p15_pos.x - 50.28).abs() < 1e-4);
        assert!((j2_p1_pos.y - 2.54).abs() < 1e-4);
        assert!((j2_p15_pos.y - 2.54).abs() < 1e-4);

        // Verify pins 1..15 of J3 span horizontally along +X axis
        let j3_pad1 = &j3.pads[0];
        let j3_pad15 = &j3.pads[14];
        let j3_p1_pos = j3_pad1.global_position(j3.position, j3.rotation);
        let j3_p15_pos = j3_pad15.global_position(j3.position, j3.rotation);
        assert!((j3_p1_pos.x - 14.72).abs() < 1e-4);
        assert!((j3_p15_pos.x - 50.28).abs() < 1e-4);
        assert!((j3_p1_pos.y - 29.46).abs() < 1e-4);
        assert!((j3_p15_pos.y - 29.46).abs() < 1e-4);
    }
}

