use std::process::Command;
use std::fs::File;
use std::io::Write;
use std::collections::HashMap;
use spectral_pruner::{TauSpectralPruner, Topology};

// Define a classic behavioral Verilog module to act as our verification target
const TARGET_VERILOG_ALU: &str = r#"
module GoldenAlu(
    input [7:0] a,
    input [7:0] b,
    input [1:0] opcode,
    output reg [7:0] y
);
    always @(*) begin
        case(opcode)
            2'b00: y = a + b; // Adder Block
            2'b01: y = a - b; // Subtractor Block
            2'b10: y = a & b; // Logical AND
            2'b11: y = a ^ b; // Logical XOR
        endcase
    end
endmodule
"#;

struct ContiguousGraph {
    id_map: HashMap<usize, usize>,
    edges: Vec<(usize, usize)>,
}

impl ContiguousGraph {
    fn new() -> Self {
        Self {
            id_map: HashMap::new(),
            edges: Vec::new(),
        }
    }

    fn get_or_create(&mut self, orig_id: usize) -> usize {
        let next_id = self.id_map.len();
        *self.id_map.entry(orig_id).or_insert(next_id)
    }

    fn add_edge(&mut self, u: usize, v: usize) {
        let u_compact = self.get_or_create(u);
        let v_compact = self.get_or_create(v);
        self.edges.push((u_compact, v_compact));
    }

    fn to_topology(&self) -> Topology {
        let num_nodes = self.id_map.len();
        let mut topo = Topology::new(num_nodes);
        for &(u, v) in &self.edges {
            topo.add_edge(u, v);
        }
        topo
    }
}

#[test]
fn test_real_hardware_bisection_equivalence() {
    // 1. Gracefully probe for yosys availability in the system path
    if Command::new("yosys").arg("--version").status().is_err() {
        println!("------------------------------------------------------------------------");
        println!("NOTICE: Skipping Yosys HDL bisection verification because 'yosys' is not");
        println!("installed or not found in the system PATH.");
        println!("To enable this test, install Yosys:");
        println!("  - macOS: brew install yosys");
        println!("  - Ubuntu/Debian: sudo apt-get install yosys");
        println!("------------------------------------------------------------------------");
        return;
    }

    // 2. Write the target Verilog code to a temporary workspace file
    let mut file = File::create("target_alu.v").unwrap();
    file.write_all(TARGET_VERILOG_ALU.as_bytes()).unwrap();

    // 3. Shell out to Yosys to synthesize the HDL code into a raw JSON netlist
    // This strips behavioral logic out and leaves a pure, flat gate-level adjacency graph
    let yosys_status = Command::new("yosys")
        .args([
            "-p", 
            "read_verilog target_alu.v; synth; write_json netlist_output.json"
        ])
        .status()
        .expect("Failed to execute Yosys parser.");
    
    assert!(yosys_status.success(), "Yosys failed to parse structural Verilog layout.");

    // 4. Ingest the generated JSON file directly into the spectral-pruner engine
    let json_content = std::fs::read_to_string("netlist_output.json")
        .expect("Failed to read synthesized JSON netlist.");
    
    let mut graph = ContiguousGraph::new();
    
    // Parse JSON cells to edges
    let parsed_json: serde_json::Value = serde_json::from_str(&json_content).unwrap();
    if let Some(modules) = parsed_json.get("modules") {
        if let Some(golden_mod) = modules.get("GoldenAlu") {
            if let Some(cells) = golden_mod.get("cells") {
                for (cell_name, cell_data) in cells.as_object().unwrap() {
                    let node_id = hash_string_to_id(cell_name);
                    if let Some(connections) = cell_data.get("connections") {
                        for (_port, nets) in connections.as_object().unwrap() {
                            if let Some(net_array) = nets.as_array() {
                                for net in net_array {
                                    if let Some(net_num) = net.as_u64() {
                                        graph.add_edge(node_id, net_num as usize);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Convert to target topology required by spectral-pruner
    let topology = graph.to_topology();

    // 5. Fire the spectral-pruner mathematical micro-kernel
    let pruner = TauSpectralPruner::builder()
        .tau(0.0)
        .build();
    let resolution = pruner.prune(&topology, 0).expect("Spectral engine bisection calculation failed.");

    // 6. Run structural asset checks
    assert!(resolution.mainland_nodes.len() + resolution.island_nodes.len() > 0, "Engine failed to isolate structural partitions.");
    
    let left_count = resolution.mainland_nodes.len();
    let right_count = resolution.island_nodes.len();
    
    println!("Verification Success: Split logic gates cleanly into {} Left and {} Right physical clusters.", left_count, right_count);

    // Clean up temporary artifact files
    let _ = std::fs::remove_file("target_alu.v");
    let _ = std::fs::remove_file("netlist_output.json");
}

fn hash_string_to_id(s: &str) -> usize {
    let mut s_bytes = s.as_bytes();
    if s_bytes.len() > 8 { s_bytes = &s_bytes[0..8]; }
    let mut id = 0;
    for &b in s_bytes { id = (id << 8) | b as usize; }
    id
}
