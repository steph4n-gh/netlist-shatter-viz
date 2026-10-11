//! JSON stdin/stdout bridge shared by the local workbench and batch experiments.
use netlist_shatter_viz::pcb::*;
use serde::Deserialize;
use std::io::{self, Read};
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Request {
    board: PcbBoard,
    constraints: Option<PlacementConstraints>,
    #[serde(default)]
    seed: u64,
}
fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut input = String::new();
    io::stdin().take(4_000_001).read_to_string(&mut input)?;
    if input.len() > 4_000_000 {
        return Err("Input exceeds 4 MB".into());
    }
    let mut r: Request = serde_json::from_str(&input)?;
    let c = r
        .constraints
        .unwrap_or_else(|| conservative_constraints(&r.board));
    let report = optimize_placement(&mut r.board, &c, r.seed)?;
    println!(
        "{}",
        serde_json::to_string(
            &serde_json::json!({"board":r.board,"constraints":c,"report":report})
        )?
    );
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(1)
    }
}
