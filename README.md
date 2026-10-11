# Netlist Shatter

A local Rust and Python workbench for PCB layout experiments. Supply a circuit
with exact footprints or a KiCad project, inspect placements, try routing or
repair, and review the resulting native checks and manufacturing-file exports.

**Public alpha: [`v0.1.0-alpha.1`](https://github.com/steph4n-gh/netlist-shatter-viz/releases/tag/v0.1.0-alpha.1).** This is an experimental software release.
**Pico Ultimate remains incomplete: `RGB_DATA` and `I2C_SCL` are open, and there
is no completed Pico engineering handoff.** No included design has been
physically validated or approved for manufacturing release.

## Start

See [Quickstart](docs/QUICKSTART.md) for the tested installation, tool setup and
first routing example. The workbench runs on your computer and listens on
`127.0.0.1`; it is not a hosted service.

After starting it, open `http://127.0.0.1:8766/`. Use **Design** to explore
placement candidates, or **Your board** to create, finish or improve a project.
The LED-bank example is a small starting point. The RP2040 bring-up example
includes supplied placement and exact native footprint data.

## What the alpha does

- Generates bounded placement candidates for supplied model circuits.
- Creates native boards from explicit circuit and footprint JSON, including
  supplied anchors and rigid groups on rectangular boards.
- Routes an existing placement and attempts bounded repair of open connections.
- Tries improvements while preserving the original as fallback and checking
  the declared rules, source contacts and protected copper.
- Produces inspectable job reports and, when the required checks pass, matched
  KiCad, Gerber, drill and assembly-review files.

Inputs must supply the circuit, footprints and rules. The software does not
infer a circuit, select qualified parts or guarantee that an arbitrary board can
be routed. A passing report covers the checks actually run; read
[Limitations](docs/LIMITATIONS.md) before relying on a result.

## Files and licensing

The source release includes the software, tests, runnable examples and license
notices. Historical development jobs, experimental policy variants and selected
board manufacturing archives are outside this alpha package.

See [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for the
software and included assets. KiCad, Java and Freerouting are separate tools
with their own licenses. Preserve the licenses of projects you import or
redistribute.

[Release notes](docs/RELEASE_NOTES.md) describe the alpha scope and its recorded
verification. [Testing](docs/TESTING.md) lists the included local checks. Report reproducible problems through the repository's Issues
page, including the alpha version, platform, tool versions, commands and the
relevant job findings. Review project files before attaching them publicly.
