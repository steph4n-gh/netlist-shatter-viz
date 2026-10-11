# Quickstart

This alpha is a source distribution. It builds the Rust programs and creates a
local Python environment. It does not install KiCad, Java or Freerouting.

## Prerequisites

The release's installation check uses macOS on Apple silicon, Python 3.12.9
and a Rust toolchain supporting edition 2024. Other platforms are not qualified
by that check. Have `git`, `cargo` and `uv` available in your terminal. The
installer downloads Python 3.12.9 through `uv` when that interpreter is absent.

For routing and native checks, also install:

- KiCad 10.x, including `kicad-cli` and a Python runtime that imports both
  `pcbnew` and `wx`.
- The supported Freerouting JAR and a compatible Java runtime. The original
  Freerouting 2.4.1 JAR used for this alpha requires Java 25 or newer.

Follow [Toolchain](../TOOLCHAIN.md) for acquisition links and the supported JAR hash.
`KICAD_CLI`, `KICAD_PYTHON`, `JAVA` and `FREEROUTING_JAR` can identify tools when
automatic detection is insufficient. `JAVA` identifies the executable;
`FREEROUTING_JAR` identifies the JAR file. Paths with spaces should be quoted.
The doctor reports missing tools and their actual versions.

## Install and check

Clone the tagged source, or unpack the release source archive and enter its
directory:

```sh
git clone --branch v0.1.0-alpha.1 --depth 1 https://github.com/steph4n-gh/netlist-shatter-viz.git
cd netlist-shatter-viz
./scripts/install-alpha.sh
.venv/bin/python scripts/doctor.py --require-routing
```

The installer uses the included hash-checked Python dependency lock and
`Cargo.lock`, creates `.venv/`, and builds `target/release/`. Compilation and
dependency downloads require internet access. It does not modify global Python
packages. The separate KiCad, Java and Freerouting tools must already be present
for routing.

To check only the software environment before adding routing tools, omit
`--require-routing`. A successful preview environment check does not establish
that native routing is available.

## Start and try the LED example

```sh
./scripts/start-alpha.sh
```

The launcher first requires a passing routing-tool check. Open
`http://127.0.0.1:8766/native` in your browser. Select **LED bank with
electrical scenarios**, press **Create selected example**, then **Run selected
task**. Inspect the report's
connections, enabled findings, supplied profile checks and qualification limits.
If all required checks pass, the result offers its exact engineering archive.

To use another port:

```sh
./scripts/start-alpha.sh --port 8767
```

Use that port in the browser URL. Stop the server with Ctrl-C. Job files remain
under `build/layout-jobs/`; restarting does not make an incomplete result pass.

## Supply your own project

On **Your board**, choose a workflow and select one board or new-design JSON
with its matching `.kicad_pro`. Include matching `.kicad_dru`, `.dc.json` and
`.routing.json` when used. Native boards retain their supplied placement.
New-design JSON must specify exact footprint data, numbered-pad net assignments,
board dimensions and rules; it does not infer those properties from part names.

A project ZIP must contain one board and matching project/profiles at its root,
plus `fp-lib-table` and the referenced local footprint libraries. Library URIs
must use `${KIPRJMOD}/` paths within the project. Keep upstream license and notice
files. Upload size and file limits are displayed beside the file picker.

**Finish** attempts open-connection repair. **Improve** retains the original as
fallback and accepts only changes satisfying its measured nonregression checks.
List protected nets for critical copper and read the result's findings. Missing
external libraries and source-rule violations remain visible.

## Troubleshooting

- If the port is busy, choose another with `--port`.
- If the doctor cannot import `pcbnew`/`wx`, point `KICAD_PYTHON` at the KiCad
  Python runtime rather than the project's `.venv` interpreter.
- If Java rejects the Freerouting class version, select a Java 25+ executable
  with `JAVA` and rerun the doctor.
- If routing fails or times out, inspect the retained job report. A timeout is
  not evidence of physical impossibility; do not use incomplete outputs as a
  completed board.

Read [Limitations](LIMITATIONS.md) before fabricating or assembling a result.
