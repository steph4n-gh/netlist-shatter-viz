# v0.1.0-alpha.1

First public source alpha of the local Netlist Shatter PCB workbench.

The release provides bounded placement experiments, native KiCad creation and
routing/repair, guarded improvement, local browser review and conditional
engineering-file exports. It includes runnable examples and tests; it excludes
historical job caches, private routing policy variants and preselected
manufacturing archives.

**Known limitation: Pico Ultimate is incomplete. `RGB_DATA` and `I2C_SCL` remain
open; no completed Pico handoff is provided.** Isolated candidate path checks
do not establish simultaneous routing or native board acceptance. No included
board has been physically validated or approved for manufacturing release.

See [Quickstart](QUICKSTART.md) for installation and [Limitations](LIMITATIONS.md)
for the checked scope. The release attaches `VERIFICATION.json` recording the
exact tagged commit, source archive SHA-256, test results, platform/tool versions
and clean-install/native-routing checks. `SHA256SUMS` covers the uploaded assets.

The supported alpha host is macOS on Apple silicon. Install and launch scripts
use pinned Python packages, a locked Rust build and separately installed KiCad
10.x, Java 25+ and the exact stock Freerouting 2.4.1 JAR. Historical results using
the private patched router do not qualify this package.
