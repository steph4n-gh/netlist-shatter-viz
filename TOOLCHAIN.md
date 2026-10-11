# External tools

The alpha source archive does not bundle KiCad, Java, Freerouting, uv or Rust.
Install those tools from their upstream distributors. Keep their license files
with each installation. The local install script only creates `.venv` and builds
this checkout's Rust binaries.

## Install and build

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) and
[Rust through rustup](https://rustup.rs/). Rust needs Apple's command-line build
tools on macOS (`xcode-select --install` if they are missing). The installer uses
Python **3.12.9**, downloading that interpreter through uv when necessary.
`scripts/requirements-lock.txt` pins all Python packages and accepted artifact
hashes. `Cargo.lock` pins Rust dependencies; compilation uses `--locked`.

The tested host is **macOS 27.0 arm64**. Older macOS versions and Intel Macs have
not been qualified by this alpha verification. Linux and Windows are not claimed
as supported alpha targets.

## Native PCB tools

Install the complete **KiCad 10.x** application, including its Python runtime,
from the [official macOS download page](https://www.kicad.org/download/macos/).
The alpha verification uses **KiCad 10.0.6**; the upstream page also links
previous releases. The workbench requires `kicad-cli` and KiCad's own Python with
both `pcbnew` and `wx`; installing a generic Python package named `pcbnew` does
not supply these tools.

Standard `/Applications/KiCad/KiCad.app` and `/Applications/KiCad.app` locations
are detected. For another location, set the executable paths:

```sh
export KICAD_CLI='/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
# Optional: set only if bundled Python is not detected from KICAD_CLI.
export KICAD_PYTHON='/path/to/KiCad/Python.framework/Versions/3.9/bin/python3.9'
```

Install **Java 25 or newer**, for example from
[Eclipse Temurin](https://adoptium.net/temurin/releases/). Java's executable must
actually run: macOS's `/usr/bin/java` can be a placeholder that reports no Java
runtime. The pinned Freerouting JAR has Java class version 69, which needs Java 25.

Download the upstream
[Freerouting 2.4.1 JAR](https://github.com/freerouting/freerouting/releases/download/v2.4.1/freerouting-2.4.1.jar).
The [upstream release](https://github.com/freerouting/freerouting/releases/tag/v2.4.1)
points to commit `ae3d377740b6ffa744bed1bab26625fe0278fa90`. Its JAR SHA-256 is:

```text
251101c3eeac22d7e7dfcf6796603279e5d1000283eb82d8f093780f7afc6aa9
```

After choosing a location for the JAR, check it and set paths in the shell used
to launch the workbench:

```sh
shasum -a 256 "$HOME/Downloads/freerouting-2.4.1.jar"
export JAVA="$(/usr/libexec/java_home -v 25)/bin/java"
export FREEROUTING_JAR="$HOME/Downloads/freerouting-2.4.1.jar"
.venv/bin/python scripts/doctor.py --require-routing
./scripts/start-alpha.sh
```

The startup check requires this exact upstream JAR. Its automatic fanout stays
disabled by the conservative production policy. Earlier local measurements used
a private patched router with different fanout behavior; those timings and
completion results do not qualify the stock-JAR alpha. No patched JAR or local
`.tools` directory is included in the public package.
