#!/usr/bin/env python3
"""Check the local alpha installation without routing or changing a board."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
ORIGINAL_ROUTER_SHA256 = '251101c3eeac22d7e7dfcf6796603279e5d1000283eb82d8f093780f7afc6aa9'


def command(arguments):
    try:
        result = subprocess.run(list(map(str, arguments)), capture_output=True, text=True, timeout=20)
        return result.returncode == 0, (result.stdout + result.stderr).strip()
    except (OSError, subprocess.SubprocessError) as error:
        return False, str(error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--require-routing', action='store_true', help='Fail when external routing tools are missing or incompatible')
    parser.add_argument('--json', action='store_true', help='Print the same findings as JSON')
    args = parser.parse_args()
    findings = []

    def record(name, passed, detail, routing=False):
        findings.append({'name': name, 'passed': bool(passed), 'detail': detail, 'required': not routing or args.require_routing})

    record('Python', sys.version_info[:2] == (3, 12), sys.version.split()[0] + ' (tested: 3.12.9)')
    record('Platform', sys.platform == 'darwin' and platform.machine() == 'arm64',
           sys.platform + ' ' + platform.machine() + ' (public alpha verified on macOS arm64)')
    lock = ROOT / 'scripts/requirements-lock.txt'
    try:
        versions = dict(re.findall(r'^([A-Za-z0-9_-]+)==([^\s]+)', lock.read_text(), re.M))
        mismatches = []
        for package, expected in versions.items():
            try:
                actual = importlib.metadata.version(package)
            except importlib.metadata.PackageNotFoundError:
                actual = 'missing'
            if actual != expected:
                mismatches.append(f'{package}: {actual}, expected {expected}')
        record('Python dependency lock', bool(versions) and not mismatches, '; '.join(mismatches) or f'{len(versions)} pinned packages match')
        for module in ('numpy', 'scipy', 'shapely', 'gerbonara'):
            __import__(module)
        record('Native Python libraries', True, 'NumPy, SciPy, Shapely and Gerbonara import')
    except (OSError, ImportError) as error:
        record('Python dependencies', False, str(error))

    cargo = shutil.which('cargo')
    ok, output = command([cargo, '--version']) if cargo else (False, 'cargo missing; install Rust with rustup')
    record('Cargo', ok, output)
    binaries = ('export_manufacturing', 'preview_layout', 'route_dsn')
    missing = [name for name in binaries if not os.access(ROOT / 'target/release' / name, os.X_OK)]
    record('Workbench binaries', not missing, ', '.join(missing) + ' missing; run installer' if missing else ', '.join(binaries))

    sys.path.insert(0, str(ROOT / 'scripts'))
    try:
        from build_board import find_kicad
        from solve_kicad import find_python
        kicad = find_kicad()
        ok, output = command([kicad, 'version']) if kicad else (False, 'Install KiCad 10.x, or set KICAD_CLI')
        version = re.search(r'\b(\d+)\.(\d+)\.(\d+)\b', output)
        record('KiCad CLI', ok and version is not None and int(version[1]) == 10, output, routing=True)
        python = find_python(kicad) if kicad else None
        ok, output = command([python, '-c', 'import pcbnew,wx;print(pcbnew.GetBuildVersion());print(wx.version())']) if python else (False, 'KiCad Python unavailable; set KICAD_PYTHON')
        record('KiCad Python', ok, output, routing=True)
    except (ImportError, ValueError, TypeError) as error:
        record('KiCad', False, str(error), routing=True)

    java = os.getenv('JAVA') or shutil.which('java')
    ok, output = command([java, '-version']) if java else (False, 'Install Java 25+ and set JAVA to its executable')
    version = re.search(r'version "(\d+)', output)
    record('Java', ok and version is not None and int(version[1]) >= 25, output, routing=True)
    jar = os.getenv('FREEROUTING_JAR')
    try:
        path = Path(jar) if jar else None
        digest = hashlib.sha256(path.read_bytes()).hexdigest() if path else None
        if path:
            with zipfile.ZipFile(path) as archive:
                class_bytes = archive.read('app/freerouting/Freerouting.class')
                router_class = class_bytes[:4] == b'\xca\xfe\xba\xbe'
        else:
            router_class = False
        record('Freerouting 2.4.1', digest == ORIGINAL_ROUTER_SHA256 and router_class,
               str(path) + ' SHA256=' + str(digest) if path else 'Set FREEROUTING_JAR to the upstream 2.4.1 JAR; see QUICKSTART.md', routing=True)
    except (OSError, zipfile.BadZipFile, KeyError) as error:
        record('Freerouting 2.4.1', False, str(error), routing=True)
    passed = all(item['passed'] for item in findings if item['required'])
    result = {'passed': passed, 'routing_required': args.require_routing, 'checks': findings}
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        for item in findings:
            label = 'PASS' if item['passed'] else 'FAIL' if item['required'] else 'OPTIONAL'
            print(f"{label}: {item['name']}: {item['detail']}")
        print('Doctor passed.' if passed else 'Doctor failed; correct the findings above.')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
