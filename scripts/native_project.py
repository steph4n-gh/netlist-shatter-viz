"""Carry project-local footprint libraries with native board jobs."""
from pathlib import Path
import json
import shutil
import re
from kicad_sexpr import sexpr


def copy_assets(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    table = source / 'fp-lib-table'
    paths = [Path(name) for name in ('LICENSE','LICENSE.txt','LICENSE.md','NOTICE','NOTICE.txt') if (source/name).is_file()]
    if table.exists():paths.append(Path('fp-lib-table'))
    for entry in sexpr(table.read_text())[1:] if table.exists() else []:
        if not isinstance(entry, list) or not entry or entry[0] != 'lib':
            continue
        uri = next((v[1] for v in entry if isinstance(v, list) and v[0] == 'uri'), None)
        uri = json.loads(uri) if uri and uri.startswith('"') else uri
        if not uri or not uri.startswith('${KIPRJMOD}/'):
            continue  # Global libraries remain governed by KiCad's environment.
        relative = Path(uri[len('${KIPRJMOD}/'):])
        library = (source / relative).resolve()
        if relative.is_absolute() or '..' in relative.parts or not library.is_relative_to(source):
            raise ValueError('Project footprint libraries must remain inside the project')
        if not library.is_dir():
            raise ValueError('Missing project footprint library: ' + str(relative))
        paths.extend(p.relative_to(source) for p in sorted(library.rglob('*')) if p.is_file())
    for relative in paths:
        original, target = source / relative, destination / relative
        if original.resolve() == target.resolve():
            continue
        if not original.resolve().is_relative_to(source):
            raise ValueError('Project library symlink escapes its project')
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original, target)
    return [str(p) for p in paths]


def routing_references(dsn, footprints):
    """Resolve KiCad's DSN aliases by exact exported pose, including duplicate refs."""
    if not re.search(r'\(unit\s+um\)',dsn):raise ValueError('Expected native DSN micrometre units')
    token=r'("(?:\\.|[^"\\])*"|[^\s()]+)'
    pattern=r'\(place\s+'+token+r'\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+(front|back)\s+'
    result={};aliases=set()
    for match in re.finditer(pattern,dsn):
        name,x,y,side=match.groups();name=json.loads(name) if name.startswith('"') else name
        x,y=float(x)/1000,-float(y)/1000
        candidates=[f for f in footprints if f['id'] not in result and f['side']==side
                    and abs(f['position'][0]-x)<=.00001 and abs(f['position'][1]-y)<=.00001]
        # Disambiguate coincident pads/footprints with their reference when possible.
        exact=[f for f in candidates if f['reference']==name]
        if len(exact)==1:candidates=exact
        if len(candidates)!=1 or name in aliases:raise ValueError('Ambiguous DSN component alias: '+name)
        result[candidates[0]['id']]=name;aliases.add(name)
    if set(result)!={f['id'] for f in footprints}:raise ValueError('DSN footprint inventory does not match native board')
    return result
