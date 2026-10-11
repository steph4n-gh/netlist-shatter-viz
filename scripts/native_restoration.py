"""Immutable first-source connectivity for native routing and continuations."""
import hashlib
import json
import copy
from pathlib import Path
import shutil

from native_project import copy_assets

NAME = 'restoration-source'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate(directory, contacts_required=True):
    directory = Path(directory).resolve()
    manifest = json.loads((directory/'manifest.json').read_text())
    if manifest.get('schema') != 'native-restoration-source/1':
        raise ValueError('Unknown original restoration baseline')
    files = manifest.get('files', {})
    if not isinstance(files, dict) or not {'candidate.kicad_pcb', 'candidate.kicad_pro'} <= files.keys():
        raise ValueError('Original restoration source or project is missing')
    for name, digest in files.items():
        relative = Path(name)
        path = (directory/relative).resolve()
        if relative.is_absolute() or '..' in relative.parts or not path.is_relative_to(directory):
            raise ValueError('Original restoration asset escapes its directory')
        if not path.is_file() or sha(path) != digest:
            raise ValueError('Original restoration asset hash changed: '+name)
    if contacts_required:
        if 'native-connectivity.json' not in files:
            raise ValueError('Original native connectivity baseline is missing')
        native = json.loads((directory/'native-connectivity.json').read_text())
        if native.get('source_sha256') != files['candidate.kicad_pcb']:
            raise ValueError('Original native contacts belong to a different board')
        if native.get('contact_schema')!='physical-Cu-islands/1' or native.get('contact_graph_unambiguous')is not True:
            raise ValueError('Original restoration needs unambiguous native physical contacts')
    return manifest


def copy_baseline(source,destination):
    source,destination=Path(source),Path(destination)
    manifest=validate(source,contacts_required=False)
    destination.mkdir(parents=True,exist_ok=False)
    for name in (*manifest['files'],'manifest.json'):
        target=destination/name;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source/name,target)
    return validate(destination,contacts_required=False)


def snapshot(source, destination, profiles=None):
    """Copy the first source, or an already verified inherited baseline."""
    source, destination = Path(source).resolve(), Path(destination)
    inherited = source.parent/NAME
    if inherited.exists():
        manifest=validate(inherited, contacts_required=False)
        assets={s:source.with_suffix(s)for s in ('.kicad_pro','.kicad_dru','.dc.json','.routing.json')}
        assets.update(profiles or {})
        for suffix,original in assets.items():
            name='candidate'+suffix;expected=manifest['files'].get(name)
            present=original is not None and Path(original).is_file()
            if (expected is None and present) or (expected is not None and (not present or sha(original)!=expected)):
                raise ValueError('Continuation changed the first supplied rules/profile: '+suffix)
        return copy_baseline(inherited,destination)
    destination.mkdir(parents=True, exist_ok=False)
    names = copy_assets(source.parent, destination)
    for suffix in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
        original = source.with_suffix(suffix)
        if original.is_file():
            name = 'candidate'+suffix
            shutil.copy2(original, destination/name)
            names.append(name)
    for suffix, original in (profiles or {}).items():
        if original is not None:
            name = 'candidate'+suffix
            shutil.copy2(original, destination/name)
            names.append(name)
    if (source.parent/'source-design.json').is_file():
        shutil.copy2(source.parent/'source-design.json', destination/'source-design.json')
        names.append('source-design.json')
    manifest = {'schema':'native-restoration-source/1',
                'files':{name:sha(destination/name) for name in sorted(set(names))}}
    (destination/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return validate(destination, contacts_required=False)


def bind_contacts(directory):
    directory = Path(directory)
    manifest = validate(directory, contacts_required=False)
    native = json.loads((directory/'native-connectivity.json').read_text())
    if native.get('source_sha256') != manifest['files']['candidate.kicad_pcb']:
        raise ValueError('Native contact export changed the original restoration source')
    manifest['files']['native-connectivity.json'] = sha(directory/'native-connectivity.json')
    (directory/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return validate(directory)


def audit(directory, candidate_native, protected=()):
    """Account for every missing original owner, never only the latest cut."""
    from repair_native_congestion import native_partition_preservation
    directory = Path(directory)
    manifest = validate(directory)
    before = json.loads((directory/'native-connectivity.json').read_text())
    old, new = before['native_items'], candidate_native['native_items']
    removed = sorted(set(old)-set(new))
    allowed = sorted({item['net'] for item in old.values()}-set(protected))
    modified=sorted(uid for uid in old.keys()&new.keys()if old[uid]!=new[uid])
    edits_valid=all(old[uid]['class']in ('PCB_TRACK','PCB_VIA') and not old[uid].get('locked',True)
                    and new[uid]['class']==old[uid]['class'] and new[uid]['net']==old[uid]['net']
                    and old[uid]['net']in allowed and new[uid].get('locked',False)==old[uid].get('locked',False)
                    for uid in modified)
    # Native widening can intentionally retain a track UUID. Declare that owner
    # replaced, while all its original component's other copper/pads stay joined.
    survivors=copy.deepcopy(before)
    if edits_valid:
        for uid in modified:survivors['native_items'].pop(uid)
        nodes={node for node,row in survivors['contact_nodes'].items()if row['uuid']in modified}
        for node in nodes:
            survivors['contact_nodes'].pop(node);survivors['contact_node_items'].pop(node)
        for groups in survivors['groups'].values():
            for group in groups:
                group['contact_nodes']=[node for node in group['contact_nodes']if node not in nodes]
    result = native_partition_preservation(survivors, candidate_native, removed, allowed)
    result['pass'] &= edits_valid
    rows = result['partitions']
    result.update(required=len(rows), restored=sum(row['pass'] for row in rows),
                  original_source_sha256=manifest['files']['candidate.kicad_pcb'],
                  removed_original_copper_uuids=removed,
                  modified_original_copper_uuids=modified,original_copper_edits_valid=edits_valid,
                  removed_generated_copper_excluded=True)
    result['empty_copper_inventory_exact'] = set(candidate_native.get('empty_cu_items', [])) == set(before.get('empty_cu_items', []))-set(removed)
    result['pass'] &= result['empty_copper_inventory_exact']
    return result
