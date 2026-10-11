#!/usr/bin/env python3
"""KiCad-owned geometry operations. Run with KiCad's pcbnew Python runtime."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import re
from pathlib import Path
import pcbnew
import wx
from kicad_sexpr import sexpr,zone_definition


def measure(board):
    nets = {}
    for net in board.GetNetsByNetcode().values():
        nets[str(net.GetNetCode())] = {'name': net.GetNetname(), 'length_mm': 0, 'vias': 0,
            'locked': False, 'width_mm': 0, 'min_width_mm': None, 'layers': [], 'pads': 0, 'zone': False}
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            nets[str(pad.GetNetCode())]['pads'] += 1
    for zone in board.Zones():
        if not zone.GetIsRuleArea() and not zone.IsTeardropArea():nets[str(zone.GetNetCode())]['zone'] = True
    for track in board.GetTracks():
        n = nets[str(track.GetNetCode())]
        n['locked'] |= track.IsLocked()
        if isinstance(track, pcbnew.PCB_VIA):n['vias'] += 1
        else:
            n['length_mm'] += pcbnew.ToMM(track.GetLength())
            n['width_mm'] = max(n['width_mm'], pcbnew.ToMM(track.GetWidth()))
            width = pcbnew.ToMM(track.GetWidth())
            n['min_width_mm'] = width if n['min_width_mm'] is None else min(n['min_width_mm'], width)
            if track.GetLayerName() not in n['layers']:n['layers'].append(track.GetLayerName())
    return {'length_mm': sum(n['length_mm'] for n in nets.values()),
            'vias': sum(n['vias'] for n in nets.values()), 'nets': nets, 'inventory': inventory(board)}


def inventory(board):
    return {
        'copper_layers': board.GetCopperLayerCount(),
        'footprints': len(list(board.GetFootprints())),
        'pads': sum(len(list(f.Pads())) for f in board.GetFootprints()),
        'tracks_and_vias': len(list(board.GetTracks())),
        'zones': len(list(board.Zones())),
        'locked_tracks_and_vias': sum(t.IsLocked() for t in board.GetTracks()),
        'nets': len(board.GetNetsByNetcode()),
    }


def export_assembly(board, directory):
    """Record native footprint anchors and literal fields; no machine-angle conversion."""
    target = directory / 'assembly'
    target.mkdir(exist_ok=True)
    frames, bom, dnp_bom, positions, dnp_positions = [], [], [], [], []
    for fp in sorted(board.GetFootprints(), key=lambda f: (f.GetReference(), f.m_Uuid.AsString())):
        point = fp.GetPosition()
        properties = [{'name': field.GetName(), 'value': field.GetText()} for field in fp.GetFields()]
        fields = {field['name']: field['value'] for field in properties}
        frame = {'uuid': fp.m_Uuid.AsString(), 'reference': fp.GetReference(), 'value': fp.GetValue(),
                 'footprint_id': str(fp.GetFPID().GetUniStringLibId()),
                 'position_mm': [pcbnew.ToMM(point.x), pcbnew.ToMM(point.y)],
                 'rotation_degrees': fp.GetOrientationDegrees(),
                 'side': 'bottom' if fp.IsFlipped() else 'top', 'dnp': bool(fp.IsDNP()),
                 'excluded_from_bom': bool(fp.IsExcludedFromBOM()),
                 'excluded_from_positions': bool(fp.IsExcludedFromPosFiles()),
                 'native_attributes': int(fp.GetAttributes()), 'properties': properties}
        frames.append(frame)
        flags = {'DNP': str(frame['dnp']).upper(), 'ExcludedFromBOM': str(frame['excluded_from_bom']).upper(),
                 'ExcludedFromPositions': str(frame['excluded_from_positions']).upper()}
        identity = {'Reference': frame['reference'], 'Value': frame['value'],
                    'Footprint': frame['footprint_id'], 'FootprintUUID': frame['uuid']}
        part = dict(identity, Quantity=1, Manufacturer=fields.get('Manufacturer', ''), MPN=fields.get('MPN', ''),
                    **flags, NativeProperties=json.dumps(properties, ensure_ascii=False),
                    ProcurementQualified='FALSE', AssemblyOrientation='UNQUALIFIED')
        pose = dict(identity, X_mm=frame['position_mm'][0], Y_mm=frame['position_mm'][1],
                    Rotation_degrees=frame['rotation_degrees'], Side=frame['side'],
                    **flags, MachineOrientation='UNQUALIFIED')
        if frame['dnp']:
            dnp_bom.append(part);dnp_positions.append(pose)
        else:
            if not frame['excluded_from_bom']:bom.append(part)
            if not frame['excluded_from_positions']:positions.append(pose)
    bom_fields = ['Reference','Value','Footprint','FootprintUUID','Quantity','Manufacturer','MPN','DNP',
                  'ExcludedFromBOM','ExcludedFromPositions','NativeProperties','ProcurementQualified','AssemblyOrientation']
    position_fields = ['Reference','Value','Footprint','FootprintUUID','X_mm','Y_mm','Rotation_degrees','Side',
                       'DNP','ExcludedFromBOM','ExcludedFromPositions','MachineOrientation']
    for name, rows, fields in [('BOM.csv',bom,bom_fields), ('BOM-DNP.csv',dnp_bom,bom_fields),
                               ('positions.csv',positions,position_fields), ('positions-DNP.csv',dnp_positions,position_fields)]:
        with (target / name).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader();writer.writerows(rows)
    result = {'source_board_sha256': hashlib.sha256((directory/'candidate.kicad_pcb').read_bytes()).hexdigest(),
              'coordinate_convention': 'Native board origin, millimetres, +X right, +Y down; raw native KiCad orientation degrees.',
              'machine_orientation_qualified': False, 'procurement_qualified': False,
              'footprints': len(frames), 'populated_BOM_rows': len(bom), 'populated_position_rows': len(positions),
              'DNP_rows': len(dnp_bom), 'frames': frames}
    (target/'native-assembly-frames.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    return result


def export_native_drills(board, directory, source_sha256):
    """Exact native drill geometry in the Excellon axis convention."""
    layers = list(board.GetEnabledLayers().CuStack())
    rank = {layer:index+1 for index,layer in enumerate(layers)}
    holes = []
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            drill = pad.GetDrillSize()
            if not drill.x and not drill.y:continue
            if not drill.x or not drill.y:raise ValueError('Incomplete native drill dimensions')
            segment = pad.GetEffectiveHoleShape().GetSeg()
            holes.append({'uuid':pad.m_Uuid.AsString(), 'reference':footprint.GetReference(),
                'pin':pad.GetNumber(), 'diameter_nm':min(drill.x,drill.y),
                'ends_nm':sorted([[point.x,-point.y] for point in (segment.A,segment.B)]),
                'plated':pad.GetAttribute()!=pcbnew.PAD_ATTRIB_NPTH, 'layer_pair':[1,len(layers)]})
    for via in board.GetTracks():
        if not isinstance(via,pcbnew.PCB_VIA):continue
        point = via.GetPosition()
        holes.append({'uuid':via.m_Uuid.AsString(), 'reference':None, 'pin':None,
            'diameter_nm':via.GetDrillValue(), 'ends_nm':[[point.x,-point.y],[point.x,-point.y]],
            'plated':True, 'layer_pair':sorted([rank[via.TopLayer()],rank[via.BottomLayer()]])})
    if any(hole['diameter_nm']<=0 for hole in holes):raise ValueError('Nonpositive native drill')
    result = {'source_board_sha256':source_sha256, 'native_version':pcbnew.Version(),
        'coordinate_convention':'Native integer nanometres; Excellon +X right, +Y up (native Y reflected).',
        'copper_layers':[board.GetLayerName(layer) for layer in layers], 'holes':holes}
    target = directory/'native-audit'
    target.mkdir(exist_ok=True)
    (target/'native-drill-holes.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def export_gerbers(board, directory, source_sha256):
    """Native plot only: finer transient curve approximation; never save or refill."""
    settings = board.GetDesignSettings()
    original_error = int(settings.m_MaxError)
    if original_error <= 0:raise ValueError('Invalid native curve approximation setting')
    plot_error = min(original_error, pcbnew.FromMM(.001))
    io = pcbnew.PCB_IO_KICAD_SEXPR()
    items = list(board.GetFootprints())+list(board.GetTracks())+list(board.GetDrawings())+list(board.Zones())
    before = {q.m_Uuid.AsString():io.Format(q) for q in items}
    source_files = [p for p in directory.iterdir() if p.is_file() and
        (p.name in ('candidate.kicad_pcb','candidate.kicad_pro','candidate.kicad_dru','candidate.routing.json','candidate.dc.json','source-design.json'))]
    source_hashes = {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}
    target = directory/'cam';target.mkdir(exist_ok=True)
    controller = None
    files = []
    try:
        settings.m_MaxError = plot_error
        for fp in board.GetFootprints():
            for pad in fp.Pads():pad.SetDirty()
        controller = pcbnew.PLOT_CONTROLLER(board)
        options = controller.GetPlotOptions()
        options.SetOutputDirectory(str(target.resolve()))
        options.SetPlotFrameRef(False);options.SetPlotReference(True);options.SetPlotValue(True);options.SetPlotFPText(True)
        options.SetUseGerberProtelExtensions(True);options.SetUseGerberX2format(True);options.SetIncludeGerberNetlistInfo(True)
        options.SetDisableGerberMacros(False);options.SetCreateGerberJobFile(False);options.SetGerberPrecision(6)
        options.SetUseAuxOrigin(False);options.SetMirror(False);options.SetScale(1);options.SetNegative(False)
        options.SetDrillMarksType(pcbnew.DRILL_MARKS_NO_DRILL_SHAPE);options.SetSubtractMaskFromSilk(False)
        layers = list(board.GetEnabledLayers().CuStack())+[pcbnew.F_Paste,pcbnew.B_Paste,
            pcbnew.F_SilkS,pcbnew.B_SilkS,pcbnew.F_Mask,pcbnew.B_Mask,pcbnew.Edge_Cuts]
        job = pcbnew.GERBER_JOBFILE_WRITER(board)
        for layer in layers:
            name = board.GetLayerName(layer);controller.SetLayer(layer)
            if not controller.OpenPlotfile(name.replace('.', '_'),pcbnew.PLOT_FORMAT_GERBER,name):
                raise ValueError('Native Gerber file did not open')
            if not controller.PlotLayer():raise ValueError('Native Gerber layer export failed')
            path = Path(controller.GetPlotFileName());controller.ClosePlot()
            job.AddGbrFile(layer,path.name)
            files.append({'layer':name,'path':str(path.relative_to(directory.resolve())),
                'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
        path = target/'candidate-job.gbrjob'
        if not job.CreateJobFile(str(path)):raise ValueError('Native Gerber job metadata export failed')
        files.append({'path':str(path.relative_to(directory)),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    finally:
        if controller is not None and controller.IsPlotOpen():controller.ClosePlot()
        settings.m_MaxError = original_error
        for fp in board.GetFootprints():
            for pad in fp.Pads():pad.SetDirty()
    if before!={q.m_Uuid.AsString():io.Format(q) for q in items}:raise ValueError('Native geometry or persisted fills changed during plotting')
    if source_hashes!={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}:raise ValueError('Native source files changed during plotting')
    result = {'source_board_sha256':source_sha256,'native_version':pcbnew.Version(),
        'source_curve_approximation_error_nm':original_error,'plot_curve_approximation_error_nm':plot_error,
        'source_files_unchanged':True,'native_definitions_and_fills_unchanged':True,
        'source_setting_restored':True,'source_saved':False,'zones_refilled':False,'files':files,
        'scope':'Native Gerber plotting with transient finer curve tessellation; original source/project/rules/fills retained.'}
    audit = directory/'native-audit';audit.mkdir(exist_ok=True)
    (audit/'gerber-precision.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=['prepare', 'prepare-planes', 'finish', 'geometry', 'dc-geometry', 'native-connectivity', 'assembly-export', 'gerber-export', 'apply-grid', 'prepare-repair', 'inspect', 'measure', 'rip-nets', 'widen-power','move-via'])
    p.add_argument('directory', type=Path)
    args = p.parse_args()
    app = wx.App(False)
    d = args.directory
    export_sha256 = hashlib.sha256((d/'candidate.kicad_pcb').read_bytes()).hexdigest() if args.mode in ('assembly-export','gerber-export') else None
    b = pcbnew.LoadBoard(str(d / 'candidate.kicad_pcb'))
    if args.mode == 'gerber-export':
        export_gerbers(b,d,export_sha256)
        return
    if args.mode == 'assembly-export':
        export_assembly(b, d)
        export_native_drills(b, d, export_sha256)
        if hashlib.sha256((d/'candidate.kicad_pcb').read_bytes()).hexdigest()!=export_sha256:
            raise ValueError('Selected native board changed during engineering metadata export')
        return
    if args.mode in ('inspect', 'measure'):
        (d / 'metrics.json').write_text(json.dumps(measure(b),indent=2))
        if args.mode == 'inspect':
            pcbnew.SaveBoard(str(d / 'original-normalized.kicad_pcb'),b)
            (d / 'inventory.json').write_text(json.dumps({'source':inventory(b),'routing_input':inventory(b)}))
        return
    if args.mode=='move-via':
        plan=json.loads((d/'via-move.json').read_text())
        for plan in plan.get('moves',[plan]):
            via=next(t for t in b.GetTracks() if t.m_Uuid.AsString()==plan['uuid'])
            if not isinstance(via,pcbnew.PCB_VIA) or via.IsLocked():raise ValueError('Only unlocked vias may move')
            old=pcbnew.VECTOR2I(via.GetPosition());new=pcbnew.VECTOR2I(*[pcbnew.FromMM(v) for v in plan['at']])
            joined=[]
            for t in b.GetTracks():
                if isinstance(t,pcbnew.PCB_VIA) or t.GetNetCode()!=via.GetNetCode():continue
                start,end=t.GetStart()==old,t.GetEnd()==old
                if start or end:
                    if t.IsLocked() or isinstance(t,pcbnew.PCB_ARC):raise ValueError('Via connects to locked or curved copper')
                    joined.append((t,start,end))
            if not joined:raise ValueError('Via has no movable track endpoints')
            if plan.get('jog'):
                layers={}
                for t,_,_ in joined:layers[t.GetLayer()]=max(layers.get(t.GetLayer(),0),t.GetWidth())
                for layer,width in layers.items():
                    stub=pcbnew.PCB_TRACK(b);stub.SetStart(old);stub.SetEnd(new)
                    stub.SetLayer(layer);stub.SetWidth(width);stub.SetNetCode(via.GetNetCode());b.Add(stub)
            else:
                for t,start,end in joined:
                    if start:t.SetStart(new)
                    if end:t.SetEnd(new)
            via.SetPosition(new)
        b.BuildConnectivity();pcbnew.ZONE_FILLER(b).Fill(b.Zones())
        pcbnew.SaveBoard(str(d/'candidate.kicad_pcb'),b);return
    if args.mode == 'widen-power':
        plan=json.loads((d / 'width-plan.json').read_text())
        tracks={t.m_Uuid.AsString():t for t in b.GetTracks()}
        for change in plan['changes']:
            t=tracks[change['uuid']]
            if t.IsLocked() or isinstance(t,(pcbnew.PCB_VIA,pcbnew.PCB_ARC)):raise ValueError('Only unlocked straight traces may be widened')
            if t.GetNetname()!=change['net'] or abs(pcbnew.ToMM(t.GetWidth())-change['old_width_mm'])>1e-6:raise ValueError('Width plan does not match current copper')
            width=change['width_mm']
            if not change['old_width_mm']<width<=plan['max_width_mm']:raise ValueError('Invalid width change')
            segments=change.get('segments')
            if segments:
                if t.GetParentGroup():raise ValueError('Grouped tracks cannot be split by the width repair')
                # Duplicate native track properties while retaining the original
                # UUID on the final section; added sections receive fresh UUIDs.
                for i,segment in enumerate(segments):
                    item=t if i==len(segments)-1 else t.Duplicate()
                    w=segment['width_mm']
                    if not change['old_width_mm']<=w<=plan['max_width_mm']:raise ValueError('Invalid neck width')
                    item.SetStart(pcbnew.VECTOR2I(*[pcbnew.FromMM(v) for v in segment['start']]))
                    item.SetEnd(pcbnew.VECTOR2I(*[pcbnew.FromMM(v) for v in segment['end']]))
                    item.SetWidth(pcbnew.FromMM(w))
                    if item is not t:b.Add(item)
            else:t.SetWidth(pcbnew.FromMM(width))
        b.BuildConnectivity()
        if not plan.get('preserve_functional_fills'):pcbnew.ZONE_FILLER(b).Fill(b.Zones())
        pcbnew.SaveBoard(str(d / 'candidate.kicad_pcb'),b)
        return
    if args.mode == 'rip-nets':
        plan=json.loads((d / 'route-nets.json').read_text())
        removed_items=[t for t in b.GetTracks() if t.GetNetCode() in plan['nets'] and not t.IsLocked()]
        for track in removed_items:b.Remove(track)
        b.BuildConnectivity()
        pcbnew.SaveBoard(str(d / 'candidate.kicad_pcb'),b)
        return
    if args.mode == 'prepare-repair':
        drc=json.loads((d / 'input-drc.json').read_text())
        plan_path=d/'route-nets.json'
        plan=json.loads(plan_path.read_text()) if plan_path.exists() else {}
        protected=set(plan.get('protected_nets',[]))
        excluded={t.GetNetCode() for t in b.GetTracks() if t.IsLocked()}
        excluded.update(n.GetNetCode() for n in b.GetNetsByNetcode().values() if n.GetNetname() in protected)
        bad_ids={item['uuid'] for issue in drc.get('violations',[]) for item in issue.get('items',[])}
        if plan.get('preserve_dangling_vias'):
            dangling={item['uuid'] for issue in drc.get('violations',[]) if issue.get('type')=='via_dangling'
                      for item in issue.get('items',[])}
            other={item['uuid'] for issue in drc.get('violations',[]) if issue.get('type')!='via_dangling'
                   for item in issue.get('items',[])}
            bad_ids-=dangling-other
        removed_items=[t for t in b.GetTracks() if not t.IsLocked() and t.GetNetCode() not in excluded and t.m_Uuid.AsString() in bad_ids]
        removed_void_tracks=[]
        removed_vias=[t for t in removed_items if isinstance(t,pcbnew.PCB_VIA)]
        if removed_vias:
            from native_geometry import _straight_track_inside_round_via_drill
            round_drills={}
            removed_ids={t.m_Uuid.AsString() for t in removed_items}
            for track in b.GetTracks():
                if track.m_Uuid.AsString() in removed_ids or track.IsLocked() or track.GetNetCode() in excluded:continue
                # A nominal stub wholly inside this drill has no physical Cu.
                # Delete it with the dangling via before removal exposes a new
                # capsule under an unchanged source UUID. Partial Cu stays put.
                covering=next((via for via in removed_vias if track.GetNetCode()==via.GetNetCode()
                    and _straight_track_inside_round_via_drill(track,via,track.GetLayer(),round_drills)),None)
                if covering is not None:
                    removed_items.append(track)
                    removed_void_tracks.append({'track_uuid':track.m_Uuid.AsString(),
                        'round_native_via_uuid':covering.m_Uuid.AsString(),'layer':track.GetLayer()})
        # Connectivity findings need routing even when no violating track was
        # removed. Previously a clean-but-incomplete board produced an empty
        # allowlist, so the repair pass silently attempted zero nets.
        by_id={item.m_Uuid.AsString():item for item in list(b.GetTracks())+
               [pad for fp in b.GetFootprints() for pad in fp.Pads()]}
        incomplete={by_id[item['uuid']].GetNetCode() for issue in drc.get('unconnected_items',[])
                    for item in issue.get('items',[]) if item['uuid'] in by_id}
        selected=(incomplete|{t.GetNetCode() for t in removed_items})-excluded
        points={}
        for issue in drc.get('violations',[])+drc.get('unconnected_items',[]):
            for item in issue.get('items',[]):
                obj=by_id.get(item['uuid'])
                if obj is None or obj.GetNetCode() not in selected:continue
                pts=[obj.GetStart(),obj.GetEnd()] if isinstance(obj,pcbnew.PCB_TRACK) else [obj.GetPosition()]
                points.setdefault(str(obj.GetNetCode()),[]).extend((pcbnew.ToMM(p.x),pcbnew.ToMM(p.y)) for p in pts)
        windows={n:[min(p[0] for p in pts),min(p[1] for p in pts),max(p[0] for p in pts),max(p[1] for p in pts)] for n,pts in points.items()}
        (d / 'route-nets.json').write_text(json.dumps({'nets': sorted(selected),'windows':windows,'protected_nets':sorted(protected),'excluded_net_codes':sorted(excluded)}))
        for track in removed_items:b.Remove(track)
        b.BuildConnectivity()
        pcbnew.SaveBoard(str(d / 'candidate.kicad_pcb'),b)
        (d / 'repair.json').write_text(json.dumps({'removed_unlocked_copper':len(removed_items),
            'removed_native_uuids':sorted(t.m_Uuid.AsString() for t in removed_items),
            'removed_drill_void_tracks':removed_void_tracks},indent=2))
        return
    if args.mode == 'native-connectivity':
        from native_geometry import export_native_connectivity
        result=export_native_connectivity(b)
        result['source_sha256']=hashlib.sha256((d/'candidate.kicad_pcb').read_bytes()).hexdigest()
        (d/'native-connectivity.json').write_text(json.dumps(result))
        return
    if args.mode == 'dc-geometry':
        from native_geometry import export_dc_geometry
        geometry=export_dc_geometry(b)
        geometry['source_board_sha256']=hashlib.sha256((d/'candidate.kicad_pcb').read_bytes()).hexdigest()
        (d / 'dc-geometry.json').write_text(json.dumps(geometry))
        return
    if args.mode == 'geometry':
        from native_geometry import export_geometry
        (d / 'geometry.json').write_text(json.dumps(export_geometry(b)))
        return
    if args.mode == 'apply-grid':
        route=json.loads((d / 'grid-route.json').read_text())
        remove=set(route.get('remove',[]))
        removed_items=[t for t in b.GetTracks() if t.m_Uuid.AsString() in remove and not t.IsLocked()]
        for track in removed_items:b.Remove(track)
        # FromMM truncates some decimal positions one native unit below the
        # intended coordinate (for example 16.4 mm). Generated paths are on the
        # native grid; round once instead of shortening shared endpoints.
        def vector(p):return pcbnew.VECTOR2I(*[int(round(v*pcbnew.FromMM(1))) for v in p])
        for item in route['tracks']:
            track=pcbnew.PCB_TRACK(b);track.SetStart(vector(item['start']));track.SetEnd(vector(item['end']))
            track.SetLayer(item['layer']);track.SetWidth(pcbnew.FromMM(item['width']));track.SetNetCode(item['net']);b.Add(track)
        for item in route['vias']:
            via=pcbnew.PCB_VIA(b);via.SetPosition(vector(item['at']));via.SetWidth(pcbnew.FromMM(item['diameter']))
            via.SetDrill(pcbnew.FromMM(item['drill']));via.SetViaType(pcbnew.VIATYPE_THROUGH)
            via.SetLayerPair(pcbnew.F_Cu,pcbnew.B_Cu);via.SetNetCode(item['net']);b.Add(via)
        b.BuildConnectivity()
        # Improvement retains the source's filled copper. Native DRC and the
        # independent actual-fill guard still reject invalid edits.
        if not route.get('preserve_functional_fills'):pcbnew.ZONE_FILLER(b).Fill(b.Zones())
        pcbnew.SaveBoard(str(d / 'candidate.kicad_pcb'),b)
        (d / 'routed-inventory.json').write_text(json.dumps(inventory(b),indent=2))
        return
    if args.mode in ('prepare','prepare-planes'):
        outline = pcbnew.SHAPE_POLY_SET()
        if not b.GetBoardPolygonOutlines(outline, False) or not outline.OutlineCount():
            raise ValueError('A closed, valid Edge.Cuts outline is required; no outline is inferred')
        if not list(b.GetFootprints()):
            raise ValueError('Board has no footprints')
        before = inventory(b)
        source_hash=hashlib.sha256((d/'candidate.kicad_pcb').read_bytes()).hexdigest()
        if not pcbnew.SaveBoard(str(d / 'original-normalized.kicad_pcb'), b):
            raise RuntimeError('Could not save normalized source')
        # Teardrops are generated routing copper, not functional plane outlines.
        # Keeping their obsolete copper after ripping tracks can create apparent
        # connections that a genuinely fresh route never generated. Conservatively
        # retain every teardrop on a net with locked routing, and locked zones.
        locked_nets={t.GetNetCode() for t in b.GetTracks() if t.IsLocked()}
        generated_zones=list(z for z in b.Zones() if z.IsTeardropArea())
        removed_zones=[];preserved_zones=[]
        for z in generated_zones:
            record={'uuid':z.m_Uuid.AsString(),'net_code':z.GetNetCode(),
                    'net_name':z.GetNetname(),'native_teardrop':True}
            if z.IsLocked() or z.GetNetCode() in locked_nets or z.GetIsRuleArea():
                record['reason']='Locked zone, locked routing on this net, or rule area'
                preserved_zones.append(record)
            else:
                removed_zones.append(record);b.Remove(z)
        zone_removals={
            'source_board_sha256':source_hash,
            'normalized_source_sha256':hashlib.sha256((d/'original-normalized.kicad_pcb').read_bytes()).hexdigest(),
            'removed_generated_zones':removed_zones,'preserved_generated_zones':preserved_zones,
            'scope':'Fresh route only; native generated teardrops. Functional planes and rule areas are preserved.'
        }
        # Locked critical routing remains. Everything else is freshly routed.
        removed = 0
        removed_items = list(b.GetTracks())
        for t in removed_items:
            if not t.IsLocked():
                b.Remove(t)
                removed += 1
        b.BuildConnectivity()
        # Recompute planes after removing tracks. Plane definitions are preserved.
        filler = pcbnew.ZONE_FILLER(b)
        filler.Fill(b.Zones())
        if not pcbnew.SaveBoard(str(d / 'candidate.kicad_pcb'), b):
            raise RuntimeError('Could not save routing input')
        import shutil
        shutil.copy2(d / 'candidate.kicad_pcb',d / 'routing-base.kicad_pcb')
        # Generated zones have no serialized UUID in KiCad 10. Record exact
        # definition fingerprints with multiplicity instead of assuming IDs.
        def serialized_zones(path):
            return Counter(json.dumps(zone_definition(v),separators=(',',':')) for v in sexpr(path.read_text())
                           if isinstance(v,list) and v and v[0]=='zone')
        removed_definitions=serialized_zones(d/'original-normalized.kicad_pcb')-serialized_zones(d/'routing-base.kicad_pcb')
        if sum(removed_definitions.values())!=len(removed_zones):raise ValueError('Native/serialized generated-zone removal count mismatch')
        zone_removals['removed_zone_definitions']=[]
        for definition in removed_definitions.elements():
            net=next(v for v in json.loads(definition) if isinstance(v,list) and v and v[0]=='net')
            zone_removals['removed_zone_definitions'].append({'definition_sha256':hashlib.sha256(definition.encode()).hexdigest(),
                                                          'net_name':json.loads(net[1]),'native_teardrop':True})
        (d/'generated-zone-removals.json').write_text(json.dumps(zone_removals,indent=2)+'\n')
        routing_input = inventory(b)
        # A DSN plane is an outline, not KiCad's actual filled copper with voids
        # and thermals. Route explicit connections, then refill the original planes.
        # Keep the SWIG wrappers alive until exit; temporary wrappers can invalidate
        # native objects in this bundled binding after removal.
        routing_zones = list(b.Zones())
        for zone in routing_zones:
            if not zone.GetIsRuleArea() and args.mode!='prepare-planes':
                b.Remove(zone)
        # KiCad's DSN exporter serializes even footprint-only/pour-only rule
        # areas as generic copper keepouts. Those can cover an entire edge
        # connector whose native rules explicitly permit tracks and vias.
        # Omit only these non-routing restrictions from the temporary DSN copy;
        # the saved native routing base and final DRC retain every original rule.
        nonrouting_areas=[]
        for owner in [b]+list(b.GetFootprints()):
            for zone in list(owner.Zones()):
                if zone.GetIsRuleArea() and not zone.GetDoNotAllowTracks() and not zone.GetDoNotAllowVias():
                    nonrouting_areas.append(zone);owner.Remove(zone)
        # DSN pads do not encode native hole-to-copper clearance, including
        # per-pad overrides on round mounting holes. Use the native expanded
        # hole shape for every NPTH, including rotated slots.
        from native_geometry import polygons
        hole_areas=[];graphic_areas=[];obstacles=[]
        settings=b.GetDesignSettings()
        for fp in b.GetFootprints():
            for pad in fp.Pads():
                if pad.GetAttribute()!=pcbnew.PAD_ATTRIB_NPTH or not pad.GetDrillSize().x:continue
                poly=pcbnew.SHAPE_POLY_SET()
                clearance=max(settings.m_HoleClearance,settings.m_CopperEdgeClearance,
                              max(0,pad.GetOwnClearance(pcbnew.UNDEFINED_LAYER)))
                pad.TransformHoleToPolygon(poly,clearance+pcbnew.FromMM(.003),pcbnew.FromMM(.001),pcbnew.ERROR_OUTSIDE)
                area=pcbnew.ZONE(b);area.SetIsRuleArea(True);area.SetLayerSet(pcbnew.LSET.AllCuMask())
                area.SetDoNotAllowTracks(True);area.SetDoNotAllowVias(True)
                area.Outline().Append(poly)
                b.Add(area);hole_areas.append(area)
                obstacles.append({'kind':'NPTH','uuid':pad.m_Uuid.AsString(),'reference':fp.GetReference(),
                                  'pin':pad.GetNumber(),'clearance_mm':pcbnew.ToMM(clearance),
                                  'layers':list(b.GetEnabledLayers().CuStack()),'shapes':polygons(poly)})
        # Copper text and graphics are physical native copper. The exporter
        # otherwise drops them, allowing routes straight through a printed
        # copper digit. Preserve actual stroke polygons as temporary keepouts.
        graphics=list(b.GetDrawings())
        for fp in b.GetFootprints():graphics.extend(list(fp.GraphicalItems())+[fp.Reference(),fp.Value()])
        for item in graphics:
            for layer in b.GetEnabledLayers().CuStack():
                if not item.IsOnLayer(layer):continue
                poly=pcbnew.SHAPE_POLY_SET()
                own=max(0,item.GetOwnClearance(layer)) if hasattr(item,'GetOwnClearance') else 0
                clearance=max(settings.m_MinClearance,own)
                if hasattr(item,'TransformTextToPolySet'):
                    # PCB_TEXT.TransformShapeToPolygon returns its rectangular
                    # envelope. This API emits the actual font stroke copper.
                    item.TransformTextToPolySet(poly,clearance+pcbnew.FromMM(.003),pcbnew.FromMM(.001),pcbnew.ERROR_OUTSIDE)
                else:item.TransformShapeToPolygon(poly,layer,clearance+pcbnew.FromMM(.003),pcbnew.FromMM(.001),pcbnew.ERROR_OUTSIDE)
                if not poly.OutlineCount():continue
                area=pcbnew.ZONE(b);area.SetIsRuleArea(True);area.SetLayer(layer)
                area.SetDoNotAllowTracks(True);area.SetDoNotAllowVias(True);area.Outline().Append(poly)
                b.Add(area);graphic_areas.append(area)
                obstacles.append({'kind':'copper_graphic','uuid':item.m_Uuid.AsString(),'layer':layer,
                                  'text':item.GetText() if hasattr(item,'GetText') else None,
                                  'clearance_mm':pcbnew.ToMM(clearance),'shapes':polygons(poly)})
        (d/'dsn-native-obstacles.json').write_text(json.dumps({
            'source_board_sha256':source_hash,'routing_base_sha256':hashlib.sha256((d/'routing-base.kicad_pcb').read_bytes()).hexdigest(),
            'layer_names':{str(layer):b.GetLayerName(layer) for layer in b.GetEnabledLayers().CuStack()},
            'scope':'Native stroked polygons in the temporary DSN copy only; 0.003 mm outward discretization guard.',
            'obstacles':obstacles},indent=2)+'\n')
        # Freerouting writes an invalid empty component token for unnamed embedded
        # footprints. Give only the routing copy an ID; the delivered board keeps
        # the original footprint metadata and geometry.
        for footprint in b.GetFootprints():
            if not footprint.GetFPID().GetUniStringLibItemName():
                footprint.SetFPIDAsString('Native_' + footprint.m_Uuid.AsString().replace('-', ''))
        if not pcbnew.ExportSpecctraDSN(b, str(d / 'input.dsn')):
            raise RuntimeError('KiCad could not export this board to Specctra')
        # Fanout can emit sub-minimum necks even with automatic neckdown disabled.
        # Reserve 20 um in the routing copy so restoring native minimum widths
        # does not consume the original clearance. Never relax project rules.
        dsn = d / 'input.dsn'
        minimum_um = pcbnew.ToMM(b.GetDesignSettings().m_MinClearance) * 1000
        dsn.write_text(re.sub(r'\(clearance ([0-9.]+)',
            lambda m: '(clearance ' + str(max(float(m[1]), minimum_um) + 20), dsn.read_text()))
        # Native undrilled SMD pads must not acquire guessed drill keepouts.
        from dsn_constraints import encode_native_smd_drills
        native_pads=[]
        for fp in b.GetFootprints():
            drilled={p.GetNumber() for p in fp.Pads() if p.GetDrillSize().x or p.GetDrillSize().y}
            native_pads.append({'id':fp.m_Uuid.AsString(),'reference':fp.GetReference(),
                'position':[pcbnew.ToMM(fp.GetPosition().x),pcbnew.ToMM(fp.GetPosition().y)],
                'side':'back' if fp.GetLayer()==pcbnew.B_Cu else 'front',
                'zero_drill_pins':sorted({p.GetNumber() for p in fp.Pads()}-drilled)})
        translated,smd_drills=encode_native_smd_drills(dsn.read_text(),native_pads)
        from dsn_constraints import encode_existing_seed_attachments
        from native_geometry import export_dc_geometry
        translated,attachments=encode_existing_seed_attachments(translated,export_dc_geometry(b))
        dsn.write_text(translated)
        (d/'dsn-native-seed-attachments.json').write_text(json.dumps(attachments,indent=2)+'\n')
        (d/'dsn-native-smd-drills.json').write_text(json.dumps(smd_drills,indent=2)+'\n')
        # The DSN exporter does not represent native drill-to-drill constraints.
        # Keep them in the unchanged native project: local via repair and final
        # DRC enforce the actual drilled geometry after routing, rather than
        # relying on a conservative copper-annulus approximation during search.
        settings=b.GetDesignSettings()
        info = {'routing_clearance_margin_um': 20, 'source': before, 'routing_input': routing_input, 'removed_tracks_and_vias': removed,
                'retained_locked_copper': before['locked_tracks_and_vias'],
                'preserved_zone_definitions': routing_input['zones'], 'removed_generated_zones':len(removed_zones),
                'route_all_nets_explicitly': args.mode!='prepare-planes',
                'dsn_planes_are_search_hints_only': args.mode=='prepare-planes',
                'dsn_nonrouting_rule_areas_omitted':len(nonrouting_areas),
                'dsn_native_hole_obstacles':len(hole_areas),
                'dsn_native_copper_graphic_obstacles':len(graphic_areas),
                'copper_edge_clearance_um': pcbnew.ToMM(b.GetDesignSettings().m_CopperEdgeClearance)*1000,
                'hole_clearance_um':pcbnew.ToMM(settings.m_HoleClearance)*1000,
                'native_hole_to_hole_um':pcbnew.ToMM(settings.m_HoleToHoleMin)*1000,
                'drill_spacing_enforced_by':'native via repair and final KiCad DRC',
                'kicad_version': pcbnew.Version()}
        (d / 'inventory.json').write_text(json.dumps(info, indent=2))
    else:
        poses = {f.m_Uuid.AsString(): (pcbnew.VECTOR2I(f.GetPosition()), f.GetOrientation())
                 for f in b.GetFootprints()}
        # KiCad's DSN exporter assigns unique aliases to duplicate references
        # (common for mounting holes). Importing those aliases against the native
        # duplicate names opens a modal error dialog, even in a headless worker.
        from native_project import routing_references
        refs={f.m_Uuid.AsString():f.GetReference() for f in b.GetFootprints()}
        aliases=routing_references((d/'input.dsn').read_text(),[
            {'id':f.m_Uuid.AsString(),'reference':f.GetReference(),
             'position':[pcbnew.ToMM(f.GetPosition().x),pcbnew.ToMM(f.GetPosition().y)],
             'side':'back' if f.GetLayer()==pcbnew.B_Cu else 'front'} for f in b.GetFootprints()])
        (d/'routing-reference-map.json').write_text(json.dumps(aliases,indent=2))
        for f in b.GetFootprints():f.SetReference(aliases[f.m_Uuid.AsString()])
        from dsn_constraints import canonicalize_seed_attachment_session
        session=d/'route.ses'
        raw_session=session.read_text()
        canonical,attachments=canonicalize_seed_attachment_session((d/'input.dsn').read_text(),raw_session)
        if attachments['changed_fixed_via_identifiers']:
            (d/'route-before-seed-aliases.ses').write_text(raw_session)
            session.write_text(canonical)
        (d/'ses-native-seed-attachments.json').write_text(json.dumps(attachments,indent=2)+'\n')
        if not pcbnew.ImportSpecctraSES(b, str(session)):
            raise RuntimeError('KiCad rejected the routing session')
        # SES placement precision is lower than native KiCad precision. Routing
        # is not authorized to round or move the supplied components.
        for footprint in b.GetFootprints():
            footprint.SetReference(refs[footprint.m_Uuid.AsString()])
            position, angle = poses[footprint.m_Uuid.AsString()]
            footprint.SetOrientation(angle)
            footprint.SetPosition(position)
        widths_normalized = 0
        for track in b.GetTracks():
            if not isinstance(track, pcbnew.PCB_VIA) and not track.IsLocked():
                minimum = b.GetDesignSettings().m_TrackMinWidth
                if track.GetWidth() < minimum:
                    track.SetWidth(minimum)
                    widths_normalized += 1
        b.BuildConnectivity()
        filler = pcbnew.ZONE_FILLER(b)
        filler.Fill(b.Zones())
        if not pcbnew.SaveBoard(str(d / 'candidate.kicad_pcb'), b):
            raise RuntimeError('Could not save routed board')
        info = inventory(b)
        info['widths_normalized'] = widths_normalized
        (d / 'routed-inventory.json').write_text(json.dumps(info, indent=2))


if __name__ == '__main__':
    main()
