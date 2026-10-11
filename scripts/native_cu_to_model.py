"""Import one supported native Cu inventory into an unchanged Rust board model.

Input coordinates are native-positiveY millimetres. Arcs, non-through vias,
filled zones, unknown layer names and unmapped net names fail closed.
"""
from pathlib import Path
import argparse,json,hashlib

def convert(source,geometry):
 b=json.loads(Path(source).read_text());g=json.loads(Path(geometry).read_text())
 if any(t.get('locked',False) for t in g['tracks']+g['vias']):raise ValueError('Rust model cannot preserve locked native copper')
 if g.get('zones'):raise ValueError('Rust model adapter does not support native filled zones')
 if abs(g['board_thickness_mm']-b['thickness'])>1e-6:raise ValueError('Native thickness differs from source model')
 names={int(i):n for i,n in b['net_names'].items() if n}
 if len(names)!=len(set(names.values())):raise ValueError('Source named net IDs must be unique')
 inverse={n:i for i,n in names.items()}
 native={int(i):d['name'] for i,d in g['nets'].items()}
 def net(code):
  if code not in native or native[code] not in inverse:raise ValueError(f'Unmapped native net {code}')
  return inverse[native[code]]
 supported={'F.Cu':'FCu','B.Cu':'BCu'}
 if any(n not in supported for n in g['layer_names'].values()):raise ValueError('Unmapped native copper layer')
 layer_map={int(i):supported[n] for i,n in g['layer_names'].items()}
 if set(layer_map.values())!={'FCu','BCu'}:raise ValueError('Only a complete two-layer native stack is supported')
 tracks=[];vias=[]
 for t in g['tracks']:
  if t['arc']:raise ValueError('Native arcs cannot be silently replaced by chords')
  if t['layer'] not in layer_map:raise ValueError('Unmapped track layer')
  tracks.append({'net':net(t['net']),'width':t['width'],'layer':layer_map[t['layer']], 'start':{'x':t['start'][0],'y':t['start'][1]},'end':{'x':t['end'][0],'y':t['end'][1]}})
 for v in g['vias']:
  if not v['through']:raise ValueError('Blind or buried vias are unsupported')
  if not (v['drill_mm']>0 and v['diameter_mm']>v['drill_mm']):raise ValueError('Invalid native via geometry')
  vias.append({'net':net(v['net']),'size':v['diameter_mm'],'drill':v['drill_mm'],'at':{'x':v['at'][0],'y':v['at'][1]},'layers':['FCu','BCu']})
 b['tracks']=tracks;b['vias']=vias
 return b,{'native_tracks':len(g['tracks']),'native_vias':len(g['vias']),'model_tracks':len(tracks),'model_vias':len(vias),'native_net_id_to_source_model_id':{str(n):net(n) for n in native},'native_layer_id_to_model_layer':layer_map,'coordinate_convention':'native positive Y millimetres, no reflection/board-height transform','source_non_copper_model_fields_unchanged':True,'unsupported_shapes_fail_closed':True}

def main():
 p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('geometry');p.add_argument('output');a=p.parse_args();b,proof=convert(a.source,a.geometry);out=Path(a.output);out.write_text(json.dumps(b,indent=2)+'\n');proof.update(source_model_sha256=hashlib.sha256(Path(a.source).read_bytes()).hexdigest(),native_geometry_sha256=hashlib.sha256(Path(a.geometry).read_bytes()).hexdigest(),output_model_sha256=hashlib.sha256(out.read_bytes()).hexdigest());out.with_name(out.stem+'-mapping-proof.json').write_text(json.dumps(proof,indent=2)+'\n');print(json.dumps({k:proof[k] for k in ['native_tracks','native_vias','model_tracks','model_vias','source_non_copper_model_fields_unchanged']}))
if __name__=='__main__':main()
