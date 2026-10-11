#!/usr/bin/env python3
"""Create an explicitly modified TCA9548A placement/routing benchmark from pinned Eagle source.
Retains the upstream circuit, connector/mount positions and pad geometry except
solder jumpers: closed jumpers become 0805 0R links, open address jumpers become
DNP 0805 links. Original copper/artwork/pours are not imported. CC BY-SA 3.0.
"""
import json,math,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'reference/adafruit-tca9548a'
def point(x,y):return {'x':x,'y':y}
def main():
    b=ET.parse(SOURCE/'Adafruit TCA9548A.brd').find('.//board')
    packages={(lib.get('name'),p.get('name')):p for lib in b.findall('libraries/library') for p in lib.findall('packages/package')}
    nets={'0':''};pins={}
    for i,s in enumerate(b.findall('signals/signal'),1):
        nets[str(i)]=s.get('name')
        for c in s.findall('contactref'):pins[c.get('element'),c.get('pad')]=i
    fps=[];skipped=[];replaced=[]
    for e in b.findall('elements/element'):
        ref=e.get('name');pkg=packages[e.get('library'),e.get('package')]
        if not pkg.findall('smd') and not pkg.findall('pad'):
            skipped.append(ref);continue
        rot=e.get('rot','R0');mirror='M' in rot;angle=float(rot.split('R')[-1]);s,c=math.sin(math.radians(angle)),math.cos(math.radians(angle))
        # Reflection Y after Eagle's mirror-X and CCW rotation.
        pads=[];replacement='SOLDERJUMPER' in e.get('package')
        value=e.get('value','').strip()
        if replacement:
            pkg=packages['microbuilder','0805-NO'];value='0R' if 'CLOSED' in e.get('package') else 'DNP';replaced.append({'reference':ref,'value':value})
        for p in list(pkg.findall('smd'))+list(pkg.findall('pad')):
            x,y=float(p.get('x')),float(p.get('y'));x=-x if mirror else x
            # Bake world-relative rotation into pad centers, retain footprint rotation zero.
            x,y=c*x-s*y,-(s*x+c*y)
            a=(angle+float(p.get('rot','R0').split('R')[-1]))%180
            if a not in (0,90):raise ValueError('Unsupported non-orthogonal pad rotation')
            if p.tag=='pad':
                assert p.get('shape','round')=='round';shape={'Circle':{'diameter':float(p.get('diameter'))}}
                kind={'ThroughHole':{'drill':float(p.get('drill')),'outer_diameter':float(p.get('diameter'))}}
            else:
                w,h=float(p.get('dx')),float(p.get('dy'))
                if a==90:w,h=h,w
                roundness=float(p.get('roundness','0'))
                if roundness==100 and w==h:shape={'Circle':{'diameter':w}}
                elif roundness==0:shape={'Rect':{'width':w,'height':h}}
                else:raise ValueError('Unsupported roundness')
                layer='BCu' if mirror else 'FCu'
                kind={'Smd':{'layer':layer}}
            pads.append({'pin_number':p.get('name'),'pin_name':p.get('name'),'relative_pos':point(x,y),'shape':shape,'pad_type':kind,'net':pins.get((ref,p.get('name')))})
        fps.append({'reference':ref,'value':value,'package':('R_0805_link' if replacement else e.get('package')),'position':point(float(e.get('x')),30.48-float(e.get('y'))),'rotation':0.,'pads':pads,'silkscreen_lines':[]})
    board={'width':17.78,'height':30.48,'thickness':1.6,'footprints':fps,'tracks':[],'vias':[],'net_names':nets}
    expected={(r,p):n for (r,p),n in pins.items()}
    actual={(f['reference'],p['pin_number']):p['net'] for f in fps for p in f['pads'] if p['net'] is not None}
    assert expected==actual, (expected.keys()-actual.keys(),actual.keys()-expected.keys())
    out=ROOT/'build/tca9548a';out.mkdir(parents=True,exist_ok=True)
    (out/'board.json').write_text(json.dumps(board,indent=2)+'\n')
    constraints={'fixed':['JP1','JP3','U$4','U$5','FID1','FID3'],'groups':[['U1','C1']]}
    (SOURCE/'constraints.json').write_text(json.dumps(constraints,indent=2)+'\n')
    notes={'source':json.loads((SOURCE/'provenance.json').read_text()),'license':'CC-BY-SA-3.0','attribution':'Designed by Limor Fried/Ladyada for Adafruit Industries; layout experiment and 0805 jumper substitutions by Netlist Shatter. See the preserved README.md and license.txt.','modifications':{'jumpers_replaced_with_0805_links':replaced,'non_electrical_artwork_omitted':skipped,'routing_removed':True,'ground_pours_removed':True},'connected_terminals_preserved':len(actual),'hardware_validated':False}
    (out/'source-review.json').write_text(json.dumps(notes,indent=2)+'\n')
    print(f'{len(fps)} components, {len(actual)} connected terminals preserved; jumper substitutions explicit.')
if __name__=='__main__':main()
