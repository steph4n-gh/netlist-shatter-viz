"""Obsolete drill cleanup requires real fixed-anchor copper improvements."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from shortcut_route import consolidate_dangling_vias,shortcut
from grid_route import shape
import prune_native_routing
from shapely.geometry import LineString
from test_grid_route import fixture,rectangle
from native_restoration import NAME,snapshot,bind_contacts,sha,validate


def first_source(directory):
    snapshot(directory/'candidate.kicad_pcb',directory/NAME)
    native={'source_sha256':sha(directory/NAME/'candidate.kicad_pcb'),
        'contact_schema':'physical-Cu-islands/1','contact_graph_unambiguous':True}
    (directory/NAME/'native-connectivity.json').write_text(json.dumps(native));bind_contacts(directory/NAME)


def board(points,at,drill=.3):
    data=fixture();data['source_board_sha256']='original-board'
    data['tracks']=[{'uuid':'t'+str(i),'net':1,'start':a,'end':b,'layer':0,
        'width':.15,'locked':False,'arc':False} for i,(a,b) in enumerate(zip(points,points[1:]))]
    data['vias']=[{'uuid':'via','net':1,'at':at,'through':True,'drill_mm':drill,
                  'diameter_mm':max(.6,drill+.3),'locked':False}]
    return data


class DanglingViaConsolidationTests(unittest.TestCase):
    def test_zero_copper_connector_keeps_exact_chain_anchors_for_real_consolidation(self):
        data=board([[5.8,5.8],[6,6],[6.0016,6],[6.2,6.2]],[6,6])
        original=copy.deepcopy(data);result=consolidate_dangling_vias(data,['via'])
        self.assertEqual(data,original)
        self.assertEqual(set(result['remove']),{'via','t0','t1','t2'})
        self.assertEqual(result['unrepaired_affected_native_uuids'],[])
        self.assertEqual(len(result['tracks']),1)
        self.assertEqual(result['tracks'][0]['start'],[5.8,5.8])
        self.assertEqual(result['tracks'][0]['end'],[6.2,6.2])
        row=result['chains'][0]
        self.assertLess(row['new_length_mm'],row['old_length_mm'])
        self.assertEqual((row['old_sections'],row['new_sections']),(3,1))
        self.assertTrue(result['candidate_only']);self.assertEqual(result['source_board_sha256'],'original-board')

    def test_collinear_consolidation_reduces_owners_without_artificial_geometry_edit(self):
        data=board([[5.8,5],[6,5],[6.2,5]],[6,5])
        self.assertFalse(shortcut(dict(data,vias=[]),1)['remove'])
        result=consolidate_dangling_vias(data,['via']);row=result['chains'][0]
        self.assertEqual(row['method'],'collinear consolidation')
        self.assertEqual(row['old_length_mm'],row['new_length_mm'])
        self.assertEqual((row['old_sections'],row['new_sections']),(2,1))
        # Replacing one identical section would merely relabel its UUID.
        data['tracks']=data['tracks'][:1]
        result=consolidate_dangling_vias(data,['via'])
        self.assertFalse(result['tracks']);self.assertEqual(result['unrepaired_affected_native_uuids'],['t0'])

    def test_continuous_shortening_keeps_full_width_clear_of_obstacle_after_quantization(self):
        data=board([[5.5,6],[5.5,7],[6.5,7],[6.5,6]],[6,7],drill=1.2)
        obstacle=rectangle(5.8,5.5,6.2,6.8)
        data['keepouts']=[{'layers':[0],'tracks':True,'vias':True,'shapes':obstacle}]
        result=consolidate_dangling_vias(data,['via'])
        self.assertEqual(set(result['remove']),{'via','t0','t1','t2'})
        row=result['chains'][0]
        self.assertEqual(row['method'],'continuous same-layer shortening')
        self.assertLess(row['new_length_mm'],row['old_length_mm']-.01)
        self.assertEqual(row['start'],[5.5,6]);self.assertEqual(row['end'],[6.5,6])
        forbidden=shape(obstacle).buffer(.075+.002)
        for track in result['tracks']:
            self.assertEqual(track['width'],.15);self.assertEqual(track['layer'],0)
            self.assertFalse(LineString([track['start'],track['end']]).intersects(forbidden))
            for point in (track['start'],track['end']):
                self.assertTrue(all(abs(v*1e6-round(v*1e6))<1e-7 for v in point))

    def test_protected_locked_curved_and_unrelated_copper_are_retained(self):
        for change in ('protected','locked','via-locked','blind','arc'):
            with self.subTest(change=change):
                data=board([[5.8,5],[6,5],[6.2,5]],[6,5]);protected=[]
                if change=='protected':protected=['signal']
                elif change=='locked':data['tracks'][0]['locked']=True
                elif change=='via-locked':data['vias'][0]['locked']=True
                elif change=='blind':data['vias'][0]['through']=False
                else:data['tracks'][0]['arc']=True
                result=consolidate_dangling_vias(data,['via'],protected)
                self.assertFalse(result['tracks']);self.assertNotIn('t0',result['remove'])
        data=board([[5.8,5],[6,5],[6.2,5]],[6,5])
        data['tracks'].append(dict(data['tracks'][0],uuid='foreign',net=2))
        data['tracks'].append(dict(data['tracks'][0],uuid='remote',start=[3,3],end=[4,3]))
        result=consolidate_dangling_vias(data,['via'])
        self.assertNotIn('foreign',result['remove']);self.assertNotIn('remote',result['remove'])

    def test_native_source_gate_controls_atomic_adoption_and_restores_trial_project(self):
        for source_pass in (False,True):
            with self.subTest(source_pass=source_pass),tempfile.TemporaryDirectory() as tmp:
                d=Path(tmp);original='(kicad_pcb (segment (net "signal") (uuid "t0")) (via (net "signal") (uuid "via")))'
                changed='(kicad_pcb (segment (net "signal") (uuid "replacement")))'
                (d/'candidate.kicad_pcb').write_text(original)
                project=b'{"rules":"exact"}\n';(d/'candidate.kicad_pro').write_bytes(project)
                first_source(d)
                before={'violations':[{'type':'via_dangling','severity':'warning','items':[{'uuid':'via'}]}],
                        'unconnected_items':[]}
                (d/'routed-drc.json').write_text(json.dumps(before))
                data=board([[5.8,5],[6,5],[6.2,5]],[6,5]);data['zones']=[]
                data['layer_names']={'0':'F.Cu','2':'B.Cu'}
                def native(command,**kwargs):
                    if command[1]=='pcb':
                        Path(command[command.index('-o')+1]).write_text(json.dumps({'violations':[],'unconnected_items':[]}))
                        return SimpleNamespace(returncode=0)
                    mode=command[1];trial=Path(command[2])
                    if mode=='dc-geometry':
                        self.assertEqual((trial/'candidate.kicad_pro').read_bytes(),project)
                        geometry=copy.deepcopy(data)
                        geometry['source_board_sha256']=hashlib.sha256((trial/'candidate.kicad_pcb').read_bytes()).hexdigest()
                        (trial/'dc-geometry.json').write_text(json.dumps(geometry))
                    elif mode=='apply-grid':
                        plan=json.loads((trial/'grid-route.json').read_text())
                        self.assertEqual(set(plan['remove']),{'via','t0','t1'})
                        self.assertEqual(len(plan['tracks']),1)
                        (trial/'candidate.kicad_pcb').write_text(changed)
                    elif mode=='native-connectivity':
                        (trial/'native-connectivity.json').write_text(json.dumps({
                            'source_sha256':hashlib.sha256((trial/'candidate.kicad_pcb').read_bytes()).hexdigest()}))
                    else:raise AssertionError('Unexpected native cleanup mode '+mode)
                    (trial/'candidate.kicad_pro').write_bytes(b'{"migrated":true}')
                    return SimpleNamespace(returncode=0)
                with patch.object(prune_native_routing.subprocess,'run',side_effect=native),\
                     patch('native_restoration.audit',return_value={'pass':source_pass,'required':2,'restored':2 if source_pass else 1}):
                    trials=prune_native_routing.prune(d,['native'],'kicad')
                self.assertEqual(trials[0]['accepted'],source_pass)
                self.assertEqual((d/'candidate.kicad_pcb').read_text(),changed if source_pass else original)
                self.assertEqual((d/'candidate.kicad_pro').read_bytes(),project)
                self.assertEqual((d/'prune-00-via-consolidation/candidate.kicad_pro').read_bytes(),project)

    def test_second_dangling_via_cleanup_inherits_the_same_immutable_first_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp)
            def native_board(uids):
                return '(kicad_pcb '+''.join('('+('via' if uid.startswith('via') else 'segment')+
                    ' (net "signal") (uuid "'+uid+'"))' for uid in uids)+')'
            states={'original':['t0','t1','via','t2','t3','via2'],
                'one':['new1','t2','t3','via2'],'two':['new1','new2']}
            boards={state:native_board(uids) for state,uids in states.items()}
            (d/'candidate.kicad_pcb').write_text(boards['original'])
            project=b'{"rules":"exact"}\n';(d/'candidate.kicad_pro').write_bytes(project)
            first_source(d);original_manifest=validate(d/NAME)
            before={'violations':[{'type':'via_dangling','severity':'warning','items':[{'uuid':'via'}]}],
                'unconnected_items':[]}
            (d/'routed-drc.json').write_text(json.dumps(before))
            data=board([[5.8,5],[6,5],[6.2,5]],[6,5]);data['zones']=[]
            data['layer_names']={'0':'F.Cu','2':'B.Cu'}
            data['tracks'].extend([dict(data['tracks'][i],uuid='t'+str(i+2),
                start=[v+2 if j==0 else v for j,v in enumerate(data['tracks'][i]['start'])],
                end=[v+2 if j==0 else v for j,v in enumerate(data['tracks'][i]['end'])])for i in range(2)])
            data['vias'].append(dict(data['vias'][0],uuid='via2',at=[8,5]))
            audits=[]
            def native(command,**kwargs):
                command=list(map(str,command))
                trial=Path(command[-1]).parent if command[1]=='pcb' else Path(command[2])
                state=next(s for s,value in boards.items() if (trial/'candidate.kicad_pcb').read_text()==value)
                self.assertEqual((trial/'candidate.kicad_pro').read_bytes(),project)
                if command[1]=='pcb':
                    warning='via' if state=='original' else 'via2' if state=='one' else None
                    report={'violations':[] if warning is None else [{'type':'via_dangling','severity':'warning','items':[{'uuid':warning}]}],
                        'unconnected_items':[]}
                    Path(command[command.index('-o')+1]).write_text(json.dumps(report))
                elif command[1]=='dc-geometry':
                    geometry=copy.deepcopy(data)
                    geometry['tracks']=[t for t in geometry['tracks'] if t['uuid'] in states[state]]
                    geometry['vias']=[v for v in geometry['vias'] if v['uuid'] in states[state]]
                    geometry['source_board_sha256']=sha(trial/'candidate.kicad_pcb')
                    (trial/'dc-geometry.json').write_text(json.dumps(geometry))
                elif command[1]=='apply-grid':
                    plan=json.loads((trial/'grid-route.json').read_text())
                    if state=='original':self.assertEqual(set(plan['remove']),{'via','t0','t1'})
                    else:self.assertEqual(set(plan['remove']),{'via2','t2','t3'})
                    (trial/'candidate.kicad_pcb').write_text(boards['one' if state=='original' else 'two'])
                elif command[1]=='native-connectivity':
                    (trial/'native-connectivity.json').write_text(json.dumps({'source_sha256':sha(trial/'candidate.kicad_pcb')}))
                else:raise AssertionError('Unexpected native mode '+command[1])
                (trial/'candidate.kicad_pro').write_bytes(b'{"migrated":true}')
                return SimpleNamespace(returncode=0)
            def audit(baseline,contacts,protected):
                self.assertEqual(validate(baseline),original_manifest)
                self.assertEqual((Path(baseline)/'candidate.kicad_pcb').read_text(),boards['original'])
                audits.append(Path(baseline));return {'pass':True,'required':2,'restored':2}
            with patch.object(prune_native_routing.subprocess,'run',side_effect=native),patch('native_restoration.audit',side_effect=audit):
                trials=prune_native_routing.prune(d,['native'],'kicad',rounds=12)
            self.assertTrue(trials[0]['accepted'],trials)
            nested=trials[0]['tail_prune'][0]
            self.assertTrue(nested['accepted'],nested)
            self.assertEqual((d/'candidate.kicad_pcb').read_text(),boards['two'])
            self.assertEqual(json.loads((d/'routed-drc.json').read_text())['violations'],[])
            self.assertEqual((d/'candidate.kicad_pro').read_bytes(),project)
            child=d/'prune-00-via-consolidation'
            grandchild=child/'tail-prune-00-via-consolidation'
            self.assertEqual(validate(child/NAME),original_manifest)
            self.assertEqual(validate(grandchild/NAME),original_manifest)
            self.assertEqual(set(audits),{d/NAME,child/NAME})


if __name__=='__main__':unittest.main()
