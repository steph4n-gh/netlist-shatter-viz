"""Automatic cuts must use topology and keep native acceptance independent."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import shapely

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
import repair_native_congestion as repair
from native_restoration import NAME,bind_contacts,sha,snapshot,validate
from test_native_restoration import contacts
from test_coupled_completion import enclosed


class AutomaticClusterRepairTests(unittest.TestCase):
    def drill_geometry(self):
        def track(uid,a,b,layer=0,net=7):
            return {'uuid':uid,'net':net,'start':a,'end':b,'layer':layer,'width':.2,'locked':False,'arc':False}
        return {'nets':{'7':{'name':'arbitrary-owner'},'8':{'name':'other'}},
            'vias':[{'uuid':'drill','net':7,'at':[0,0],'drill_mm':.3,'diameter_mm':.6,'locked':False}],
            'tracks':[track('front',[0,0],[1,0]),track('back',[0,0],[0,1],2),
                track('crossing',[-1,.2],[1,.2]),track('retained',[1,0],[2,0])]}

    def test_via_cut_declares_both_caps_and_nearby_shadow_without_touching_far_tracks(self):
        geometry=self.drill_geometry();before=copy.deepcopy(geometry)
        cut=repair.native_drill_cap_cuts(geometry,['drill'],{7})
        self.assertEqual(set(cut),{'drill','front','back','crossing'})
        self.assertEqual(geometry,before)
        self.assertEqual(set(repair.native_drill_cap_cuts(geometry,cut,{7})),set(cut))

    def test_drill_cap_expansion_cannot_cross_owner_lock_protection_or_arc_guards(self):
        for change in ('foreign','locked','protected','arc','missing','duplicate'):
            with self.subTest(change=change):
                geometry=self.drill_geometry();protected=[];cut=['drill']
                if change=='foreign':geometry['tracks'][0]['net']=8
                elif change=='locked':geometry['tracks'][-1]['locked']=True
                elif change=='protected':protected=['arbitrary-owner']
                elif change=='arc':geometry['tracks'][0]['arc']=True
                elif change=='missing':cut=['absent']
                else:cut=['drill','drill']
                with self.assertRaises(ValueError):repair.native_drill_cap_cuts(geometry,cut,{7},protected)

    def test_native_preflight_allows_pending_reconnection_but_not_changed_survivors(self):
        before=contacts();after=copy.deepcopy(before)
        for field in ('native_items','contact_nodes','contact_node_items'):after[field].pop('tA')
        after['groups']['A']=[{'contact_nodes':['A1']},{'contact_nodes':['A2']}]
        self.assertTrue(repair.native_cut_preparation(before,after,['tA'],{'A'})['pass'])
        self.assertFalse(repair.native_partition_preservation(before,after,['tA'],{'A'})['pass'])
        for change in ('polygon','unknown-addition','selected-owner-addition','empty-copper','ambiguous','undeclared-cut'):
            with self.subTest(change=change):
                tampered=copy.deepcopy(after)
                if change=='polygon':tampered['contact_nodes']['B1']['polygon_nm'][0]['shell'][1][0]+=1
                elif change=='unknown-addition':tampered['native_items']['new']={'class':'PCB_TRACK','net':'B','locked':False}
                elif change=='selected-owner-addition':tampered['native_items']['new']={'class':'PCB_TRACK','net':'A','locked':False}
                elif change=='empty-copper':tampered['empty_cu_items']=['generated-empty']
                elif change=='ambiguous':tampered['contact_graph_unambiguous']=False
                else:tampered['native_items'].pop('tB')
                self.assertFalse(repair.native_cut_preparation(before,tampered,['tA'],{'A'})['pass'])

    def test_frontier_includes_legal_reached_layers_and_respects_remaining_via_budget(self):
        graph={'terminal_contacts':[[(0,0)],[(2,2)]],
            'via_edges':[{'from':(0,0),'to':(2,1)},{'from':(2,1),'to':(0,3)}]}
        bad=[{'groups':[0,1]}];parts={n:shapely.box(0,0,size,size) for n,size in [((0,0),1),((2,1),2),((2,2),3),((0,3),4)]}
        self.assertEqual(repair.reachable_frontier_nodes(graph,bad,parts,0),[(0,0)])
        self.assertEqual(repair.reachable_frontier_nodes(graph,bad,parts,1),[(0,0),(2,1)])
        self.assertEqual(repair.reachable_frontier_nodes(graph,bad,parts),[(0,0),(2,1),(0,3)])

    def test_measured_local_fanout_precedes_remote_paths_and_shorter_pad_prefix_survives(self):
        geometry=self.drill_geometry();geometry['tracks']=geometry['tracks'][:2]
        for i in range(3):geometry['tracks'].append({'uuid':f'prefix-{i}','net':8,'start':[i,2],'end':[i+1,2],
            'width':.2,'layer':0,'locked':False,'arc':False})
        primitives={p['uuid']:p for p in geometry['tracks']+geometry['vias']}
        boundary={'rows':[{'editable_copper':True,'boundary_support_mm':1,
            'native_primitives':[{'uuid':uid}], 'frontier_index':0,
            'frontier_distance_mm':.4 if uid in ('front','drill') else .8} for uid in primitives]}
        paths,_=repair.boundary_cut_choices(boundary,primitives,[0,2],set(),1,4,{(7,0,1,0),(8,0,0,2)})
        self.assertEqual(paths[0],frozenset(['front','back','drill']))
        prefix2=frozenset(['prefix-0','prefix-1']);prefix3=prefix2|{'prefix-2'}
        self.assertLess(paths.index(prefix2),paths.index(prefix3))
        limited,_=repair.boundary_cut_choices(boundary,primitives,[0,2],set(),1,2,{(8,0,0,2)})
        self.assertIn(prefix2,limited);self.assertNotIn(prefix3,limited)

    def test_owner_discovery_finishes_before_mandatory_native_drill_cap_replacement(self):
        geometry=self.drill_geometry();geometry['source_board_sha256']='arbitrary-source'
        geometry['nets']['9']={'name':'opened-demand'}
        geometry['pads']=[{'net':9},{'net':9}];geometry['layers']=[0];geometry['items']=[]
        geometry['holes']=[{'circle':[0,0,.15]}]
        geometry['tracks']=[geometry['tracks'][0],dict(geometry['tracks'][-1],uuid='next-owner-wall',net=8)]
        old=copy.deepcopy(geometry)
        def scope(data,*args,**kwargs):
            retained={item['uuid']for item in data['tracks']+data['vias']}
            return {'specs':{str(net):{'name':geometry['nets'][str(net)]['name'],'width':.2,'via_diameter':.6}for net in (7,8,9)},
                'components':lambda net:[{'net':net,'retained':retained},{'net':net,'retained':retained}],
                'domains':lambda *args,**kwargs:[shapely.box(0,0,10,10)],'limits':{},'items':[]}
        def graph(domains,via,groups,layers,maximum=None):
            net=groups[0]['net'];retained=groups[0]['retained']
            # Retaining the source cap during discovery reveals the owner's
            # second wall. Early virtual cap deletion hides that demand.
            blocked=(net==9 and 'drill' in retained)or(net==7 and {'front','next-owner-wall'}<=retained)
            return {'terminal_pairs':[{'groups':[0,1],'connected_under_via_limit':not blocked}],
                'terminal_contacts':[[(0,0)],[(0,0)]]}
        def boundary(region,layer,items,spec,data):
            uid='drill' if spec['net']==9 else 'next-owner-wall'
            net=7 if uid=='drill' else 8
            return {'rows':[{'editable_copper':True,'net':net,'boundary_support_mm':1,
                'native_primitives':[{'uuid':uid}]}]}
        drc={'unconnected_items':[{'items':[{'description':'Pad1 [opened-demand] of U1'}]}]}
        with patch('diagnose_routing_topology.model_scope',side_effect=scope),\
             patch('diagnose_routing_topology.polygon_graph',side_effect=graph),\
             patch('diagnose_routing_topology.boundary_attribution',side_effect=boundary):
            selection=repair.measured_owner_cluster(geometry,drc,{})
        self.assertEqual(geometry,old)
        self.assertEqual(set(selection['selected_nets']),{'arbitrary-owner','other','opened-demand'})
        self.assertEqual(set(selection['peeled_native_uuids']),{'drill','next-owner-wall'})
        self.assertEqual(set(repair.measured_owner_cut(geometry,selection,'arbitrary-source')),
            {'drill','front','next-owner-wall'})

    def test_completed_private_cluster_is_cleaned_then_checked_against_first_source(self):
        for mode in ('complete','miss','changed-survivor'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as temp:
                directory=Path(temp);board=directory/'candidate.kicad_pcb';board.write_text('(kicad_pcb original)')
                board.with_suffix('.kicad_pro').write_bytes(b'{"unchanged":true}\n')
                snapshot(board,directory/NAME)
                initial=contacts()
                for item in initial['native_items'].values():item['net']={'A':'signal','B':'barrier'}[item['net']]
                initial['groups']={'signal':[{'contact_nodes':['A1','tA']},{'contact_nodes':['A2']}],
                    'barrier':[{'contact_nodes':['B1','B2','tB']}]}
                initial['source_sha256']=sha(directory/NAME/'candidate.kicad_pcb')
                (directory/NAME/'native-connectivity.json').write_text(json.dumps(initial));bind_contacts(directory/NAME)
                g=enclosed();g['tracks'][0]['uuid']='tB';g['items'][2]['uuid']='tB'
                open_drc={'violations':[],'schematic_parity':[],'unconnected_items':[{'items':[{'description':'Pad1 [signal] of U1'}]}]}
                calls=[]
                def native(args,**kwargs):
                    args=list(map(str,args));d=Path(args[-1]) if 'drc' not in args else Path(args[-1]).parent
                    state=(d/'candidate.kicad_pcb').read_text()
                    if 'apply-grid' in args:
                        proposal=json.loads((d/'grid-route.json').read_text())
                        (d/'candidate.kicad_pcb').write_text('(kicad_pcb cut)' if proposal.get('remove') else '(kicad_pcb routed)')
                    elif 'dc-geometry' in args:
                        data=copy.deepcopy(g);data['source_board_sha256']=sha(d/'candidate.kicad_pcb')
                        (d/'dc-geometry.json').write_text(json.dumps(data))
                    elif 'native-connectivity' in args:
                        data=copy.deepcopy(initial)
                        removed=[] if 'original' in state else ['tB']
                        if 'clean' in state:removed.append('tA')
                        for uid in removed:
                            for field in ('native_items','contact_nodes','contact_node_items'):data[field].pop(uid)
                        if 'routed' in state or 'clean' in state:
                            data['groups']={'signal':[{'contact_nodes':['A1','A2']+([] if 'tA' in removed else ['tA'])}],
                                'barrier':[{'contact_nodes':['B1','B2']}]}
                        elif removed:data['groups']['barrier']=[{'contact_nodes':['B1']},{'contact_nodes':['B2']}]
                        if mode=='changed-survivor' and removed:data['contact_nodes']['B1']['polygon_nm'][0]['shell'][1][0]+=1
                        data['source_sha256']=sha(d/'candidate.kicad_pcb');(d/'native-connectivity.json').write_text(json.dumps(data))
                    elif 'drc' in args:
                        report=copy.deepcopy(open_drc)
                        if 'routed' in state or 'clean' in state:report['unconnected_items']=[]
                        if 'routed' in state:report['violations']=[{'type':'track_dangling','severity':'warning','items':[{'uuid':'tA'}]}]
                        Path(args[args.index('-o')+1]).write_text(json.dumps(report))
                def joint(data,codes,**kwargs):
                    calls.append(('route',kwargs));return {'complete_cluster':mode=='complete','tracks':[],'vias':[],'joint_trials':[]}
                def prune(d,*args,**kwargs):
                    calls.append(('prune',kwargs));self.assertEqual(validate(Path(d)/NAME),validate(directory/NAME))
                    (Path(d)/'candidate.kicad_pcb').write_text('(kicad_pcb clean)');return [{'accepted':True}]
                selection={'target':1,'target_name':'signal','owner_nets':[2],'removed_native_uuids':['tB']}
                with patch.object(repair.subprocess,'run',side_effect=native),patch.object(repair,'screen',return_value={'pass':True}),\
                     patch.object(repair,'nonregression',return_value=True),patch.object(repair,'measured_component_cuts',return_value={'selected':[selection]}),\
                     patch.object(repair,'measured_owner_cluster',return_value=None),patch.object(repair,'route',side_effect=ValueError('miss')),\
                     patch('joint_route.route_joint',side_effect=joint),patch('prune_native_routing.prune',side_effect=prune),\
                     patch.object(repair,'plane_partitions_preserved',return_value={'pass':True}),\
                     patch('solve_kicad.audit_preservation',return_value={'pass':True}),patch('optimize_kicad.unchanged_other_nets',return_value=True):
                    result=repair.coupled_completion(directory,['worker'],'kicad',{},seconds=60)
                if mode=='complete':
                    self.assertTrue(result['accepted'],result);self.assertTrue(result['complete'])
                    self.assertEqual(result['removed_original_copper_uuids'],['tA','tB'])
                    self.assertTrue(result['first_source_restoration']['pass'])
                    self.assertEqual([c[0] for c in calls],['route','prune'])
                    self.assertTrue(calls[0][1]['ordinary_first']);self.assertEqual(calls[1][1]['rounds'],12)
                    self.assertTrue(result['no_new_native_findings'])
                else:
                    self.assertFalse(result['accepted']);self.assertEqual(board.read_text(),'(kicad_pcb original)')
                    self.assertFalse(any(c[0]=='prune' for c in calls))
                    if mode=='changed-survivor':self.assertFalse(calls)


if __name__=='__main__':unittest.main()
