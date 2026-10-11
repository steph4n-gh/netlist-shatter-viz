"""Repair selection must reach multi-net escapes without touching locked copper."""
import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import shapely
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from repair_native_congestion import repair_plans,local_removals,frontier_plans,routing_orders,next_repair_option,boundary_cut_choices,measured_component_cuts,measured_owner_cut,measured_owner_cluster


class CongestionPlansTests(unittest.TestCase):
    def test_owner_selection_keeps_existing_target_tree_and_cuts_only_measured_boundary(self):
        from test_coupled_completion import enclosed
        from test_grid_route import rectangle
        geometry=enclosed();geometry['source_board_sha256']='fixture-source'
        target_track={'uuid':'retained-target-track','net':1,'layer':0,'width':.2,
            'start':[10,5],'end':[10,8],'locked':False,'arc':False}
        target_via={'uuid':'retained-target-via','net':1,'at':[10,8],
            'diameter_mm':.6,'drill_mm':.3,'through':True,'locked':False}
        geometry['tracks'].append(target_track);geometry['vias'].append(target_via)
        geometry['holes'].append({'circle':[10,8,.15]})
        geometry['pads'].append({'net':1,'ref':'remote','pin':'1'})
        geometry['items'].extend([
            {'uuid':target_track['uuid'],'net':1,'kind':'copper','terminals':[[10,5],[10,8]],
             'clearance':.2,'shapes':{'0':rectangle(9.9,4.9,10.1,8.1)}},
            {'uuid':target_via['uuid'],'net':1,'kind':'copper','terminals':[[10,8],[10,8]],
             'clearance':.2,'shapes':{str(layer):[{'circle':[10,8,.3]}] for layer in [0,2]}},
            {'net':1,'kind':'pad','terminals':[[10,8]],'clearance':.2,'shapes':{'0':rectangle(9.7,7.7,10.3,8.3)}}])
        before=copy.deepcopy(geometry)
        drc={'unconnected_items':[{'items':[{'description':'Pad 1 [signal] of a'}]}]}
        result=measured_owner_cluster(geometry,drc,{'net_limits':[{'net':'signal','max_vias':1}]})
        self.assertIsNotNone(result);self.assertEqual(geometry,before)
        self.assertEqual(result['rounds'][0]['removed_native_uuids'],[])
        cut=set(measured_owner_cut(geometry,result,'fixture-source'))
        self.assertTrue(cut);self.assertFalse(cut&{target_track['uuid'],target_via['uuid']})
        measured={p['uuid'] for row in result['rounds'] for item in row['boundary']['rows']
                  if item['editable_copper'] for p in item['native_primitives']}
        self.assertLessEqual(cut,measured)
        target=next(row['graph'] for row in result['demand_graphs'] if row['net']=='signal')
        self.assertEqual(len(target['terminal_contacts']),2)
        self.assertTrue(all(row['connected_under_via_limit'] for row in target['terminal_pairs']))

    def test_owner_demand_uses_partial_cut_with_shadowed_via_retained(self):
        from test_coupled_completion import enclosed
        from diagnose_routing_topology import polygon_graph
        geometry=enclosed();before=copy.deepcopy(geometry);budgets=[]
        def graph(domains,via,groups,layers,maximum=None):
            if any(item.get('uuid')=='shadowed-via' for group in groups for item in group):budgets.append(maximum)
            return polygon_graph(domains,via,groups,layers,maximum)
        profile={'net_limits':[{'net':'barrier','max_vias':1}]}
        drc={'unconnected_items':[{'items':[{'description':'Pad 1 [signal] of a'}]}]}
        with patch('diagnose_routing_topology.polygon_graph',side_effect=graph):
            result=measured_owner_cluster(geometry,drc,profile)
        self.assertIsNotNone(result);self.assertEqual(geometry,before)
        self.assertNotIn('shadowed-via',result['peeled_native_uuids'])
        owner=next(row['graph'] for row in result['demand_graphs'] if row['net']=='barrier')
        self.assertEqual(len(owner['terminal_contacts']),1)
        self.assertTrue(budgets);self.assertEqual(set(budgets),{0})

    def test_connected_boundary_path_precedes_fragment_and_via_cuts(self):
        def track(uid,a,b,layer=0,net=2):return {'uuid':uid,'net':net,'layer':layer,'start':a,'end':b,'locked':False,'arc':False}
        primitives={p['uuid']:p for p in [track('z',[0,0],[1,0]),track('a',[1,0],[2,0]),track('b',[2,0],[3,0]),
            track('other-layer',[1,0],[2,0],2),track('locked-owner',[0,0],[1,0],net=3)]}
        primitives['via']={'uuid':'via','net':4,'at':[5,5],'locked':False}
        boundary={'rows':[{'editable_copper':True,'boundary_support_mm':100 if uid=='via' else 1,
            'native_primitives':[{'uuid':uid}]} for uid in primitives]}
        before=copy.deepcopy(primitives)
        paths,singles=boundary_cut_choices(boundary,primitives,[0,2],{3},1,4)
        self.assertEqual(paths,[frozenset(['a','b','z'])])
        self.assertEqual(singles[-1],frozenset(['via']))
        self.assertNotIn(frozenset(['locked-owner']),singles);self.assertEqual(primitives,before)
        self.assertEqual(boundary_cut_choices(boundary,primitives,[0,2],{3},1,2)[0],[])

    def test_bounded_measured_search_rotates_targets_and_tests_complete_path(self):
        geometry={'nets':{str(n):{'name':f'n{n}'} for n in range(1,4)},'pads':[{'net':1},{'net':2}],
            'items':[],'vias':[],'holes':[],'layers':[0],
            'tracks':[{'uuid':str(i),'net':3,'start':[i,0],'end':[i+1,0],'layer':0,'locked':False,'arc':False} for i in range(3)]}
        scope={'specs':{str(n):{'name':f'n{n}','width':.2,'via_diameter':.6} for n in range(1,4)},
            'components':lambda net:[[],[]],'domains':lambda *args,**kw:[shapely.box(0,0,10,10)],'limits':{},'items':[]}
        graph={'terminal_pairs':[{'groups':[0,1],'connected_under_via_limit':False}],
            'terminal_contacts':[[(0,0)],[(0,0)]]}
        boundary={'rows':[{'editable_copper':True,'boundary_support_mm':1,'native_primitives':[{'uuid':str(i)}]} for i in range(3)]}
        with patch('repair_native_congestion.native_open_counts',return_value={1:1,2:1,3:0}), \
             patch('diagnose_routing_topology.model_scope',return_value=scope) as model, \
             patch('diagnose_routing_topology.polygon_graph',return_value=graph), \
             patch('diagnose_routing_topology.boundary_attribution',return_value=boundary):
            result=measured_component_cuts(geometry,{}, {},states=4)
        self.assertEqual([r['target'] for r in result['rows']],[1,2,1,2])
        self.assertEqual([len(r['removed_native_uuids']) for r in result['rows']],[0,0,3,3])
        self.assertEqual(result['states_examined'],4);self.assertFalse(result['selected'])
        self.assertTrue(all(call.kwargs['preserve_source'] for call in model.call_args_list))

    def test_measured_owner_cut_retains_unpeeled_copper_and_rejects_stale_or_unsafe_ids(self):
        geometry={'source_board_sha256':'source','nets':{'1':{'name':'target'},'2':{'name':'owner'},'3':{'name':'foreign'}},
            'tracks':[{'uuid':uid,'net':net,'locked':False,'arc':False} for uid,net in [('target',1),('cut',2),('retained',2),('foreign',3)]], 'vias':[]}
        selection={'source_sha256':'source','selected_nets':['target','owner'],'peeled_native_uuids':['target','cut']}
        self.assertEqual(measured_owner_cut(geometry,selection,'source'),['target','cut'])
        for mode in ('stale','stale_geometry','unknown','foreign','duplicate','locked','arc','protected'):
            g=copy.deepcopy(geometry);s=copy.deepcopy(selection);protected=[]
            if mode=='stale':s['source_sha256']='old'
            elif mode=='stale_geometry':g['source_board_sha256']='old'
            elif mode=='unknown':s['peeled_native_uuids'].append('missing')
            elif mode=='foreign':s['peeled_native_uuids'].append('foreign')
            elif mode=='duplicate':s['peeled_native_uuids'].append('cut')
            elif mode=='locked':g['tracks'][2]['locked']=True
            elif mode=='arc':g['tracks'][1]['arc']=True
            else:protected=['owner']
            with self.assertRaises(ValueError,msg=mode):measured_owner_cut(g,s,'source',protected)

    def test_repeated_failed_target_does_not_starve_other_opens(self):
        first=(1,(2,),(1,2),None)
        expanded=(1,(2,3),(1,2,3),0)
        other=(4,(5,),(4,5),None)
        # Frontier expansions retain priority within a target, but another
        # connection gets its first trial before this one gets its second.
        self.assertEqual(next_repair_option([first,expanded,other],{}),first)
        self.assertEqual(next_repair_option([expanded,other],{1:1}),other)
        self.assertEqual(next_repair_option([expanded,other],{1:1,4:1}),expanded)

    def test_small_escape_exhausts_orders_but_large_groups_stay_bounded(self):
        self.assertEqual(len(routing_orders(1,(2,3))),6)
        self.assertIn((3,1,2),routing_orders(1,(2,3)))
        self.assertEqual(len(routing_orders(1,(2,3,4))),2)

    def test_measured_blockers_expand_failed_neighborhood_and_respect_locks(self):
        g={'nets':{str(n):{'name':f'n{n}'} for n in range(1,7)},
           'pads':[{'net':n} for n in range(1,7) for _ in range(2)],
           'tracks':[{'net':4,'locked':True},{'net':5}], 'vias':[]}
        search={'blocking_copper':[{'net':n,'frontier_contacts':10} for n in [1,2,4,5,6,3]]}
        self.assertEqual(frontier_plans(g,1,search,(2,),protected=['n6']),[(1,(2,3))])

    def test_two_blockers_are_considered_before_distant_single_moves(self):
        g={'nets':{str(n):{'name':f'n{n}'} for n in range(1,8)},'pads':[], 'tracks':[], 'vias':[]}
        for n in range(1,8):
            g['pads'].extend([{'net':n,'at':[0,0]},{'net':n,'at':[10,0]}])
            if n>1:g['tracks'].append({'net':n,'start':[0,n*.1],'end':[10,n*.1],'locked':False})
        drc={'unconnected_items':[{'items':[{'description':'Pad 1 [n1] of U1 on F.Cu'}]}]}
        before=copy.deepcopy(g);plans=repair_plans(g,drc)
        self.assertIn((1,(2,3)),plans)
        self.assertLess(plans.index((1,(2,3))),plans.index((1,(7,))))
        self.assertEqual(g,before)
        g['tracks'][0]['locked']=True
        plans=repair_plans(g,drc,protected=['n3'])
        self.assertTrue(plans)
        self.assertTrue(all(2 not in ns and 3 not in ns for _,ns in plans))
        self.assertEqual(repair_plans(g,drc,protected=['n1']),[])

    def test_missing_lock_information_is_protected_and_large_nets_can_move_locally(self):
        g={'nets':{'1':{'name':'signal'},'2':{'name':'supply'}},
           'pads':[{'net':1,'at':[0,0]},{'net':1,'at':[1,0]}]+[{'net':2,'at':[x,1]} for x in range(9)],
           'tracks':[{'net':2,'start':[0,.1],'end':[1,.1],'locked':False}], 'vias':[]}
        drc={'unconnected_items':[{'items':[{'description':'Pad 1 [signal] of U1'}]}]}
        self.assertIn((1,(2,)),repair_plans(g,drc))
        g['pads']=g['pads'][:4];del g['tracks'][0]['locked']
        self.assertEqual(repair_plans(g,drc),[])

    def test_local_cut_preserves_width_layer_outside_copper_and_locks(self):
        track={'net':7,'start':[-5,0],'end':[5,0],'width':.3,'layer':0,'uuid':'editable','locked':False,'arc':False}
        locked=dict(track,uuid='locked',locked=True)
        g={'tracks':[track,locked], 'vias':[{'net':7,'at':[0,0],'uuid':'via','locked':True}]}
        before=copy.deepcopy(g);result=local_removals(g,7,[[0,0]])
        self.assertEqual(result['remove'],['editable']);self.assertEqual(g,before)
        self.assertEqual(len(result['tracks']),2)
        self.assertTrue(all(t['width']==.3 and t['layer']==0 and t['net']==7 for t in result['tracks']))
        ends={tuple(t['start']) for t in result['tracks']}|{tuple(t['end']) for t in result['tracks']}
        self.assertEqual(ends,{(-5,0),(-3,0),(3,0),(5,0)})


if __name__=='__main__':unittest.main()
