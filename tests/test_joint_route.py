"""Independent contact/clearance checks for coordinated candidate routing."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

import shapely
from shapely.geometry import LineString,Point

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from grid_route import route,routing_geometry,append_path,shape
from joint_route import route_joint,passage_sections

FIXTURES=Path(__file__).parent/'fixtures'


def fixture(name='joint-passage.json'):
    return json.loads((FIXTURES/name).read_text())


class JointRouteTests(unittest.TestCase):
    def assert_layer_connected(self,data,result,net):
        nodes=[];edges={}
        for layer in data['layers']:
            copper=[shape(item['shapes'][str(layer)])for item in data['items']
                    if item['net']==net and str(layer)in item['shapes']]
            copper.extend(LineString([t['start'],t['end']]).buffer(t['width']/2)
                          for t in result['tracks']if t['net']==net and t['layer']==layer)
            copper.extend(Point(v['at']).buffer(v['diameter']/2)for v in result['vias']if v['net']==net)
            for polygon in shapely.get_parts(shapely.union_all(copper)):
                nodes.append((layer,polygon));edges[len(nodes)-1]=set()
        for via in (v for v in result['vias']if v['net']==net):
            contacts=[i for i,(_,polygon)in enumerate(nodes)if polygon.covers(Point(via['at']))]
            for i in contacts:edges[i].update(contacts)
        seen={0};pending=[0]
        while pending:
            for other in edges[pending.pop()]-seen:seen.add(other);pending.append(other)
        self.assertEqual(seen,set(edges),'Every layer fragment must join through literal via copper')

    def assert_literal_routes(self,data,result):
        self.assertTrue(result['complete_cluster']);self.assertFalse(result['failed'])
        self.assertLessEqual(result['states'],result['maximum_states'])
        outline=shape(data['outline'])
        for net in (1,2):
            pads=[shape(polys)for item in data['items']if item['net']==net for polys in item['shapes'].values()]
            copper=[]
            for track in result['tracks']:
                if track['net']!=net:continue
                line=LineString([track['start'],track['end']]);copper.append(line.buffer(track['width']/2))
                self.assertTrue(outline.buffer(-data['edge_clearance']).covers(copper[-1]))
                for other in result['tracks']:
                    if other['net']==net or other['layer']!=track['layer']:continue
                    clearance=max(data['nets'][str(net)]['clearance'],data['nets'][str(other['net'])]['clearance'])
                    self.assertGreaterEqual(line.distance(LineString([other['start'],other['end']])),(track['width']+other['width'])/2+clearance)
            joined=shapely.union_all(pads+copper)
            self.assertEqual(len(shapely.get_parts(joined)),1)
        self.assertFalse(result['vias'])

    def test_shared_passage_closes_both_nets_after_greedy_blocks_the_second(self):
        data=fixture();original=copy.deepcopy(data)
        greedy=route(data,continuous=True,step=.025,seconds=3)
        self.assertEqual(greedy['completed_nets'],[1]);self.assertTrue(greedy['failed'])
        result=route_joint(data,[1,2],seconds=4)
        self.assert_literal_routes(data,result)
        self.assertTrue(any(row.get('whole_lane_clearance_pass')for row in result['joint_trials']))
        self.assertEqual(data,original)

    def test_frozen_rotated_case_with_extra_obstacle_closes_without_coordinate_policy(self):
        data=fixture('joint-passage-held-out.json')
        self.assertTrue(route(data,continuous=True,step=.025,seconds=3)['failed'])
        self.assert_literal_routes(data,route_joint(data,[1,2],seconds=4))

    def test_direct_first_joins_literal_contacts_without_floating_trunks(self):
        data=fixture();original=copy.deepcopy(data)
        result=route_joint(data,[1,2],seconds=4,ordinary_first=True)
        self.assert_literal_routes(data,result)
        self.assertFalse(any('bundle'in row for row in result['joint_trials']))
        self.assertEqual(data,original)

    def test_direct_first_disconnected_cluster_emits_no_copper_or_repeated_search(self):
        data=fixture();data.pop('routing_profile')
        data['outline']=[{'shell':[[0,0],[12,0],[12,10],[0,10]],'holes':[]}]
        data['keepouts']=[{'layers':[0,2],'tracks':True,'vias':True,'shapes':[
            {'shell':[[4,0],[6,0],[6,10],[4,10]],'holes':[]}]}]
        result=route_joint(data,[1,2],seconds=3,ordinary_first=True)
        self.assertFalse(result['complete_cluster']);self.assertEqual(result['tracks'],[])
        self.assertEqual(result['vias'],[]);self.assertEqual(result['states'],1)

    def test_unequal_widths_keep_literal_pair_clearance(self):
        data=fixture();data['nets']['1']['width']=.22
        self.assert_literal_routes(data,route_joint(data,[1,2],seconds=4))

    def test_shared_trunk_can_visit_another_layer_than_both_pad_layers(self):
        data=fixture()
        for limit in data['routing_profile']['net_limits']:
            limit['allowed_layers']=['F.Cu','B.Cu'];limit['max_vias']=2
        data['keepouts']=[{'layers':[0],'tracks':True,'vias':False,'shapes':[
            {'shell':[[4,4.65],[8,4.65],[8,4.9],[4,4.9]],'holes':[]}]}]
        result=route_joint(data,[1,2],seconds=4)
        self.assertTrue(result['complete_cluster']);self.assertFalse(result['failed'])
        self.assertLessEqual(result['states'],24)
        self.assertTrue(any(row.get('whole_lane_clearance_pass') and row['bundle']['layer']==2
                            for row in result['joint_trials']))
        rejected=next(i for i,row in enumerate(result['joint_trials'])if row.get('branch_rejected_net'))
        self.assertTrue(any(row.get('connection_pass') and row.get('preferred_layer')is not None
                            for row in result['joint_trials'][rejected+1:]),
                        'A conflicting default transition must backtrack to a legal alternative')
        self.assertTrue(result['vias'])
        for net in (1,2):
            self.assertLessEqual(sum(v['net']==net for v in result['vias']),2)
            self.assert_layer_connected(data,result,net)
        for track in result['tracks']:
            line=LineString([track['start'],track['end']])
            for other in result['tracks']:
                if other['net']==track['net'] or other['layer']!=track['layer']:continue
                self.assertGreaterEqual(line.distance(LineString([other['start'],other['end']])),
                                        (track['width']+other['width'])/2+.15)
            for via in result['vias']:
                if via['net']!=track['net']:
                    self.assertGreaterEqual(line.distance(Point(via['at'])),(track['width']+via['diameter'])/2+.15)

    def test_multi_component_tree_can_reserve_a_shared_trunk_before_remote_join(self):
        data=fixture()
        for limit in data['routing_profile']['net_limits']:
            limit.update(allowed_layers=['F.Cu','B.Cu'],max_vias=3)
        data['items'][0]['shapes']['0']=[{'shell':[[1.9,4.9],[2.1,4.9],[2.1,5.1],[1.9,5.1]],'holes':[]}]
        remote=copy.deepcopy(data['items'][1]);remote['terminals']=[[10,8]]
        remote['shapes']['0']=[{'shell':[[9.8,7.8],[10.2,7.8],[10.2,8.2],[9.8,8.2]],'holes':[]}]
        data['items'].append(remote)
        data['keepouts']=[{'layers':[0],'tracks':True,'vias':False,'shapes':[
            {'shell':[[4,4.65],[8,4.65],[8,4.9],[4,4.9]],'holes':[]}]}]
        self.assertEqual(len(routing_geometry(data)['components'](1)),3)
        result=route_joint(data,[1,2],seconds=4)
        self.assertTrue(result['complete_cluster']);self.assertLessEqual(result['states'],24)
        self.assertTrue(any(row.get('whole_lane_clearance_pass')for row in result['joint_trials']))
        for net in (1,2):self.assert_layer_connected(data,result,net)

    def test_floating_bundle_allows_more_than_twelve_separate_pad_attachments(self):
        data=fixture();pad=copy.deepcopy(data['items'][0]);data['items']=[]
        locations={1:((2,5),(9.2,5.8),(10.5,5.9),(9.35,6.9),(10.7,7.1),(9.1,8.1),(10.55,8.3),(9.4,9.15)),
                   2:((2,3),(9.2,3.8),(10.6,3.7),(9.35,2.7),(10.7,2.5),(9.1,1.5),(10.5,1.3),(11.15,4.65))}
        for net,points in locations.items():
            for index,(x,y)in enumerate(points):
                item=copy.deepcopy(pad);item['net']=net;item['terminals']=[[x,y]]
                radius=.05 if index==0 else .15
                item['shapes']['0']=[{'shell':[[x-radius,y-radius],[x+radius,y-radius],
                    [x+radius,y+radius],[x-radius,y+radius]],'holes':[]}]
                data['items'].append(item)
        context=routing_geometry(data)
        self.assertEqual([len(context['components'](net))for net in (1,2)],[8,8])
        result=route_joint(data,[1,2],seconds=4,maximum_states=24)
        self.assert_literal_routes(data,result)
        joins=[row for row in result['joint_trials']if row.get('connection_pass')]
        self.assertGreater(len(joins),12)
        self.assertTrue(all(row['components_before']-row['components_after']==1 for row in joins))

    def test_split_bundle_core_does_not_substitute_a_remote_pocket(self):
        data=fixture()
        for point in data['outline'][0]['shell']:
            if point[1]==4.65:point[1]=4.9
            if point[1]==5.35:point[1]=5.1
        self.assertEqual(passage_sections(shape(data['outline']),[2,5],[10,5],2,.3,lambda:None),[])

    def test_disconnected_contact_does_not_spend_states_on_layer_preferences(self):
        data=fixture();data['nets'].pop('2');data['items']=data['items'][:2]
        data['routing_profile']['net_limits']=data['routing_profile']['net_limits'][:1]
        data['routing_profile']['net_limits'][0].update(allowed_layers=['F.Cu','B.Cu'],max_vias=4)
        data['outline']=[{'shell':[[0,0],[12,0],[12,10],[0,10]],'holes':[]}]
        data['keepouts']=[{'layers':[0,2],'tracks':True,'vias':True,'shapes':[
            {'shell':[[4,0],[6,0],[6,10],[4,10]],'holes':[]}]}]
        result=route_joint(data,[1],seconds=3)
        self.assertFalse(result['complete_cluster']);self.assertEqual(result['tracks'],[])
        self.assertEqual(result['states'],1)

    def test_disconnected_two_component_net_stops_an_append_only_branch(self):
        data=fixture();data.pop('routing_profile')
        data['outline']=[{'shell':[[0,0],[12,0],[12,10],[0,10]],'holes':[]}]
        data['items']=data['items'][:2]+[copy.deepcopy(data['items'][2])]
        remote=copy.deepcopy(data['items'][2]);remote['terminals']=[[2,8]]
        remote['shapes']['0']=[{'shell':[[1.8,7.8],[2.2,7.8],[2.2,8.2],[1.8,8.2]],'holes':[]}]
        data['items'].append(remote)
        data['keepouts']=[{'layers':[0,2],'tracks':True,'vias':True,'shapes':[
            {'shell':[[4,0],[6,0],[6,10],[4,10]],'holes':[]}]}]
        result=route_joint(data,[1,2],seconds=3)
        self.assertFalse(result['complete_cluster']);self.assertEqual(result['tracks'],[])
        self.assertEqual(result['states'],1)
        self.assertEqual(result['best_partial_components'],{1:2,2:2})

    def test_narrow_passage_returns_no_partial_copper_as_a_complete_cluster(self):
        data=fixture()
        for point in data['outline'][0]['shell']:
            if point[1]==4.65:point[1]=4.85
            if point[1]==5.35:point[1]=5.15
        result=route_joint(data,[1,2],seconds=3)
        self.assertFalse(result['complete_cluster']);self.assertTrue(result['failed'])
        self.assertEqual(result['tracks'],[]);self.assertEqual(result['vias'],[])
        self.assertEqual(result['completed_nets'],[])

    def test_remote_join_is_not_completion_of_a_three_component_net(self):
        data=fixture();data['nets'].pop('2');data['routing_profile']['net_limits']=data['routing_profile']['net_limits'][:1]
        data['outline']=[{'shell':[[0,0],[12,0],[12,10],[0,10]],'holes':[]}]
        first=copy.deepcopy(data['items'][0]);first['shapes']['0']=[{'shell':[[1,4],[3,4],[3,6],[1,6]],'holes':[]}]
        remote=[]
        for x in (9,10):
            item=copy.deepcopy(first);item['terminals']=[[x,5]]
            item['shapes']['0']=[{'shell':[[x-.1,4.9],[x+.1,4.9],[x+.1,5.1],[x-.1,5.1]],'holes':[]}];remote.append(item)
        data['items']=[first,*remote]
        data['keepouts']=[{'layers':[0,2],'tracks':True,'vias':True,'shapes':[{'shell':[[4,0],[6,0],[6,10],[4,10]],'holes':[]}]}]
        result=route_joint(data,[1],seconds=3)
        self.assertFalse(result['complete_cluster']);self.assertEqual(result['completed_nets'],[])
        self.assertEqual(result['best_partial_components'][1],2)

    def test_complete_three_component_tree_joins_every_original_pad(self):
        data=fixture();data['nets'].pop('2')
        data['routing_profile']['net_limits']=data['routing_profile']['net_limits'][:1]
        data['outline']=[{'shell':[[0,0],[12,0],[12,10],[0,10]],'holes':[]}]
        pad=copy.deepcopy(data['items'][0]);data['items']=[]
        for x,y in ((2,2),(9,2),(9,8)):
            item=copy.deepcopy(pad);item['terminals']=[[x,y]]
            item['shapes']['0']=[{'shell':[[x-.1,y-.1],[x+.1,y-.1],[x+.1,y+.1],[x-.1,y+.1]],'holes':[]}]
            data['items'].append(item)
        result=route_joint(data,[1],seconds=3)
        self.assertTrue(result['complete_cluster']);self.assertEqual(result['completed_nets'],[1])
        self.assertFalse(result['failed']);self.assertFalse(result['vias'])
        copper=[shape(item['shapes']['0'])for item in data['items']]
        copper.extend(LineString([t['start'],t['end']]).buffer(t['width']/2)for t in result['tracks'])
        self.assertEqual(len(shapely.get_parts(shapely.union_all(copper))),1)

    def test_nanometre_rounding_failure_does_not_mutate_candidate_context(self):
        data=fixture();data['items']=[];data.pop('routing_profile')
        data['outline']=[{'shell':[[0,0],[12,0],[12,10],[0,10]],'holes':[]}]
        for spec in data['nets'].values():spec['clearance']=.1500004
        context=routing_geometry(data)
        append_path(context,1,[(0,[1,5.00000049]),(0,[11,5.00000049])])
        self.assertGreater(5.00000049-4.69999999,.3000004)
        with self.assertRaises(ValueError):append_path(context,2,[(0,[1,4.69999999]),(0,[11,4.69999999])])
        self.assertEqual(len(context['items']),1)

    def test_illegal_bore_and_via_limit_are_rejected_before_emission(self):
        data=fixture();data['routing_profile']['net_limits'][0]['max_vias']=2
        context=routing_geometry(data,preserve_source=True)
        with self.assertRaisesRegex(ValueError,'legal through via'):append_path(context,1,[(0,[2,5]),(2,[2,5])])
        self.assertEqual(len(context['items']),4)
        data['items']=[];context=routing_geometry(data)
        data['routing_profile']['net_limits'][0]['max_vias']=0
        with self.assertRaises(ValueError):append_path(context,1,[(0,[2,5]),(2,[2,5])])

    def test_exhaustion_and_invalid_budgets_are_explicit(self):
        result=route_joint(fixture(),[1,2],seconds=2,maximum_states=1)
        self.assertFalse(result['complete_cluster']);self.assertEqual(result['tracks'],[])
        self.assertLessEqual(result['states'],1)
        for seconds in (0,math.inf,math.nan):
            with self.assertRaises(ValueError):route_joint(fixture(),[1,2],seconds=seconds)


if __name__=='__main__':unittest.main()
