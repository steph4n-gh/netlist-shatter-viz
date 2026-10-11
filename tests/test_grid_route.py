"""Geometric invariants of the bounded candidate router, independent of KiCad."""
import copy
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from grid_route import route, shape, routing_geometry, append_path
from shapely.geometry import LineString, Point


def rectangle(x0,y0,x1,y1):
    return [{'shell':[[x0,y0],[x1,y0],[x1,y1],[x0,y1]],'holes':[]}]


def fixture(back=False):
    def pad(x,layer):
        return {'net':1,'kind':'pad','terminals':[[x,5]],'clearance':.2,'shapes':{str(layer):rectangle(x-.5,4.5,x+.5,5.5)}}
    return {'layers':[0,2], 'outline':rectangle(0,0,12,10), 'items':[pad(2,0),pad(10,2 if back else 0)],
            'holes':[], 'keepouts':[], 'edge_clearance':.4,'hole_clearance':.25,
            'nets':{'1':{'width':.2,'clearance':.2,'via_diameter':.6,'via_drill':.3,'name':'signal'}}}


class GridRouteTests(unittest.TestCase):
    def test_domain_reuse_includes_candidate_copper_holes_and_rollback(self):
        data=fixture();data['nets']['2']=dict(data['nets']['1'],name='other')
        context=routing_geometry(data,preserve_source=True)
        holes=context['state']['holes'];count=len(context['items'])
        trace_point=Point(6,5);via_point=Point(6,7)
        self.assertTrue(context['domains'](2,.1,padding=0)[0].covers(trace_point))
        self.assertTrue(context['domains'](1,.3,via=True,padding=0)[0].covers(via_point))
        append_path(context,1,[(0,[4,5]),(0,[8,5])])
        append_path(context,1,[(0,[6,7]),(2,[6,7])])
        self.assertFalse(context['domains'](2,.1,padding=0)[0].covers(trace_point))
        self.assertFalse(context['domains'](1,.3,via=True,padding=0)[0].covers(via_point))
        del context['items'][count:];context['state']['holes']=holes
        self.assertTrue(context['domains'](2,.1,padding=0)[0].covers(trace_point))
        self.assertTrue(context['domains'](1,.3,via=True,padding=0)[0].covers(via_point))

    def test_large_local_search_can_coarsen_without_ignoring_its_budget(self):
        result=route(fixture(),step=.025,only_nets=[1],max_states=100000,seconds=2)
        self.assertFalse(result['failed']);self.assertEqual(result['completed_nets'],[1])
        self.assertEqual(result['net_grid_mm']['1'],.05)

    def test_explicit_layers_width_and_via_budget_guide_search(self):
        data=fixture();data['layer_names']={'0':'F.Cu','2':'B.Cu'}
        data['routing_profile']={'net_limits':[{'net':'signal','min_width_mm':.3,'max_vias':0,'allowed_layers':['F.Cu']}]}
        original=copy.deepcopy(data);r=route(data,step=.2,seconds=2)
        self.assertFalse(r['failed']);self.assertFalse(r['vias'])
        self.assertTrue(all(t['layer']==0 and t['width']==.3 for t in r['tracks']))
        self.assertEqual(data,original)
        data['keepouts']=[{'layers':[0],'tracks':True,'vias':True,'shapes':rectangle(5,0,7,10)}]
        data['routing_profile']['net_limits'][0]['allowed_layers']=['F.Cu','B.Cu']
        data['routing_profile']['net_limits'][0]['max_vias']=1
        self.assertTrue(route(data,step=.2,seconds=2)['failed'])
        data['routing_profile']['net_limits'][0]['max_vias']=2
        r=route(data,step=.2,seconds=2)
        self.assertFalse(r['failed']);self.assertEqual(len(r['vias']),2)

    def test_pad_can_connect_at_exposed_copper_when_centre_is_blocked(self):
        data=fixture()
        data['items'][0]['shapes']['0']=rectangle(1,4.5,4,5.5)
        data['keepouts']=[{'layers':[0,2],'tracks':True,'vias':True,'shapes':rectangle(0,0,2.5,10)}]
        r=route(data,seconds=2)
        self.assertFalse(r['failed'])
        self.assertTrue(any(LineString([t['start'],t['end']]).intersects(shape(data['items'][0]['shapes']['0'])) for t in r['tracks']))
        for t in r['tracks']:
            self.assertGreaterEqual(LineString([t['start'],t['end']]).distance(shape(data['keepouts'][0]['shapes'])),t['width']/2-1e-6)

    def test_opposite_side_pads_require_a_real_via(self):
        data=fixture(True);original=copy.deepcopy(data)
        r=route(data)
        self.assertFalse(r['failed']);self.assertEqual(r['completed_nets'],[1]);self.assertEqual(len(r['vias']),1)
        self.assertEqual(data,original)
        self.assertEqual({t['layer'] for t in r['tracks']},{0,2})
        v=Point(r['vias'][0]['at'])
        for layer in data['layers']:
            self.assertTrue(any(LineString([t['start'],t['end']]).distance(v)<1e-6 for t in r['tracks'] if t['layer']==layer))

    def test_cutout_and_hole_clearance_are_respected(self):
        data=fixture();hole=rectangle(5,3,7,7)
        data['outline'][0]['holes']=[hole[0]['shell']]
        data['drill_obstacles']=[{'net':0,'shapes':rectangle(3,4,4,6)}]
        r=route(data)
        self.assertFalse(r['failed'])
        for t in r['tracks']:
            line=LineString([t['start'],t['end']])
            self.assertGreaterEqual(line.distance(shape(hole)),.4+t['width']/2-1e-6)
            self.assertGreaterEqual(line.distance(shape(data['drill_obstacles'][0]['shapes'])),.25+t['width']/2-1e-6)

    def test_all_layer_barrier_reports_failure_without_crossing_it(self):
        data=fixture();data['keepouts']=[{'layers':[0,2],'tracks':True,'vias':True,'shapes':rectangle(5,0,7,10)}]
        r=route(data,seconds=2)
        self.assertTrue(r['failed']);self.assertEqual(r['tracks'],[]);self.assertEqual(r['completed_nets'],[])
        self.assertTrue(r['candidate_only'])

    def test_invalid_or_oversized_search_fails_explicitly(self):
        for step in [0,-1,math.inf,math.nan]:
            with self.assertRaises(ValueError):route(fixture(),step=step)
        with self.assertRaises(ValueError):route(fixture(),max_states=1)

    def test_local_net_repair_does_not_rasterize_unrelated_board_area(self):
        data=fixture();data['outline']=rectangle(0,0,1000,1000)
        with self.assertRaises(ValueError):route(data,max_states=100000)
        result=route(data,only_nets=[1],max_states=100000)
        self.assertFalse(result['failed']);self.assertEqual(result['completed_nets'],[1])
        self.assertLess(result['search_windows']['1'][2],20)


if __name__=='__main__':unittest.main()
