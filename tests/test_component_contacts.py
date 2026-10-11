"""Native lock authority and connected-component contact selection."""
import ast
import math
from pathlib import Path
import sys
import unittest

import shapely
from shapely.geometry import LineString,Point,Polygon,box

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from polygon_path import find_path


def endpoint_function(data):
    path=ROOT/'scripts/grid_route.py'
    tree=ast.parse(path.read_text())
    node=next(n for n in ast.walk(tree)if isinstance(n,ast.FunctionDef)and n.name=='endpoints')
    scope={'Point':Point,'data':data,'layers':[0]}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(path),'exec'),scope)
    return scope['endpoints']


def connected_group(item_locked=None):
    copper={'uuid':'native-track','net':1,'kind':'copper','terminals':[[0,0],[2,0]],
            'geometry':{0:LineString([(0,0),(2,0)]).buffer(.1)}}
    if item_locked is not None:copper['locked']=item_locked
    pad={'net':1,'kind':'pad','terminals':[[1,0]],'geometry':{0:Point(1,0).buffer(.25)}}
    return [copper,pad]


class NativeLockContactTests(unittest.TestCase):
    def test_unlocked_native_tip_does_not_mask_other_component_contacts(self):
        points=endpoint_function({'tracks':[{'uuid':'native-track','locked':False}] })(connected_group())
        self.assertIn(((1,0),0),points)
        self.assertIn(((2,0),0),points)
        self.assertIn(((0,0),0),points)

    def test_true_locked_tip_uses_uuid_inventory_when_item_omits_lock(self):
        # Both ends are dangling; the middle pad is an existing connection.
        points=endpoint_function({'tracks':[{'uuid':'native-track','locked':True}]})(connected_group())
        self.assertEqual(set(points),{((0,0),0),((2,0),0)})
        self.assertNotIn(((1,0),0),points)

    def test_native_uuid_lock_state_overrides_stale_item_flag(self):
        points=endpoint_function({'tracks':[{'uuid':'native-track','locked':False}]})(connected_group(True))
        self.assertIn(((1,0),0),points)

    def test_explicit_generated_locked_tip_still_has_preference(self):
        points=endpoint_function({})(connected_group(True))
        self.assertEqual(set(points),{((0,0),0),((2,0),0)})


class PolygonComponentContactTests(unittest.TestCase):
    def test_second_contact_avoids_lexical_first_detour(self):
        # One connected legal component wraps around the barrier. The lexical
        # first contact needs a detour; the other contact has a direct route.
        domain=box(0,0,10,4).difference(box(3,0,7,3))
        def domains(radius,via=False,padding=0):return [Polygon()if via else domain.buffer(-padding)]
        def endpoints(group,legal):return group
        source=[((1,1),0),((8,1),0)]
        target=[((9,1),0)]
        calls=0
        def bounded_check():
            nonlocal calls
            calls+=1
            if calls>25:raise TimeoutError('Detouring contact exhausted the synthetic search budget')
        path,detail=find_path(domains,endpoints,source,target,[0],.2,.3,0,.025,bounded_check)
        self.assertEqual(path[0],(0,[8,1]))
        self.assertEqual(path[-1],(0,[9,1]))
        self.assertEqual(detail['minimum_new_vias'],0)
        self.assertEqual(detail['triangle_counts'],[0])
        self.assertLessEqual(sum(math.dist(a[1],b[1])for a,b in zip(path,path[1:])),1.01)
        self.assertTrue(all(domain.covers(LineString([a[1],b[1]]))for a,b in zip(path,path[1:])))

    def test_attachment_distance_does_not_replace_minimum_via_chain(self):
        domain=box(0,0,10,4)
        def domains(radius,via=False,padding=0):return [domain.buffer(-padding),domain.buffer(-padding)]
        def endpoints(group,legal):return group
        source=[((1,1),0),((8.9,1),2)]
        target=[((9,1),0)]
        path,detail=find_path(domains,endpoints,source,target,[0,2],.2,.3,1,.025,lambda:None)
        self.assertEqual(detail['minimum_new_vias'],0)
        self.assertEqual(path[0],(0,[1,1]))
        self.assertTrue(all(layer==0 for layer,point in path))


if __name__=='__main__':unittest.main()
