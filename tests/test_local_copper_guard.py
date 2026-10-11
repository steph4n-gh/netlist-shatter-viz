"""Local edits cannot trade native copper outside the authorized window."""
import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from repair_native_congestion import local_copper_preserved,local_removals


def track(start=(-5,0),end=(5,0),**fields):
    return dict(net=7,start=list(start),end=list(end),width=.3,layer=0,uuid='source',locked=False,arc=False)|fields


def geometry(tracks=(),vias=()):
    return {'tracks':list(tracks),'vias':list(vias),'layers':[0,2]}


def fragments(original,ends):
    """Known-good native fixtures; no clipping algorithm is implemented here."""
    return geometry([dict(original,start=list(a),end=list(b),uuid=f'fragment-{i}') for i,(a,b) in enumerate(ends)])


class LocalCopperGuardTests(unittest.TestCase):
    def test_exact_collinear_fragments_preserve_outside_native_anchors(self):
        source=track();before=geometry([source])
        after=fragments(source,[((-5,0),(-2.8,0)),((2.8,0),(5,0))])
        self.assertTrue(local_copper_preserved(before,after,[7],[[0,0]])['pass'])
        diagonal=track((-5,-5),(5,5));before=geometry([diagonal])
        after=fragments(diagonal,[((-5,-5),(-2,-2)),((2,2),(5,5))])
        self.assertTrue(local_copper_preserved(before,after,[7],[[0,0]])['pass'])

    def test_actual_raw_proximity_cut_fails_physical_outside_gate(self):
        before=geometry([track()]);proposal=local_removals(before,7,[[0,0]])
        after=geometry([dict(t,uuid=str(i),locked=False,arc=False) for i,t in enumerate(proposal['tracks'])])
        self.assertTrue(proposal['remove'])
        self.assertFalse(local_copper_preserved(before,after,[7],[[0,0]])['pass'])

    def test_one_native_nm_drift_and_layer_swap_rejected(self):
        source=track();before=geometry([source]);after=fragments(source,[((-5,0),(-2.8,0)),((2.8,0),(5,0))])
        drift=copy.deepcopy(after);drift['tracks'][0]['end'][1]=.000001
        self.assertFalse(local_copper_preserved(before,drift,[7],[[0,0]])['pass'])
        swapped=copy.deepcopy(after);swapped['tracks'][0]['layer']=2
        self.assertFalse(local_copper_preserved(before,swapped,[7],[[0,0]])['pass'])

    def test_locked_and_arc_source_primitives_cannot_change_even_inside(self):
        for flags in ({'locked':True},{'arc':True}):
            before=geometry([track((0,0),(1,0),**flags)]);after=copy.deepcopy(before)
            after['tracks'][0]['end']=[1.001,0]
            self.assertFalse(local_copper_preserved(before,after,[7],[[0,0]])['pass'])

    def test_partially_outside_via_is_preserved_exactly(self):
        via={'net':7,'at':[2.9,0],'uuid':'via','diameter_mm':.6,'locked':False,'through':True}
        before=geometry(vias=[via])
        self.assertEqual(local_removals(before,7,[[0,0]],primitive_uuids=['via'],retain_crossing=True)['remove'],[])
        after=copy.deepcopy(before);after['vias'][0]['at']=[2.899,0]
        self.assertFalse(local_copper_preserved(before,after,[7],[[0,0]])['pass'])
        self.assertFalse(local_copper_preserved(before,geometry(),[7],[[0,0]])['pass'])

    def test_internal_new_copper_allowed_but_outside_extension_rejected(self):
        before=geometry([track()]);after=copy.deepcopy(before)
        after['tracks'].append(track((0,0),(0,1),uuid='new'))
        self.assertTrue(local_copper_preserved(before,after,[7],[[0,0]])['pass'])
        after['tracks'][-1]['end']=[0,4]
        self.assertFalse(local_copper_preserved(before,after,[7],[[0,0]])['pass'])

    def test_measured_cut_keeps_unselected_interior_uuid_exact(self):
        before=geometry([track((0,0),(1,0))]);after=copy.deepcopy(before)
        after['tracks'][0]['end']=[1.001,0]
        self.assertTrue(local_copper_preserved(before,after,[7],[[0,0]])['pass'])
        self.assertFalse(local_copper_preserved(before,after,[7],[[0,0]],removed=set())['pass'])

    def test_whole_cut_rejects_duplicate_outside_addition(self):
        before=geometry([track((5,0),(6,0))]);after=copy.deepcopy(before)
        after['tracks'].append(dict(before['tracks'][0],uuid='duplicate'))
        self.assertTrue(local_copper_preserved(before,after,[7],[[0,0]])['pass'])
        self.assertFalse(local_copper_preserved(before,after,[7],[[0,0]],removed=set())['pass'])

    def test_whole_inside_selected_removal_keeps_unselected_uuid(self):
        before=geometry([track((0,0),(1,0),uuid='selected'),track((0,1),(1,1),uuid='unselected')])
        after=geometry([before['tracks'][1]])
        self.assertTrue(local_copper_preserved(before,after,[7],[[0,0]],removed={'selected'})['pass'])
        self.assertFalse(local_copper_preserved(before,geometry(),[7],[[0,0]],removed={'selected'})['pass'])

    def test_incomplete_via_stack_metadata_fails_closed(self):
        before={'tracks':[],'vias':[{'net':7,'at':[5,0],'uuid':'v','diameter_mm':.6,'locked':False}]}
        self.assertFalse(local_copper_preserved(before,before,[7],[[0,0]])['pass'])


if __name__=='__main__':unittest.main()
