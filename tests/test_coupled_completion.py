"""Joint completion cannot trade native findings or miss shadowed owner copper."""
import copy
import json
import tempfile
from unittest.mock import patch
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from repair_native_congestion import measured_owner_cluster,no_new_native_findings,native_open_counts
from test_grid_route import fixture,rectangle


def enclosed():
    g=fixture();g.update(tracks=[],vias=[],pads=[{'net':1,'ref':'a','pin':'1'},{'net':1,'ref':'b','pin':'1'}],layer_names={'0':'F.Cu','2':'B.Cu'})
    g['nets']['2']=dict(g['nets']['1'],name='barrier')
    for l in [0,2]:
        ident='track-'+str(l);g['tracks'].append({'uuid':ident,'net':2,'layer':l,'width':1.,'start':[6,0],'end':[6,10],'arc':False,'locked':False})
        g['items'].append({'uuid':ident,'net':2,'kind':'copper','terminals':[[6,0],[6,10]],'clearance':.2,'shapes':{str(l):rectangle(5.5,-.5,6.5,10.5)}})
    v={'uuid':'shadowed-via','net':2,'at':[6,5],'diameter_mm':.6,'drill_mm':.3,'through':True,'locked':False};g['vias'].append(v)
    g['items'].append({'uuid':v['uuid'],'net':2,'kind':'copper','terminals':[[6,5],[6,5]],'clearance':.2,'shapes':{str(l):[{'circle':[6,5,.3]}] for l in [0,2]}});g['holes']=[{'circle':[6,5,.15]}]
    return g


class CoupledCompletionTests(unittest.TestCase):
    def setUp(self):
        self.drc={'unconnected_items':[{'items':[{'description':'Pad1 [signal] of a'},{'description':'Pad1 [signal] of b'}]}]}
        self.profile={'net_limits':[{'net':'signal','min_width_mm':.2,'allowed_layers':['F.Cu','B.Cu']}]}

    def test_owner_closure_includes_via_shadowed_by_wider_tracks(self):
        g=enclosed();old=copy.deepcopy(g);r=measured_owner_cluster(g,self.drc,self.profile)
        self.assertIsNotNone(r);self.assertEqual(r['selected_nets'],['signal','barrier']);self.assertTrue(r['graph_reachable'])
        # The separating tracks are the observed frontier. A same-owner via
        # hidden inside them is omitted together with the owner, not retained.
        visible={p['uuid'] for row in r['rounds'][0]['boundary']['rows'] for p in row['native_primitives']}
        self.assertNotIn('shadowed-via',visible);self.assertEqual(g,old)

    def test_displaced_owner_demand_expands_closure_after_target_opens(self):
        g=enclosed();g['nets']['3']=dict(g['nets']['1'],name='enclosure')
        for x in [2,10]:
            g['pads'].append({'net':2,'ref':'n'+str(x),'pin':'1'})
            g['items'].append({'net':2,'kind':'pad','terminals':[[x,2]],'clearance':.2,'shapes':{'0':rectangle(x-.3,1.7,x+.3,2.3)}})
        edges=[([1,1],[3,1]),([3,1],[3,3]),([3,3],[1,3]),([1,3],[1,1])]
        for layer in [0,2]:
            for j,(a,b) in enumerate(edges):
                ident=f'ring-{layer}-{j}';poly=rectangle(min(a[0],b[0])-.1,min(a[1],b[1])-.1,max(a[0],b[0])+.1,max(a[1],b[1])+.1)
                g['tracks'].append({'uuid':ident,'net':3,'layer':layer,'width':.2,'start':a,'end':b,'arc':False,'locked':False})
                g['items'].append({'uuid':ident,'net':3,'kind':'copper','terminals':[a,b],'clearance':.2,'shapes':{str(layer):poly}})
        r=measured_owner_cluster(g,self.drc,self.profile)
        self.assertEqual(r['selected_nets'],['signal','barrier','enclosure'])
        self.assertEqual({row['net'] for row in r['demand_graphs']},{'signal','barrier','enclosure'})

    def test_protected_or_partially_locked_owner_cannot_be_selected(self):
        g=enclosed();self.assertIsNone(measured_owner_cluster(g,self.drc,self.profile,protected=['barrier']))
        g['vias'][0]['locked']=True;self.assertIsNone(measured_owner_cluster(g,self.drc,self.profile))

    def test_fixed_pad_enclosure_is_not_ripped(self):
        g=enclosed()
        for i in g['items'][2:]:i['kind']='pad'
        self.assertIsNone(measured_owner_cluster(g,self.drc,self.profile))

    def test_expired_diagnostic_budget_retains_no_false_selection(self):
        self.assertIsNone(measured_owner_cluster(enclosed(),self.drc,self.profile,seconds=1e-12))

    def test_equal_warning_counts_cannot_hide_a_new_identity(self):
        a={'violations':[{'type':'silk_over_copper','severity':'warning','description':'overlap','items':[{'uuid':'old'}]}]}
        b=copy.deepcopy(a);b['violations'][0]['items'][0]['uuid']='new'
        self.assertFalse(no_new_native_findings(a,b));self.assertTrue(no_new_native_findings(a,a));self.assertTrue(no_new_native_findings(a,{'violations':[]}))

    def test_native_open_count_is_by_connection_not_two_items(self):
        self.assertEqual(native_open_counts(enclosed(),self.drc),{1:1,2:0})

    def test_one_failed_append_does_not_abort_other_eligible_nets(self):
        import repair_native_congestion as repair
        g=enclosed();g['pads'].extend([{'net':2,'ref':'n1','pin':'1'},{'net':2,'ref':'n2','pin':'1'}])
        drc={'violations':[],'schematic_parity':[],'unconnected_items':self.drc['unconnected_items']+[{'items':[{'description':'Pad1 [barrier] of n1'}]}]}
        seen=[]
        def native(args,**kwargs):
            if 'dc-geometry' in args:
                d=Path(args[-1]);(d/'dc-geometry.json').write_text(json.dumps(g))
            elif 'native-connectivity' in args:
                (Path(args[-1])/'native-connectivity.json').write_text(json.dumps({'native_items':{}}))
            elif 'drc' in args:Path(args[args.index('-o')+1]).write_text(json.dumps(drc))
        def fail_route(data,**kwargs):
            seen.extend(kwargs['only_nets']);raise ValueError('Endpoint outside exact legal component')
        with tempfile.TemporaryDirectory() as temp:
            d=Path(temp);(d/'candidate.kicad_pcb').write_text('(kicad_pcb)');(d/'candidate.kicad_pro').write_text('{}')
            with patch.object(repair.subprocess,'run',side_effect=native),patch.object(repair,'copy_assets'),patch.object(repair,'screen',return_value={'pass':True}),patch.object(repair,'nonregression',return_value=True),patch.object(repair,'measured_component_cuts',return_value={'selected':[]}),patch.object(repair,'measured_owner_cluster',return_value=None),patch.object(repair,'route',side_effect=fail_route),patch.object(repair,'plane_partitions_preserved',return_value={'pass':True}),patch.object(repair,'native_partition_preservation',return_value={'pass':True}),patch('solve_kicad.audit_preservation',return_value={'pass':True}),patch('optimize_kicad.unchanged_other_nets',return_value=True):
                r=repair.coupled_completion(d,['worker'],'kicad',self.profile,seconds=30)
            self.assertEqual(seen,[1,2,1,2]);self.assertEqual(len(r['append_trials']),4)
            self.assertTrue(all(t['error']=='Endpoint outside exact legal component' and not t['accepted'] for t in r['append_trials']))
            self.assertNotIn('error',r);self.assertFalse(r['accepted']);self.assertFalse(r['complete'])
            self.assertEqual((d/'candidate.kicad_pcb').read_text(),'(kicad_pcb)')

if __name__=='__main__':unittest.main()
