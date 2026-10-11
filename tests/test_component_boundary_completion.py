"""Retained copper and complete native contact sets govern local cut repair."""
import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from repair_native_congestion import measured_component_cuts,native_partition_preservation
from test_coupled_completion import enclosed
from test_grid_route import rectangle


class ComponentBoundaryCompletionTests(unittest.TestCase):
    def setUp(self):
        self.drc={'unconnected_items':[{'items':[{'description':'Pad1 [signal] of a'}]}]}
        self.profile={'net_limits':[{'net':'signal','min_width_mm':.2}]}

    def test_two_separators_need_two_precise_cuts_not_whole_owner(self):
        g=enclosed();g['layers']=[0]
        g['tracks']=[p for p in g['tracks'] if p['layer']==0]
        g['items']=[p for p in g['items'] if p.get('uuid')!='track-2']
        g['tracks'].append(dict(g['tracks'][0],uuid='second-track',start=[8,0],end=[8,10]))
        g['items'].append({'uuid':'second-track','net':2,'kind':'copper','terminals':[[8,0],[8,10]],
            'clearance':.2,'shapes':{'0':rectangle(7.5,-.5,8.5,10.5)}})
        original=copy.deepcopy(g)
        r=measured_component_cuts(g,self.drc,self.profile)
        self.assertTrue(r['selected']);self.assertEqual(g,original)
        for row in r['selected']:
            self.assertEqual(set(row['removed_native_uuids']),{'track-0','second-track'})
            self.assertEqual(row['owner_nets'],[2])
        # The same owner's hidden via is retained, unlike whole-owner rip-up.
        self.assertNotIn('shadowed-via',r['selected'][0]['removed_native_uuids'])

    def test_fixed_locked_and_protected_barriers_cannot_be_cut(self):
        for mode in ('protected','locked','arc','fixed'):
            g=enclosed();protected=[]
            if mode=='protected':protected=['barrier']
            elif mode=='locked':g['tracks'][0]['locked']=True
            elif mode=='arc':g['tracks'][0]['arc']=True
            else:
                for item in g['items'][2:]:item['kind']='pad'
            self.assertFalse(measured_component_cuts(g,self.drc,self.profile,protected)['selected'],mode)

    def test_budget_expiry_is_a_retained_miss(self):
        r=measured_component_cuts(enclosed(),self.drc,self.profile,seconds=0)
        self.assertEqual(r['states_examined'],0);self.assertFalse(r['selected'])
        self.assertFalse(r['physical_infeasibility_claim'])

    def test_retained_via_consumes_supplied_budget_before_graph_selection(self):
        g=enclosed();via=dict(g['vias'][0],uuid='retained-signal-via',net=1,at=[2,5])
        g['vias'].append(via);g['holes'].append({'circle':[2,5,.15]})
        g['items'].append({'uuid':via['uuid'],'net':1,'kind':'copper','terminals':[[2,5],[2,5]],
            'clearance':.2,'shapes':{str(l):[{'circle':[2,5,.3]}] for l in [0,2]}})
        profile={'net_limits':[{'net':'signal','min_width_mm':.2,'max_vias':1}]}
        r=measured_component_cuts(g,self.drc,profile)
        self.assertTrue(r['selected'])
        # Removing only the back-layer separator would still need a second via.
        self.assertTrue(all('track-0' in row['removed_native_uuids'] for row in r['selected']))
        self.assertTrue(all(row['remaining_new_via_budget']==0 for row in r['rows']))

    def test_matching_component_counts_cannot_hide_split_survivors(self):
        items={u:{'class':'PAD' if u.startswith('p') else 'PCB_TRACK','net':'signal','locked':False}
               for u in ('p1','p2','tip','cut','island')}
        before={'native_items':items,'groups':{'signal':[{'native_item_uuids':['p1','p2','tip','cut']},
            {'native_item_uuids':['island']}]},'cross_net_contacts':[],'unrepresented_net_items':[]}
        after=copy.deepcopy(before);after['native_items'].pop('cut')
        after['groups']['signal']=[{'native_item_uuids':['p1','p2']},{'native_item_uuids':['tip','island']}]
        self.assertFalse(native_partition_preservation(before,after,['cut'],{'signal'})['pass'])
        after['groups']['signal']=[{'native_item_uuids':['p1','p2','tip','island']}]
        self.assertTrue(native_partition_preservation(before,after,['cut'],{'signal'})['pass'])
        for tamper in ('changed','foreign','locked','stale','retained_cut','cross_net','missing_item'):
            a=copy.deepcopy(after);b=copy.deepcopy(before);cut=['cut']
            if tamper=='changed':a['native_items']['tip']['width_nm']=199999
            elif tamper=='foreign':a['native_items']['new']={'class':'PCB_TRACK','net':'other'}
            elif tamper=='locked':b['native_items']['cut']['locked']=True
            elif tamper=='stale':cut=['unknown']
            elif tamper=='retained_cut':a['native_items']['cut']=b['native_items']['cut']
            elif tamper=='cross_net':a['cross_net_contacts']=[['p1','foreign']]
            else:a['unrepresented_net_items']=['tip']
            self.assertFalse(native_partition_preservation(b,a,cut,{'signal'})['pass'],tamper)


if __name__=='__main__':unittest.main()
