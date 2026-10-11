"""A continuation must restore the first source, not its latest partial board."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from native_restoration import NAME,audit,bind_contacts,sha,snapshot,validate


def contacts():
    rows={uid:{'class':'PCB_TRACK' if uid.startswith('t') else 'PAD',
               'net':net,'locked':False} for net in ('A','B') for uid in (net+'1',net+'2','t'+net)}
    nodes={uid:{'uuid':uid,'layer':0,'polygon_nm':[{'shell':[[0,0],[2,0],[2,2],[0,2]],'holes':[]}]} for uid in rows}
    return {'native_items':rows,'contact_nodes':nodes,'contact_node_items':{uid:uid for uid in rows},
            'groups':{net:[{'contact_nodes':[net+'1',net+'2','t'+net]}]for net in ('A','B')},
            'contact_schema':'physical-Cu-islands/1','contact_graph_unambiguous':True,
            'empty_cu_items':[],'cross_net_contacts':[],'unrepresented_net_items':[]}


def baseline(root,native=None):
    source=root/'upload'/'board.kicad_pcb';source.parent.mkdir()
    source.write_text('(kicad_pcb original)');source.with_suffix('.kicad_pro').write_bytes(b'{"rules":"exact"}\n')
    directory=root/NAME;snapshot(source,directory)
    native=contacts() if native is None else native;native['source_sha256']=sha(directory/'candidate.kicad_pcb')
    (directory/'native-connectivity.json').write_text(json.dumps(native));bind_contacts(directory)
    return source,directory,native


class NativeRestorationTests(unittest.TestCase):
    def test_cumulative_cuts_are_checked_against_the_first_upload(self):
        with tempfile.TemporaryDirectory() as td:
            _,directory,native=baseline(Path(td));after=copy.deepcopy(native)
            for uid in ('tA','tB'):
                after['native_items'].pop(uid);after['contact_nodes'].pop(uid);after['contact_node_items'].pop(uid)
            after['groups']={'A':[{'contact_nodes':['A1']},{'contact_nodes':['A2']}],
                             'B':[{'contact_nodes':['B1','B2']}]}
            result=audit(directory,after)
            self.assertFalse(result['pass']);self.assertEqual(result['restored'],1)
            self.assertEqual(result['required'],2)
            self.assertEqual(result['removed_original_copper_uuids'],['tA','tB'])
            after['groups']['A']=[{'contact_nodes':['A1','A2']}]
            self.assertTrue(audit(directory,after)['pass'])

    def test_continuation_copies_the_immutable_baseline_instead_of_partial_copper(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);_,directory,native=baseline(root)
            partial=root/'candidate.kicad_pcb';partial.write_text('(kicad_pcb missing_A)')
            partial.with_suffix('.kicad_pro').write_bytes((directory/'candidate.kicad_pro').read_bytes())
            destination=root/'next'/NAME;snapshot(partial,destination)
            self.assertEqual(validate(destination)['files'],validate(directory)['files'])
            self.assertNotEqual((destination/'candidate.kicad_pcb').read_bytes(),partial.read_bytes())
            partial.with_suffix('.kicad_pro').write_text('{"changed":true}')
            with self.assertRaisesRegex(ValueError,'first supplied rules'):snapshot(partial,root/'invalid'/NAME)
            partial.with_suffix('.kicad_pro').write_bytes((directory/'candidate.kicad_pro').read_bytes())
            partial.with_suffix('.kicad_dru').write_text('(version 1)')
            with self.assertRaisesRegex(ValueError,'first supplied rules'):snapshot(partial,root/'added-rules'/NAME)

    def test_locked_or_protected_cuts_and_changed_survivors_are_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            _,directory,native=baseline(Path(td));after=copy.deepcopy(native)
            after['native_items'].pop('tA');after['contact_nodes'].pop('tA')
            after['groups']['A']=[{'contact_nodes':['A1','A2']}]
            self.assertFalse(audit(directory,after,protected=['A'])['pass'])
            after=copy.deepcopy(native);after['contact_nodes']['A1']['polygon_nm'][0]['shell'][1][0]=3
            self.assertFalse(audit(directory,after)['pass'])

    def test_authorized_width_edit_is_declared_and_keeps_other_source_contacts(self):
        with tempfile.TemporaryDirectory() as td:
            _,directory,native=baseline(Path(td));after=copy.deepcopy(native)
            after['native_items']['tA']['width_nm']=220000
            result=audit(directory,after)
            self.assertTrue(result['pass']);self.assertEqual(result['modified_original_copper_uuids'],['tA'])
            self.assertFalse(audit(directory,after,protected=['A'])['pass'])
            after['groups']['A']=[{'contact_nodes':['A1','tA']},{'contact_nodes':['A2']}]
            self.assertFalse(audit(directory,after)['pass'])

    def test_removed_drill_cannot_expose_unchanged_source_stub_without_declared_deletion(self):
        native=contacts();native['native_items']['tA']['class']='PCB_VIA'
        native['native_items']['voidA']={'class':'PCB_TRACK','net':'A','locked':False}
        native['empty_cu_items']=['voidA']
        native['exact_drill_void_tracks']=[{'track_uuid':'voidA','layer':0,'round_native_via_uuid':'tA'}]
        with tempfile.TemporaryDirectory() as td:
            _,directory,native=baseline(Path(td),native);after=copy.deepcopy(native)
            after['native_items'].pop('tA');after['contact_nodes'].pop('tA');after['contact_node_items'].pop('tA')
            # Removing a drill exposes physical copper even though the nominal
            # track definition is unchanged. Every original pad stays joined.
            after['contact_nodes']['voidA']=dict(copy.deepcopy(native['contact_nodes']['tA']),uuid='voidA')
            after['contact_node_items']['voidA']='voidA'
            after['groups']['A']=[{'contact_nodes':['A1','A2','voidA']}]
            after['empty_cu_items']=[]
            result=audit(directory,after)
            self.assertFalse(result['pass']);self.assertFalse(result['surviving_contact_polygons_exact'])
            self.assertFalse(result['empty_copper_inventory_exact']);self.assertEqual(result['restored'],2)
            after['native_items'].pop('voidA');after['contact_nodes'].pop('voidA');after['contact_node_items'].pop('voidA')
            after['groups']['A']=[{'contact_nodes':['A1','A2']}]
            result=audit(directory,after)
            self.assertTrue(result['pass']);self.assertTrue(result['empty_copper_inventory_exact'])
            self.assertEqual(result['removed_original_copper_uuids'],['tA','voidA'])
            self.assertFalse(audit(directory,after,protected=['A'])['pass'])

    def test_stale_board_contacts_assets_and_escaping_paths_fail(self):
        for change in ('board','contacts','escape'):
            with self.subTest(change=change),tempfile.TemporaryDirectory() as td:
                _,directory,native=baseline(Path(td))
                if change=='board':(directory/'candidate.kicad_pcb').write_text('changed')
                elif change=='contacts':
                    native['source_sha256']='stale';(directory/'native-connectivity.json').write_text(json.dumps(native))
                else:
                    manifest=json.loads((directory/'manifest.json').read_text());manifest['files']['../outside']='x'
                    (directory/'manifest.json').write_text(json.dumps(manifest))
                with self.assertRaises(ValueError):validate(directory)

    def test_native_worker_failure_and_timeout_restore_project_bytes_and_withhold_zip(self):
        from solve_kicad import solve
        for error in (subprocess.CalledProcessError(2,['worker']),subprocess.TimeoutExpired(['worker'],1)):
            with self.subTest(error=type(error).__name__),tempfile.TemporaryDirectory() as td:
                root=Path(td);source=root/'board.kicad_pcb';source.write_text('(kicad_pcb)')
                source.with_suffix('.kicad_pro').write_bytes(b'{"exact":true}\n');out=root/'out'
                def fail(command,**kw):
                    (out/NAME/'candidate.kicad_pro').write_bytes(b'{"migrated":true}')
                    raise error
                with patch('solve_kicad.native_worker_command',return_value=['python','worker']),patch('solve_kicad.subprocess.run',side_effect=fail):
                    result=solve(source,out,kicad='fake',mode='repair')
                self.assertEqual(result['state'],'failed');self.assertFalse(result['review_pass'])
                self.assertEqual((out/NAME/'candidate.kicad_pro').read_bytes(),source.with_suffix('.kicad_pro').read_bytes())
                self.assertFalse((out/'prototype.zip').exists())

    def test_legacy_continuation_chain_recovers_first_source_and_rejects_cycles(self):
        import layout_server as server
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);jobs=root/'build/layout-jobs';ids=['a'*32,'b'*32]
            for index,job in enumerate(ids):
                d=jobs/job;source=d/'source'/'board.kicad_pcb';source.parent.mkdir(parents=True)
                source.write_text('(kicad_pcb '+str(index)+')');source.with_suffix('.kicad_pro').write_text('{}')
                (d/'native').mkdir();(d/'native/candidate.kicad_pcb').write_text('(kicad_pcb partial)')
                (d/'native/source-manifest.json').write_text(json.dumps({f.name:sha(f)for f in source.parent.iterdir()}))
                status={'native':True,'state':'done','source':str(source)}
                if index:status['source_job']=ids[0]
                (d/'status.json').write_text(json.dumps(status))
            with patch.object(server,'ROOT',root):
                _,first=server.original_restoration_job(ids[1]);self.assertEqual(first,(jobs/ids[0]/'source/board.kicad_pcb').resolve())
                path=jobs/ids[0]/'status.json';status=json.loads(path.read_text());status['source_job']=ids[1];path.write_text(json.dumps(status))
                with self.assertRaisesRegex(ValueError,'cyclic'):server.original_restoration_job(ids[1])

    def test_legacy_changed_or_missing_first_source_hash_requires_original_upload(self):
        import layout_server as server
        for stale in (False,True):
            with self.subTest(stale=stale),tempfile.TemporaryDirectory()as td:
                root=Path(td);job='a'*32;d=root/'build/layout-jobs'/job
                source=d/'source/board.kicad_pcb';source.parent.mkdir(parents=True)
                source.write_text('(kicad_pcb original)');source.with_suffix('.kicad_pro').write_text('{}')
                native=d/'native';native.mkdir();(native/'candidate.kicad_pcb').write_text('(kicad_pcb partial)')
                (d/'status.json').write_text(json.dumps({'native':True,'state':'done','source':str(source)}))
                if stale:
                    (native/'source-manifest.json').write_text(json.dumps({f.name:sha(f)for f in source.parent.iterdir()}))
                    source.write_text('(kicad_pcb changed)')
                with patch.object(server,'ROOT',root),self.assertRaisesRegex(ValueError,'upload the original project'):
                    server.original_restoration_job(job)


if __name__=='__main__':unittest.main()
