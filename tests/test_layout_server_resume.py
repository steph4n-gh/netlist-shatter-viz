"""Retained native job boundaries and the existing continuation endpoint."""
import http.client
import hashlib
from http.server import HTTPServer
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
module = ROOT / 'scripts/layout_server.py'
spec = importlib.util.spec_from_file_location('layout_server_resume_tested', module)
server = importlib.util.module_from_spec(spec);spec.loader.exec_module(server)


class Slot:
    def __init__(self): self.released = 0
    def acquire(self, **kwargs): return True
    def release(self): self.released += 1


class RetainedNativeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.patch = patch.object(server, 'ROOT', self.root);self.patch.start()
        self.slot = Slot();self.slot_patch = patch.object(server, 'ROUTING', self.slot);self.slot_patch.start()
        self.job = 'a' * 32;self.directory = self.root / 'build/layout-jobs' / self.job
        self.original = self.directory / 'source/design.kicad_pcb'
        self.original.parent.mkdir(parents=True)
        self.original.write_text('(kicad_pcb original_source_copper)')
        self.original.with_suffix('.kicad_pro').write_bytes(b'{"original_rules":true}')
        self.original.with_suffix('.kicad_dru').write_bytes(b'(version 1)')
        self.native = self.directory / 'native';self.native.mkdir()
        (self.native / 'candidate.kicad_pcb').write_bytes(b'(kicad_pcb retained_valid_copper)')
        (self.native / 'candidate.kicad_pro').write_bytes(b'{"migration":true}')
        (self.native / 'candidate.dc.json').write_bytes(b'{"retained_dc":true}')
        (self.native / 'candidate.routing.json').write_bytes(b'{"retained_fixed_profile":true}')
        (self.native / 'fp-lib-table').write_text('(fp_lib_table (lib (name "Local") (type "KiCad") (uri "${KIPRJMOD}/Local.pretty")))')
        (self.native / 'Local.pretty').mkdir()
        (self.native / 'Local.pretty/part.kicad_mod').write_text('(footprint local_footprint)')
        (self.native/'source-manifest.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest()for f in self.original.parent.iterdir()}))
        self.status = {'native': True, 'state': 'done', 'review_pass': False, 'source': str(self.original),
            'dc_screen':{'profile_sha256':hashlib.sha256((self.native/'candidate.dc.json').read_bytes()).hexdigest()},
            'reference_planes':{'profile_sha256':hashlib.sha256((self.native/'candidate.routing.json').read_bytes()).hexdigest()}}
        self.write_status()

    def write_status(self): (self.directory / 'status.json').write_text(json.dumps(self.status))

    def tearDown(self):
        self.slot_patch.stop();self.patch.stop();self.temp.cleanup()

    def test_source_job_rejects_invalid_running_non_native_or_escaped_candidates(self):
        for value in [None, 7, '../outside', 'A' * 32, 'f' * 31]:
            with self.assertRaises(ValueError): server.native_source_job(value)
        for field, value in [('state', 'running'), ('native', False)]:
            changed = dict(self.status, **{field: value});(self.directory / 'status.json').write_text(json.dumps(changed))
            with self.assertRaises(ValueError): server.native_source_job(self.job)
        self.write_status()
        (self.native / 'candidate.kicad_pcb').unlink()
        outside = self.root / 'outside.kicad_pcb';outside.write_text('outside')
        (self.native / 'candidate.kicad_pcb').symlink_to(outside)
        with self.assertRaises(ValueError): server.native_source_job(self.job)

    def test_failed_terminal_job_requires_retained_original_rules_and_rejects_escaped_source(self):
        self.status = {'native': True, 'state': 'failed'};self.write_status()
        (self.native / 'status.json').write_text(json.dumps({'source': str(self.original)}))
        self.assertEqual(server.native_source_job(self.job), (self.native.resolve(), self.original.resolve()))
        (self.native / 'status.json').write_text(json.dumps({'source': str(self.root / 'external.kicad_pcb')}))
        with self.assertRaises(ValueError): server.native_source_job(self.job)
        (self.native / 'status.json').write_text(json.dumps({'source': str(self.original)}))
        self.original.with_suffix('.kicad_pro').unlink()
        with self.assertRaises(ValueError): server.native_source_job(self.job)

    def test_continuation_copies_retained_copper_original_rules_profiles_and_libraries(self):
        def solve(board, out, **kwargs):
            self.assertEqual(board.read_bytes(), b'(kicad_pcb retained_valid_copper)')
            self.assertEqual(board.with_suffix('.kicad_pro').read_bytes(), self.original.with_suffix('.kicad_pro').read_bytes())
            self.assertEqual(board.with_suffix('.kicad_dru').read_bytes(), b'(version 1)')
            self.assertEqual((board.parent / 'Local.pretty/part.kicad_mod').read_text(), '(footprint local_footprint)')
            self.assertEqual(kwargs['dc_profile'].read_bytes(), b'{"retained_dc":true}')
            self.assertEqual(kwargs['routing_profile'].read_bytes(), b'{"retained_fixed_profile":true}')
            self.assertEqual(kwargs['mode'], 'repair');self.assertEqual(kwargs['protected'], ['USB_D+'])
            self.assertIsNone(kwargs['allowed'])
            return {'state': 'done', 'review_pass': False}
        with patch.object(server, 'solve_native', side_effect=solve) as invoked:
            server.run_native('b' * 32, [], 'candidate.kicad_pcb', 'kicad',
                              mode='repair', protected=['USB_D+'], source_job=self.job)
        self.assertEqual(invoked.call_count, 1)
        result = json.loads((self.root / 'build/layout-jobs' / ('b' * 32) / 'status.json').read_text())
        self.assertEqual(result['source_job'], self.job);self.assertIsNone(result['archive_url'])
        self.assertEqual(self.slot.released, 1)
        self.assertEqual((self.native / 'candidate.kicad_pro').read_bytes(), b'{"migration":true}')

    def test_improve_reuses_assets_and_a_failed_copy_never_exposes_an_archive(self):
        with patch.object(server, 'solve_native', return_value={'state': 'done', 'review_pass': False}) as invoked:
            server.run_native('c' * 32, [], 'candidate.kicad_pcb', 'kicad', mode='improve',
                              allowed=['LED'], source_job=self.job)
        self.assertEqual(invoked.call_args.kwargs['allowed'], ['LED'])
        self.assertTrue((self.root / 'build/layout-jobs' / ('c' * 32) / 'source/Local.pretty/part.kicad_mod').exists())
        (self.native / 'Local.pretty/part.kicad_mod').unlink();(self.native / 'Local.pretty').rmdir()
        with patch.object(server, 'solve_native') as invoked:
            server.run_native('d' * 32, [], 'candidate.kicad_pcb', 'kicad', mode='repair', source_job=self.job)
        self.assertFalse(invoked.called)
        result = json.loads((self.root / 'build/layout-jobs' / ('d' * 32) / 'status.json').read_text())
        self.assertEqual(result['state'], 'failed');self.assertFalse(result['review_pass'])
        self.assertNotIn('archive_url', result)

    def test_failed_solver_result_never_gets_an_archive_link(self):
        with patch.object(server, 'solve_native', return_value={'state': 'failed', 'review_pass': True}):
            server.run_native('e' * 32, [], 'candidate.kicad_pcb', 'kicad',
                              mode='repair', source_job=self.job)
        result = json.loads((self.root / 'build/layout-jobs' / ('e' * 32) / 'status.json').read_text())
        self.assertIsNone(result['archive_url'])

    def test_changed_legacy_profile_blocks_continuation_before_solver(self):
        (self.native/'candidate.routing.json').write_text('{"changed":true}')
        with patch.object(server,'solve_native')as invoked:
            server.run_native('f'*32,[],'candidate.kicad_pcb','kicad',mode='repair',source_job=self.job)
        self.assertFalse(invoked.called)
        result=json.loads((self.root/'build/layout-jobs'/('f'*32)/'status.json').read_text())
        self.assertEqual(result['state'],'failed');self.assertFalse(result['review_pass'])
        self.assertIn('upload the original project',result['stage'])
        self.assertNotIn('archive_url',result)

    def test_saved_result_offers_only_existing_files_and_preserves_job_context(self):
        from review_ui import native_review
        self.status.update(review_pass=True);self.write_status()
        (self.native / 'preview').mkdir()
        (self.native / 'preview/candidate-F_Cu.svg').write_text('<svg/>')
        srv = HTTPServer(('127.0.0.1', 0), server.Handler)
        thread = threading.Thread(target=srv.serve_forever, daemon=True);thread.start()
        def get():
            client = http.client.HTTPConnection('127.0.0.1', srv.server_port, timeout=5)
            client.request('GET', '/api/job/' + self.job)
            response = client.getresponse();value = json.loads(response.read());client.close()
            self.assertEqual(response.status, 200);return value
        try:
            value = get()
            self.assertIn('preview/candidate-F_Cu.svg', value['available_files'])
            self.assertNotIn('preview/board.svg', value['available_files'])
            self.assertIsNone(value['archive_url'])
            markup = native_review(value, self.native)
            self.assertIn('/native?job=' + self.job, markup)
            self.assertNotIn('href="prototype.zip"', markup)
            # Retained HTML is presented with current navigation without editing the archive.
            saved = markup.replace('/native?job=' + self.job, '/native')
            (self.native / 'report.html').write_text(saved)
            client = http.client.HTTPConnection('127.0.0.1', srv.server_port, timeout=5)
            client.request('GET', '/jobs/' + self.job + '/native/report.html')
            response = client.getresponse();served = response.read().decode();client.close()
            self.assertEqual(response.status, 200)
            self.assertIn('/native?job=' + self.job, served)
            self.assertIn('preview/candidate-F_Cu.svg', served)
            self.assertIn('Main navigation', served)
            self.assertEqual((self.native / 'report.html').read_text(), saved)
            (self.native / 'prototype.zip').write_bytes(b'archive')
            self.assertIn('prototype.zip', get()['available_files'])
            self.assertIn('href="prototype.zip"', native_review(value, self.native))
            (self.native / 'candidate.kicad_pcb').unlink()
            self.assertFalse(get()['retained_candidate'])
        finally: srv.shutdown();thread.join();srv.server_close()

    def test_existing_endpoint_accepts_repair_and_rejects_wrong_source_or_editable_scope(self):
        srv = HTTPServer(('127.0.0.1', 0), server.Handler);srv.routing_tools = ('kicad', None, None)
        thread = threading.Thread(target=srv.serve_forever, daemon=True);thread.start()
        def post(body):
            client = http.client.HTTPConnection('127.0.0.1', srv.server_port, timeout=5)
            client.request('POST', '/api/native-route', json.dumps(body), {'Content-Type': 'application/json'})
            response = client.getresponse();value = json.loads(response.read());client.close();return response.status, value
        try:
            with patch.object(server.threading, 'Thread') as worker:
                status, value = post({'source_job': self.job, 'mode': 'repair', 'protected_nets': ['USB_D+']})
                self.assertEqual(status, 202)
                self.assertEqual(worker.call_args.kwargs['args'][4:], ('repair', ['USB_D+'], None, self.job))
                for body in [
                    {'source_job': self.job, 'mode': 'repair', 'editable_nets': ['LED']},
                    {'source_job': self.job, 'mode': 'route'},
                    {'source_job': self.job, 'mode': 'repair', 'files': []},
                    {'source_job': '../outside', 'mode': 'repair'},
                ]:
                    self.assertEqual(post(body)[0], 400)
                files = [{'name': 'board.kicad_pcb', 'text': '(kicad_pcb)'},
                         {'name': 'board.kicad_pro', 'text': '{}'}]
                self.assertEqual(post({'files': files, 'mode': 'repair'})[0], 202)
                files[0] = {'name': 'board.json', 'text': '{}'}
                self.assertEqual(post({'files': files, 'mode': 'repair'})[0], 400)
                self.assertEqual(worker.call_count, 2)
        finally: srv.shutdown();thread.join();srv.server_close()


if __name__ == '__main__': unittest.main()
