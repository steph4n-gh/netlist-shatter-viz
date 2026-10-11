import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import solve_kicad
from optimize_kicad import optimize


class NativeWorkerStartupTests(unittest.TestCase):
    def test_improvement_search_does_not_treat_launcher_option_as_script_path(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            source = root / 'source.kicad_pcb'
            source.write_text('(kicad_pcb)')
            out = root / 'out'
            out.mkdir()
            (out / 'candidate.kicad_pcb').write_bytes(source.read_bytes())
            (out / 'original-normalized.kicad_pcb').write_bytes(source.read_bytes())
            metrics = {'length_mm': 10, 'vias': 0, 'nets': {'1': {
                'name': 'SIGNAL', 'length_mm': 10, 'vias': 0, 'pads': 2,
                'locked': False, 'zone': False, 'width_mm': .2}}}
            (out / 'source-metrics.json').write_text(json.dumps(metrics))
            (out / 'source-drc.json').write_text('{}')
            with patch.object(solve_kicad.sys, 'platform', 'darwin'), \
                 patch.object(solve_kicad, 'find_python', return_value=sys.executable):
                worker = solve_kicad.native_worker_command('kicad', root / 'worker.py')
            searches = []

            def run(command, log, *seconds):
                if 'geometry' in command:
                    directory = Path(command[-1])
                    (directory / 'geometry.json').write_text(json.dumps({
                        'nets': {'1': {'name': 'SIGNAL'}}}))
                elif len(command) > 1 and str(command[1]).endswith('grid_route.py'):
                    searches.append(Path(command[1]))
                    directory = Path(command[2])
                    (directory / 'grid-route.json').write_text(json.dumps({
                        'failed': [{'net': 1}], 'completed_nets': []}))

            with patch('shortcut_route.shortcut', return_value={'remove': []}):
                result = optimize(out, source, worker, 'kicad', run,
                                  lambda **fields: None, lambda *args: {}, 10)
            self.assertEqual(searches, [Path(solve_kicad.__file__).parent / 'grid_route.py'])
            self.assertTrue(searches[0].is_file())
            self.assertFalse(result['improved'])
            self.assertTrue(result['original_retained'])
            self.assertEqual((out / 'candidate.kicad_pcb').read_bytes(), source.read_bytes())

    def test_macos_launch_preserves_script_arguments_and_sibling_imports(self):
        with tempfile.TemporaryDirectory(prefix='native worker ') as name:
            directory = Path(name)
            (directory / 'sibling.py').write_text('VALUE = 123\n')
            script = directory / 'worker.py'
            script.write_text('import json,sys,sibling\nprint(json.dumps([sys.argv,sibling.VALUE]))\n')
            with patch.object(solve_kicad.sys, 'platform', 'darwin'), \
                 patch.object(solve_kicad, 'find_python', return_value=sys.executable):
                command = solve_kicad.native_worker_command('kicad', script)
            result = subprocess.run(command + ['dc-geometry', 'a directory with spaces'],
                                    capture_output=True, text=True, check=True, timeout=10)
            arguments, imported = json.loads(result.stdout)
            self.assertEqual(arguments, [str(script), 'dc-geometry', 'a directory with spaces'])
            self.assertEqual(imported, 123)


if __name__ == '__main__':
    unittest.main()
