"""Real KiCad round/slot exports keep commands while restoring exact native tools."""
import copy
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from native_drill_precision import restore_tool_precision

FIXTURE=Path(__file__).parent/'fixtures/native-drill-precision'


class NativeDrillPrecisionTests(unittest.TestCase):
    def fixture(self):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        root=Path(temporary.name);raw=root/'raw';raw.mkdir()
        for path in FIXTURE.glob('*.drl'):shutil.copy2(path,raw/path.name)
        native=json.loads((FIXTURE/'native-drill-holes.json').read_text())
        return raw,native,root/'cam'

    def test_exact_tools_preserve_every_other_byte_and_raw_files(self):
        raw,native,output=self.fixture()
        before={p.name:p.read_bytes() for p in raw.glob('*.drl')}
        result=restore_tool_precision(raw,native,native['source_board_sha256'],output)
        self.assertTrue(result['pass']);self.assertEqual(result['matched_native_features'],6)
        self.assertEqual({r['native_uuid'] for r in result['records']},{h['uuid'] for h in native['holes']})
        expected={'candidate-PTH.drl':{1:406400,2:600123,3:600124,4:600124},'candidate-NPTH.drl':{1:650321}}
        for name,data in before.items():
            self.assertEqual((raw/name).read_bytes(),data)
            restored=(output/name).read_bytes()
            strip=lambda b:re.sub(rb'^T\d+C[\d.]+$',b'TOOL',b,flags=re.M)
            self.assertEqual(strip(restored),strip(data))
            tools={int(n):round(float(d)*1e6) for n,d in re.findall(rb'^T(\d+)C([\d.]+)$',restored,re.M)}
            self.assertEqual(tools,expected[name])
            record=next(f for f in result['files'] if Path(f['raw_file']).name==name)
            self.assertEqual(record['raw_sha256'],hashlib.sha256(data).hexdigest())
            self.assertEqual(record['restored_sha256'],hashlib.sha256(restored).hexdigest())

    def test_bad_source_or_missing_ambiguous_geometry_rejects_before_output(self):
        for defect in ('source','missing_metadata','missing_hole','duplicate_hole','diameter','position','plating','layer_span'):
            with self.subTest(defect=defect):
                raw,native,output=self.fixture();expected=native['source_board_sha256']
                if defect=='source':expected='0'*64
                elif defect=='missing_metadata':native.pop('holes')
                elif defect=='missing_hole':native['holes'].pop()
                elif defect=='duplicate_hole':native['holes'].append(copy.deepcopy(native['holes'][0]))
                elif defect=='diameter':native['holes'][0]['diameter_nm']+=1000
                elif defect=='position':native['holes'][0]['ends_nm'][0][0]+=1000
                elif defect=='plating':native['holes'][0]['plated']=True
                else:native['holes'][0]['layer_pair']=[1,4]
                with self.assertRaises(ValueError):restore_tool_precision(raw,native,expected,output)
                self.assertFalse(output.exists())

    def test_missing_file_or_file_function_rejects_before_output(self):
        for defect in ('missing_NPTH','missing_function','duplicate_function'):
            with self.subTest(defect=defect):
                raw,native,output=self.fixture();path=raw/'candidate-NPTH.drl'
                if defect=='missing_NPTH':path.unlink()
                else:
                    original=path.read_text();line='; #@! TF.FileFunction,NonPlated,1,2,NPTH'
                    path.write_text(original.replace(line,'' if defect=='missing_function' else line+'\n'+line))
                with self.assertRaises(ValueError):restore_tool_precision(raw,native,native['source_board_sha256'],output)
                self.assertFalse(output.exists())

    def test_different_exact_native_diameters_cannot_share_rounded_tool(self):
        raw,native,output=self.fixture();path=raw/'candidate-PTH.drl'
        text=path.read_text().replace('T3C0.600\n','').replace('\nT3\n','\nT2\n')
        path.write_text(text)
        with self.assertRaisesRegex(ValueError,'one exact native diameter'):
            restore_tool_precision(raw,native,native['source_board_sha256'],output)
        self.assertFalse(output.exists())

    def test_any_existing_final_drill_is_preserved_and_rejected(self):
        raw,native,output=self.fixture();output.mkdir()
        sentinel=output/'unrelated-old-name.drl';sentinel.write_bytes(b'old immutable bytes')
        with self.assertRaisesRegex(ValueError,'already exists'):
            restore_tool_precision(raw,native,native['source_board_sha256'],output)
        self.assertEqual(list(output.iterdir()),[sentinel]);self.assertEqual(sentinel.read_bytes(),b'old immutable bytes')


if __name__=='__main__':unittest.main()
