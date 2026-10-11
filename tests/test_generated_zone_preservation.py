import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from solve_kicad import audit_preservation
from kicad_sexpr import sexpr,zone_definition


class GeneratedZonePreservationTest(unittest.TestCase):
    generated='(zone (net "SIGNAL") (attr (teardrop (type padvia))) (polygon (pts (xy 1 2) (xy 2 3))) (fill yes (thermal_gap 0.5)))'
    functional='(zone (net "GND") (polygon (pts (xy 0 0) (xy 8 8))) (fill yes (thermal_gap 0.5)))'

    def audit(self,before,after,manifest_edit=None):
        with tempfile.TemporaryDirectory() as root:
            a,b=Path(root)/'source',Path(root)/'after';a.write_text(before);b.write_text(after)
            z=next(v for v in sexpr(before) if isinstance(v,list) and v and v[0]=='zone')
            m={'normalized_source_sha256':hashlib.sha256(a.read_bytes()).hexdigest(),
               'removed_generated_zones':[{'uuid':'native-only','net_name':'SIGNAL','native_teardrop':True}],
               'removed_zone_definitions':[{'definition_sha256':hashlib.sha256(json.dumps(zone_definition(z),separators=(',',':')).encode()).hexdigest(),
                                             'net_name':'SIGNAL','native_teardrop':True}]}
            if manifest_edit:manifest_edit(m)
            return audit_preservation(a,b,m),audit_preservation(a,b)

    def test_exact_generated_removal_requires_explicit_manifest(self):
        before='(kicad_pcb '+self.generated+' '+self.functional+')'
        result,strict=self.audit(before,'(kicad_pcb '+self.functional+')')
        self.assertTrue(result['pass']);self.assertFalse(strict['pass'])
        self.assertEqual(len(result['removed_generated_zone_definition_sha256']),1)

    def test_functional_zone_changes_are_never_covered_by_manifest(self):
        before='(kicad_pcb '+self.generated+' '+self.functional+')'
        for changed in ('',self.functional.replace('(xy 8 8)','(xy 8 9)'),self.functional.replace('0.5','0.4')):
            self.assertFalse(self.audit(before,'(kicad_pcb '+changed+')')[0]['pass'])

    def test_locked_routing_and_locked_generated_zone_cannot_be_removed(self):
        for protection in ('(segment (net "SIGNAL") (locked yes))', '(via (net "SIGNAL") locked)'):
            self.assertFalse(self.audit('(kicad_pcb '+self.generated+' '+protection+')','(kicad_pcb '+protection+')')[0]['pass'])
        locked=self.generated.replace('(zone ','(zone locked ')
        self.assertFalse(self.audit('(kicad_pcb '+locked+')','(kicad_pcb)')[0]['pass'])

    def test_source_hash_native_classification_and_counts_must_match(self):
        before='(kicad_pcb '+self.generated+')'
        edits=(lambda m:m.update(normalized_source_sha256='wrong'),
               lambda m:m['removed_generated_zones'][0].update(native_teardrop=False),
               lambda m:m['removed_zone_definitions'].append(dict(m['removed_zone_definitions'][0])))
        for edit in edits:self.assertFalse(self.audit(before,'(kicad_pcb)',edit)[0]['pass'])

    def test_functional_zone_cannot_impersonate_generated_zone(self):
        for original in (self.functional,self.generated.replace('padvia','unknown')):
            self.assertFalse(self.audit('(kicad_pcb '+original+')','(kicad_pcb)')[0]['pass'])


if __name__=='__main__':unittest.main()
