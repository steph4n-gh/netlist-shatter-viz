"""Root project ZIP boundaries and exact native asset delivery."""
import base64,hashlib,http.client,io,json,stat,struct,sys,tempfile,threading,unittest,zipfile
from http.server import HTTPServer
from pathlib import Path
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import layout_server as server


def archive(entries=None,compression=zipfile.ZIP_DEFLATED):
    stream=io.BytesIO()
    with zipfile.ZipFile(stream,'w',compression=compression) as z:
        for name,content in (entries or basic()):z.writestr(name,content)
    return stream.getvalue()


def basic():
    return [('board.kicad_pcb','(kicad_pcb exact_source)'),('board.kicad_pro','{"rule":0.1524}'),
            ('fp-lib-table','(fp_lib_table (lib (name "Local") (type "KiCad") (uri "${KIPRJMOD}/Local.pretty")))'),
            ('Local.pretty/part.kicad_mod','(footprint "part" (property "Value" "µF"))'),('LICENSE','source license')]


def body(raw=None):return {'project_zip':{'name':'project.zip','base64':base64.b64encode(raw if raw is not None else archive()).decode('ascii')}}


class NativeProjectUploadTests(unittest.TestCase):
    def test_positive_exact_utf8_assets_stored_deflated_and_profiles(self):
        for compression in (zipfile.ZIP_STORED,zipfile.ZIP_DEFLATED):
            entries=basic()+[('board.routing.json',json.dumps({'net_limits':[{'net':'A','min_width_mm':.2,'max_vias':1,'allowed_layers':['F.Cu']}]}))]
            files,board=server.native_files(body(archive(entries,compression)))
            self.assertEqual(board,'board.kicad_pcb');self.assertEqual({f['name']:f['text'] for f in files},dict(entries))
        self.assertEqual(server.native_files({'files':[{'name':n,'text':t} for n,t in basic()[:2]]})[1],'board.kicad_pcb')

    def test_exact_nested_assets_reach_native_solver_without_board_or_rule_change(self):
        files,name=server.native_files(body());original=dict(basic())
        with tempfile.TemporaryDirectory() as root:
            def solve(board,out,**kwargs):
                self.assertEqual(board.name,'board.kicad_pcb')
                for path,text in original.items():self.assertEqual((board.parent/path).read_bytes(),text.encode('utf-8'))
                self.assertEqual(kwargs['mode'],'repair');return {'state':'done','review_pass':False}
            with patch.object(server,'ROOT',Path(root)),patch.object(server,'solve_native',side_effect=solve) as called,patch.object(server,'ROUTING'):
                server.run_native('a'*32,files,name,'kicad',mode='repair')
            self.assertEqual(called.call_count,1)
            result=json.loads((Path(root)/'build/layout-jobs'/('a'*32)/'status.json').read_text())
            self.assertIsNone(result['archive_url'])

    def test_traversal_absolute_windows_controls_and_duplicates_rejected(self):
        for name in ['../escape','/absolute','a/../escape','./board','a//b','a//','a\\b','C:/board','a\x00b','a/\nb']:
            with self.subTest(name=name),self.assertRaises(ValueError):server.native_files(body(archive(basic()+[(name,'x')])))
        with self.assertRaisesRegex(ValueError,'Duplicate project ZIP path'):server.native_files(body(archive(basic()+[('Local.pretty/caf\u00e9.kicad_mod','(footprint)'),('Local.pretty/cafe\u0301.kicad_mod','(footprint)')])))
        for name in ['board.kicad_pcb','BOARD.kicad_pcb','Local.pretty/part.kicad_mod']:
            with self.subTest(name=name),self.assertRaises(ValueError):server.native_files(body(archive(basic()+[(name,'x')])))

    def test_retained_job_carries_exact_circuit_with_native_source(self):
        with tempfile.TemporaryDirectory() as root:
            jobs=Path(root)/'build/layout-jobs';previous=jobs/('b'*32)
            native=previous/'native';native.mkdir(parents=True)
            source=previous/'source';source.mkdir()
            (native/'candidate.kicad_pcb').write_bytes(b'(kicad_pcb retained_copper)')
            circuit='{"components":[{"reference":"C1","value":"1 µF"}]}\n'.encode('utf-8')
            (native/'source-design.json').write_bytes(circuit)
            original=source/'board.kicad_pcb';original.write_bytes(b'(kicad_pcb original)')
            original.with_suffix('.kicad_pro').write_bytes(b'{"rule":0.1524}')
            (native/'source-manifest.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest()for f in source.iterdir()}))
            (previous/'status.json').write_text(json.dumps({'native':True,'state':'done','source':str(original)}))
            def solve(board,out,**kwargs):
                self.assertEqual(board.read_bytes(),b'(kicad_pcb retained_copper)')
                self.assertEqual((board.parent/'source-design.json').read_bytes(),circuit)
                self.assertEqual(board.with_suffix('.kicad_pro').read_bytes(),b'{"rule":0.1524}')
                return {'state':'done','review_pass':False}
            with patch.object(server,'ROOT',Path(root)),patch.object(server,'solve_native',side_effect=solve) as called,patch.object(server,'ROUTING'):
                server.run_native('a'*32,[],'candidate.kicad_pcb','kicad',mode='improve',source_job='b'*32)
            self.assertEqual(called.call_count,1)
            self.assertEqual((native/'source-design.json').read_bytes(),circuit)

    def test_symlinks_fifo_compression_and_encrypted_entries_rejected(self):
        for kind in (stat.S_IFLNK,stat.S_IFIFO):
            info=zipfile.ZipInfo('Local.pretty/link.kicad_mod');info.create_system=3;info.external_attr=(kind|0o777)<<16
            with self.assertRaises(ValueError):server.native_files(body(archive(basic()+[(info,'outside')])))
        with self.assertRaises(ValueError):server.native_files(body(archive(basic(),zipfile.ZIP_BZIP2)))
        raw=bytearray(archive());local=raw.index(b'PK\x03\x04');central=raw.index(b'PK\x01\x02')
        struct.pack_into('<H',raw,local+6,struct.unpack_from('<H',raw,local+6)[0]|1)
        struct.pack_into('<H',raw,central+8,struct.unpack_from('<H',raw,central+8)[0]|1)
        with self.assertRaises(ValueError):server.native_files(body(raw))

    def test_escaping_library_uris_missing_libraries_and_malformed_table_rejected(self):
        for uri in ['/outside','${KIPRJMOD}/../outside','${KIPRJMOD}//absolute','${KIPRJMOD}/a\\b','${KIPRJMOD}/C:/outside','${GLOBAL}/Local.pretty','${KIPRJMOD}/missing.pretty']:
            entries=basic();entries[2]=('fp-lib-table','(fp_lib_table (lib (uri '+json.dumps(uri)+')))')
            with self.subTest(uri=uri),self.assertRaises(ValueError):server.native_files(body(archive(entries)))
        for table in ['(wrong_table)','(fp_lib_table (lib (uri)))','(fp_lib_table (lib (uri "unterminated)))']:
            entries=basic();entries[2]=('fp-lib-table',table)
            with self.assertRaises(ValueError):server.native_files(body(archive(entries)))

    def test_unsupported_unreferenced_binary_and_ambiguous_core_rejected(self):
        for extra in [('notes.txt','x'),('Extra.pretty/x.kicad_mod','(footprint)'),('Local.pretty/model.step','3D'),('extra.kicad_pcb','(board)'),('nested/board.kicad_pcb','(board)'),('Local.pretty/bin.kicad_mod',b'\xff\xfe')]:
            with self.subTest(extra=extra[0]),self.assertRaises(ValueError):server.native_files(body(archive(basic()+[extra])))
        for entries in [[('board.kicad_pcb','(board)')],[('board.kicad_pcb','(board)'),('other.kicad_pro','{}')],[('board.json','{}'),('board.kicad_pro','{}')]]:
            with self.assertRaises(ValueError):server.native_files(body(archive(entries)))
        with self.assertRaises(ValueError):server.native_files(body(archive(basic()+[('board.routing.json','{"unknown":1}')])))

    def test_compressed_entry_advertised_actual_and_base64_limits(self):
        with patch.object(server,'ZIP_COMPRESSED_LIMIT',10),self.assertRaises(ValueError):server.native_files(body())
        with patch.object(server,'ZIP_DECOMPRESSED_LIMIT',32),self.assertRaises(ValueError):server.native_files(body())
        with patch.object(server,'ZIP_ENTRY_LIMIT',4),self.assertRaises(ValueError):server.native_files(body())
        for payload in ['%%%','é',base64.b64encode(b'not a ZIP').decode()]:
            with self.assertRaises(ValueError):server.native_files({'project_zip':{'name':'x.zip','base64':payload}})
        with self.assertRaises(ValueError):server.native_files({'project_zip':{'name':'../x.zip','base64':''}})
        # Falsified advertised size is still rejected by the actual CRC/decoder.
        raw=bytearray(archive(compression=zipfile.ZIP_STORED));c=raw.index(b'PK\x01\x02');struct.pack_into('<I',raw,c+24,1)
        with self.assertRaises(ValueError):server.native_files(body(raw))

    def test_existing_endpoint_validates_and_queues_exact_zip_files(self):
        slot=Mock();slot.acquire.return_value=True;srv=HTTPServer(('127.0.0.1',0),server.Handler);srv.routing_tools=('kicad',None,None)
        thread=threading.Thread(target=srv.serve_forever,daemon=True);thread.start()
        try:
            with patch.object(server,'ROUTING',slot),patch.object(server.threading,'Thread') as worker:
                connection=http.client.HTTPConnection('127.0.0.1',srv.server_port,timeout=5);connection.request('POST','/api/native-route',json.dumps(dict(body(),mode='repair')),{'Content-Type':'application/json'});response=connection.getresponse();self.assertEqual(response.status,202);self.assertEqual(len(json.loads(response.read())['id']),32);connection.close()
                args=worker.call_args.kwargs['args'];self.assertEqual(args[2],'board.kicad_pcb');self.assertEqual(args[4],'repair');self.assertEqual({f['name']:f['text'] for f in args[1]},dict(basic()));slot.acquire.assert_called_once();worker.return_value.start.assert_called_once()
        finally:srv.shutdown();thread.join();srv.server_close()

    def test_cp1252_license_is_queued_and_delivered_as_exact_solver_source(self):
        license_bytes=b'License: \x93source\x94 \x96 original bytes\r\n'
        entries=basic()[:-1]+[('LICENSE.md',license_bytes),('Local.pretty/NOTICE.txt',license_bytes)]
        original={name:content.encode('utf-8') if isinstance(content,str) else content for name,content in entries}
        slot=Mock();slot.acquire.return_value=True;srv=HTTPServer(('127.0.0.1',0),server.Handler);srv.routing_tools=('kicad',None,None)
        thread=threading.Thread(target=srv.serve_forever,daemon=True);thread.start()
        try:
            with tempfile.TemporaryDirectory() as root:
                def solve(board,out,**kwargs):
                    for name,data in original.items():self.assertEqual((board.parent/name).read_bytes(),data)
                    self.assertEqual(kwargs['mode'],'improve');self.assertEqual(kwargs['protected'],['A']);self.assertEqual(kwargs['allowed'],[])
                    return {'state':'done','review_pass':False}
                with patch.object(server,'ROOT',Path(root)),patch.object(server,'ROUTING',slot),patch.object(server.threading,'Thread') as worker,patch.object(server,'solve_native',side_effect=solve) as called:
                    connection=http.client.HTTPConnection('127.0.0.1',srv.server_port,timeout=5);connection.request('POST','/api/native-route',json.dumps(dict(body(archive(entries)),mode='improve',protected_nets=['A'],editable_nets=[])),{'Content-Type':'application/json'});response=connection.getresponse();self.assertEqual(response.status,202);response.read();connection.close()
                    args=worker.call_args.kwargs['args'];server.run_native(*args)
                    self.assertEqual(called.call_count,1);slot.acquire.assert_called_once();slot.release.assert_called_once()
        finally:srv.shutdown();thread.join();srv.server_close()

    def test_non_utf8_native_core_table_modules_and_unreferenced_license_stay_rejected(self):
        for name in ['board.kicad_pcb','board.kicad_pro','fp-lib-table','Local.pretty/part.kicad_mod']:
            entries=[(n,b'\xff\xfe' if n==name else content) for n,content in basic()]
            with self.subTest(name=name),self.assertRaisesRegex(ValueError,'Invalid project ZIP or UTF-8 file content'):
                server.native_files(body(archive(entries)))
        with self.assertRaisesRegex(ValueError,'unsupported or unreferenced'):
            server.native_files(body(archive(basic()+[('Unreferenced/NOTICE.txt',b'\xff\xfe')])))

    def test_crc_and_mixed_upload_errors_precede_semaphore_and_worker(self):
        raw=bytearray(archive(compression=zipfile.ZIP_STORED));raw[raw.index(b'exact_source')]=ord('X')
        invalid=[body(raw),dict(body(),files=[]),dict(body(),source_job='a'*32)]
        slot=Mock();srv=HTTPServer(('127.0.0.1',0),server.Handler);srv.routing_tools=('kicad',None,None)
        thread=threading.Thread(target=srv.serve_forever,daemon=True);thread.start()
        try:
            with patch.object(server,'ROUTING',slot),patch.object(server.threading,'Thread') as worker:
                for request in invalid:
                    connection=http.client.HTTPConnection('127.0.0.1',srv.server_port,timeout=5);connection.request('POST','/api/native-route',json.dumps(dict(request,mode='repair')),{'Content-Type':'application/json'});response=connection.getresponse();self.assertEqual(response.status,400);self.assertIn('error',json.loads(response.read()));connection.close()
                slot.acquire.assert_not_called();worker.assert_not_called()
        finally:srv.shutdown();thread.join();srv.server_close()


if __name__=='__main__':unittest.main()
