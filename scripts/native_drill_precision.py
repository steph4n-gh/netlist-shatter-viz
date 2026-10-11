"""Restore native tool diameters in KiCad's metric Excellon; keep every other byte."""
import hashlib,json,re,warnings
from pathlib import Path
from gerbonara.excellon import ExcellonParser
from gerbonara.utils import MM


def restore_tool_precision(directory, native, expected_source_sha256, output):
    if not isinstance(native,dict) or native.get('source_board_sha256')!=expected_source_sha256:
        raise ValueError('Native drill metadata does not match the selected board SHA256')
    if not isinstance(native.get('holes'),list):raise ValueError('Missing native drilled feature metadata')
    holes=native['holes'];remaining=set(range(len(holes)));proposed={};records=[]
    for path in sorted(Path(directory).glob('*.drl')):
        original=path.read_bytes().decode('utf-8')
        metadata=re.findall(r'^;\s*#@! TF.FileFunction,(Plated|NonPlated),(\d+),(\d+),(?:PTH|NPTH)$',original,re.M)
        if len(metadata)!=1:raise ValueError('Missing or ambiguous native Excellon plating/layer span')
        plating,top,bottom=metadata[0];plated=plating=='Plated';span=[int(top),int(bottom)]
        parser=ExcellonParser()
        with warnings.catch_warnings(record=True) as notices:
            warnings.simplefilter('always');parser.do_parse(original,filename=str(path))
        if parser.settings.unit!=MM:raise ValueError('Only the current metric native export is supported')
        if any('G90 header statement' not in str(v.message) for v in notices):raise ValueError('Unexpected drill parser warning')
        tools={}
        for obj in parser.objects:
            if type(obj).__name__=='Flash':ends=sorted([[obj.x,obj.y],[obj.x,obj.y]])
            elif type(obj).__name__=='Line':ends=sorted([[obj.x1,obj.y1],[obj.x2,obj.y2]])
            else:raise ValueError('Unsupported native drill command geometry')
            matches=[i for i in remaining if holes[i]['plated']==plated and holes[i]['layer_pair']==span
                     and abs(holes[i]['diameter_nm']/1e6-obj.tool.diameter)<=.000501
                     and all(abs(a/1e6-b)<=.000501 for p,q in zip(holes[i]['ends_nm'],ends) for a,b in zip(p,q))]
            if len(matches)!=1:raise ValueError('Native drill feature has no unique Excellon match')
            index=matches[0];remaining.remove(index)
            ids=[number for number,tool in parser.tools.items() if tool is obj.tool]
            if len(ids)!=1:raise ValueError('Drill tool identity is ambiguous')
            tools.setdefault(ids[0],set()).add(holes[index]['diameter_nm'])
            records.append({'file':path.name,'tool':ids[0],'native_uuid':holes[index]['uuid'],'native_diameter_nm':holes[index]['diameter_nm']})
        if set(tools)!=set(parser.tools) or any(len(diameters)!=1 for diameters in tools.values()):raise ValueError('Tool group does not have one exact native diameter')
        substitutions=[]
        def replace(match):
            number=int(match[1]);diameter=next(iter(tools[number]));substitutions.append(number)
            return f'T{number}C{diameter//1000000}.{diameter%1000000:06d}'
        rewritten=re.sub(r'^T(\d+)C([\d.]+)$',replace,original,flags=re.M)
        if len(substitutions)!=len(tools) or set(substitutions)!=set(tools):raise ValueError('Duplicate or unsupported tool table')
        check=ExcellonParser()
        with warnings.catch_warnings():warnings.simplefilter('ignore');check.do_parse(rewritten,filename=str(path))
        if len(check.objects)!=len(parser.objects):raise ValueError('Drill commands changed')
        for a,b in zip(parser.objects,check.objects):
            if type(a)!=type(b):raise ValueError('Drill command shape changed')
            for coordinate in ['x','y','x1','y1','x2','y2']:
                if getattr(a,coordinate,None)!=getattr(b,coordinate,None):raise ValueError('Drill coordinates changed')
        for number,diameters in tools.items():
            if abs(check.tools[number].diameter-next(iter(diameters))/1e6)>1e-12:raise ValueError('Native tool diameter did not survive reparse')
        proposed[path]=rewritten
    if remaining:raise ValueError('Not every native drilled feature is represented')
    if holes and not proposed:raise ValueError('Missing native drill files')
    output=Path(output)
    if any(output.glob('*.drl')):raise ValueError('Final drill output already exists')
    output.mkdir(parents=True,exist_ok=True)
    files=[]
    for path,content in proposed.items():
        encoded=content.encode('utf-8');target=output/path.name;target.write_bytes(encoded)
        files.append({'raw_file':str(path),'raw_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                      'restored_file':str(target),'restored_sha256':hashlib.sha256(encoded).hexdigest()})
    return {'pass':True,'native_source_sha256':native['source_board_sha256'],'matched_native_features':len(holes),'tool_only_precision_restored':True,'native_matching_axis_tolerance_mm':.000501,'tool_diameter_error_mm':0,'records':records,'files':files,
            'CAM_scope':'Native-derived tool-restored Excellon; all raw commands and coordinates retained, not byte-exact raw native export.'}

if __name__=='__main__':
    import sys
    result=restore_tool_precision(Path(sys.argv[1]),json.loads(Path(sys.argv[2]).read_text()),sys.argv[3],Path(sys.argv[4]))
    Path(sys.argv[5]).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='records'}))
