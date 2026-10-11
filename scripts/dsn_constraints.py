"""Carry explicit width and layer restrictions into Freerouting's search input.

Native rules and final independent checks remain authoritative. Positive via
counts, terminal path bounds and pair skew have no equivalent enforced here.
"""
import copy
import math
from kicad_sexpr import sexpr

QUOTE='(string_quote ")'

def decode(value):
    # Specctra's quoted identifiers are literal, not JSON string escapes.
    # For example /GPIO16\I2C-SCL contains one actual backslash.
    return value[1:-1] if value.startswith('"') and value.endswith('"') else value

def quote(value):
    if '"' in value or any(ord(c)<32 for c in value):
        raise ValueError('Unsupported quote/control character in DSN identifier')
    return '"'+value+'"'

def parse(text):
    if text.count(QUOTE)!=1:raise ValueError('Expected KiCad DSN quote declaration')
    tree=sexpr(text.replace(QUOTE,'(string_quote __native_quote__)'),literal_strings=True)
    if ['unit','um'] not in tree:raise ValueError('Expected micrometre DSN units')
    return tree

def serialize(tree):
    def render(node,depth=0):
        if not isinstance(node,list):return node
        if all(not isinstance(v,list) for v in node):return '('+' '.join(node)+')'
        first=next(i for i,v in enumerate(node) if isinstance(v,list))
        return '('+' '.join(node[:first])+''.join('\n'+'  '*(depth+1)+render(v,depth+1) for v in node[first:])+')'
    return render(tree).replace('(string_quote __native_quote__)',QUOTE)+'\n'


def apply(text,profile,clearance_margin_um=0):
    from routing_screen import validate
    validate(profile)
    tree=parse(text)
    reserved=any(isinstance(scope,list) and scope and scope[0]=='wiring' and
                 any(isinstance(item,list) and item and item[0] in ('wire','via') and
                     any(['type',kind] in item for kind in ('protect','fix'))
                     for item in scope) for scope in tree)
    removed_global=0
    if reserved and clearance_margin_um:
        # KiCad exports locked routing as "fix"; generated seeds use "protect".
        # Both native-valid routes may occupy the optional 20 um search
        # guard. Apply the native clearances consistently on both sides of them.
        def unpad(node):
            if not isinstance(node,list):return
            if node and node[0]=='clearance':
                value=float(node[1])-clearance_margin_um
                if value<0:raise ValueError('Invalid exporter clearance margin')
                node[1]=str(value)
            else:
                for child in node:unpad(child)
        unpad(tree);removed_global=clearance_margin_um;clearance_margin_um=0
    network=next(v for v in tree if isinstance(v,list) and v and v[0]=='network')
    if any(isinstance(v,list) and v and v[0]=='class_class' for v in network):
        raise ValueError('Cross-class DSN rules require explicit constraint translation')
    classes=[v for v in network if isinstance(v,list) and v and v[0]=='class']
    layer_names={decode(v[1]) for scope in tree if isinstance(scope,list) and scope and scope[0]=='structure'
                 for v in scope if isinstance(v,list) and v and v[0]=='layer'}
    additions=[];applied=[]
    for limit in profile.get('net_limits',[]):
        if not {'min_width_mm','allowed_layers'}&set(limit):continue
        name=limit['net']
        matches=[c for c in classes if name in [decode(v) for v in c[2:] if isinstance(v,str)]]
        if len(matches)!=1:raise ValueError('Expected exactly one native class for '+name)
        original=matches[0];rules=copy.deepcopy([v for v in original[2:] if isinstance(v,list)])
        allowed=limit.get('allowed_layers')
        if allowed is not None:
            if set(allowed)-layer_names:raise ValueError('Unknown copper layer for '+name)
            circuits=[v for v in rules if v[0]=='circuit']
            for circuit in circuits:
                for rule in circuit[1:]:
                    if rule[0]=='use_layer':allowed=[l for l in allowed if l in [decode(v) for v in rule[1:]]]
            if not allowed:raise ValueError('Conflicting layer restrictions for '+name)
            combined=['circuit']+[v for c in circuits for v in c[1:] if v[0]!='use_layer']
            combined.append(['use_layer',*[quote(l) for l in allowed]])
            rules=[v for v in rules if v[0]!='circuit']+[combined]
        requested_width=limit.get('min_width_mm',0)*1000
        # The retained Freerouting SES uses 0.1 um width resolution. Round
        # minima outward: 200.66 um must not come back as 200.6 um.
        width=math.ceil(requested_width*10-1e-9)/10
        def widths(node):
            if isinstance(node,list):
                if node and node[0]=='width':node[1]=str(math.ceil(max(float(node[1]),width)*10-1e-9)/10)
                elif node and node[0]=='clearance' and clearance_margin_um:
                    # Remove only the known search-only padding added by our
                    # native exporter. Native clearances remain unchanged.
                    value=float(node[1])-clearance_margin_um
                    if value<0:raise ValueError('Invalid exporter clearance margin')
                    node[1]=str(value)
                else:
                    for v in node:widths(v)
        widths(rules)
        rules.append(['rule',['width',str(max(width,max((float(r[1]) for scope in rules if scope[0]=='rule' for r in scope[1:] if r[0]=='width'),default=0)))]] )
        additions.append(['class',quote('native_constraint_'+str(len(additions))),quote(name),*rules])
        original[:]=[v for i,v in enumerate(original) if i<2 or isinstance(v,list) or decode(v)!=name]
        applied.append({'net':name,'minimum_width_mm':requested_width/1000,'rounded_minimum_width_mm':width/1000,
                        'allowed_layers':allowed,'removed_search_padding_um':clearance_margin_um})
    network.extend(additions)
    return serialize(tree),{'applied':applied,'removed_global_search_padding_um':removed_global,
        'scope':'Search widths and copper layers; remaining constraints checked independently'}


def add_seed(text,geometry,names):
    """Reserve freshly generated, checked signal copper in the search copy."""
    import re
    tree=parse(text);wiring=next(v for v in tree if isinstance(v,list) and v and v[0]=='wiring')
    library=next(v for v in tree if isinstance(v,list) and v and v[0]=='library')
    padstacks={}
    for item in library:
        if not isinstance(item,list) or item[0]!='padstack':continue
        token=item[1];name=decode(token)
        match=re.fullmatch(r'Via\[0-\d+\]_([\d.]+):([\d.]+)_um',name)
        if match:padstacks[tuple(round(float(v),3) for v in match.groups())]=token
    nets={int(code):n['name'] for code,n in geometry['nets'].items()}
    mm=lambda value:format(value*1000,'.6f')
    added={'tracks':0,'vias':0,'nets':sorted(names)}
    for t in geometry['tracks']:
        if nets[t['net']] not in names or t['locked']:continue
        if t['arc']:raise ValueError('Seed arcs are unsupported')
        wiring.append(['wire',['path',quote(geometry['layer_names'][str(t['layer'])]),mm(t['width']),
            mm(t['start'][0]),mm(-t['start'][1]),mm(t['end'][0]),mm(-t['end'][1])],['net',quote(nets[t['net']])],['type','protect']])
        added['tracks']+=1
    for v in geometry['vias']:
        if nets[v['net']] not in names:continue
        if not v['through']:raise ValueError('Seed requires through vias')
        key=round(v['diameter_mm']*1000,3),round(v['drill_mm']*1000,3)
        if key not in padstacks:raise ValueError('Missing native via padstack for seed')
        wiring.append(['via',padstacks[key],mm(v['at'][0]),mm(-v['at'][1]),['net',quote(nets[v['net']])],['type','protect']])
        added['vias']+=1
    seeded,attachments=encode_existing_seed_attachments(serialize(tree),geometry)
    added['native_seed_attachments']=attachments
    return seeded,added


def encode_native_smd_drills(text,footprints):
    """Stop Freerouting inferring holes in native undrilled SMD padstacks."""
    from native_project import routing_references
    tree=parse(text)
    aliases=routing_references(text,footprints)
    undrilled={aliases[f['id']]:set(f['zero_drill_pins']) for f in footprints}
    library=next(v for v in tree if isinstance(v,list) and v and v[0]=='library')
    images={decode(v[1]):v for v in library if isinstance(v,list) and v and v[0]=='image'}
    placement=next(v for v in tree if isinstance(v,list) and v and v[0]=='placement')
    uses={}
    for component in placement[1:]:
        for pose in component[2:]:
            if pose[0]!='place':continue
            pins=undrilled[decode(pose[1])]
            for pin in images[decode(component[1])][2:]:
                if isinstance(pin,list) and pin and pin[0]=='pin':
                    uses.setdefault(pin[1],[]).append(decode(next(v for v in pin[2:] if isinstance(v,str))) in pins)
    renames={};existing={decode(v[1]) for v in library if isinstance(v,list) and v and v[0]=='padstack'}
    for item in library:
        if not isinstance(item,list) or not item or item[0]!='padstack':continue
        old=item[1]
        layers=[v for v in item[2:] if isinstance(v,list) and v and v[0]=='shape']
        if len(layers)!=1 or not uses.get(old) or not all(uses[old]):continue
        # Upstream 2.4.1 Padstack.getDrillRadius reads diameter ratios from
        # _outer:drill_ names, otherwise guesses 45% of copper half-width.
        # A positive denominator and zero numerator returns exactly zero.
        # Use a fresh identifier so a colon in the old name cannot interfere.
        if decode(old).startswith('NativeSMD_') and decode(old).endswith('_1:0_um'):continue
        name='NativeSMD_'+str(len(renames))+'_1:0_um'
        if name in existing:raise ValueError('Native SMD DSN identifier collision')
        existing.add(name);renames[old]=quote(name)
    for item in library:
        if not isinstance(item,list) or not item:continue
        if item[0]=='padstack' and item[1] in renames:item[1]=renames[item[1]]
        elif item[0]=='image':
            for pin in item[2:]:
                if isinstance(pin,list) and pin and pin[0]=='pin' and pin[1] in renames:
                    pin[1]=renames[pin[1]]
    return serialize(tree),{'renamed_padstacks':{decode(k):decode(v) for k,v in renames.items()},
        'native_undrilled_pin_instances':sum(len(uses[k]) for k in renames),
        'scope':'Single-layer padstack identifiers only; every native pin use has zero drill; physical shapes and rules unchanged'}

def encode_existing_seed_attachments(text,geometry):
    """Represent existing protected native via/SMD copper contacts only.

    The attach-enabled aliases stay outside every selectable via rule. Their
    original physical stacks are restored in SES before native import. Native
    DRC remains mandatory; this does not enable new via-in-pad routing.
    """
    import re
    tree=parse(text)
    library=next(v for v in tree if isinstance(v,list) and v[0]=='library')
    structure=next(v for v in tree if isinstance(v,list) and v[0]=='structure')
    wiring=next(v for v in tree if isinstance(v,list) and v[0]=='wiring')
    stacks={v[1]:v for v in library if isinstance(v,list) and v[0]=='padstack'}
    aliases=_existing_seed_aliases(stacks)
    original_stacks={k:v for k,v in stacks.items() if k not in aliases}
    report={'fixed_via_changes':[],'contacts':[],
            'scope':'Existing protected native zero-drill contacts only; selectable via permissions unchanged'}
    # Enabling the global parser flag must not enable any original via stack.
    # Conservative no-op for other exporters with missing/attach-on stacks.
    if any([v for v in s if isinstance(v,list) and v[0]=='attach']!=[['attach','off']]
           for s in original_stacks.values()):
        report['skipped']='Original padstack attachment permissions are not explicitly off'
        return text,report
    selectable=[]
    def allowed(node):
        if not isinstance(node,list):return
        if node and node[0] in ('via','use_via'):selectable.extend(node[1:])
        else:
            for child in node:allowed(child)
    allowed(structure)
    network=next(v for v in tree if isinstance(v,list) and v[0]=='network')
    allowed(network)
    if not selectable or any(v not in original_stacks for v in selectable):
        report['skipped']='Selectable via stacks are absent, unknown or attachment aliases'
        return text,report
    controls=[v for v in structure if isinstance(v,list) and v[0]=='control']
    settings=[v for c in controls for v in c[1:] if isinstance(v,list) and v[0]=='via_at_smd']
    if len(settings)>1 or any(v not in (['via_at_smd','off'],['via_at_smd','on']) for v in settings):
        raise ValueError('Ambiguous DSN via attachment control')
    nets={int(c):n['name'] for c,n in geometry['nets'].items()}
    # Native polygons have an outward approximation error <=1 um. Require
    # >2 um physical overlap; ambiguous tangencies do not change permissions.
    def segment_distance(p,a,b):
        dx,dy=b[0]-a[0],b[1]-a[1];den=dx*dx+dy*dy
        t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den)) if den else 0
        return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
    def ring_inside(p,ring):
        inside=False
        for a,b in zip(ring,ring[1:]+ring[:1]):
            if (a[1]>p[1])!=(b[1]>p[1]) and p[0]<(b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]:inside=not inside
        return inside
    def overlaps(p,r,poly):
        shell=poly['shell'];holes=poly.get('holes',[])
        if ring_inside(p,shell) and not any(ring_inside(p,h) for h in holes):return True
        return min((segment_distance(p,a,b) for ring in [shell]+holes
                    for a,b in zip(ring,ring[1:]+ring[:1])),default=float('inf'))<r-.002
    for index,item in enumerate(wiring[1:],1):
        if not isinstance(item,list) or item[0]!='via':continue
        fixed=next((v[1] for v in item if isinstance(v,list) and v[0]=='type'),None)
        if fixed not in ('protect','fix'):continue
        token=item[1]
        if token in aliases:continue
        if token not in original_stacks:raise ValueError('Unknown fixed seed via padstack')
        at=[float(item[2])/1000,-float(item[3])/1000]
        name=decode(next(v[1] for v in item if isinstance(v,list) and v[0]=='net'))
        tolerance=.0008 if fixed=='fix' else .000001
        matches=[v for v in geometry.get('vias',[]) if nets.get(v['net'])==name and math.dist(v['at'],at)<tolerance and (fixed!='fix' or v.get('locked'))]
        if len(matches)!=1:continue
        via=matches[0];contacts=[]
        dimensions=re.fullmatch(r'Via\[0-\d+\]_([\d.]+):([\d.]+)_um',decode(token))
        if not dimensions or any(abs(float(value)/1000-via[key])>.000001 for value,key in zip(dimensions.groups(),('diameter_mm','drill_mm'))):continue
        shapes=[v[1] for v in stacks[token] if isinstance(v,list) and v[0]=='shape']
        if (len(shapes)!=len(geometry['layers']) or
            {decode(s[1]) for s in shapes if s[0]=='circle'}!={geometry['layer_names'][str(l)] for l in geometry['layers']} or
            any(s[0]!='circle' or abs(float(s[2])/1000-via['diameter_mm'])>.000001 or
                any(float(c)!=0 for c in s[3:]) for s in shapes)):
            continue
        for pad in geometry.get('pads',[]):
            if not via.get('through') or pad['net']!=via['net'] or pad.get('drill')!=[0,0] or len(pad['shapes'])!=1:continue
            for layer,shapes in pad['shapes'].items():
                if any(overlaps(via['at'],via['diameter_mm']/2,s) for s in shapes):
                    contacts.append({'native_via_uuid':via['uuid'],'pad_ref':pad['ref'],'native_pin':pad['pin'],'layer':layer,'native_pad_drill_mm':pad['drill']})
        if not contacts:continue
        clone_token=quote('NativeSeedAttach_'+decode(token))
        if clone_token not in stacks:
            clone=copy.deepcopy(stacks[token]);clone[1]=clone_token
            next(v for v in clone if isinstance(v,list) and v[0]=='attach')[1]='on'
            library.append(clone);stacks[clone_token]=clone;aliases[clone_token]=token
        elif aliases.get(clone_token)!=token:raise ValueError('Native seed alias collision')
        item[1]=clone_token
        report['fixed_via_changes'].append({'wiring_index':index,'native_uuid':via['uuid'],'before_padstack':token,'after_padstack':clone_token,'net':name,'at_mm':at})
        report['contacts'].extend(contacts)
    if report['fixed_via_changes']:
        if settings:settings[0][1]='on'
        else:structure.append(['control',['via_at_smd','on']])
        return serialize(tree),report
    return text,report


def _existing_seed_aliases(stacks):
    aliases={}
    for token,stack in stacks.items():
        name=decode(token)
        if not name.startswith('NativeSeedAttach_'):continue
        original=quote(name[len('NativeSeedAttach_'):])
        candidates=[v for v in stacks if decode(v)==decode(original)]
        if len(candidates)!=1:raise ValueError('Missing original native seed via stack')
        original=candidates[0];check=copy.deepcopy(stack);check[1]=original
        attachment=[v for v in check if isinstance(v,list) and v[0]=='attach']
        if attachment!=[['attach','on']]:raise ValueError('Invalid native seed attachment alias')
        attachment[0][1]='off'
        if check!=stacks[original]:raise ValueError('Physically different native seed alias')
        aliases[token]=original
    return aliases


def canonicalize_seed_attachment_session(dsn_text,ses_text):
    """Restore identical original stack names for nonselectable fixed SES vias."""
    dsn=parse(dsn_text);library=next(v for v in dsn if isinstance(v,list) and v[0]=='library')
    stacks={v[1]:v for v in library if isinstance(v,list) and v[0]=='padstack'}
    aliases=_existing_seed_aliases(stacks)
    if not aliases:return ses_text,{'changed_fixed_via_identifiers':0}
    session=sexpr(ses_text,literal_strings=True)
    routes=next(v for v in session if isinstance(v,list) and v[0]=='routes')
    lib=next(v for v in routes if isinstance(v,list) and v[0]=='library_out')
    definitions={decode(v[1]):v for v in lib if isinstance(v,list) and v[0]=='padstack'}
    network=next(v for v in routes if isinstance(v,list) and v[0]=='network_out')
    resolution=next(v for v in routes if isinstance(v,list) and v[0]=='resolution')
    if resolution[1]!='um':raise ValueError('Unknown SES via coordinate units')
    scale=float(resolution[2])
    if scale<=0:raise ValueError('Invalid SES resolution')
    wiring=next(v for v in dsn if isinstance(v,list) and v[0]=='wiring')
    originals=[v for v in wiring if isinstance(v,list) and v[0]=='via' and v[1] in aliases and any(['type',f] in v for f in ('protect','fix'))]
    def physical_shapes(stack,units):
        shapes=[]
        for item in stack[2:]:
            if not isinstance(item,list) or item[0]!='shape':continue
            shape=item[1]
            if shape[0]!='circle':raise ValueError('Unsupported native seed SES stack shape')
            values=[float(v)/units for v in shape[2:]]
            if len(values)==1:values.extend([0,0])
            if len(values)!=3:raise ValueError('Invalid seed via circle')
            shapes.append((decode(shape[1]),*values))
        return sorted(shapes)
    used=set();changes=0
    # Scope-only edits: equal strings in net names, geometry and rules stay exact.
    for net in network[1:]:
        if not isinstance(net,list) or net[0]!='net':continue
        for via in net[2:]:
            if not isinstance(via,list) or via[0]!='via' or via[1] not in aliases:continue
            fixed=next((v[1] for v in via if isinstance(v,list) and v[0]=='type'),None)
            if fixed not in ('protect','fix'):raise ValueError('Movable attachment alias in routing session')
            name=decode(net[1])
            matches=[i for i,v in enumerate(originals) if i not in used and v[1]==via[1] and ['type',fixed] in v and
                decode(next(c[1] for c in v if isinstance(c,list) and c[0]=='net'))==name and
                all(abs(float(v[j])-float(via[j])/scale)<=.5/scale+1e-6 for j in (2,3))]
            if len(matches)!=1:raise ValueError('Session alias is not the original protected seed via')
            used.add(matches[0])
            original=aliases[via[1]]
            if decode(original) not in definitions:raise ValueError('Original seed via missing from SES library')
            native_shapes=physical_shapes(stacks[original],1)
            ses_shapes=physical_shapes(definitions[decode(original)],scale)
            if len(native_shapes)!=len(ses_shapes) or any(a[0]!=b[0] or any(abs(x-y)>1e-6 for x,y in zip(a[1:],b[1:])) for a,b in zip(native_shapes,ses_shapes)):
                raise ValueError('Physically different original SES via stack')
            via[1]=original;changes+=1
    return (serialize(session) if changes else ses_text),{'changed_fixed_via_identifiers':changes,
        'scope':'Protected via identifiers only; original physical stack geometry unchanged'}
