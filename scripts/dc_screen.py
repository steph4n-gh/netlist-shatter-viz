"""Explicit, user-supplied DC acceptance criteria for native routing jobs."""
import math
from pcb_dc import run

def number(value,label,minimum=None,positive=False):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):raise ValueError(label+' must be a finite number')
    if minimum is not None and (value<minimum or (positive and value==minimum)):raise ValueError(label+' is outside the supported range')

def validate(profile):
    if not isinstance(profile,dict):raise ValueError('DC profile must be an object')
    required={'nets','sources_v','loads_a','voltage_limits_v','copper'}
    if required-set(profile):raise ValueError('DC profile requires '+', '.join(sorted(required-set(profile))))
    if set(profile)-required-{'components','shorted_pins','purpose','returns','plane_mesh','rc_hold'}:raise ValueError('Unknown DC profile field')
    nets=profile['nets']
    if not isinstance(nets,list) or not nets or any(not isinstance(n,str) or not n for n in nets) or len(nets)!=len(set(nets)):raise ValueError('DC nets must be unique nonempty names')
    for field in ('sources_v','loads_a'):
        values=profile[field]
        if not isinstance(values,dict) or not values:raise ValueError(field+' must map terminals to values')
        for terminal,value in values.items():
            if not isinstance(terminal,str) or '.' not in terminal:raise ValueError('Terminals use reference.pin notation')
            number(value,field+'.'+terminal,0)
    if not any(profile['loads_a'].values()):raise ValueError('At least one positive DC load is required')
    if 'returns' in profile:
        returns=profile['returns']
        if not isinstance(returns,dict) or set(returns)!=set(profile['loads_a']) or any(not isinstance(v,str) or '.' not in v for v in returns.values()):raise ValueError('returns must map every loaded terminal to an explicit return terminal')
    if 'plane_mesh' in profile:
        mesh=profile['plane_mesh']
        if not isinstance(mesh,dict) or set(mesh)!={'pitch_mm','voltage_tolerance_v'}:raise ValueError('plane_mesh requires pitch_mm and voltage_tolerance_v')
        number(mesh['pitch_mm'],'Mesh pitch',0,True);number(mesh['voltage_tolerance_v'],'Mesh tolerance',0,True)
        if not .02<=mesh['pitch_mm']<=2:raise ValueError('Mesh pitch must be between 0.02 and 2 mm')
    bounds=profile['voltage_limits_v']
    if not isinstance(bounds,dict) or set(bounds)!=set(profile['loads_a']):raise ValueError('Voltage limits must cover exactly every loaded terminal')
    for terminal,limits in bounds.items():
        if not isinstance(limits,list) or len(limits)!=2:raise ValueError('Voltage limits require [minimum, maximum]')
        for value in limits:number(value,'Voltage limit',0)
        if limits[0]>=limits[1]:raise ValueError('Voltage minimum must be below maximum')
    copper=profile['copper'];keys={'temperature_c','copper_mm','plating_mm','board_mm'}
    if not isinstance(copper,dict) or set(copper)!=keys:raise ValueError('Explicit temperature_c, copper_mm, plating_mm and board_mm are required')
    for key,value in copper.items():number(value,key,-100 if key=='temperature_c' else 0,key!='temperature_c')
    if copper['temperature_c']>200:raise ValueError('DC copper temperature exceeds this model range')
    components=profile.get('components',[])
    if not isinstance(components,list):raise ValueError('components must be a list')
    for c in components:
        if not isinstance(c,dict) or set(c)!={'a','b','ohm','reference'}:raise ValueError('A component requires a, b, ohm and reference')
        if any(not isinstance(c[k],str) or not c[k] for k in ('a','b','reference')):raise ValueError('Component terminal/reference must be a string')
        number(c['ohm'],'Component resistance',0)
    links=profile.get('shorted_pins',[])
    if not isinstance(links,list):raise ValueError('shorted_pins must be a list')
    for pins in links:
        if not isinstance(pins,list) or len(pins)<2 or any(not isinstance(p,str) or '.' not in p for p in pins):raise ValueError('shorted_pins requires lists of terminal names')
    if 'rc_hold' in profile:
        from rc_screen import validate as validate_rc
        validate_rc(profile)
    return profile

def screen(geometry,profile):
    validate(profile)
    result=run(geometry,profile)
    violations=[]
    for terminal,limits in profile['voltage_limits_v'].items():
        value=result['loads'][terminal]['voltage_v']
        if not math.isfinite(value) or not limits[0]<=value<=limits[1]:violations.append({'terminal':terminal,'voltage_v':value,'limits_v':limits})
    for terminal,value in result.get('mesh_check',{}).get('coarse_loads',{}).items():
        limits=profile['voltage_limits_v'][terminal]
        if not limits[0]<=value['voltage_v']<=limits[1]:violations.append({'terminal':terminal,'voltage_v':value['voltage_v'],'limits_v':limits,'mesh_resolution':'coarse'})
    result.update(pass_limits=not violations and result.get('mesh_check',{}).get('pass',True),violations=violations)
    if 'rc_hold' in profile:
        from rc_screen import evaluate
        result['rc_hold']=evaluate(geometry,profile)
        result['pass_limits']=result['pass_limits'] and result['rc_hold']['pass_limits']
        if not result['rc_hold']['pass_limits']:violations.append({'check':'rc_hold','message':'Declared RC voltage limits or numerical convergence failed; inspect rc_hold evidence'})
    return result

def nonregression(before,after):
    """No load loses voltage and no total loss grows, within numerical tolerance.

    Upper and lower limits are mandatory as increasing a supply is not always
    desirable. The retained assumptions must be byte-equivalent JSON values.
    """
    if before['assumptions']!=after['assumptions']:return False
    if 'rc_hold' in before or 'rc_hold' in after:
        from rc_screen import nonregression as rc_nonregression
        if 'rc_hold' not in before or 'rc_hold' not in after or not rc_nonregression(before['rc_hold'],after['rc_hold']):return False
    if not after['pass_limits'] or before['loads'].keys()!=after['loads'].keys():return False
    if 'mesh_check' in before or 'mesh_check' in after:
        if 'mesh_check' not in before or 'mesh_check' not in after:return False
        a,b=before['mesh_check'],after['mesh_check']
        if not b['pass'] or b['coarse_loss_w']>a['coarse_loss_w']+1e-9:return False
        if any(v['voltage_v']<a['coarse_loads'][pin]['voltage_v']-1e-8 for pin,v in b['coarse_loads'].items()):return False
    return (all(math.isfinite(v['voltage_v']) and v['voltage_v']>=before['loads'][pin]['voltage_v']-1e-8 for pin,v in after['loads'].items())
            and math.isfinite(after['loss_w']) and after['loss_w']<=before['loss_w']+1e-9)
