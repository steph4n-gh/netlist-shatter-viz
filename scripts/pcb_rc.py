"""Passive RC reservoir response of extracted PCB copper during a declared current hold.

This is not a regulator control-loop, switching, inductance or EMI simulation.
"""
import math
from collections import defaultdict,deque
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import splu
from scipy.linalg import cho_factor,cho_solve

def reduce_ports(network,terminals,ports,ground):
    """Exact resistor-network Schur reduction; no artificial grounding resistors."""
    if len(ports)!=len(set(ports)) or ground in ports:raise ValueError('Ports must be unique and exclude ground')
    roots={p:network.root(terminals[p]) for p in [*ports,ground]}
    boundary=list(dict.fromkeys(roots.values()));boundary_set=set(boundary)
    adjacency=defaultdict(set);edges=[]
    for a,b,r,_ in network.edges:
        a,b=network.root(a),network.root(b)
        if a==b:continue
        if not math.isfinite(r) or r<=0:raise ValueError('Finite positive resistance required')
        edges.append((a,b,r));adjacency[a].add(b);adjacency[b].add(a)
    reached=set(boundary);todo=deque(boundary)
    while todo:
        for n in adjacency[todo.popleft()]:
            if n not in reached:reached.add(n);todo.append(n)
    interior=list(reached-boundary_set);nodes=boundary+interior;idx={n:i for i,n in enumerate(nodes)}
    rows=[];cols=[];values=[]
    for a,b,r in edges:
        if a not in reached:continue
        i,j=idx[a],idx[b]
        rows.extend((i,j,i,j));cols.extend((i,j,j,i));values.extend((1/r,1/r,-1/r,-1/r))
    matrix=coo_matrix((values,(rows,cols)),shape=(len(nodes),len(nodes))).tocsc();nb=len(boundary)
    reduced=matrix[:nb,:nb].toarray()
    if interior:
        coupling=matrix[nb:,:nb].toarray();factor=splu(matrix[nb:,nb:]);solution=factor.solve(coupling)
        residual=float(np.max(np.abs(matrix[nb:,nb:]@solution-coupling)))
        if residual>1e-6:raise ValueError('Port reduction residual too large')
        reduced-=matrix[:nb,nb:]@solution
    else:residual=0.
    asym=float(np.max(abs(reduced-reduced.T)));reduced=(reduced+reduced.T)/2
    # Preserve KCL exactly after cancellation of large sheet conductances.
    row_error=float(np.max(abs(reduced.sum(axis=1))))
    if asym>1e-6 or row_error>1e-5:raise ValueError('Port reduction lost symmetry or KCL')
    reduced-=np.diag(reduced.sum(axis=1))
    ground_i=idx[roots[ground]];keep=[i for i in range(nb) if i!=ground_i]
    reduced=reduced[np.ix_(keep,keep)];mapping={p:keep.index(idx[r]) if idx[r]!=ground_i else None for p,r in roots.items()}
    if np.linalg.eigvalsh(reduced).min(initial=0)<-1e-6:raise ValueError('Non-passive port reduction')
    return {'conductance_s':reduced.tolist(),'terminal_index':mapping,'full_nodes':len(nodes),'port_nodes':len(keep),'schur_residual':residual,'kcl_roundoff_s':row_error}

def reservoir(model,capacitors,source,loads_before,loads_after,voltage_v,duration_s,step_s):
    """Hold regulator output current at its pre-step value for the entire window.

    Loads/capacitors are explicit differential terminal pairs. Capacitances are
    effective operating values; ESR is explicit. Copper inductance is omitted.
    """
    if not all(math.isfinite(v) and v>0 for v in (voltage_v,duration_s,step_s)):raise ValueError('Invalid time/voltage')
    if duration_s/step_s>100000:raise ValueError('Too many time steps')
    mapping=model['terminal_index'];base=np.array(model['conductance_s'],float);n=len(base);count=len(capacitors)
    g=np.zeros((n+count,n+count));g[:n,:n]=base;c=np.zeros_like(g);vectors=[]
    def vector(a,b,size):
        v=np.zeros(size)
        for pin,sign in ((a,1),(b,-1)):
            k=mapping[pin]
            if k is not None:v[k]+=sign
        return v
    for j,cap in enumerate(capacitors):
        cv,esr=cap['farad'],cap['esr_ohm']
        if not math.isfinite(cv) or cv<=0 or not math.isfinite(esr) or esr<=0:raise ValueError('Positive finite C and ESR required')
        positive=mapping[cap['a']];negative=mapping[cap['b']]
        series=np.zeros(n+count);series[n+j]=1
        if positive is not None:series[positive]-=1
        g+=np.outer(series,series)/esr
        branch=np.zeros(n+count);branch[n+j]=1
        if negative is not None:branch[negative]-=1
        vectors.append(branch);c+=cv*np.outer(branch,branch)
    def injection(loads):
        b=np.zeros(n+count)
        for load in loads:
            if not math.isfinite(load['amp']) or load['amp']<0:raise ValueError('Invalid load current')
            b-=load['amp']*vector(load['a'],load['b'],n+count)
        return b
    src=vector(source[0],source[1],n+count)
    before=injection(loads_before)
    # Initial DC operating point, with an ideal source enforcing the setpoint.
    initial=np.block([[g,src[:,None]],[src[None,:],np.zeros((1,1))]])
    answer=np.linalg.solve(initial,np.r_[before,voltage_v]);v0=answer[:-1];source_a=-float(answer[-1])
    after=injection(loads_after)+source_a*src
    bc=np.array(vectors).T
    # Capacitor voltages are continuous; algebraic load-port jumps include ESR.
    instant=np.block([[g,bc],[bc.T,np.zeros((count,count))]])
    v=np.linalg.solve(instant,np.r_[after,bc.T@v0])[:n+count]
    measured={load['a']:vector(load['a'],load['b'],n+count) for load in [*loads_before,*loads_after]}
    measured['source']=src
    ratio=duration_s/step_s
    steps=max(1,round(ratio) if math.isclose(ratio,round(ratio),rel_tol=1e-13) else math.ceil(ratio));dt=duration_s/steps
    factor=cho_factor(g+c/dt);history=[{'time_s':0.,**{k:float(w@v) for k,w in measured.items()}}]
    maximum_residual=0.
    for i in range(steps):
        old=v;v=cho_solve(factor,after+c@old/dt)
        maximum_residual=max(maximum_residual,float(np.max(abs(g@v+c@(v-old)/dt-after))))
        history.append({'time_s':(i+1)*dt,**{k:float(w@v) for k,w in measured.items()}})
    if maximum_residual>1e-6:raise ValueError('Transient KCL residual too large')
    return {'history':history,'initial_v':{k:float(w@v0) for k,w in measured.items()},'source_hold_a':source_a,'step_s':dt,'duration_s':duration_s,'max_kcl_residual_a':maximum_residual,'minimum_v':min(row[k] for row in history for k in measured),'maximum_v':max(row[k] for row in history for k in measured)}
