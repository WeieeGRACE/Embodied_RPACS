"""Reproducible oriented-lattice planner and independent convex sweep audit.

No external packages. A straight action's hull is exact. For a pure rotation,
the union of sampled rectangles is enclosed by their convex hull plus
R*(1-cos(dtheta/2)); this safely fills nonconvex sweep regions. The resulting
convex hull is conservative, never a proof of an actual collision when it fails.
Endpoint boxes use X/Y +/-50, not a 50 mm circular dilation.
"""
import argparse, functools, heapq, json, math, random, time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
def read(name): return json.loads((ROOT/'data'/name).read_text(encoding='utf-8'))
M=read('map_v1.json'); C=read('safety.json'); R=math.hypot(320,120)
STEP=100
DIR=[(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1)]

def hull(points):
    pts=sorted(set(points))
    def cross(a,b,c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lo=[]; hi=[]
    for p in pts:
        while len(lo)>1 and cross(lo[-2],lo[-1],p)<=0:lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi)>1 and cross(hi[-2],hi[-1],p)<=0:hi.pop()
        hi.append(p)
    return lo[:-1]+hi[:-1]

def expand(poly,box):
    return hull([(x+sx*box,y+sy*box) for x,y in poly for sx in (-1,1) for sy in (-1,1)]) if box else poly

def rect(x,y,h):
    a=math.radians(h); s,c=math.sin(a),math.cos(a)
    return [(x-f*s+l*c,y+f*c+l*s) for f,l in ((0,-120),(320,-120),(320,120),(0,120))]

def bbox(p):return min(x for x,y in p),min(y for x,y in p),max(x for x,y in p),max(y for x,y in p)

def overlap(a,b):
    # Boundary contact counts as overlap. SAT is exact for convex polygons.
    for poly in (a,b):
        for p,q in zip(poly,poly[1:]+poly[:1]):
            nx,ny=-(q[1]-p[1]),q[0]-p[0]
            ap=[x*nx+y*ny for x,y in a]; bp=[x*nx+y*ny for x,y in b]
            if max(ap)<min(bp)-1e-7 or max(bp)<min(ap)-1e-7:return False
    return True

def pointseg(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];d=dx*dx+dy*dy
    t=0 if d==0 else max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/d))
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)

def distance(a,b):
    if overlap(a,b):return 0
    return min(pointseg(p,q,r) for src,dst in ((a,b),(b,a)) for p in src for q,r in zip(dst,dst[1:]+dst[:1]))

def split_walls():
    """Split every intersection so physical junctions share a random vertex."""
    result=[]
    for w in M['walls']:
        a,b=w['a'],w['b'];vx,vy=b[0]-a[0],b[1]-a[1]; ts={0.,1.}
        for z in M['walls']:
            c,d=z['a'],z['b'];wx,wy=d[0]-c[0],d[1]-c[1];det=vx*wy-vy*wx
            if abs(det)<1e-8:continue
            t=((c[0]-a[0])*wy-(c[1]-a[1])*wx)/det
            u=((c[0]-a[0])*vy-(c[1]-a[1])*vx)/det
            if -1e-6<=t<=1+1e-6 and -1e-6<=u<=1+1e-6:ts.add(round(max(0,min(1,t)),8))
        ts=sorted(ts)
        for i,(t,u) in enumerate(zip(ts,ts[1:])):
            result.append({'id':w['id']+f'.{i}', 'a':tuple(round(a[k]+t*(b[k]-a[k]),3) for k in (0,1)), 'b':tuple(round(a[k]+u*(b[k]-a[k]),3) for k in (0,1))})
    return result

W=split_walls()
def walls(uncertainty=0,offsets=None):
    out=[]
    for w in W:
        a,b=w['a'],w['b']
        if offsets:a=tuple(a[i]+offsets[a][i] for i in (0,1));b=tuple(b[i]+offsets[b][i] for i in (0,1))
        # Square half-thickness safely contains any 3 mm perpendicular slab.
        p=expand([a,b],uncertainty+1.5);out.append((w['id'],p,bbox(p)))
    return out

def directed_endpoint_offsets(vertices, kind):
    """Deterministic wall-end combinations; shared split vertices move together."""
    if kind.startswith('corner_'):
        sx=-50 if kind.endswith('left') else 50
        sy=-50 if '_down' in kind else 50
        return {p:(sx,sy) for p in vertices}
    out={p:(0.0,0.0) for p in vertices}
    for x,y in vertices:
        if kind=='a_side_inward' and abs(x-2400)<1 and y<=600:
            out[(x,y)]=(-50.0,0.0)
        elif kind=='mid_h_tilt' and abs(y-1200)<1 and x>=1200:
            out[(x,y)]=(0.0,-50.0 if x<2100 else 50.0)
        elif kind=='b_junction_shift' and x<=600 and y>=600 and y<=1200:
            out[(x,y)]=(0.0,50.0)
        elif kind=='d_corner_squeeze':
            if abs(x-2400)<1 and 1800<=y<=2400:out[(x,y)]=(-50.0,-50.0 if abs(y-1800)<1 else 0.0)
            elif abs(y-1800)<1 and 1800<=x<=2400:out[(x,y)]=(0.0,-50.0)
        elif kind=='diagonal_x_shift' and 400<=x<=1350 and 1650<=y<=2650:
            out[(x,y)]=(50.0,-50.0)
    return out

def budget():
    position=max(10,C['tracking_position_error_mm'])
    yaw=R*math.sin(math.radians(max(2,C['tracking_yaw_error_deg'])))
    delay=C['initial_speed_mm_s']*(C['range_age_ms']+C['control_latency_ms'])/1000
    return dict(position_mm=position,yaw_corner_mm=yaw,tof_position_mm=C['tof_error_mm'],reaction_travel_mm=delay,brake_mm=C['braking_distance_mm'],required_clearance_mm=C['minimum_clearance_mm'])

BUDGET=budget(); INFLATE=sum(BUDGET.values())

@functools.lru_cache(None)
def local_sweep(h,kind,inflate):
    if kind in (-1,1): # rotation by 45 degrees
        n=math.ceil(45/C['turn_sweep_step_deg']); da=45/n
        p=hull([v for i in range(n+1) for v in rect(0,0,h*45+kind*da*i)])
        err=R*(1-math.cos(math.radians(da/2)))
    else:
        sign=-1 if kind==2 else 1
        dx,dy=DIR[h];p=hull(rect(0,0,h*45)+rect(sign*dx*STEP,sign*dy*STEP,h*45));err=0
    return expand(p,inflate+err)

def moved(poly,x,y):return [(px+x,py+y) for px,py in poly]

def check_poly(poly,ww,clearance=False):
    a,b,c,d=bbox(poly); best=(float('inf'),None)
    for wid,p,(e,f,g,h) in ww:
        broad=c<e or g<a or d<f or h<b
        if not broad and overlap(poly,p):return False,0.,wid
        if clearance:
            q=distance(poly,p)
            if q<best[0]:best=q,wid
    return True,best[0],best[1]

class Planner:
    def __init__(self,uncertainty=0,inflate=INFLATE):self.ww=walls(uncertainty);self.inflate=inflate;self.checked={}
    def edge(self,p,kind):
        key=(p,kind)
        if key not in self.checked:self.checked[key]=check_poly(moved(local_sweep(p[2],kind,self.inflate),p[0],p[1]),self.ww)[0]
        return self.checked[key]
    def plan(self,start,goal):
        def heuristic(p):return math.hypot(p[0]-goal[0],p[1]-goal[1])
        todo=[(heuristic(start),0,start)];cost={start:0};prev={};expanded=0
        while todo:
            _,g,p=heapq.heappop(todo)
            if g!=cost[p]:continue
            expanded+=1
            if heuristic(p)<=60:
                path=[p]
                while p in prev:p=prev[p];path.append(p)
                return list(reversed(path)),expanded
            x,y,h=p
            for kind in (0,2,-1,1):
                dx,dy=DIR[h]
                if kind==0:q=(x+dx*STEP,y+dy*STEP,h)
                elif kind==2:q=(x-dx*STEP,y-dy*STEP,h)
                else:q=(x,y,(h+kind)%8)
                if not(0<q[0]<3000 and 0<q[1]<3000):continue
                ng=g+(STEP*math.hypot(dx,dy) if kind in (0,2) else 250)
                if ng>=cost.get(q,float('inf')) or not self.edge(p,kind):continue
                cost[q]=ng;prev[q]=p;heapq.heappush(todo,(ng+heuristic(q),ng,q))
        return None,expanded

def actions(path):
    for p,q in zip(path,path[1:]):
        if p[2]!=q[2]:kind=1 if (q[2]-p[2])%8==1 else -1
        else:
            dx,dy=DIR[p[2]]
            kind=0 if (q[0]-p[0])*dx+(q[1]-p[1])*dy>0 else 2
        yield p,kind

def route_sweeps(path,inflate=INFLATE):return [moved(local_sweep(p[2],k,inflate),p[0],p[1]) for p,k in actions(path)]

def audit(polys,ww):
    best=(float('inf'),None,None)
    for i,poly in enumerate(polys):
        ok,d,w=check_poly(poly,ww,True)
        if d<best[0]:best=d,i,w
        if not ok:return {'passed':False,'clearance_mm':0,'first_action':i,'wall':w}
    return {'passed':True,'clearance_mm':round(best[0],3),'first_action':best[1],'wall':best[2]}

def preview(routes,name,ww=None,failure=None):
    ww=ww or walls();s=['<svg xmlns="http://www.w3.org/2000/svg" width="900" height="980" viewBox="-100 -100 3200 3500">','<rect width="3000" height="3000" fill="white"/>']
    for wid,p,_ in ww:
        pts=' '.join(f'{x:.1f},{3000-y:.1f}' for x,y in p);s.append(f'<polygon points="{pts}" fill="#986d3e" stroke="#6a4018" stroke-width="5"/><text x="{p[0][0]:.0f}" y="{3000-p[0][1]:.0f}" font-size="34">{wid}</text>')
    for label,(x,y) in M['goals_mm'].items():s.append(f'<circle cx="{x}" cy="{3000-y}" r="22" fill="red"/><text x="{x+30}" y="{3000-y}" font-size="80" fill="red">{label}</text>')
    colors=['#0067bb','#1b854b','#b31c9c','#ef8800','#222']
    for (key,path),col in zip(routes.items(),colors):
        if not path:continue
        pts=' '.join(f'{x},{3000-y}' for x,y,h in path);s.append(f'<polyline points="{pts}" fill="none" stroke="{col}" stroke-width="10"/>')
        for x,y,h in path:
            a=math.radians(h*45);s.append(f'<line x1="{x}" y1="{3000-y}" x2="{x-45*math.sin(a):.1f}" y2="{3000-y-45*math.cos(a):.1f}" stroke="{col}" stroke-width="8"/>')
    if failure:
        pts=' '.join(f'{x},{3000-y}' for x,y in failure);s.append(f'<polygon points="{pts}" fill="red" fill-opacity=".3" stroke="red" stroke-width="5"/>')
    s.append('<text x="0" y="3200" font-size="65">Nominal estimates / 3 mm walls / simulated margins only</text></svg>')
    (ROOT/'records'/name).write_text('\n'.join(s),encoding='utf-8')

def main():
    t=time.monotonic();out=ROOT/'records';out.mkdir(exist_ok=True)
    # Plan the nominal mechanical envelope once, then audit that fixed path
    # with the entire uncertainty/delay/braking budget. Never shrink the
    # acceptance budget to make an unsafe route pass.
    routes={};result={};pl=Planner(inflate=C['minimum_clearance_mm']);rob=Planner(50)
    for key in ['B','C','D','E','PATROL']:
        path=[(2700,300,0)];exp=0
        for goal in ('BCDE' if key=='PATROL' else key):
            sub,n=pl.plan(path[-1],M['goals_mm'][goal]);exp+=n
            if sub is None:path=None;break
            path+=sub[1:]
        routes[key]=path
        if path is None:result[key]={'nominal':'NO_PATH_IN_LATTICE','expanded':exp};print(key,result[key],flush=True);continue
        pp=route_sweeps(path,inflate=C['minimum_clearance_mm'])
        nom=audit(pp,pl.ww)
        robust_pp=route_sweeps(path,inflate=INFLATE)
        envelope=audit(robust_pp,rob.ww)
        result[key]={'nominal':nom,'endpoint_box_envelope':envelope,'actions':len(path)-1,'expanded':exp}
        print(key,result[key],flush=True)
    # Fixed nominal paths in all scenarios: never replan after perturbation.
    polys={k:route_sweeps(p) for k,p in routes.items() if p}
    vertices=sorted({tuple(w[p]) for w in W for p in ('a','b')})
    rng=random.Random(C['random_seed']);worst={};rows=[]
    directed=['corner_left_down','corner_left_up','corner_right_down','corner_right_up',
              'a_side_inward','mid_h_tilt','b_junction_shift','d_corner_squeeze','diagonal_x_shift']
    for i in range(C['random_scenarios']+len(directed)):
        if i<len(directed):
            offsets=directed_endpoint_offsets(vertices,directed[i]);kind=directed[i]
        else:
            offsets={v:(rng.uniform(-50,50),rng.uniform(-50,50)) for v in vertices};kind='random_shared_vertices'
        ww=walls(offsets=offsets);row={'scene':i,'kind':kind,'routes':{}}
        for key,pp in polys.items():
            # early collision test; distances are evaluated for proof/first failure only
            bad=None
            for j,poly in enumerate(pp):
                ok,_,wid=check_poly(poly,ww)
                if not ok:bad=(j,wid);break
            row['routes'][key]={'passed':bad is None,'first_action':None if bad is None else bad[0],'wall':None if bad is None else bad[1]}
            if bad and key not in worst:
                worst[key]=i;preview({key:routes[key]},f'failure_{key}.svg',ww,pp[bad[0]])
        rows.append(row)
    for key in result:
        if key in polys:
            result[key]['random_passes']=sum(r['routes'][key]['passed'] for r in rows[len(directed):])
            result[key]['random_failures']=C['random_scenarios']-result[key]['random_passes']
        result[key]['board_enabled']=False
    summary={'status':'INCOMPLETE_NAVIGATION_VALIDATION','map_version':M['map_version'],'wall_thickness_mm':3,'seed':C['random_seed'],'random_scenarios':C['random_scenarios'],'directed_scenarios':len(directed),'budget_components':BUDGET,'total_pose_motion_inflation_mm':INFLATE,'endpoint_boxes_mm':50,'routes':result,'elapsed_s':round(time.monotonic()-t,2),'limitations':['100 mm XY / 45 degree lattice with forward/reverse straight motion and stationary turns; any no-path result is planner-resolution/model specific, not physical impossibility','Convex rotation hull is a conservative swept enclosure with a chord error bound. A failed enclosure means safety unproven, not necessarily actual contact','All safety parameters except vehicle envelope, start tolerance and wall thickness require board measurement','Directed scenes combine shared wall endpoints for A-side narrowing, B and D junction shifts, a tilted MID_H span and the diagonal X; 1000 random shared-vertex scenarios remain exploratory, while the +/-50 mm envelope decides route release']}
    (out/'routes.json').write_text(json.dumps({'map_version':M['map_version'],'routes':routes},indent=2),encoding='utf-8')
    (out/'geometry_report.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    (out/'scenarios.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
    preview(routes,'routes_preview.svg');preview({},'map_v1_preview.svg')
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
