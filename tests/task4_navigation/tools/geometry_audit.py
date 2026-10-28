#!/usr/bin/env python3
"""Conservative task4 map preview and initial-turn sweep audit (stdlib only).

This intentionally does not invent a route where the image and route book do
not establish traversable centerlines. It tests the mandatory first in-place
turn at A under the specified body envelope and perturbation model.
"""
import json, math, random
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAP = json.loads((ROOT / 'data/map_v1.json').read_text(encoding='utf-8'))
OUT = ROOT / 'records'
OUT.mkdir(exist_ok=True)
WALLS = MAP['walls']
START = MAP['assumptions']['start_xy_mm']
RADIUS = math.hypot(MAP['assumptions']['body_front_mm'], MAP['assumptions']['body_half_width_mm'])
THICK = MAP['assumptions']['wall_thickness_mm'] / 2
U = MAP['assumptions']['wall_endpoint_xy_uncertainty_mm']
POSE = MAP['assumptions']['start_xy_error_mm'] + RADIUS * math.sin(math.radians(MAP['assumptions']['start_yaw_error_mdeg']/1000))

def point_seg(p, a, b):
    vx,vy=b[0]-a[0],b[1]-a[1]
    d=vx*vx+vy*vy
    t=0 if d==0 else max(0,min(1,((p[0]-a[0])*vx+(p[1]-a[1])*vy)/d))
    return math.hypot(p[0]-(a[0]+t*vx),p[1]-(a[1]+t*vy))

def perturbed_walls(rng, extreme=None):
    # Coordinate-keyed perturbations preserve shared corners/junctions.
    coords={tuple(p) for w in WALLS for p in (w['a'],w['b'])}
    offsets={}
    for p in coords:
        if extreme is None: offsets[p]=(rng.uniform(-U,U),rng.uniform(-U,U))
        else: offsets[p]=extreme
    result=[]
    for w in WALLS:
        a,b=w['a'],w['b']
        result.append((w['id'],(a[0]+offsets[tuple(a)][0],a[1]+offsets[tuple(a)][1]),(b[0]+offsets[tuple(b)][0],b[1]+offsets[tuple(b)][1])))
    return result

def evaluate(walls):
    required=RADIUS+THICK+POSE
    clearance=min(point_seg(START,a,b)-THICK-POSE for _,a,b in walls)
    hit=min((point_seg(START,a,b)-THICK-POSE, wid, a,b) for wid,a,b in walls)
    return {'required_radius_mm':round(required,2),'available_clearance_mm':round(clearance,2),'pass':clearance>RADIUS,'nearest_wall':hit[1], 'signed_margin_mm':round(clearance-RADIUS,2)}

def svg():
    s=['<svg xmlns="http://www.w3.org/2000/svg" width="720" height="720" viewBox="-80 -80 3160 3160">','<rect x="0" y="0" width="3000" height="3000" fill="#fff" stroke="#333" stroke-width="28"/>']
    for w in WALLS:
        a,b=w['a'],w['b']; s.append(f'<line x1="{a[0]}" y1="{3000-a[1]}" x2="{b[0]}" y2="{3000-b[1]}" stroke="#8b5a2b" stroke-width="30"/>')
        mx,my=(a[0]+b[0])/2,3000-(a[1]+b[1])/2
        s.append(f'<text x="{mx}" y="{my}" font-size="42" fill="#333">{w["id"]}</text>')
    for name,(x,y) in MAP['goals_mm'].items():
        s.append(f'<circle cx="{x}" cy="{3000-y}" r="35" fill="#d22"/><text x="{x+35}" y="{2990-y}" font-size="100" fill="#d22">{name}</text>')
    r=RADIUS+THICK+POSE
    s.append(f'<circle cx="{START[0]}" cy="{3000-START[1]}" r="{r:.1f}" fill="#f00" fill-opacity=".15" stroke="#d00" stroke-width="8"/>')
    s.append('<text x="80" y="2900" font-size="65" fill="#900">A first 360-degree turn sweep (conservative disk)</text></svg>')
    (OUT/'map_v1_preview.svg').write_text('\n'.join(s),encoding='utf-8')

def main():
    rng=random.Random(0x5441534B34)
    nominal=evaluate(perturbed_walls(rng, (0,0)))
    trials=[evaluate(perturbed_walls(rng)) for _ in range(1000)]
    passed=sum(t['pass'] for t in trials)
    report={'seed':'0x5441534B34','random_scenarios':len(trials),'body_corner_radius_mm':round(RADIUS,2),'pose_inflation_mm':round(POSE,2),'wall_half_thickness_assumption_mm':THICK,'endpoint_xy_perturbation_mm':U,'nominal_first_turn':nominal,'random_first_turn_passes':passed,'random_first_turn_collisions':len(trials)-passed,'minimum_signed_margin_mm':round(min(t['signed_margin_mm'] for t in trials),2),'maximum_signed_margin_mm':round(max(t['signed_margin_mm'] for t in trials),2),'route_release':'NOT PROVEN / NOT RELEASED: route centerlines, measured braking distance, ToF error and survey-grade wall endpoints are not available. The A-point full in-place turn fails this geometry model; this does not prove every route must turn at A or is physically impossible.'}
    (OUT/'first_turn_geometry_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    lines=['# 任务四几何审计结果','',f"- 地图：`{MAP['map_version']}`（图片标注估算，非实测）",f"- 车体旋转角点半径：{RADIUS:.2f} mm",f"- 墙厚半值假设：{THICK:.1f} mm；端点扰动：±{U} mm/坐标轴",f"- 起步位置/航向误差膨胀：{POSE:.2f} mm",f"- 固定随机种子：`{report['seed']}`，场景数：{len(trials)}",f"- 名义 A 点原地旋转：{'通过' if nominal['pass'] else '失败'}；最近墙段 `{nominal['nearest_wall']}`，有符号净空余量 {nominal['signed_margin_mm']:.2f} mm",f"- 扰动统计：{passed}/1000 通过，{len(trials)-passed}/1000 失败；最差余量 {report['minimum_signed_margin_mm']:.2f} mm",'',f"**路线结论：尚未证明安全，暂不放行上板。** {report['route_release']}",'','此审计只检查 A 点中心的全向原地旋转扫掠，不包含转弯前的平移退出、沿路线扫掠、制动滑行或其他目标点。因此不能推出完整路线必然碰撞或物理不可达。未知制动距离、ToF/定位误差未伪装为零；完成这些参数和逐路段检查前不能作路线安全结论。']
    (OUT/'GEOMETRY_REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    svg(); print('\n'.join(lines))

if __name__=='__main__': main()
