"""Render the contest scene STL meshes as a top-down surface-height map."""
from pathlib import Path
import re, struct, xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
MJCF = Path('<workspace-legacy>/resources/submission_mac_retest_20260820/20260820_215224_main3660b81/source/goai26-s10-racing/upstream/goai_embodied_future_material/src/S10_sdk_deploy/S10_description/s10_mjcf/mjcf')
SCENE = MJCF / 'scene.xml'
COURSE = Path('<workspace-legacy>/goai26-s10-racing/src/s10_bringup/config/course.yaml')
OUT = ROOT / 'poster_assets/img/course_height_overview.png'

W, H = 2100, 1450
MARGIN = (90, 85, 230, 100)  # left, top, right, bottom
BG = (10, 30, 42)
FONT = '/Library/Fonts/Arial Unicode.ttf'
BOLD = '/System/Library/Fonts/STHeiti Medium.ttc'

# Compact, perceptually ordered blue -> teal -> green -> yellow -> orange palette.
STOPS = [
    (0.00, (28, 55, 102)),
    (0.22, (23, 116, 143)),
    (0.45, (42, 160, 132)),
    (0.68, (147, 197, 81)),
    (0.84, (241, 196, 74)),
    (1.00, (238, 108, 55)),
]

def cmap(t):
    t = max(0.0, min(1.0, float(t)))
    for (a, ca), (b, cb) in zip(STOPS, STOPS[1:]):
        if t <= b:
            u = (t-a)/(b-a)
            return tuple(round(ca[i]*(1-u)+cb[i]*u) for i in range(3))
    return STOPS[-1][1]

def parse_stl(path):
    b = path.read_bytes()
    n = struct.unpack_from('<I', b, 80)[0]
    dt = np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')])
    if 84 + 50*n != len(b):
        raise ValueError(f'Expected binary STL: {path}')
    return np.frombuffer(b, dtype=dt, offset=84, count=n)['v'].astype(np.float64)

root = ET.parse(SCENE).getroot()
assets = {}
for mesh in root.findall('./asset/mesh'):
    scale = np.array([float(x) for x in mesh.attrib.get('scale','1 1 1').split()])
    assets[mesh.attrib['name']] = (MJCF / mesh.attrib['file']).resolve(), scale
body = root.find('./worldbody/body[@name="main_body"]')
body_pos = np.array([float(x) for x in body.attrib.get('pos','0 0 0').split()])
triangles = []
for geom in body.findall('./geom[@type="mesh"]'):
    path, scale = assets[geom.attrib['mesh']]
    v = parse_stl(path) * scale + body_pos
    # Keep projected triangles with non-negligible horizontal area.
    xy1 = v[:,1,:2]-v[:,0,:2]
    xy2 = v[:,2,:2]-v[:,0,:2]
    area2 = np.abs(xy1[:,0]*xy2[:,1]-xy1[:,1]*xy2[:,0])
    v = v[area2 > 1e-7]
    triangles.append(v)
tri = np.concatenate(triangles, axis=0)
mins = tri.min(axis=(0,1)); maxs = tri.max(axis=(0,1))
zmin, zmax = float(mins[2]), float(maxs[2])

# Full map extent with padding and equal x/y scale.
xmin, xmax = float(mins[0])-1.5, float(maxs[0])+1.5
ymin, ymax = -3.0, float(maxs[1])+1.5
plot_w = W-MARGIN[0]-MARGIN[2]; plot_h = H-MARGIN[1]-MARGIN[3]
scale = min(plot_w/(xmax-xmin), plot_h/(ymax-ymin))
actual_w=(xmax-xmin)*scale; actual_h=(ymax-ymin)*scale
x0=MARGIN[0]+(plot_w-actual_w)/2; y0=MARGIN[1]+(plot_h-actual_h)/2

def pix(x,y):
    return (round(x0+(x-xmin)*scale), round(y0+actual_h-(y-ymin)*scale))

img=Image.new('RGB',(W,H),BG)
d=ImageDraw.Draw(img)
# Draw lower triangles first; highest surface remains visible.
order=np.argsort(tri[:,:,2].mean(axis=1))
for idx in order:
    v=tri[idx]
    z=float(v[:,2].mean())
    d.polygon([pix(v[j,0],v[j,1]) for j in range(3)], fill=cmap((z-zmin)/(zmax-zmin)))

# Subtle border and 10 m scale grid.
for gx in np.arange(np.ceil(xmin/10)*10, xmax, 10):
    p1=pix(gx,ymin); p2=pix(gx,ymax)
    d.line([p1,p2],fill=(255,255,255,35),width=1)
for gy in np.arange(np.ceil(ymin/10)*10, ymax, 10):
    p1=pix(xmin,gy); p2=pix(xmax,gy)
    d.line([p1,p2],fill=(255,255,255,35),width=1)

# Parse official waypoints directly from course.yaml.
course=COURSE.read_text()
pts=[]
for m in re.finditer(r'- index:\s*(\d+)\s+position:\s*\[([^\]]+)\]',course):
    i=int(m.group(1)); xyz=[float(x.strip()) for x in m.group(2).split(',')]
    pts.append((i,*xyz))
assert len(pts)==33 and pts[0][0]==0 and pts[-1][0]==32
path=[pix(x,y) for _,x,y,z in pts]
# Route halo and centerline.
d.line(path,fill=(4,15,22),width=16,joint='curve')
d.line(path,fill=(246,249,250),width=8,joint='curve')
# Direction chevrons at selected segments.
for i in [3,8,13,18,23,28]:
    x1,y1=path[i-1]; x2,y2=path[i]
    u=.62; cx=x1*(1-u)+x2*u; cy=y1*(1-u)+y2*u
    vx=x2-x1; vy=y2-y1; n=(vx*vx+vy*vy)**.5 or 1
    vx/=n; vy/=n; px=-vy; py=vx
    tip=(cx+10*vx,cy+10*vy); a=(cx-8*vx+7*px,cy-8*vy+7*py); b=(cx-8*vx-7*px,cy-8*vy-7*py)
    d.polygon([tip,a,b],fill=(246,249,250))

font_s=ImageFont.truetype(FONT,24)
font_b=ImageFont.truetype(BOLD,27)
font_title=ImageFont.truetype(BOLD,36)
font_small=ImageFont.truetype(FONT,21)
key={0:'起点 WP0',7:'WP7',15:'WP15',16:'WP16',18:'WP18',23:'WP23',28:'WP28',32:'终点 WP32'}
label_offsets={
    0:(14,8), 7:(14,12), 15:(-78,16), 16:(14,-42),
    18:(14,-38), 23:(-82,14), 28:(14,26), 32:(14,-42),
}
for i,x,y,z in pts:
    p=pix(x,y); t=(z-min(q[3] for q in pts))/(max(q[3] for q in pts)-min(q[3] for q in pts))
    r=8 if i not in key else 12
    d.ellipse([p[0]-r,p[1]-r,p[0]+r,p[1]+r],fill=cmap(t),outline=(255,255,255),width=3)
    if i in key:
        label=key[i]
        # Manual positions keep the labels apart at the dense upper-platform section.
        dx,dy=label_offsets[i]
        box=d.textbbox((0,0),label,font=font_s); tw=box[2]-box[0]
        lx=p[0]+dx; ly=p[1]+dy
        d.rounded_rectangle([lx-6,ly-3,lx+tw+6,ly+27],radius=5,fill=(8,28,39),outline=(220,236,240),width=1)
        d.text((lx,ly),label,font=font_s,fill='white')

# Title and map facts.
d.rounded_rectangle([40,25,920,82],radius=12,fill=(7,26,37))
d.text((65,34),'GOAI 2026 赛道全览｜颜色表示地表高度',font=font_title,fill='white')
facts='33 航点  ·  水平路线 224.21 m  ·  累计爬升 6.70 m  ·  到点半径 0.20 m'
d.text((65,H-67),facts,font=font_b,fill=(223,238,241))

# Height legend.
cbx=W-150; cby=150; cbh=820; cbw=38
for yy in range(cbh):
    t=1-yy/(cbh-1)
    d.rectangle([cbx,cby+yy,cbx+cbw,cby+yy+1],fill=cmap(t))
d.rectangle([cbx,cby,cbx+cbw,cby+cbh],outline='white',width=2)
d.text((W-205,100),'高度 / m',font=font_b,fill='white')
for val in [zmin,1.0,1.5,2.0,2.5,3.0,zmax]:
    yy=cby+(1-(val-zmin)/(zmax-zmin))*cbh
    d.line([(cbx+cbw,yy),(cbx+cbw+11,yy)],fill='white',width=2)
    d.text((cbx+cbw+16,yy-13),f'{val:.2f}',font=font_small,fill='white')
# North arrow and scale bar.
nx=W-140; ny=1080
d.line([(nx,ny+75),(nx,ny)],fill='white',width=6)
d.polygon([(nx,ny-15),(nx-12,ny+10),(nx+12,ny+10)],fill='white')
d.text((nx-12,ny+82),'N',font=font_b,fill='white')
bar=10*scale; bx=W-215; by=1260
d.line([(bx,by),(bx+bar,by)],fill='white',width=5)
d.line([(bx,by-8),(bx,by+8),(bx+bar,by+8),(bx+bar,by-8)],fill='white',width=3)
d.text((bx+bar/2-28,by+15),'10 m',font=font_small,fill='white')

OUT.parent.mkdir(parents=True,exist_ok=True)
img.save(OUT,quality=95)
print({'output':str(OUT),'triangles':int(len(tri)),'scene_bounds_m':[mins.tolist(),maxs.tolist()],'height_range_m':[zmin,zmax],'waypoints':len(pts),'image_px':[W,H]})
