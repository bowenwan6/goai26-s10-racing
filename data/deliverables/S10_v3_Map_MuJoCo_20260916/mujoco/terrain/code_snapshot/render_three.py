"""Offline, unexaggerated geometry/evidence figures and self-contained viewer."""
from pathlib import Path
import sys,json,argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import fontManager
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import plotly.graph_objects as go
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent/'route_structured_assumed_v1/runtime'))
import shapely
from shapely.geometry import shape
fontManager.addfont('/System/Library/Fonts/Supplemental/Arial Unicode.ttf')
plt.rcParams.update({'font.family':'Arial Unicode MS','axes.unicode_minus':False,'font.size':12,'figure.facecolor':'#f6f8fb','axes.facecolor':'#f6f8fb','axes.spines.top':False,'axes.spines.right':False})
COLORS=['#7095ac','#ad8353','#7196a8','#82ae94'];BC='#d9a958'
def parts(g):
    if g.is_empty:return []
    if g.geom_type=='Polygon':return [g]
    return [p for c in getattr(g,'geoms',[]) for p in parts(c)]
def main():
    ap=argparse.ArgumentParser();ap.add_argument('name');out=ROOT/ap.parse_args().name;media=out/'media'
    g=np.load(out/'geometry.npz');v=g['vertices'];f=g['faces'];sid=g['surface_id'];bg=np.load(out/'data/B_geometry_preserved.npz');bv=bg['vertices'];bf=bg['faces']
    report=json.loads((out/'build_report.json').read_text());regs=json.loads((out/'surface_regions.json').read_text())
    def save(fig,name):fig.savefig(media/f'{name}.png',dpi=180,bbox_inches='tight');plt.close(fig)
    def add3(ax,vertices,faces,color):
        if len(faces):ax.add_collection3d(Poly3DCollection(vertices[faces],facecolors=color,linewidth=0,antialiased=False,shade=True,lightsource=matplotlib.colors.LightSource(azdeg=315,altdeg=50)))
    def scene(ax,bounds,elev=25,az=-60):
        (x0,x1),(y0,y1),(z0,z1)=bounds
        cc=v[f].mean(1);cut=(cc[:,0]>=x0)&(cc[:,0]<=x1)&(cc[:,1]>=y0)&(cc[:,1]<=y1)
        for k,c in enumerate(COLORS):add3(ax,v,f[(sid==k)&cut],c)
        bc=bv[bf].mean(1);cutb=(bc[:,0]>=x0)&(bc[:,0]<=x1)&(bc[:,1]>=y0)&(bc[:,1]<=y1);add3(ax,bv,bf[cutb],BC)
        ax.set(xlim=(x0,x1),ylim=(y0,y1),zlim=(z0,z1),xlabel='map X (m)',ylabel='map Y (m)',zlabel='map Z (m)')
        ax.set_box_aspect([x1-x0,y1-y0,z1-z0]);ax.view_init(elev,az);ax.grid(False)
        for axis in (ax.xaxis,ax.yaxis,ax.zaxis):axis.pane.fill=False
    bounds=[(-2,31),(-2,28),(-.8,5.4)]
    fig=plt.figure(figsize=(16,10));ax=fig.add_subplot(111,projection='3d');scene(ax,bounds,30,-66)
    ax.text(2,1,.1,'① Start\n局部分面 + 原样曲边',color='#294c63');ax.text(19,11,3.4,'② 原 B 保留\n踏步、坡面与平台',color='#8f5a0c');ax.text(28,24,5.25,'③ B 后短平台\n4.88 m',color='#2f6650')
    fig.suptitle('本轮交付：Start → 原 B → B 后约 4.9 m',fontsize=22,fontweight='bold',y=.96)
    fig.text(.08,.05,'坐标与高度不夸张。颜色表示区域，不是纹理；裁剪边界 ≠ 实地道路边界。未知区未补成地板。',fontsize=12,color='#526071')
    save(fig,'00_scene_overview')
    fig,ax=plt.subplots(figsize=(13,11))
    for k,c in enumerate(COLORS):ax.add_collection(PolyCollection(v[f[sid==k],:2],facecolor=c,edgecolor='none'))
    ax.add_collection(PolyCollection(bv[bf,:2],facecolor=BC,edgecolor='none'))
    for r in regs:
        for p in parts(shape(r['requested'])):
            xy=np.asarray(p.exterior.coords);ax.plot(*xy.T,'--',lw=1,color='#374151')
        for p in parts(shape(r['unknown'])):ax.fill(*np.asarray(p.exterior.coords).T,color='#ee6072',alpha=.9)
    for xy,txt,label in [([2,1],'① Start\n已确认样板 + 剩余路面',[-.8,7]),([20,12],'② 原 B：完整保留', [6,17]),([28,24],'③ B 后短平台\n只到这条截断线',[17,28])]:
        ax.annotate(txt,xy=xy,xytext=label,fontsize=14,arrowprops=dict(arrowstyle='->',color='#334155'),bbox=dict(boxstyle='round,pad=.6',fc='white',ec='#d1d9e3'))
    for edge in ['protected_entry_edge','protected_exit_edge']:
        aa=np.array(report[edge]);ax.plot(*aa.T,color='#7b3bd1',lw=3)
    ax.plot([-1,4],[-2,-2],lw=4,color='#152637');ax.text(1.5,-2.9,'5 m',ha='center')
    ax.set(xlim=(-3,32),ylim=(-4,30),xlabel='map X (m)',ylabel='map Y (m)',title='范围与分区图｜红色 = 同层观测支持不足，不代表现实有洞')
    ax.set_aspect('equal');ax.grid(alpha=.12);save(fig,'01_scope_chart')
    fig=plt.figure(figsize=(18,7))
    for i,(title,bb,el,az) in enumerate([('Start：分面微坡，保留抬高曲边',[(-1,13.5),(-1.3,4),(-.65,.3)],23,-65),('原 B：没有重新平滑台阶',[(12,28),(-.3,23.5),(-.7,5.4)],24,-45),('B 后：有支持的主平台',[(25,31),(21,27),(4.3,5.15)],28,-60)]):
        ax=fig.add_subplot(1,3,i+1,projection='3d');scene(ax,bb,el,az);ax.set_title(title,fontsize=14)
    fig.suptitle('局部形态检查｜各视窗独立取景，均保持 XYZ 真实比例',fontsize=19);save(fig,'02_region_details')
    # Fixed section points are NOT residual-filtered; whole cropped source arrays remain in data/.
    cloud=np.load(out/'data/Start_sources.npz')['xyz'];pc=np.load(out/'data/Post_B_sources.npz')['xyz']
    triangles=shapely.polygons(v[f,:2]);tree=shapely.STRtree(triangles)
    ns=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]]);off=-np.sum(ns*v[f[:,0]],axis=1)
    def sample_height(xy):
        pairs=tree.query(shapely.points(xy),predicate='intersects');z=np.full(len(xy),np.nan)
        if pairs.shape[1]:z[pairs[0]]=-(np.sum(xy[pairs[0]]*ns[pairs[1],:2],axis=1)+off[pairs[1]])/ns[pairs[1],2]
        return z
    postsrc=np.load(ROOT/'three_sources/post_B.npz');origin=postsrc['origin'];d=postsrc['direction'];l=postsrc['lateral'];ps=(pc[:,:2]-origin)@d;pt=(pc[:,:2]-origin)@l
    fig,axs=plt.subplots(2,2,figsize=(16,9));section_settings=[]
    sections=[('Start 中线 y=0.5 m',cloud[:,0],cloud[:,1]-.5,cloud[:,2],np.linspace(-1,12.7,700),lambda s:np.c_[s,np.full(len(s),.5)],(-.75,.1)),('Start 曲边附近 y=-0.85 m',cloud[:,0],cloud[:,1]+.85,cloud[:,2],np.linspace(3,8,500),lambda s:np.c_[s,np.full(len(s),-.85)],(-.75,.1)),('B 后路线中心',ps,pt,pc[:,2],np.linspace(0,4.88,500),lambda s:origin+s[:,None]*d,(4.25,5.25)),('B 后侧边 cross=-0.95 m',ps,pt+.95,pc[:,2],np.linspace(0,4.88,500),lambda s:origin+s[:,None]*d-.95*l,(4.25,5.25))]
    for ax,(name,ss,cc,zz,s,fn,zlim) in zip(axs.ravel(),sections):
        select=(abs(cc)<.06)&(ss>=s[0])&(ss<=s[-1]);ax.scatter(ss[select],zz[select],s=2,c='#53758f',alpha=.2,rasterized=True,label='全部截面源点（未按残差过滤）')
        ax.plot(s,sample_height(fn(s)),c='#ee7d27',lw=2,label='导出 mesh');ax.set(title=name,xlim=(s[0],s[-1]),ylim=zlim,xlabel='沿截面距离 (m)',ylabel='map Z (m)');ax.grid(alpha=.18)
        section_settings.append(dict(name=name,halfwidth_m=.06,points=int(select.sum()),z_display_window=zlim,outside_z_window=int(np.sum(select&((zz<zlim[0])|(zz>zlim[1]))))))
    axs[0,0].legend(fontsize=9);fig.suptitle('固定截面对照：平整是结构假设，不是“源点已无误差”',fontsize=20)
    fig.tight_layout();save(fig,'03_source_sections');(out/'section_display_settings.json').write_text(json.dumps(section_settings,ensure_ascii=False,indent=2))
    # Side evidence shown in full local plan, including returns above the candidate plane.
    fig,axs=plt.subplots(1,2,figsize=(16,8))
    for ax,r,pts,bb in zip(axs,regs,[cloud,pc],[[-1.2,13.6,-1.4,4.1],[24.8,30.8,20.8,27]]):
        sel=np.arange(0,len(pts),max(1,len(pts)//35000));ax.scatter(pts[sel,0],pts[sel,1],s=1,c=pts[sel,2],cmap='cividis',alpha=.35)
        for p in parts(shape(r['modeled'])):ax.plot(*np.asarray(p.exterior.coords).T,color='#16735c',lw=.7)
        for p in parts(shape(r['unknown'])):ax.fill(*np.asarray(p.exterior.coords).T,color='#ee6072')
        for p in parts(shape(r['requested'])):ax.plot(*np.asarray(p.exterior.coords).T,'--',lw=1,color='#111827')
        ax.set(xlim=bb[:2],ylim=bb[2:],xlabel='map X (m)',ylabel='map Y (m)',title=r['name']+'：源点 + 支持范围');ax.set_aspect('equal');ax.grid(alpha=.15)
    fig.suptitle('边界与未建模区｜黑虚线是截取范围；红色是观测不足的掩码',fontsize=18);fig.text(.1,.03,'B 后侧边抬高结构保留为源点证据，尚未作为可靠障碍物碰撞体；不要把空白区域当作可通行。',fontsize=12);save(fig,'04_support_masks')
    check=json.loads((out/'mujoco_checks.json').read_text());inter=json.loads((out/'interface_checks.json').read_text())
    fig,axs=plt.subplots(1,3,figsize=(17,6));names=['Start','B后'];res=np.array([r['all_low_candidate_abs_z_p50_p95_max_m'][:2] for r in regs])*100;x=np.arange(2)
    axs[0].bar(x-.17,res[:,0],.34,label='P50',color='#7095ac');axs[0].bar(x+.17,res[:,1],.34,label='P95',color='#e5aa58');axs[0].set(xticks=x,xticklabels=names,ylabel='绝对高度偏差 (cm)',title='源点与假设面的偏差');axs[0].legend(loc='upper left');axs[0].text(.03,.78,'非内点专用统计\n包含错层/边缘混合',transform=axs[0].transAxes,va='top',fontsize=9)
    depths=[];labels=[]
    for t in check['tests']:
        if t['collisions']:labels.append(('默认' if t['variant']=='baseline' else '候选')+f' {t["dt"]*1000:.0f}ms');depths.append(-1000*t['deepest_contact_m'])
    axs[1].bar(labels,depths,color=['#de7b77','#de7b77','#72a992','#72a992']);axs[1].axhline(15,c='#4f5661',ls='--',label='预设上限 15mm');axs[1].set(ylabel='最大瞬时穿入 (mm)',title='相同落球条件：保留默认参数失败');axs[1].tick_params(axis='x',rotation=20);axs[1].legend(fontsize=9)
    vals=[z['max_adjacent_difference_m']*1000 for z in inter['B_interfaces']];axs[2].bar(['Start→B','B→短平台'],vals,color='#8a77b5');axs[2].axhline(2,c='#4f5661',ls='--',label='预设上限 2mm');axs[2].set(ylabel='相邻射线高度差 (mm)',title='接缝两侧相距 4mm；41点/处');axs[2].legend(fontsize=9)
    fig.suptitle('检查仪表板｜左：数据符合程度；中/右：仿真实现检查；二者不能互相替代',fontsize=17);fig.tight_layout();save(fig,'05_quality_dashboard')
    fig,ax=plt.subplots(figsize=(16,6));ax.set(xlim=(0,16),ylim=(0,6));ax.axis('off')
    boxes=[(2,3.7,'保存帧 + 既有位姿\n原始文件只读','#dfe9f1'),(6,3.7,'区域/边界分类\n平面 · 曲边 · 台阶 · 未知','#dfeee8'),(10,3.7,'可视化 mesh\n原 B + Start 样板不变','#f6e9d2'),(14,3.7,'有限凸碰撞体\n不把整图变成单一凸包','#e9e0f2'),(10,1.5,'源点/截面对照\n公开残差与未建模区','#f4e5d5'),(14,1.5,'MuJoCo 实测\n射线 + 落球 + 导轨轮','#dce9ef')]
    for xx,yy,label,c in boxes:ax.text(xx,yy,label,ha='center',va='center',fontsize=13,bbox=dict(boxstyle='round,pad=.8',fc=c,ec='#80909e'))
    for a,b in [((3.5,3.7),(4.4,3.7)),((7.6,3.7),(8.4,3.7)),((11.6,3.7),(12.4,3.7)),((10,2.95),(10,2.25)),((14,2.95),(14,2.25))]:ax.annotate('',xy=b,xytext=a,arrowprops=dict(arrowstyle='->',lw=2,color='#536777'))
    ax.text(8,.35,'停止线：未做现场尺寸验收、未运行机器狗 policy/follower、未修改真机',ha='center',color='#ad4848',fontsize=14);ax.set_title('本轮处理链与验收边界',fontsize=23,pad=18);save(fig,'06_pipeline_diagram')
    # Self-contained Plotly scene. Source overlay and wireframe begin hidden.
    fig=go.Figure()
    for k,name in enumerate(['Start 已确认样板','Start 抬高曲边','Start 其余路面','B 后短平台']):
        ff=f[sid==k];fig.add_trace(go.Mesh3d(x=v[:,0],y=v[:,1],z=v[:,2],i=ff[:,0],j=ff[:,1],k=ff[:,2],name=name,color=COLORS[k],flatshading=True,showlegend=True,lighting=dict(ambient=.7,diffuse=.5,specular=.08),hovertemplate=name+'<br>X=%{x:.2f} Y=%{y:.2f} Z=%{z:.2f}<extra></extra>'))
    fig.add_trace(go.Mesh3d(x=bv[:,0],y=bv[:,1],z=bv[:,2],i=bf[:,0],j=bf[:,1],k=bf[:,2],name='原 B（字节保留）',color=BC,flatshading=True,showlegend=True))
    for name,pts in [('Start 全高度源点',cloud),('B 后全高度源点',pc)]:
        take=pts[::max(1,len(pts)//30000)];fig.add_trace(go.Scatter3d(x=take[:,0],y=take[:,1],z=take[:,2],mode='markers',marker=dict(size=1,color='#344c63',opacity=.35),name=name,visible='legendonly'))
    for r in regs:
        for j,p in enumerate(parts(shape(r['unknown']))):
            xy=np.asarray(p.exterior.coords);zz=np.full(len(xy),-.33 if r['name']=='Start' else 4.62)
            fig.add_trace(go.Scatter3d(x=xy[:,0],y=xy[:,1],z=zz,mode='lines',line=dict(color='#f05265',width=3),legendgroup='unknown',showlegend=j==0,name=r['name']+' 未知掩码（仅标注）',hoverinfo='name'))
    edges=v[f[:,[0,1,2,0]]];e=np.concatenate([edges,np.full((len(edges),1,3),np.nan)],axis=1).reshape(-1,3)
    fig.add_trace(go.Scatter3d(x=e[:,0],y=e[:,1],z=e[:,2],mode='lines',line=dict(color='#435261',width=1),name='新表面线框',visible='legendonly',hoverinfo='skip'))
    buttons=[]
    for name,bb in [('全段',bounds),('Start',[(-1.5,13.7),(-1.5,4.3),(-.75,.5)]),('B',[(11.5,28),(-.5,23.5),(-.8,5.5)]),('B后',[(24.8,31),(20.8,27),(4.1,5.4)])]:
        buttons.append(dict(label=name,method='relayout',args=[{'scene.xaxis.range':bb[0],'scene.yaxis.range':bb[1],'scene.zaxis.range':bb[2],'scene.camera':dict(eye=dict(x=1.4,y=-1.7,z=1.1))}]))
    fig.update_layout(title='Start → 原 B → B 后短平台 · 结构假设模型',height=820,paper_bgcolor='#f6f8fb',margin=dict(l=0,r=0,t=100,b=0),legend=dict(x=.01,y=.95,bgcolor='rgba(255,255,255,.85)'),scene=dict(xaxis=dict(title='map X / m',range=bounds[0]),yaxis=dict(title='map Y / m',range=bounds[1]),zaxis=dict(title='map Z / m',range=bounds[2]),aspectmode='data',camera=dict(eye=dict(x=1.4,y=-1.7,z=1.1))),updatemenus=[dict(type='buttons',direction='right',buttons=buttons,x=.98,xanchor='right',y=1.08)])
    html=fig.to_html(include_plotlyjs=True,full_html=False,config={'displaylogo':False,'scrollZoom':True})
    (out/'interactive.html').write_text('<!doctype html><html lang="zh"><meta charset="utf-8"><title>三段精细模型</title><style>body{margin:0;background:#f6f8fb;font:16px system-ui;color:#203246}header{padding:20px 30px}a{color:#166b9d}.note{background:#fff3dc;padding:14px;border-radius:8px}</style><header><a href="index.html">← 交付总览 / 视频 / 报告</a><h2>三段精细模型 · 可离线旋转查看</h2><p>拖动旋转、滚轮缩放；点击图例开关源点/线框。红色只是未知区域标注，不是碰撞地板。</p><p class="note">默认只看当前地表视窗；源点层保留全高度，视窗外的源点没有删除。没有扩展全场；真实台阶与曲边没有整体抹平。B 后侧边抬高障碍仍是未建模证据。</p></header>'+html+'</html>')
    print('Geometry images, chart/diagram and offline viewer complete',flush=True)
if __name__=='__main__':main()
