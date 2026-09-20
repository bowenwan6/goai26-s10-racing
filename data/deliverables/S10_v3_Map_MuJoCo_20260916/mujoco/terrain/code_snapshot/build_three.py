"""Bounded Start + byte-preserved B + <=5 m post-B; no robot/network code."""
from pathlib import Path
import sys,json,hashlib,argparse,shutil,copy,time
import numpy as np
from scipy.spatial import cKDTree
import open3d as o3d
ROOT=Path(__file__).resolve().parents[1];MAP=ROOT.parents[1]
sys.path.insert(0,str(ROOT.parent/'route_structured_assumed_v1/runtime'))
import shapely
from shapely.geometry import Polygon,LineString,box,mapping,shape,Point
from shapely.ops import unary_union
import mapbox_earcut
from extract_features import plane_fit
from export_precise_obj import export as objwrite
import xml.etree.ElementTree as ET

def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def dump(p,v):
    with p.open('x') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
def parts(g):
    if g.is_empty:return []
    if g.geom_type=='Polygon':return [g]
    return [p for sub in getattr(g,'geoms',[]) for p in parts(sub)]
def smooth(t):
    t=np.clip(t,0,1);return t*t*(3-2*t)
def xml(path,root):
    ET.indent(root);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
def force_plane_at_edge(c,edge,source):
    a,b=edge;e=(b-a)/np.linalg.norm(b-a);n=np.array([-e[1],e[0]])
    side=(source[:,:2]-a)@n;zref=source[:,:2]@c[:2]+c[2]
    r=source[:,2]-zref;take=(abs(r)<.2)&(abs(side)>.2)
    k=np.sum(side[take]*r[take])/np.sum(side[take]**2)
    return np.r_[c[:2]+k*n,c[2]-k*(a@n)]

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--name',default='three_regions_01');args=ap.parse_args();out=ROOT/args.name
    out.mkdir(exist_ok=False)
    for sub in ['visual','collision','media','data']: (out/sub).mkdir()
    started=time.monotonic();sample=ROOT/'sample_05';B=ROOT.parent/'B_structured_repair_v1/candidate_05'
    source=np.load(ROOT/'sources/Start_saved_frames.npz');q=source['xyz'];fid=source['frame'];poses=source['poses']
    post=np.load(ROOT/'three_sources/post_B.npz');pq=post['xyz'];pfid=post['frame'];origin=post['origin'];direction=post['direction'];lateral=post['lateral']
    sg=np.load(sample/'geometry.npz');sample_plane=sg['plane'];sample_box=box(3,-1.1,8,1.4)
    bp=[p for p in json.loads((B/'repair_report.json').read_text())['surface_candidates'] if 'footprint_map_xy' in p]
    bpolys=[Polygon(p['footprint_map_xy']) for p in bp];bunion=unary_union(bpolys)
    entry=np.array(bp[0]['footprint_map_xy'])[:2];exit_edge=np.array(bp[-1]['footprint_map_xy'])[:2]
    # Scope cuts are straight/dashed computational limits, never surveyed curbs.
    low,high=entry[np.argmin(entry[:,1])],entry[np.argmax(entry[:,1])]
    start_domain=Polygon([[-1,-1.1],[10.5,-1.1],low,high,[10.5,3.8],[-1,3.8]]).difference(bunion)
    seeds=[('W_s',(-1,3,-1.1,1.4)),('W_n',(-1,3,1.4,3.8)),('M_n',(3,8,1.4,3.8)),('E_s',(8,12,-1.1,1.4)),('E_n',(8,12,1.4,3.8))]
    planes={'M_s':sample_plane};fits=[]
    for name,(x0,x1,y0,y1) in seeds:
        take=(q[:,0]>x0)&(q[:,0]<x1)&(q[:,1]>y0)&(q[:,1]<y1)&(q[:,2]>-.8)&(q[:,2]<-.12)
        c=plane_fit(q[take]);planes[name]=c;rr=q[take,2]-q[take,:2]@c[:2]-c[2]
        fits.append(dict(name=name,plane=c.tolist(),source_points=int(take.sum()),all_candidate_abs_z_p50_p95_max_m=np.percentile(abs(rr),[50,95,100]).tolist(),seed_bounds=[x0,x1,y0,y1]))
    fits.append(dict(name='M_s',plane=sample_plane.tolist(),source='accepted sample_05',unchanged=True))
    # Plane mixtures exist ONLY within explicitly declared transition strips.
    # The sample is not modified; every adjacent side evaluates to its plane.
    def start_height(xy):
        x,y=xy.T;hs=xy@sample_plane[:2]+sample_plane[2];hn=xy@planes['M_n'][:2]+planes['M_n'][2]
        w=smooth((3-x)/1.);es=smooth((x-8)/1.)
        hs=(1-w)*hs+w*(xy@planes['W_s'][:2]+planes['W_s'][2]);hn=(1-w)*hn+w*(xy@planes['W_n'][:2]+planes['W_n'][2])
        hs=(1-es)*hs+es*(xy@planes['E_s'][:2]+planes['E_s'][2]);hn=(1-es)*hn+es*(xy@planes['E_n'][:2]+planes['E_n'][2])
        north=smooth((y-1.4)/.75);z=(1-north)*hs+north*hn
        # Local, finite B entry patch; does not pull an entire infinite line.
        e=(entry[1]-entry[0])/np.linalg.norm(entry[1]-entry[0]);u=(xy-entry[0])@e
        dist=np.abs(np.cross(np.tile(e,(len(xy),1)),xy-entry[0]))
        near=(u>=-.001)&(u<=np.linalg.norm(entry[1]-entry[0])+.001)
        wb=(1-smooth(dist/.8))*near;c=np.array(bp[0]['plane'])
        return z*(1-wb)+(xy@c[:2]+c[2])*wb
    # Post-B is one bounded plane fitted with the original exit edge as a hard
    # height constraint. This avoids changing B or adding an arbitrary slope gap.
    st=post['station'];ct=post['cross'];pc=np.array(bp[-1]['plane'])
    core=(st>.2)&(st<4.7)&(abs(ct)<.65)&(pq[:,2]>4.3)&(pq[:,2]<4.85)
    pc=force_plane_at_edge(pc,exit_edge,pq[core])
    ec=exit_edge.mean(0);ed=(exit_edge[1]-exit_edge[0]);ed/=np.linalg.norm(ed)
    pn=np.array([-ed[1],ed[0]])
    if pn@direction<0:pn=-pn
    scope=json.loads((ROOT/'scope.json').read_text());stop=np.array(scope['post_B_stop_map_xy'])
    length=float((stop-ec)@pn);half=np.linalg.norm(exit_edge[1]-exit_edge[0])/2
    # Follow the saved route endpoint, not only the exit edge normal. The
    # footprint is an oblique strip; its outer sides remain scope cuts.
    post_shift=stop-ec
    post_domain=Polygon([exit_edge[0],exit_edge[1],exit_edge[1]+post_shift,exit_edge[0]+post_shift]).difference(bunion)
    def post_height(xy):return xy@pc[:2]+pc[2]
    # Record geometry of a persistent raised side as UNKNOWN/obstacle evidence.
    # Do not invent its material or flatten it into the main platform.
    post_res=pq[:,2]-post_height(pq[:,:2]);post_raised=(st>2.3)&(st<5.1)&(ct<-.72)&(post_res>.11)&(post_res<.55)
    raised_xy=pq[post_raised,:2]
    side_obstacle=None
    if len(raised_xy)>100:
        # A source-supported conservative side band; not a measured curb face.
        ss=(pq[post_raised,:2]-ec)@pn;tt=(pq[post_raised,:2]-ec)@ed
        profile=[]
        for s in np.arange(2.3,length,.1):
            sel=abs(ss-s)<.1
            if sel.sum()>=8:profile.append([float(s),float(np.percentile(tt[sel],10)),int(sel.sum())])
        dump(out/'data/post_raised_evidence.json',dict(points=int(post_raised.sum()),frames=np.unique(pfid[post_raised]).tolist(),profiles=profile,material='unknown',modeled=False))
    # Observation support affects modeled extent, not the recovered feature
    # shapes. No high-return XY projection is used to punch the floor.
    masks=[];domain_records=[];domains=[];support_records=[]
    for name,dom,cloud,frames,height,zlimits in [('Start',start_domain,q,fid,start_height,(-.8,-.12)),('Post_B',post_domain,pq,pfid,post_height,(4.25,4.9))]:
        inside=shapely.contains_xy(dom,cloud[:,0],cloud[:,1]);res=cloud[:,2]-height(cloud[:,:2]);same=inside&(abs(res)<.08)
        tree=cKDTree(cloud[same,:2]);x0,y0,x1,y1=dom.bounds;step=.05
        xx,yy=np.meshgrid(np.arange(x0+step/2,x1,step),np.arange(y0+step/2,y1,step));xy=np.c_[xx.ravel(),yy.ravel()];valid=shapely.contains_xy(dom,xy[:,0],xy[:,1]);xy=xy[valid]
        dist=tree.query(xy,workers=2)[0]
        bad=xy[dist>.05];holes=unary_union([box(x-step/2,y-step/2,x+step/2,y+step/2) for x,y in bad]).intersection(dom)
        holes=holes.buffer(.0001,quad_segs=2).intersection(dom)
        if name=='Start':holes=holes.difference(sample_box)
        allowed=dom.difference(holes)
        if name=='Start':allowed=allowed.difference(sample_box)
        domains.append(allowed);masks.append(holes)
        low=inside&(cloud[:,2]>zlimits[0])&(cloud[:,2]<zlimits[1])
        domain_records.append(dict(name=name,requested=mapping(dom),modeled=mapping(allowed),unknown=mapping(holes),
            requested_area_m2=dom.area,modeled_area_without_sample_m2=allowed.area,unknown_area_m2=holes.area,
            source_points=int(inside.sum()),same_layer_points=int(same.sum()),candidate_points=int(low.sum()),
            all_low_candidate_abs_z_p50_p95_max_m=np.percentile(abs(res[low]),[50,95,100]).tolist(),
            source_frame_ids=np.unique(frames[inside]).tolist(),boundary_type='explicit crop plus measured support, not surveyed pavement edge'))
        np.savez_compressed(out/f'data/{name}_sources.npz',xyz=cloud[inside],frame=frames[inside],residual=res[inside],same_layer=same[inside])
        np.savez_compressed(out/f'data/{name}_support.npz',xy=xy,nearest_xy_m=dist,unknown_threshold_m=np.array(.05))
    # New visual top surfaces; sample geometric data copied without movement.
    vertices=sg['vertices'].tolist();faces=sg['faces'].tolist();kinds=sg['face_classes'].tolist();index={tuple(np.round(p,9)):i for i,p in enumerate(vertices)}
    def vi(p):
        key=tuple(np.round(p,9))
        if key not in index:index[key]=len(vertices);vertices.append(p.tolist())
        return index[key]
    # 20 cm tessellation inside known smooth/planar patches, fine boundary
    # vertices retained. This is not a 20 cm reconstructed footprint.
    for dom,height,tag in [(domains[0],start_height,2),(domains[1],post_height,3)]:
        x0,y0,x1,y1=dom.bounds;step=.20
        for x in np.arange(np.floor(x0/step)*step,x1,step):
            for y in np.arange(np.floor(y0/step)*step,y1,step):
                for p in parts(dom.intersection(box(x,y,x+step,y+step))):
                    if p.area<1e-9:continue
                    rings=[np.array(p.exterior.coords)[:-1]]+[np.array(h.coords)[:-1] for h in p.interiors]
                    xy=np.vstack(rings);ends=np.cumsum([len(r) for r in rings]).astype(np.uint32);tri=mapbox_earcut.triangulate_float64(xy,ends).reshape(-1,3);zz=height(xy)
                    for t in tri:
                        a,b=xy[t[1]]-xy[t[0]],xy[t[2]]-xy[t[0]];area=a[0]*b[1]-a[1]*b[0]
                        if abs(area)<1e-9:continue
                        if area<0:t=t[[0,2,1]]
                        faces.append([vi(np.r_[xy[j],zz[j]]) for j in t]);kinds.append(tag)
    v=np.array(vertices);f=np.array(faces);tag=np.array(kinds)
    # Keep the accepted sample's faces/coordinates exact; outside mesh interfaces
    # may contain T-junctions but represent the identical height. Separate actual
    # interface-ray checks are required before use as a collision scene.
    cross=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
    assert np.isfinite(v).all() and np.min(cross[:,2])>0
    poly=shapely.polygons(v[f,:2]);pairs=shapely.STRtree(poly).query(poly,predicate='intersects');pairs=pairs[:,pairs[0]<pairs[1]]
    overlap=shapely.area(shapely.intersection(poly[pairs[0]],poly[pairs[1]]));assert np.max(overlap,initial=0)<1e-8
    np.savez_compressed(out/'geometry.npz',vertices=v,faces=f,surface_id=tag,poses=poses,sample_vertex_count=len(sg['vertices']),sample_face_count=len(sg['faces']))
    colors={0:([.66,.72,.76],'Start_sample_plane'),1:([.73,.50,.27],'Start_raised_band'),2:([.62,.73,.77],'Start_remaining'),3:([.63,.75,.67],'Post_B_platform')}
    for k,(color,name) in colors.items():
        ff=f[tag==k];mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(v),o3d.utility.Vector3iVector(ff));mesh.remove_unreferenced_vertices();mesh.compute_vertex_normals();mesh.paint_uniform_color(color)
        objwrite(out/'visual'/f'{name}.obj',np.asarray(mesh.vertices),np.asarray(mesh.triangles));o3d.io.write_triangle_mesh(str(out/'visual'/f'{name}.ply'),mesh)
    bg=np.load(B/'geometry.npz');bv=bg['vertices'];bf=bg['faces']
    combined=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.r_[v,bv]),o3d.utility.Vector3iVector(np.r_[f,bf+len(v)]));combined.compute_vertex_normals()
    objwrite(out/'visual/Start_B_short_combined.obj',np.asarray(combined.vertices),np.asarray(combined.triangles));o3d.io.write_triangle_mesh(str(out/'visual/Start_B_short_combined.ply'),combined)
    shutil.copy2(B/'assets/B_structured_map.obj',out/'visual/B_preserved.obj');shutil.copy2(B/'geometry.npz',out/'data/B_geometry_preserved.npz')
    shutil.copy2(B/'B_structured_collision.xml',out/'collision/B_original.xml')
    # Collision merge: merge only exactly coplanar, convex, hole-free neighbors.
    # Nonplanar band remains fine. A concave global mesh is never a collider.
    normals=cross/np.linalg.norm(cross,axis=1)[:,None];offset=-np.sum(normals*v[f[:,0]],axis=1)
    labels=np.c_[tag,np.round(normals,7),np.round(offset,7)];_,groups=np.unique(labels,axis=0,return_inverse=True)
    solids=[];merged_from=0
    for gi in np.unique(groups):
        ii=np.flatnonzero(groups==gi);remaining={int(j):poly[j] for j in ii}
        if len(ii)>1:
            changed=True
            # Bounded passes; convex pieces never cross an unknown hole.
            for _ in range(12):
                keys=list(remaining);geoms=[remaining[k] for k in keys];tree=shapely.STRtree(geoms);changed=False
                for a in keys:
                    if a not in remaining:continue
                    ga=remaining[a]
                    for j in tree.query(ga,predicate='intersects'):
                        b=keys[int(j)]
                        if b==a or b not in remaining:continue
                        gb=remaining[b]
                        if ga.boundary.intersection(gb.boundary).length<1e-6:continue
                        gg=ga.union(gb)
                        if gg.geom_type!='Polygon' or len(gg.interiors) or len(gg.exterior.coords)>35:continue
                        if abs(gg.convex_hull.area-gg.area)>1e-9:continue
                        remaining[a]=gg;del remaining[b];ga=gg;changed=True;merged_from+=1
                if not changed:break
        n=normals[ii[0]];off=offset[ii[0]]
        for polygon in remaining.values():
            xy=np.array(polygon.exterior.coords)[:-1];z=-(xy@n[:2]+off)/n[2];top=np.c_[xy,z];solid=np.r_[top,top-[0,0,.20]]
            solids.append((solid,int(tag[ii[0]])))
    inc=ET.Element('mujocoinclude');assets=ET.SubElement(inc,'asset');world=ET.SubElement(inc,'worldbody')
    for i,(sv,k) in enumerate(solids):
        center=sv.mean(0);ET.SubElement(assets,'mesh',name=f'fine_{i}',vertex=' '.join(f'{x:.12g}' for x in (sv-center).ravel()))
        ET.SubElement(world,'geom',name=f'fine_{i}',type='mesh',mesh=f'fine_{i}',pos=' '.join(f'{x:.12g}' for x in center),group='3',contype='1',conaffinity='1',friction='.8 .005 .0001',rgba=' '.join(map(str,[*colors[k][0],1])))
    xml(out/'collision/new_surfaces.xml',inc)
    bi=ET.parse(out/'collision/B_original.xml').getroot()
    for geom in bi.findall('./worldbody/geom'):geom.set('group','3')
    xml(out/'collision/B_preserved.xml',bi)
    for variant,solref in [('baseline',None),('contact_v1','.01 1')]:
        model=ET.Element('mujoco',model='Start + preserved B + bounded post-B, assumed geometry')
        ET.SubElement(model,'compiler',angle='radian');ET.SubElement(model,'option',timestep='.001',integrator='implicitfast',gravity='0 0 -9.81')
        if solref:ET.SubElement(ET.SubElement(model,'default'),'geom',solref=solref)
        ET.SubElement(model,'include',file='collision/new_surfaces.xml');ET.SubElement(model,'include',file='collision/B_preserved.xml')
        aa=ET.SubElement(model,'asset');ww=ET.SubElement(model,'worldbody')
        for _,(color,name) in colors.items():
            ET.SubElement(aa,'mesh',name=name,file=f'visual/{name}.obj',inertia='shell');ET.SubElement(ww,'geom',name=name,type='mesh',mesh=name,group='2',contype='0',conaffinity='0',rgba=' '.join(map(str,[*color,1])))
        ET.SubElement(aa,'mesh',name='B_visual',file='visual/B_preserved.obj',inertia='shell');ET.SubElement(ww,'geom',name='B_visual',type='mesh',mesh='B_visual',group='2',contype='0',conaffinity='0',rgba='.88 .64 .34 1')
        vis=ET.SubElement(model,'visual');ET.SubElement(vis,'global',offwidth='1600',offheight='1000');ET.SubElement(vis,'headlight',ambient='.6 .6 .6',diffuse='.6 .6 .6')
        ET.SubElement(model,'statistic',center='14 11 2',extent='36');xml(out/f'scene_{variant}.xml',model)
    records=json.loads((ROOT/'sources/manifest.json').read_text())+json.loads((ROOT/'three_sources/manifest.json').read_text())
    for p in [sample/'geometry.npz',sample/'assets/Start_sample.ply',B/'geometry.npz',B/'B_structured_collision.xml']:records.append(dict(path=str(p),sha256=sha(p)))
    unchanged=all(sha(r['path'])==r['sha256'] for r in records);assert unchanged
    dump(out/'input_manifest.json',records);dump(out/'surface_regions.json',domain_records);dump(out/'plane_fits.json',fits)
    report=dict(status='THREE_REGION_GEOMETRY_CANDIDATE_NOT_FIELD_ACCEPTED',new_vertices=len(v),new_triangles=len(f),B_vertices=len(bv),B_triangles=len(bf),
        collision_solids=len(solids),coplanar_merges=merged_from,new_positive_area_overlap_m2=float(np.max(overlap,initial=0)),
        sample_geometry_exact_copy=bool(np.array_equal(v[:len(sg['vertices'])],sg['vertices']) and np.array_equal(f[:len(sg['faces'])],sg['faces'])),
        B_geometry_sha256=sha(B/'geometry.npz'),B_copied_geometry_sha256=sha(out/'data/B_geometry_preserved.npz'),
        post_plane=pc.tolist(),post_length_along_exit_normal_m=length,post_halfwidth_is_crop_m=half,
        post_centerline_end_map_xy=stop.tolist(),post_centerline_length_m=float(np.linalg.norm(post_shift)),
        protected_entry_edge=entry.tolist(),protected_exit_edge=exit_edge.tolist(),post_direction=pn.tolist(),
        source_files_unchanged=unchanged,source_files_verified=len(records),field_accuracy=False,robot_used=False,policy_run=False,
        all_scope_originals_preserved=True,post_side_raised_is_unmodeled_evidence=True,
        continuous_transition_strips=dict(west_to_middle_x=[2,3],middle_to_east_x=[8,9],south_to_north_y=[1.4,2.15],B_entry_distance_m=.8),
        engine_check_required=True,elapsed_s=time.monotonic()-started)
    dump(out/'build_report.json',report);print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':main()
