"""Avoid the stock OBJ writer's six-digit rounding at thin interface triangles."""
from pathlib import Path
import numpy as np
import open3d as o3d
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'sample_05'

def export(path,v,f):
    used,inv=np.unique(f,return_inverse=True);v=v[used];f=inv.reshape(-1,3)
    mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(v),o3d.utility.Vector3iVector(f));mesh.compute_vertex_normals()
    n=np.asarray(mesh.vertex_normals)
    with path.open('w') as stream:
        stream.write('# Start sample in unchanged v3 map coordinates, metres. Float64 round-trip precision.\n')
        for p in v:stream.write('v '+' '.join(f'{a:.17g}' for a in p)+'\n')
        for p in n:stream.write('vn '+' '.join(f'{a:.17g}' for a in p)+'\n')
        for tri in f+1:stream.write('f '+' '.join(f'{a}//{a}' for a in tri)+'\n')

def main():
    d=np.load(OUT/'geometry.npz');v=d['vertices'];f=d['faces'];k=d['face_classes']
    for name,sel in [('Start_sample',np.ones(len(f),bool)),('pavement',k==0),('raised_transition',k==1)]:
        export(OUT/'assets'/f'{name}.obj',v,f[sel])
    print('Exported float64 round-trip OBJ: same geometry, no writer rounding',flush=True)

if __name__=='__main__':main()
