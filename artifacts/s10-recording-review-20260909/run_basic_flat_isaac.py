"""Isaac Sim 5.1 standalone: SDK MJCF -> free articulation -> measured-state PD tracking."""
import argparse
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import match_basic_flat as matching
from match_basic_flat import OUT, JOBS, DT, controller, dump, model_xml, rotation, save_run, target

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--only', choices=[r[0] for r in JOBS])
parser.add_argument('--output-dir', type=Path, help='Separate result directory; source references stay unchanged')
parser.add_argument('--ground-torsion', type=float, nargs=2, metavar=('RADIUS', 'MIN_RADIUS'),
                    help='Ground torsional patch radii in meters; omitted uses the existing cuboid defaults')
parser.add_argument('--ground-contact-offset', type=float,
                    help='Ground contact detection distance in meters; omitted preserves the cuboid default')
args, unknown = parser.parse_known_args()
if args.ground_torsion is not None:
    import math
    if not all(math.isfinite(v) and v >= 0 for v in args.ground_torsion):
        parser.error('Ground torsion radii must be finite and nonnegative')
if args.ground_contact_offset is not None:
    import math
    if not math.isfinite(args.ground_contact_offset) or args.ground_contact_offset <= 0:
        parser.error('Ground contact offset must be positive and finite')
if args.output_dir:
    source_out = OUT
    OUT = args.output_dir.resolve()
    OUT.mkdir(parents=True, exist_ok=True)
    if OUT != source_out:
        for name, *_ in JOBS:
            if not args.only or name == args.only:
                (OUT/name).mkdir(exist_ok=True)
                for filename in ('reference.npz', 'source.json'):
                    shutil.copy2(source_out/name/filename, OUT/name/filename)
    matching.OUT = OUT

from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'width': 640, 'height': 480,
                     'anti_aliasing': 0, 'multi_gpu': False})

import numpy as np
import omni.kit.commands
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation, RigidPrim
from isaacsim.core.utils.extensions import enable_extension
from pxr import UsdPhysics, PhysxSchema

try:
    world = World(stage_units_in_meters=1., physics_dt=DT, rendering_dt=.02, backend='numpy', device='cpu')
    world.get_physics_context().enable_gpu_dynamics(False)
    world.get_physics_context().set_broadphase_type('MBP')
    enable_extension('isaacsim.asset.importer.mjcf')
    app.update()
    tree = ET.parse(model_xml())
    compiler = tree.getroot().find('compiler')
    compiler.set('meshdir', os.path.relpath(compiler.get('meshdir'), OUT))
    wb = tree.getroot().find('worldbody')
    for node in list(wb):
        if node.tag != 'body':
            wb.remove(node)
    xml = OUT / 'isaac_model.xml'
    tree.write(xml, encoding='utf-8')
    ok, cfg = omni.kit.commands.execute('MJCFCreateImportConfig')
    assert ok
    cfg.merge_fixed_joints = False
    cfg.convex_decomp = False
    cfg.import_inertia_tensor = True
    cfg.fix_base = False
    cfg.distance_scale = 1.
    cfg.self_collision = True
    ok, imported = omni.kit.commands.execute('MJCFCreateAsset', mjcf_path=str(xml),
        import_config=cfg, prim_path='/World/S10')
    assert ok, imported
    stage = omni.usd.get_context().get_stage()
    # MJCF importer adds an empty worldBody articulation even when floor/light were removed.
    stage.RemovePrim('/World/S10/worldBody')
    roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    print('ARTICULATION_ROOTS', roots, flush=True)
    assert len(roots) == 1
    # Imported actuator drives must not silently add position control to explicit effort PD.
    rigid_paths = []
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith('/World/S10'):
            rigid_paths.append(str(p.GetPath()))
            rb = PhysxSchema.PhysxRigidBodyAPI.Apply(p)
            rb.CreateLinearDampingAttr(0.)
            rb.CreateAngularDampingAttr(0.)
        if p.IsA(UsdPhysics.Joint):
            for axis in ('angular', 'linear'):
                drive = UsdPhysics.DriveAPI.Get(p, axis)
                if drive:
                    drive.GetStiffnessAttr().Set(0.)
                    drive.GetDampingAttr().Set(0.)
    assert len(rigid_paths) == 17, rigid_paths
    material = PhysicsMaterial('/World/groundPhysics', static_friction=1., dynamic_friction=1., restitution=0.)
    # A local slab has the same z=0 support surface and avoids this runtime's native-plane material warnings.
    world.scene.add(FixedCuboid('/World/Ground', position=np.array([0., 0., -.05]),
        scale=np.array([200., 200., .1]), size=1., physics_material=material))
    ground_collision = PhysxSchema.PhysxCollisionAPI(stage.GetPrimAtPath('/World/Ground'))
    if args.ground_torsion is not None:
        ground_collision.CreateTorsionalPatchRadiusAttr(args.ground_torsion[0])
        ground_collision.CreateMinTorsionalPatchRadiusAttr(args.ground_torsion[1])
    if args.ground_contact_offset is not None:
        ground_collision.CreateContactOffsetAttr(args.ground_contact_offset)
    robot = world.scene.add(SingleArticulation(prim_path=roots[0], name='s10'))
    contacts = world.scene.add(RigidPrim(prim_paths_expr=rigid_paths, name='body_contacts',
        track_contact_forces=True, contact_filter_prim_paths_expr=[[] for _ in rigid_paths]))
    world.reset()
    robot.set_solver_position_iteration_count(16)
    robot.set_solver_velocity_iteration_count(4)
    robot.set_enabled_self_collisions(True)
    robot.set_sleep_threshold(0.)
    robot.get_articulation_controller().set_gains(np.zeros(16), np.zeros(16), save_to_usd=True)
    assert robot.num_dof == 16, robot.dof_names
    dump(OUT / 'isaac_asset.json', dict(articulation_root=roots[0], dof_names=robot.dof_names,
        body_names_in_mass_order=robot._articulation_view.body_names,
        rigid_paths=rigid_paths, masses=robot._articulation_view.get_body_masses().tolist(),
        engine_version='Isaac Sim 5.1.0', backend='PhysX CPU, numpy',
        fixed_base=False, floor_friction=1., physics_dt_s=DT,
        ground_contact_offset_m=ground_collision.GetContactOffsetAttr().Get(),
        ground_torsion_radius_m=ground_collision.GetTorsionalPatchRadiusAttr().Get(),
        ground_min_torsion_radius_m=ground_collision.GetMinTorsionalPatchRadiusAttr().Get()))
    stage.GetRootLayer().Export(str(OUT / 'isaac_flat_scene.usdc'))
    for name, _, start, end in JOBS:
        if args.only and name != args.only:
            continue
        ref = dict(np.load(OUT / name / 'reference.npz'))
        ids = np.array([robot.get_dof_index(str(n)) for n in ref['joint_names']], dtype='i4')
        assert len(set(ids)) == 16
        world.reset()
        robot.get_articulation_controller().set_gains(np.zeros(16), np.zeros(16))
        robot.set_world_pose(ref['initial_root_pose'][:3], ref['initial_root_pose'][3:])
        robot.set_joint_positions(ref['joint_position_rad'][0], joint_indices=ids)
        robot.set_joint_velocities(ref['joint_velocity_rad_s'][0], joint_indices=ids)
        robot.set_linear_velocity(ref['initial_root_linear_velocity'])
        robot.set_angular_velocity(rotation(ref['initial_root_pose'][3:]) @ ref['initial_root_angular_velocity_body'])
        rows, contact_rows = [], []
        saturation = np.zeros(16)
        print('RUN', name, flush=True)
        for step in range(round((end-start)/DT)):
            q, dq = target(ref, step*DT)
            torque, sat = controller(robot.get_joint_positions(ids), robot.get_joint_velocities(ids), q, dq)
            robot.set_joint_efforts(torque, joint_indices=ids)
            world.step(render=False)
            saturation += sat
            if (step+1) % 20 == 0:
                pos, quat = robot.get_world_pose()
                r = rotation(quat)
                rows.append(np.r_[(step+1)*DT, pos, quat, robot.get_joint_positions(ids),
                    robot.get_joint_velocities(ids), r.T@robot.get_linear_velocity(),
                    r.T@robot.get_angular_velocity(), torque])
                contact_rows.append(contacts.get_net_contact_forces(dt=DT))
        forces = np.asarray(contact_rows)
        np.savez_compressed(OUT / name / 'isaac_contacts.npz', time_s=np.asarray(rows)[:, 0],
                            body_paths=np.array(rigid_paths), net_force_n=forces)
        nonwheel = [i for i, p in enumerate(rigid_paths) if not p.endswith('_wheel')]
        save_run(name, 'isaac', rows, saturation/(step+1), dict(engine_version='Isaac Sim 5.1.0',
            physics_dt_s=DT, controller_dt_s=DT, nonwheel_net_contact_samples=int(np.sum(
                np.any(np.linalg.norm(forces[:, nonwheel], axis=2) > 1., axis=1))),
            contact_note='50 Hz net forces, includes self contact; not an exact floor contact count'))
except Exception:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
