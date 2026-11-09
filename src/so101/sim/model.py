"""MuJoCo model of the SO-101 follower, generated from the follower URDF in so101_config/urdf.

MuJoCo reads URDF directly, so the joints, limits, inertias and meshes come straight from `so101_new_calib.urdf`
(the same "new_calib" zero as the real arm and the Rerun view). On top of that this module adds what a URDF cannot
describe:

- a position actuator per joint, named after the joint (`JOINTS`), with the STS3215 parameters tuned in MuJoCo
  Menagerie's `robotstudio_so101` model (Apache-2.0): kp, kv, force range, joint damping, friction loss, armature
- solver options suited to grasping (elliptic friction cones, implicitfast integrator)
- visual meshes in geom group 2 and collision meshes in group 3, so the viewer shows only the visual ones
- collision for the jaws as a stack of slab hulls (see `_slab_hulls`), so a box fits between them
- contact exclusions between linked bodies, whose convex hulls overlap at the joints

The result is written to so101_config/mujoco/so101.xml. Scenes (`scene_box.xml`: floor, light and a box to pick up)
are shipped with the package and copied next to it; they `<include>` so101.xml.
"""

import itertools
import os
import re
import shutil
from pathlib import Path

import mujoco
import numpy as np
import trimesh

from so101.conventions import JOINTS
from so101.paths import FOLLOWER_URDF, MUJOCO_DIR, PACKAGE_DIR, SO101_HOME, ensure_home, pretty

ROBOT_XML = "so101.xml"
DEFAULT_SCENE = "scene_box.xml"
SCENES_DIR = PACKAGE_DIR / "sim" / "scenes"

# Home pose: every joint at 0 rad, the calibration mid stance of the "new_calib" URDF.
HOME_QPOS = {joint: 0.0 for joint in JOINTS}

# STS3215 servo parameters from MuJoCo Menagerie robotstudio_so101 (class "sts3215").
JOINT_DAMPING = 0.60
JOINT_FRICTIONLOSS = 0.052
JOINT_ARMATURE = 0.028
ACTUATOR_KP = 998.22
ACTUATOR_KV = 2.731
ACTUATOR_FORCE = 2.94  # N·m

VISUAL_GROUP = 2
COLLISION_GROUP = 3
GRIPPER_SLABS = 12  # slab hulls per jaw mesh, see _slab_hulls()


def _urdf_to_spec(urdf: Path) -> mujoco.MjSpec:
    """Parse the URDF with MuJoCo, keeping the visual meshes and the mesh paths relative to the URDF folder."""
    compiler = (
        f'<mujoco><compiler meshdir="{urdf.parent.resolve()}" strippath="false" discardvisual="false" '
        'fusestatic="false" balanceinertia="true"/></mujoco>'
    )
    text = re.sub(r"(<robot\b[^>]*>)", lambda m: m.group(1) + compiler, urdf.read_text(), count=1)
    return mujoco.MjSpec.from_string(text)


def _slab_hulls(mesh_path: Path, slabs: int) -> list[np.ndarray]:
    """Cut a mesh into `slabs` slices along its longest axis and return the convex hull vertices of each slice.

    MuJoCo collides with the convex hull of a mesh. For the jaws that hull is a solid wedge filling the gap between
    them, so nothing fits in between. A stack of thin hulls follows the finger shape and keeps the gap open.
    """
    mesh = trimesh.load(mesh_path, force="mesh")
    axis = int(np.argmax(mesh.extents))
    normal = np.eye(3)[axis]
    lo, hi = mesh.bounds[:, axis]
    edges = np.linspace(lo, hi, slabs + 1)
    hulls = []
    for start, end in itertools.pairwise(edges):
        part = mesh.slice_plane(normal * start, normal)
        part = part.slice_plane(normal * end, -normal) if part is not None and len(part.faces) else None
        if part is None or len(part.faces) == 0:
            continue
        points = part.vertices[np.unique(part.faces)]
        if np.linalg.matrix_rank(points - points.mean(axis=0), tol=1e-6) < 3:
            continue
        hulls.append(trimesh.convex.convex_hull(points).vertices)
    return hulls


def _split_gripper_collisions(spec: mujoco.MjSpec, meshdir: Path, slabs: int = GRIPPER_SLABS) -> None:
    """Replace the collision meshes on both sides of the gripper joint by stacks of slab hulls."""
    gripper = next(joint for joint in spec.joints if joint.name == "gripper")
    jaw_bodies = {gripper.parent.name, gripper.parent.parent.name}  # moving jaw + the body with the fixed jaw
    meshes = {mesh.name: mesh for mesh in spec.meshes}
    for geom in list(spec.geoms):
        if geom.parent.name not in jaw_bodies or geom.group != COLLISION_GROUP:
            continue
        if geom.type != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        source = meshes[geom.meshname]
        for i, vertices in enumerate(_slab_hulls(meshdir / source.file, slabs)):
            hull = spec.add_mesh(name=f"{source.name}_slab{i}", uservert=vertices.flatten().tolist())
            hull.scale = source.scale
            geom.parent.add_geom(
                type=mujoco.mjtGeom.mjGEOM_MESH,
                meshname=hull.name,
                pos=geom.pos,
                quat=geom.quat,
                group=COLLISION_GROUP,
                condim=geom.condim,
                rgba=geom.rgba,
                friction=geom.friction,
                contype=geom.contype,
                conaffinity=geom.conaffinity,
            )
        spec.delete(geom)


def build_spec(urdf: Path = FOLLOWER_URDF) -> mujoco.MjSpec:
    """The robot as an MjSpec: the URDF plus actuators, servo parameters and solver options."""
    spec = _urdf_to_spec(urdf)
    spec.modelname = "so101"

    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10

    for geom in spec.geoms:
        if geom.contype == 0 and geom.conaffinity == 0:  # URDF <visual>
            geom.group = VISUAL_GROUP
        else:  # URDF <collision>
            geom.group = COLLISION_GROUP
            geom.condim = 4  # torsional friction too, so a grasped box does not spin out of the jaws
            geom.rgba = [0.8, 0.3, 0.3, 0.5]
    _split_gripper_collisions(spec, urdf.parent)

    # Collision meshes become convex hulls, so the hulls of linked bodies overlap around the joint. MuJoCo already
    # skips parent-child contacts, except when the parent is welded to the world (base_link -> shoulder_link).
    for body in spec.bodies:
        parent = body.parent
        if parent is not None and parent.name != "world" and body.name != "world":
            spec.add_exclude(bodyname1=parent.name, bodyname2=body.name)

    joints = {joint.name: joint for joint in spec.joints}
    missing = [name for name in JOINTS if name not in joints]
    if missing:
        raise ValueError(f"{pretty(urdf)} has no joint(s) {missing}; expected {JOINTS}.")

    for name in JOINTS:
        joint = joints[name]
        joint.damping = [JOINT_DAMPING, 0, 0]  # a 3-vector since MuJoCo 4; only the linear term is used
        joint.frictionloss = JOINT_FRICTIONLOSS
        joint.armature = JOINT_ARMATURE
        actuator = spec.add_actuator(name=name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        actuator.set_to_position(kp=ACTUATOR_KP, kv=ACTUATOR_KV)
        actuator.ctrlrange = joint.range
        actuator.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
        actuator.forcerange = [-ACTUATOR_FORCE, ACTUATOR_FORCE]
        actuator.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
    return spec


def _stale(dest: Path, urdf: Path) -> bool:
    """True when so101.xml or a scene is missing, or older than the URDF, this module or a shipped scene."""
    outputs = [dest / ROBOT_XML, *(dest / scene.name for scene in SCENES_DIR.glob("*.xml"))]
    if not all(path.exists() for path in outputs):
        return True
    inputs = [urdf, Path(__file__), *SCENES_DIR.glob("*.xml")]
    return max(p.stat().st_mtime for p in inputs) > min(p.stat().st_mtime for p in outputs)


def ensure_model(urdf: Path = FOLLOWER_URDF, dest: Path = MUJOCO_DIR, force: bool = False) -> Path:
    """Generate so101.xml from the URDF (and copy the scenes) into `dest` when missing or out of date.

    Regenerates whenever the URDF, this module or a shipped scene is newer than the files in `dest`, so the model
    always follows the current URDF. The meshes are not copied: so101.xml points at the URDF's folder. Returns `dest`.
    """
    if not urdf.exists():
        raise FileNotFoundError(f"No URDF found at {pretty(urdf)}. Fetch it with `uv run so101-fetch-urdfs`.")
    if not force and not _stale(dest, urdf):
        return dest

    spec = build_spec(urdf)
    spec.compile()  # fail here, before writing anything, if the model is invalid
    xml = spec.to_xml()
    absolute = f'meshdir="{spec.meshdir}"'
    if absolute not in xml:
        raise RuntimeError(f"Could not find {absolute} in the generated MJCF.")
    xml = xml.replace(absolute, f'meshdir="{os.path.relpath(urdf.parent.resolve(), dest.resolve())}"', 1)

    if dest.resolve().is_relative_to(SO101_HOME.resolve()):
        ensure_home()
    dest.mkdir(parents=True, exist_ok=True)
    (dest / ROBOT_XML).write_text(xml)
    for scene in SCENES_DIR.glob("*.xml"):
        shutil.copyfile(scene, dest / scene.name)
    return dest


def load_model(scene: str = DEFAULT_SCENE, urdf: Path = FOLLOWER_URDF, dest: Path = MUJOCO_DIR) -> mujoco.MjModel:
    """Compile a scene from `dest`, generating the model first if needed. Pass `ROBOT_XML` for the bare arm."""
    ensure_model(urdf, dest)
    path = dest / scene
    if not path.exists():
        available = sorted(p.name for p in dest.glob("*.xml"))
        raise FileNotFoundError(f"No scene {scene!r} in {pretty(dest)}; available: {available}")
    return mujoco.MjModel.from_xml_path(str(path))


def home(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Reset `data` to the home pose, with every actuator holding it."""
    mujoco.mj_resetData(model, data)
    for joint, value in HOME_QPOS.items():
        data.qpos[model.joint(joint).qposadr[0]] = value
        data.ctrl[model.actuator(joint).id] = value
    mujoco.mj_forward(model, data)
