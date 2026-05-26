#!/usr/bin/env python3
"""Generate a legs-only variant of the kbot-v2 MJCF (and URDF on demand).

The physical robot under test in this directory has TORSO + LEGS only — no
arms. The upstream kbot-v2 mjcf includes both arms, so any sim driven from it
carries arm mass + dynamics that don't exist on the bench. This script reads
the upstream mjcf, removes the two arm subtrees and their actuators, and
writes a new file alongside the originals.

Usage:
    python3 strip_arms.py                            # produces both .mjcf variants
    python3 strip_arms.py --kbot-dir /path/to/kbot-v2
    python3 strip_arms.py --include-urdf             # also produce legs-only URDF
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

DEFAULT_KBOT_DIR = "/Users/faisaljaffer/Documents/GitHub/kscale-assets/kbot-v2"

# Root bodies of the two arm kinematic chains (everything below these is arm)
ARM_ROOT_BODIES = ["KC_C_104R_PitchHardstopDriven", "KC_C_104L_PitchHardstopDriven"]

# All arm joints (used to filter <actuator> and URDF <joint>)
ARM_JOINTS = {f"dof_{side}_{j}" for side in ("left", "right") for j in (
    "shoulder_pitch_03", "shoulder_roll_03", "shoulder_yaw_02",
    "elbow_02", "wrist_00",
)}


def _collect_descendant_body_names(node):
    """Return the set of every body name in this subtree (including the node)."""
    names = set()
    if node.tag == 'body' and node.get('name'):
        names.add(node.get('name'))
    for child in node:
        names |= _collect_descendant_body_names(child)
    return names


def _remove_body_subtree(root, body_name):
    """Find <body name=body_name> anywhere in the tree, collect all body names
    in its subtree, remove it from its parent, and return the collected names
    (empty set if not found)."""
    stack = [root]
    while stack:
        parent = stack.pop()
        for child in list(parent):
            if child.tag == 'body' and child.get('name') == body_name:
                removed_names = _collect_descendant_body_names(child)
                parent.remove(child)
                return removed_names
            stack.append(child)
    return set()


def _scale_inertials(root, scale):
    """Multiply every <inertial mass="..."/> and its diaginertia/fullinertia values
    in-place by the given scale factor. Returns (n_inertials, mass_before, mass_after)."""
    n = 0
    before = after = 0.0
    for inertial in root.iter('inertial'):
        m = inertial.get('mass')
        if m is None: continue
        m = float(m); before += m
        m_new = m * scale; after += m_new
        inertial.set('mass', f'{m_new:.6f}')
        for attr in ('diaginertia', 'fullinertia'):
            v = inertial.get(attr)
            if v is not None:
                vals = [float(x) * scale for x in v.split()]
                inertial.set(attr, ' '.join(f'{x:.8f}' for x in vals))
        n += 1
    return n, before, after


def _scale_urdf_inertials(root, scale):
    """URDF inertials are <link><inertial><mass value="..."/>... <inertia ixx=... .../></link>"""
    n = 0
    before = after = 0.0
    for link in root.findall('link'):
        inert = link.find('inertial')
        if inert is None: continue
        mass_el = inert.find('mass')
        if mass_el is None or mass_el.get('value') is None: continue
        m = float(mass_el.get('value')); before += m
        m_new = m * scale; after += m_new
        mass_el.set('value', f'{m_new:.6f}')
        inertia_el = inert.find('inertia')
        if inertia_el is not None:
            for k in ('ixx','ixy','ixz','iyy','iyz','izz'):
                v = inertia_el.get(k)
                if v is not None:
                    inertia_el.set(k, f'{float(v)*scale:.8f}')
        n += 1
    return n, before, after


def strip_arms_mjcf(src_path, dst_path, target_mass_kg=None):
    """Read MJCF at src_path, remove arm bodies + arm actuators, write to dst_path."""
    tree = ET.parse(src_path)
    root = tree.getroot()

    # Remove arm body subtrees; collect every body name that disappeared
    removed_names = set()
    for bname in ARM_ROOT_BODIES:
        removed_names |= _remove_body_subtree(root, bname)

    # Remove arm actuators
    removed_actuators = 0
    actuator_el = root.find('actuator')
    if actuator_el is not None:
        for motor in list(actuator_el):
            if motor.get('joint') in ARM_JOINTS:
                actuator_el.remove(motor)
                removed_actuators += 1

    # Prune <contact><exclude> pairs that reference removed bodies (dangling refs)
    removed_excludes = 0
    contact_el = root.find('contact')
    if contact_el is not None:
        for ex in list(contact_el):
            if ex.tag == 'exclude' and (
                ex.get('body1') in removed_names or ex.get('body2') in removed_names
            ):
                contact_el.remove(ex); removed_excludes += 1
        # If the contact block is now empty, drop it
        if not list(contact_el):
            root.remove(contact_el)

    # Prune orphan <mesh name="..."/> declarations no longer referenced by any geom
    used_mesh_names = set()
    for geom in root.iter('geom'):
        if geom.get('mesh'):
            used_mesh_names.add(geom.get('mesh'))
    removed_meshes = 0
    asset_el = root.find('asset')
    if asset_el is not None:
        for mesh in list(asset_el):
            if mesh.tag == 'mesh' and mesh.get('name') not in used_mesh_names:
                asset_el.remove(mesh); removed_meshes += 1

    scaled_msg = ""
    if target_mass_kg is not None:
        _, current_total, _ = _scale_inertials(root, 1.0)
        if current_total <= 0:
            raise RuntimeError(f"Total mass after stripping arms is zero in {src_path}")
        scale = target_mass_kg / current_total
        n, before, after = _scale_inertials(root, scale)
        scaled_msg = (f"; rescaled {n} inertials by {scale:.4f}x to match "
                      f"--target-mass-kg={target_mass_kg} "
                      f"({before:.3f} kg -> {after:.3f} kg)")

    tree.write(dst_path, xml_declaration=False, encoding='unicode')
    print(f"  {src_path.name} -> {dst_path.name}: "
          f"removed {len(removed_names)} arm bodies, {removed_actuators} actuators, "
          f"{removed_excludes} contact excludes, {removed_meshes} unused mesh declarations"
          f"{scaled_msg}")


def _link_tree(urdf_root):
    """Return {parent_link: [(joint_name, child_link), ...]} from a URDF tree."""
    out = {}
    for j in urdf_root.findall('joint'):
        p = j.find('parent'); c = j.find('child')
        if p is None or c is None:
            continue
        out.setdefault(p.get('link'), []).append((j.get('name'), c.get('link')))
    return out


def strip_arms_urdf(src_path, dst_path, target_mass_kg=None):
    """Read URDF, remove arm joints + every descendant link reachable from
    arm-shoulder-joint children. Write to dst_path."""
    tree = ET.parse(src_path)
    root = tree.getroot()

    # Build link tree, then BFS from arm shoulder-joint children
    edges = _link_tree(root)
    seeds = []
    for j in root.findall('joint'):
        if j.get('name') in ARM_JOINTS and j.find('child') is not None:
            seeds.append(j.find('child').get('link'))
    arm_links = set()
    while seeds:
        link = seeds.pop()
        if link in arm_links:
            continue
        arm_links.add(link)
        for _, child_link in edges.get(link, []):
            seeds.append(child_link)

    removed_links = 0
    for link in list(root.findall('link')):
        if link.get('name') in arm_links:
            root.remove(link); removed_links += 1

    removed_joints = 0
    for j in list(root.findall('joint')):
        name = j.get('name')
        child = j.find('child')
        if name in ARM_JOINTS or (child is not None and child.get('link') in arm_links):
            root.remove(j); removed_joints += 1

    scaled_msg = ""
    if target_mass_kg is not None:
        _, current_total, _ = _scale_urdf_inertials(root, 1.0)
        if current_total <= 0:
            raise RuntimeError(f"Total mass after stripping arms is zero in {src_path}")
        scale = target_mass_kg / current_total
        n, before, after = _scale_urdf_inertials(root, scale)
        scaled_msg = (f"; rescaled {n} link inertials by {scale:.4f}x to match "
                      f"--target-mass-kg={target_mass_kg} "
                      f"({before:.3f} kg -> {after:.3f} kg)")

    tree.write(dst_path, xml_declaration=False, encoding='unicode')
    print(f"  {src_path.name} -> {dst_path.name}: "
          f"removed {removed_links} links, {removed_joints} joints{scaled_msg}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--kbot-dir', default=DEFAULT_KBOT_DIR,
                    help='kbot-v2 asset directory (default: %(default)s)')
    ap.add_argument('--include-urdf', action='store_true',
                    help='Also generate robot.legs_only.urdf')
    ap.add_argument('--target-mass-kg', type=float, default=None,
                    help='If set, scale every body mass + inertia uniformly so the '
                         'total robot mass matches this value (kg). For our build: '
                         '13.06 kg (28.8 lb measured). Without this flag the CAD '
                         'masses are kept as-is (~28.3 kg for legs-only kbot-v2).')
    ap.add_argument('--out-suffix', default='legs_only',
                    help='Suffix to insert before the extension. Default "legs_only" '
                         '-> robot.legs_only.mjcf. Use e.g. "legs_only.real_mass" to '
                         'differentiate mass-scaled variants. Ignored if --out-dir is set '
                         '(in that case files are named robot.mjcf / robot.urdf etc).')
    ap.add_argument('--out-dir', default=None,
                    help='Output directory. If set, writes files as robot.mjcf / '
                         'robot.urdf etc. in this dir and copies only the meshes referenced '
                         'by the legs-only mjcf. Creates a self-contained asset package.')
    args = ap.parse_args()

    import re
    import shutil

    src_dir = Path(args.kbot_dir)
    if not src_dir.is_dir():
        raise SystemExit(f"kbot-v2 directory not found: {src_dir}")

    if args.out_dir:
        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        # Self-contained mode: write robot.mjcf / robot.urdf in out_dir, copy meshes
        print(f"Stripping arms -> self-contained package in {out_dir}:")
        produced_mjcfs = []
        for name in ['robot.mjcf', 'robot.suspended.mjcf']:
            src = src_dir / name
            if not src.exists():
                print(f"  skip {name} (not found)"); continue
            dst = out_dir / name
            strip_arms_mjcf(src, dst, target_mass_kg=args.target_mass_kg)
            produced_mjcfs.append(dst)

        if args.include_urdf:
            src = src_dir / 'robot.urdf'
            if src.exists():
                dst = out_dir / 'robot.urdf'
                strip_arms_urdf(src, dst, target_mass_kg=args.target_mass_kg)
            else:
                print(f"  skip robot.urdf (not found)")

        # Find all meshes referenced by the produced mjcfs and copy them
        mesh_refs = set()
        for mjcf_path in produced_mjcfs:
            xml = mjcf_path.read_text()
            for m in re.finditer(r'(?:file|mesh)="(meshes/[^"]+\.stl)"', xml):
                mesh_refs.add(m.group(1))
            # Also catch <mesh file="meshes/..."> in <asset>
            for m in re.finditer(r'file="(meshes/[^"]+\.stl)"', xml):
                mesh_refs.add(m.group(1))
        mesh_out = out_dir / 'meshes'
        mesh_out.mkdir(exist_ok=True)
        copied = 0
        for rel in sorted(mesh_refs):
            s = src_dir / rel
            d = out_dir / rel
            if s.exists():
                shutil.copy2(s, d); copied += 1
        size_mb = sum((out_dir/r).stat().st_size for r in mesh_refs if (out_dir/r).exists()) / 1024 / 1024
        print(f"\n  Copied {copied}/{len(mesh_refs)} referenced meshes ({size_mb:.1f} MB) -> {mesh_out}")
        # Also copy metadata.json if present (used by ksim)
        meta = src_dir / 'metadata.json'
        if meta.exists():
            shutil.copy2(meta, out_dir / 'metadata.json')
            print(f"  Copied metadata.json")
    else:
        # Legacy mode: write *.legs_only.mjcf alongside originals in src_dir
        print(f"Stripping arms from MJCFs in {src_dir}:")
        for name in ['robot.mjcf', 'robot.suspended.mjcf']:
            src = src_dir / name
            if not src.exists():
                print(f"  skip {name} (not found)"); continue
            dst = src_dir / name.replace('.mjcf', f'.{args.out_suffix}.mjcf')
            strip_arms_mjcf(src, dst, target_mass_kg=args.target_mass_kg)
        if args.include_urdf:
            src = src_dir / 'robot.urdf'
            if src.exists():
                dst = src_dir / f'robot.{args.out_suffix}.urdf'
                strip_arms_urdf(src, dst, target_mass_kg=args.target_mass_kg)
            else:
                print(f"  skip robot.urdf (not found)")


if __name__ == '__main__':
    main()
