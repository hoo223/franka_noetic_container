#!/usr/bin/env python3

import argparse
import sys

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft


def parse_vec3(text):
    values = [v for v in text.replace(",", " ").split() if v]
    if len(values) != 3:
        raise argparse.ArgumentTypeError("Expected 3 values, e.g. '0 0 0' or '0,0,0'")
    return np.array([float(v) for v in values], dtype=np.float64)


def transform_to_matrix(transform):
    q = [
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w,
    ]
    T = tft.quaternion_matrix(q)
    T[0, 3] = transform.transform.translation.x
    T[1, 3] = transform.transform.translation.y
    T[2, 3] = transform.transform.translation.z
    return T


def load_mesh_mass_properties(mesh_path, mesh_scale, target_mass_kg, use_convex_hull):
    try:
        import trimesh
    except ImportError as exc:
        raise RuntimeError(
            "trimesh is required. Install with: pip install trimesh"
        ) from exc

    loaded = trimesh.load(mesh_path, force="mesh")
    if isinstance(loaded, trimesh.Scene):
        meshes = [g for g in loaded.geometry.values()]
        if not meshes:
            raise RuntimeError(f"No geometry found in mesh: {mesh_path}")
        mesh = trimesh.util.concatenate(meshes)
    else:
        mesh = loaded

    if mesh.is_empty:
        raise RuntimeError(f"Mesh is empty: {mesh_path}")

    if use_convex_hull and not mesh.is_watertight:
        rospy.logwarn(
            "Input mesh is not watertight. Using convex hull for stable mass properties."
        )
        mesh = mesh.convex_hull

    if mesh_scale <= 0.0:
        raise RuntimeError("mesh_scale must be > 0")
    if abs(mesh_scale - 1.0) > 1e-12:
        mesh.apply_scale(mesh_scale)

    mp = mesh.mass_properties
    pseudo_mass = float(mp["mass"])
    if pseudo_mass <= 1e-12:
        raise RuntimeError(
            "Mesh volume is too small or invalid. Check watertight mesh and unit scale."
        )

    com_mesh = np.array(mp["center_mass"], dtype=np.float64)
    inertia_mesh_density1 = np.array(mp["inertia"], dtype=np.float64)

    density_scale = float(target_mass_kg) / pseudo_mass
    inertia_com = inertia_mesh_density1 * density_scale

    return com_mesh, inertia_com


def compute_set_load_params(
    tf_buffer,
    object_frame,
    flange_frame,
    base_frame,
    com_object,
    inertia_object_com,
):
    T_base_object = transform_to_matrix(
        tf_buffer.lookup_transform(base_frame, object_frame, rospy.Time(0), rospy.Duration(1.0))
    )
    T_base_flange = transform_to_matrix(
        tf_buffer.lookup_transform(base_frame, flange_frame, rospy.Time(0), rospy.Duration(1.0))
    )

    T_flange_object = np.linalg.inv(T_base_flange).dot(T_base_object)
    R_flange_object = T_flange_object[:3, :3]
    p_flange_object = T_flange_object[:3, 3]

    p_flange_com = p_flange_object + R_flange_object.dot(com_object)
    inertia_flange_com = R_flange_object.dot(inertia_object_com).dot(R_flange_object.T)
    load_inertia_column_major = inertia_flange_com.reshape(9, order="F")

    return p_flange_com, load_inertia_column_major


def validate_set_load_values(mass, p_flange_com, load_inertia_col_major):
    issues = []
    warnings = []

    if not np.isfinite(mass) or mass <= 0.0:
        issues.append(f"mass must be > 0, got {mass}")

    if not np.all(np.isfinite(p_flange_com)):
        issues.append("F_x_center_load has non-finite values")
    else:
        com_norm = float(np.linalg.norm(p_flange_com))
        if com_norm > 0.25:
            warnings.append(
                f"|F_x_center_load|={com_norm:.3f} m is large. "
                "This usually means object frame is not attached to gripper/flange."
            )

    I = np.array(load_inertia_col_major, dtype=np.float64).reshape(3, 3, order="F")
    if not np.all(np.isfinite(I)):
        issues.append("load_inertia has non-finite values")
        return issues, warnings

    symmetry_error = float(np.max(np.abs(I - I.T)))
    if symmetry_error > 1e-8:
        warnings.append(f"Inertia is not perfectly symmetric (max |I-I^T|={symmetry_error:.3e})")

    I = 0.5 * (I + I.T)
    eigvals = np.linalg.eigvalsh(I)
    if np.min(eigvals) < -1e-10:
        issues.append(f"Inertia is not positive semidefinite. min eigenvalue={np.min(eigvals):.3e}")

    Ixx, Iyy, Izz = float(I[0, 0]), float(I[1, 1]), float(I[2, 2])
    if Ixx < 0 or Iyy < 0 or Izz < 0:
        issues.append("Inertia diagonal terms must be >= 0")

    eps = 1e-10
    if Ixx > Iyy + Izz + eps or Iyy > Ixx + Izz + eps or Izz > Ixx + Iyy + eps:
        issues.append(
            "Inertia violates triangle inequality (Ixx<=Iyy+Izz, Iyy<=Ixx+Izz, Izz<=Ixx+Iyy)"
        )

    return issues, warnings


def main():
    parser = argparse.ArgumentParser(description="Compute /franka_control/set_load parameters from OBJ + TF")
    parser.add_argument("--mesh", required=True, help="Path to OBJ mesh")
    parser.add_argument("--mass-kg", type=float, default=0.38, help="Payload mass in kg")
    parser.add_argument(
        "--mesh-scale",
        type=float,
        default=1.0,
        help="Mesh coordinate scale to meters (e.g. 0.001 for mm)",
    )
    parser.add_argument(
        "--object-frame",
        type=str,
        default="object_part11",
        help="Object frame name used by TF",
    )
    parser.add_argument(
        "--flange-frame",
        type=str,
        default="panda_link8",
        help="Franka flange frame (usually panda_link8)",
    )
    parser.add_argument(
        "--base-frame",
        type=str,
        default="panda_link0",
        help="Base frame for TF lookup",
    )
    parser.add_argument(
        "--com-offset",
        type=parse_vec3,
        default=np.array([0.0, 0.0, 0.0], dtype=np.float64),
        help="Additional COM offset in object frame [m], format: 'x y z'",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Call /franka_control/set_load service after computing values",
    )
    parser.add_argument(
        "--no-convex-hull",
        action="store_true",
        help="Do not replace non-watertight mesh with convex hull",
    )
    parser.add_argument(
        "--diagonal-inertia-only",
        action="store_true",
        help="Zero out inertia off-diagonal terms (safer for strict firmware checks)",
    )

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("compute_set_load_from_obj", anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    try:
        com_mesh, inertia_obj_com = load_mesh_mass_properties(
            args.mesh,
            args.mesh_scale,
            args.mass_kg,
            not args.no_convex_hull,
        )
        com_object = com_mesh + args.com_offset

        p_flange_com, load_inertia = compute_set_load_params(
            tf_buffer=tf_buffer,
            object_frame=args.object_frame,
            flange_frame=args.flange_frame,
            base_frame=args.base_frame,
            com_object=com_object,
            inertia_object_com=inertia_obj_com,
        )
    except Exception as exc:
        rospy.logerr(f"Failed to compute set_load values: {exc}")
        return 1

    load_inertia_list = [float(v) for v in load_inertia.tolist()]
    p_flange_com_list = [float(v) for v in p_flange_com.tolist()]

    if args.diagonal_inertia_only:
        I = np.array(load_inertia_list, dtype=np.float64).reshape(3, 3, order="F")
        I = np.diag(np.diag(0.5 * (I + I.T)))
        load_inertia_list = [float(v) for v in I.reshape(9, order="F").tolist()]

    issues, warns = validate_set_load_values(args.mass_kg, np.array(p_flange_com_list), load_inertia_list)

    print("\nComputed /franka_control/set_load request:")
    print(f"mass: {args.mass_kg:.6f}")
    print(
        "F_x_center_load: "
        f"[{p_flange_com_list[0]:.9f}, {p_flange_com_list[1]:.9f}, {p_flange_com_list[2]:.9f}]"
    )
    print(
        "load_inertia: [" + ", ".join(f"{v:.12g}" for v in load_inertia_list) + "]"
    )

    com_norm = float(np.linalg.norm(np.array(p_flange_com_list)))
    print(f"|F_x_center_load|: {com_norm:.6f} m")

    if warns:
        print("\nWarnings:")
        for w in warns:
            print(f"- {w}")

    if issues:
        print("\nValidation issues:")
        for i in issues:
            print(f"- {i}")

    print("\nEquivalent rosservice call:")
    print(
        "rosservice call /franka_control/set_load \"mass: "
        f"{args.mass_kg:.6f}\n"
        f"F_x_center_load: [{p_flange_com_list[0]:.9f}, {p_flange_com_list[1]:.9f}, {p_flange_com_list[2]:.9f}]\n"
        "load_inertia: ["
        + ", ".join(f"{v:.12g}" for v in load_inertia_list)
        + "]\""
    )

    if args.apply:
        if issues:
            rospy.logerr("Refusing to call set_load due to validation issues.")
            return 1
        try:
            from franka_msgs.srv import SetLoad, SetLoadRequest

            rospy.wait_for_service("/franka_control/set_load", timeout=2.0)
            proxy = rospy.ServiceProxy("/franka_control/set_load", SetLoad)
            req = SetLoadRequest()
            req.mass = float(args.mass_kg)
            req.F_x_center_load = p_flange_com_list
            req.load_inertia = load_inertia_list

            resp = proxy(req)
            rospy.loginfo(f"set_load response: success={resp.success}, error='{resp.error}'")
        except Exception as exc:
            rospy.logerr(f"Failed to call /franka_control/set_load: {exc}")
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
