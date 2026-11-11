"""Package an mjlab R1 tracking policy for the r1_ctrl Mimic state.

mjlab exports policy.onnx with the joint gains, action scales, default pose and
observation layout stored as metadata. This script reads that metadata and
writes the policy directory r1_ctrl expects:

    <out>/exported/policy.onnx   actor only (obs -> actions)
    <out>/params/deploy.yaml     joint map, gains, actions and observations
    <out>/params/motion.npz      the reference motion from csv_to_npz

mjlab's ONNX also takes a time_step input and returns the reference motion,
which r1_ctrl does not use, so only the obs -> actions subgraph is kept.

Usage:
    python scripts/export_mimic_policy.py \
        --onnx /path/to/mjlab/logs/rsl_rl/r1_tracking/<run>/<run>.onnx \
        --motion /path/to/motion.npz
"""

import argparse
import shutil
import tempfile
from pathlib import Path

import numpy as np
import onnx
from onnx.utils import Extractor

# Policy joint name -> SDK motor slot, from unitree_sdk2's
# include/unitree/dds_wrapper/robots/r1/defines.h.
R1_MOTOR_SLOTS = {
    "left_hip_pitch_joint": 0,
    "left_hip_roll_joint": 1,
    "left_hip_yaw_joint": 2,
    "left_knee_joint": 3,
    "left_ankle_pitch_joint": 4,
    "left_ankle_roll_joint": 5,
    "right_hip_pitch_joint": 6,
    "right_hip_roll_joint": 7,
    "right_hip_yaw_joint": 8,
    "right_knee_joint": 9,
    "right_ankle_pitch_joint": 10,
    "right_ankle_roll_joint": 11,
    "waist_roll_joint": 12,
    "waist_yaw_joint": 13,
    "left_shoulder_pitch_joint": 15,
    "left_shoulder_roll_joint": 16,
    "left_shoulder_yaw_joint": 17,
    "left_elbow_joint": 18,
    "left_wrist_roll_joint": 19,
    "right_shoulder_pitch_joint": 22,
    "right_shoulder_roll_joint": 23,
    "right_shoulder_yaw_joint": 24,
    "right_elbow_joint": 25,
    "right_wrist_roll_joint": 26,
    "head_pitch_joint": 29,
    "head_yaw_joint": 30,
}

# mjlab observation term -> r1_ctrl observation, in the order the actor reads
# them. These are the terms of Mjlab-Tracking-Flat-Unitree-R1-No-State-Estimation;
# r1_ctrl has no estimate of the anchor position or base linear velocity that
# the state-estimation variant also uses.
OBSERVATIONS = {
    "command": "motion_command",
    "motion_anchor_ori_b": "motion_anchor_ori_b",
    "base_ang_vel": "base_ang_vel",
    "joint_pos": "joint_pos_rel",
    "joint_vel": "joint_vel_rel",
    "actions": "last_action",
}


def floats(text):
    return [float(x) for x in text.split(",")]


def fmt(values):
    return "[" + ", ".join(f"{v:.6g}" for v in values) + "]"


def read_metadata(model):
    meta = {p.key: p.value for p in model.metadata_props}
    missing = {"joint_names", "joint_stiffness", "joint_damping",
               "default_joint_pos", "action_scale", "observation_names"} - set(meta)
    if missing:
        raise ValueError(f"ONNX metadata is missing {sorted(missing)}")
    return meta


def term_sizes(num_joints):
    return {
        "command": 2 * num_joints,
        "motion_anchor_ori_b": 6,
        "base_ang_vel": 3,
        "joint_pos": num_joints,
        "joint_vel": num_joints,
        "actions": num_joints,
    }


def actor_only(model, path):
    """Keep only the obs -> actions part of the graph."""
    inputs = [i.name for i in model.graph.input]
    outputs = [o.name for o in model.graph.output]
    if "obs" not in inputs or "actions" not in outputs:
        raise ValueError(f"Expected an obs input and actions output, got {inputs} -> {outputs}")
    actor = Extractor(model).extract_model(["obs"], ["actions"])
    actor.metadata_props.extend(model.metadata_props)
    onnx.checker.check_model(actor)
    onnx.save(actor, str(path))


def deploy_yaml(meta, num_obs):
    joints = meta["joint_names"].split(",")
    unknown = [j for j in joints if j not in R1_MOTOR_SLOTS]
    if unknown:
        raise ValueError(f"No R1 motor slot for {unknown}")

    names = meta["observation_names"].split(",")
    if names != list(OBSERVATIONS):
        raise ValueError(
            f"Observation terms {names} do not match {list(OBSERVATIONS)}. "
            "Train with Mjlab-Tracking-Flat-Unitree-R1-No-State-Estimation."
        )
    sizes = term_sizes(len(joints))
    if sum(sizes.values()) != num_obs:
        raise ValueError(f"Policy takes {num_obs} observations, expected {sum(sizes.values())}")
    scales = floats(meta.get("observation_terms_scale", ",".join(["1"] * len(names))))

    default = floats(meta["default_joint_pos"])
    lines = [
        f"# Generated from the policy's ONNX metadata by {Path(__file__).name}.",
        f"# Policy joint order: {', '.join(joints)}",
        "joint_ids_map: " + fmt(R1_MOTOR_SLOTS[j] for j in joints),
        "step_dt: 0.02",
        "stiffness: " + fmt(floats(meta["joint_stiffness"])),
        "damping: " + fmt(floats(meta["joint_damping"])),
        "default_joint_pos: " + fmt(default),
        "commands: {}",
        "actions:",
        "  JointPositionAction:",
        "    clip: null",
        "    joint_names: [.*]",
        "    scale: " + fmt(floats(meta["action_scale"])),
        "    offset: " + fmt(default),
        "    joint_ids: null",
        "observations:",
    ]
    for (term, deploy_name), scale in zip(OBSERVATIONS.items(), scales):
        params = "{command_name: motion}" if deploy_name.startswith("motion_") else "{}"
        lines += [
            f"  {deploy_name}:",
            f"    params: {params}",
            "    clip: null",
            "    scale: " + fmt([scale] * sizes[term]),
            "    history_length: 1",
        ]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--onnx", required=True, type=Path, help="Policy exported by mjlab.")
    parser.add_argument("--motion", required=True, type=Path, help="NPZ from csv_to_npz.")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config/policy/mimic/current",
        help="Policy directory to write (default: the one config.yaml's Mimic state uses).",
    )
    args = parser.parse_args()

    model = onnx.load(str(args.onnx))
    meta = read_metadata(model)
    obs_dims = [d.dim_value for d in next(i for i in model.graph.input if i.name == "obs").type.tensor_type.shape.dim]

    motion = np.load(args.motion)
    num_joints = len(meta["joint_names"].split(","))
    if motion["joint_pos"].shape[1] != num_joints:
        raise ValueError(f"{args.motion} has {motion['joint_pos'].shape[1]} joints, policy has {num_joints}")
    if not np.isclose(float(np.ravel(motion["fps"])[0]), 50.0):
        raise ValueError(f"{args.motion} is at {motion['fps']} fps; r1_ctrl plays motions at 50 fps")

    yaml_text = deploy_yaml(meta, obs_dims[-1])
    (args.out / "exported").mkdir(parents=True, exist_ok=True)
    (args.out / "params").mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        actor_path = Path(tmp) / "policy.onnx"
        actor_only(model, actor_path)
        shutil.copy(actor_path, args.out / "exported" / "policy.onnx")
    (args.out / "params" / "deploy.yaml").write_text(yaml_text)
    shutil.copy(args.motion, args.out / "params" / "motion.npz")
    duration = (motion["joint_pos"].shape[0] - 1) / 50.0
    print(f"Wrote {args.out} ({num_joints} joints, {obs_dims[-1]} observations, {duration:.1f} s motion)")


if __name__ == "__main__":
    main()
