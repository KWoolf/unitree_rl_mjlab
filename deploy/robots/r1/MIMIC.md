Running a motion-tracking policy on the R1

This covers policies trained with mjlab's `Mjlab-Tracking-Flat-Unitree-R1-No-State-Estimation` task. The controller has no estimate of base position or linear velocity, so a policy from the state-estimation task cannot run here, and the export script rejects it.

Export the policy and its motion into the controller's config folder. The script reads the joint order, PD gains, action scales, default pose and observation layout from the ONNX metadata, writes them to `deploy.yaml` with the joints mapped onto the R1's motor slots, strips the training-only `time_step` input, and copies the motion. Use the same NPZ the policy was trained on.

The script needs `onnx`, `numpy` and `pyyaml`, which mjlab's environment already has.

```sh
cd deploy/robots/r1
uv run --project /path/to/mjlab python scripts/export_mimic_policy.py --onnx /path/to/run.onnx --motion /path/to/motion.npz
mkdir -p build && cd build && cmake .. && make -j
```

The controller is run with the network interface that reaches the robot, for example `./r1_ctrl --network=enp5s0`. It needs exclusive use of the low-level command channel and exits with a message if another process (the robot's own motion service) is still publishing on it, so put the robot into its low-level development mode first, following Unitree's R1 documentation.

The controller starts in Passive, which applies damping only. LT + up moves to FixStand, which interpolates every joint to the stand pose. From FixStand, RB + A starts the tracking motion. LT + B returns to Passive from FixStand, Velocity or the motion and is the stop to use if anything looks wrong. LT + up from the motion returns to FixStand. The controller also drops to Passive by itself if the torso tilts more than about 57 degrees (1 rad) from upright, returns to FixStand when the motion ends, and drops to Passive if the low-level state stream from the robot times out.

For the first run, hang the robot from a gantry with the feet just touching the ground and the strap slack enough that it does not carry weight in normal stance but catches a fall. Keep one person on the gamepad with a thumb ready on LT + B and a second person on the robot's hardware emergency stop. Start with a short window by setting `time_end` in `config/config.yaml` (under `Mimic`) to two or three seconds, and pick a motion whose first frame is close to the stand pose, since the policy starts tracking from that frame immediately and a large difference produces a sudden jump. Check FixStand first and confirm every joint holds still and the head and waist are the right way round before pressing RB + A. Extend `time_end` and lower the gantry in stages only once the short window looks the same as the policy did in mjlab's play viewer.

At the start of the motion the controller aligns the reference heading with the robot's current heading, so the robot can face any direction when the motion begins. It does not correct position, so the robot will drift relative to where the original performer moved.
