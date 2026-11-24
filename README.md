# Solo12 + OpenMANIPULATOR-X: red-cube grasp demo

Minimal reproduction of the final MuJoCo demonstration: Solo12 lowers its legs,
OpenMANIPULATOR-X grasps a red cube, lifts it, and keeps it suspended while the
legs extend again. The object is a **free rigid body**, held by frictional finger
contacts: no weld, attachment constraint, prescribed object trajectory, or RL.

The sequence lasts **17 s**. The cube is 30 mm wide and weighs 50 g; in the
validated run its bottom remains approximately 20 cm above the floor during the
last 3 s. The gripper uses feedback IK with the current floating-base pose.

Only the successful demo's model, support meshes, controller, renderer, dependency
pins and asset downloader are included. Earlier designs, failed attempts,
trajectories, generated images/GIFs and unrelated workspace files are excluded.

## Docker: build and reproduce the English GIF

```bash
git clone https://github.com/giulianodejulio/solo12-manipulation.git
cd solo12-manipulation
docker build -t solo12-manipulation:demo .
bash run_demo.sh --render --language en
```

The build downloads required upstream meshes at pinned revisions and verifies
SHA-256 hashes. No sibling workspace, ROS, Isaac Gym, Gazebo or GPU is required.

Output:

```text
results/support_front_crossmember/dynamics/grasp_pickup/
├── grasp_pickup_en.gif
├── dynamic_scene.xml
├── trajectory.npz
├── report.json
├── start.png
└── end.png
```

The GIF contains the full simulation at 20 fps. `report.json` checks that both
fingers hold the lifted object during the final window. For Italian captions,
use `--language it`; the output is `grasp_pickup.gif`.

## Interactive viewer (Linux, X11)

```bash
bash run_demo.sh --viewer
```

The script passes the desktop X11 authentication file without `xhost +` or a
privileged container. Set `DISPLAY` and `XAUTHORITY` if your session does not use
the usual GDM path. Close the viewer to stop early; a full recorded demo requires
allowing the entire 17 s to complete. Set `SOLO12_MANIPULATION_IMAGE` to override
the Docker image name.

## Native Python (3.10)

On Debian/Ubuntu, install `libosmesa6` for headless rendering and `libglfw3` for
the interactive viewer. Create and activate a Python 3.10 virtual environment:

```bash
python3.10 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python tools/fetch_assets.py
MUJOCO_GL=osmesa OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python simulate_grasp.py --render --language en
# Desktop viewer:
MUJOCO_GL=glfw python simulate_grasp.py --viewer
```

## Files

- `simulate_grasp.py`: free cube/contact model, arm IK and coordinated controller.
- `manipulation/demo_model.py`: portable loader and project output location.
- `manipulation/animation.py`: GIF generation from saved states; no second physics run.
- `assets/robot.xml`: frozen initial folded assembly and motor setup, relative paths.
- `assets/support/`: only the four support parts used in the final demonstration.
- `assets/manifest.json`, `tools/fetch_assets.py`: exact external mesh inputs.
- `requirements.txt`, `Dockerfile`, `run_demo.sh`: reproducible runtime and launcher.

Downloaded meshes and generated results are ignored by Git. To regenerate only
the GIF from an existing successful trajectory:

```bash
MUJOCO_GL=osmesa python -c "from manipulation.animation import transition_gif; from manipulation.demo_model import OUT; from simulate_grasp import phase; transition_gif(OUT/'grasp_pickup', phase_function=lambda t: phase(t, 'en'), output_name='grasp_pickup_en.gif')"
```

## Scope and modeling assumptions

This is an initial simulation proof of concept, not a hardware-validated grasp or
a structural certification. The bracket is rigid; bolt seats, joint compliance,
loads and manufacturability still require engineering verification. Arm/self/robot
collision response is not enabled. The successful local trajectory passed sampled
conservative CAD-clearance checks; this is not continuous collision certification.

Finger collision pads approximate the existing flat CAD faces. Friction 1,
maximum 2 N per finger and the arm torque limit 3 Nm are test assumptions, not
verified continuous hardware ratings. The contact solver uses Newton, elliptic
cones and `impratio=100` to reduce soft-contact creep; NoSlip is disabled. Cube-floor
contact disappears after pickup. The original 594 g arm inertials are retained;
the difference from approximately 700 g quoted hardware mass remains unresolved.
Numerical results can vary across platforms; versions match the validated CPU run.

Asset sources, revisions and licenses are documented in [THIRD_PARTY.md](THIRD_PARTY.md).
