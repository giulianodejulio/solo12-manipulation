"""Portable, frozen robot assembly used by the successful grasp demo."""
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/support_front_crossmember/dynamics'


def fmt(values):
    return ' '.join('{:.12g}'.format(float(v)) for v in np.asarray(values).flat)


def load_robot():
    scene = ROOT/'assets/robot.xml'
    root = ET.parse(scene).getroot()
    root.find('compiler').set('meshdir', str(scene.parent))
    try:
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    except ValueError as exc:
        raise RuntimeError('Unable to load demo assets. Run: python3 tools/fetch_assets.py') from exc
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return root, model, data
