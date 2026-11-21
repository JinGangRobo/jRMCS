#!/usr/bin/env bash
set -euo pipefail

sdk=${1:?Usage: bash run_simulate.sh MUJOCO_SDK_ROOT MUJOCO_PYTHON}
python=${2:?Use the Python executable from the existing MuJoCo environment}
here=$(cd -- "$(dirname -- "$0")" && pwd)
work=$(mktemp -d -t jrmcs-simulate.XXXXXXXX)
trap 'rm -rf -- "$work"' EXIT

"${CXX:-c++}" -std=c++20 -O2 -Wall -Wextra -Wpedantic -shared -fPIC \
    -I"$sdk/include" "$here/src/simulate_control.cpp" \
    -L"$sdk/lib" -Wl,-rpath,"$sdk/lib" -lmujoco -o "$work/librmcs_simulate_control.so"

"$python" - "$here/../../rmcs_ws/src/arm_description" "$work/JGarm.xml" <<'PY'
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

package = Path(sys.argv[1]).resolve()
robot = ET.parse(package / 'urdf/JGarm.urdf').getroot()
for mesh in robot.iter('mesh'):
    path = mesh.attrib['filename'].removeprefix('package://arm_description/')
    mesh.set('filename', str(package / path))
ET.SubElement(ET.SubElement(robot, 'mujoco'), 'compiler', {'strippath': 'false'})
spec = mujoco.MjSpec.from_string(ET.tostring(robot, encoding='unicode'))

# Start at MoveIt's Gripper_open pose, keeping both slide joints.
for i in (8, 9):
    name = f'joint{i}'
    joint = spec.joint(name)
    body = spec.body(f'Link{i}')
    if joint.type != mujoco.mjtJoint.mjJNT_SLIDE:
        raise ValueError(f'Expected a gripper slider: {name}')
    opened = joint.range[i - 8]
    shift = np.empty(3)
    mujoco.mju_rotVecQuat(shift, joint.axis * (opened - joint.ref), body.quat)
    body.pos += shift
    joint.ref = opened

# Both rack displacements are measured from the open pose and have opposite signs.
rack = spec.add_equality(name='gripper_rack', type=mujoco.mjtEq.mjEQ_JOINT,
                         name1='joint8', name2='joint9', objtype=mujoco.mjtObj.mjOBJ_JOINT)
rack.data[:5] = [0.0, -1.0, 0.0, 0.0, 0.0]
# Keep the two racks oppositely coupled under gravity, with a time constant
# of two physics steps at 1 kHz.
rack.solref[0] = 0.002
rack.solimp[:2] = 0.9999

model = spec.compile()
scene = ET.fromstring(spec.to_xml())

# MJCF export rounds numeric fields. Preserve the geometry and inertial data
# used by the URDF gravity model so tiny residual torques do not build up.
def vector_text(values):
    return ' '.join(f'{value:.17g}' for value in values)


for element in scene.findall('./worldbody//body'):
    body = model.body(element.attrib['name'])
    element.attrib.update(pos=vector_text(body.pos), quat=vector_text(body.quat))
    inertial = element.find('inertial')
    if inertial is not None:
        inertial.attrib.update(
            pos=vector_text(body.ipos), quat=vector_text(body.iquat),
            mass=f'{body.mass[0]:.17g}', diaginertia=vector_text(body.inertia),
        )

for element in scene.findall('./worldbody//joint'):
    joint = model.joint(element.attrib['name'])
    element.attrib.update(pos=vector_text(joint.pos), axis=vector_text(joint.axis))

option = scene.find('option')
if option is None:
    option = ET.SubElement(scene, 'option')
option.attrib.update(timestep='0.001', gravity='0 0 -9.81')
ET.SubElement(option, 'flag', {'contact': 'disable', 'gravity': 'disable'})

actuators = ET.SubElement(scene, 'actuator')
for i in range(7):
    name = f'joint{i + 1}'
    joint = model.joint(name)
    if joint.qposadr[0] != i or joint.dofadr[0] != i:
        raise ValueError('The JGarm joint order differs from J1–J7')
    lower, upper = joint.range
    # The official Control panel writes angle targets. RMCS supplies all applied torque.
    ET.SubElement(actuators, 'general', {
        'name': f'rmcs_J{i + 1}', 'joint': name, 'gainprm': '0',
        'ctrllimited': 'true', 'ctrlrange': f'{lower:.17g} {upper:.17g}',
    })

# A virtual 10 mm pinion gives a transmission of -100 rad/m at joint9.
# The control slider expresses closure, from fully open (0) to closed (1).
ET.SubElement(actuators, 'general', {
    'name': 'rmcs_gripper', 'joint': 'joint9', 'gear': '-100', 'gainprm': '0',
    'ctrllimited': 'true', 'ctrlrange': '0 1',
})
ET.ElementTree(scene).write(sys.argv[2], encoding='unicode')
PY

LD_PRELOAD="$work/librmcs_simulate_control.so${LD_PRELOAD:+:$LD_PRELOAD}" \
    "$sdk/bin/simulate" "$work/JGarm.xml"
