#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wheel_leg_real2sim.py -- 实车 -> MuJoCo 显示

链路:
    实车 value_broadcaster 发 6 个 /chassis/*/angle (std_msgs/msg/Float64)
      + WheelLegImuBroadcaster 发 /wheel_leg/imu/data (sensor_msgs/msg/Imu)
      -> Foxglove bridge (ws://127.0.0.1:8080)
      -> 本脚本: Python 连杆解算 -> 串联 mjmodel.xml 显示
                 IMU 四元数 -> 底座 free joint 姿态

依赖: mujoco, websockets   (不需要 rosbags)
安装: micromamba run -n mujoco pip install websockets

运行:
    python wheel_leg_real2sim.py \
      --model /znchi/workspace/fudan_rl_wheel_leg/mujoco/assert_now/wheeleg_urdf/meshes/mjmodel.xml
"""
import argparse
import asyncio
import json
import math
import struct
import threading
import time

import mujoco
import mujoco.viewer

URL_DEFAULT = "ws://127.0.0.1:8080"
SUBPROTOCOLS = ("foxglove.sdk.v1", "foxglove.websocket.v1")

TOPICS = {
    "/chassis/left_front_hip/angle":  "rf0",
    "/chassis/left_back_hip/angle":   "r20",
    "/chassis/left_wheel/angle":      "rw",
    "/chassis/right_front_hip/angle": "lf0",
    "/chassis/right_back_hip/angle":  "l20",
    "/chassis/right_wheel/angle":     "lw",
}
IMU_TOPIC = "/wheel_leg/imu/data"

JOINTS = ["lf0_Joint", "lf1_Joint", "l_wheel_Joint",
          "rf0_Joint", "rf1_Joint", "r_wheel_Joint"]


# ---------------------- CDR (sensor_msgs/Imu) ----------------------
class CdrReader:
    """极简 CDR 读取器: 只实现 Imu 需要的字段, 对齐相对消息体起点。"""

    def __init__(self, buf, body_start):
        self.buf = buf
        self.start = body_start
        self.off = 0

    def _align(self, n):
        r = self.off % n
        if r:
            self.off += n - r

    def i32(self):
        self._align(4)
        v = struct.unpack_from("<i", self.buf, self.start + self.off)[0]
        self.off += 4
        return v

    def u32(self):
        self._align(4)
        v = struct.unpack_from("<I", self.buf, self.start + self.off)[0]
        self.off += 4
        return v

    def f64(self):
        self._align(8)
        v = struct.unpack_from("<d", self.buf, self.start + self.off)[0]
        self.off += 8
        return v

    def string(self):
        n = self.u32()
        s = self.buf[self.start + self.off:self.start + self.off + n].decode("utf-8", "replace")
        self.off += n
        return s


def parse_imu_cdr(pkt):
    """解析 Foxglove CDR 二进制帧里的 sensor_msgs/Imu。

    帧布局: [opcode=1][subId u32][ts u64][CDR 封装头 4B][消息体]
    -> body_start = 1 + 4 + 8 + 4 = 17

    返回 ((qx, qy, qz, qw), (wx, wy, wz))。
    """
    c = CdrReader(pkt, 17)
    c.i32(); c.u32()          # header.stamp.sec / nanosec
    c.string()               # header.frame_id
    quat = tuple(c.f64() for _ in range(4))   # orientation x, y, z, w
    for _ in range(9):
        c.f64()              # orientation_covariance
    gyro = tuple(c.f64() for _ in range(3))   # angular_velocity x, y, z
    return quat, gyro


# ---------------------- 四元数工具 (w, x, y, z) ----------------------
def quat_normalize(q):
    n = math.sqrt(sum(x * x for x in q))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    w, x, y, z = q
    return (w / n, x / n, y / n, z / n)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


# ---------------------- 连杆解算 ----------------------
def solve_leg_geometry(phi1, phi4, l1, l2):
    x_b, y_b = l1 * math.cos(phi1), l1 * math.sin(phi1)
    x_d, y_d = l1 * math.cos(phi4), l1 * math.sin(phi4)
    dx, dy = x_d - x_b, y_d - y_b
    a0, b0, c0 = 2 * l2 * dx, 2 * l2 * dy, dx * dx + dy * dy
    disc = max(a0 * a0 + b0 * b0 - c0 * c0, 0.0)
    phi2 = 2 * math.atan2(b0 + math.sqrt(disc), a0 + c0)
    x_c, y_c = x_b + l2 * math.cos(phi2), y_b + l2 * math.sin(phi2)
    phi3 = math.atan2(y_c - y_d, x_c - x_d)
    return phi2, phi3, math.atan2(y_c, x_c), math.hypot(x_c, y_c)


def solve_phi3_left(phi1, phi4, l1, l2):
    return solve_leg_geometry(phi1, phi4, l1, l2)[1] - phi4 - math.pi / 2


def solve_phi3_right(phi1, phi4, l1, l2):
    return -solve_leg_geometry(phi1, phi4, l1, l2)[1] + phi4 + math.pi / 2


class LegLinkage:
    """lf0/l20 = 同一侧两个电机角; 输出虚拟膝角 lf1。"""

    def __init__(self, l1=0.175, l2=0.208, offset=1.6614):
        self.l1, self.l2, self.offset = l1, l2, offset

    def left(self, lf0, l20):
        return solve_phi3_left(self.offset + l20, lf0, self.l1, self.l2)

    def right(self, rf0, r20):
        return solve_phi3_right(self.offset - r20, -rf0, self.l1, self.l2)


# ---------------------- Foxglove 订阅 ----------------------
class FoxgloveReader(threading.Thread):
    def __init__(self, url, values, status, dump_all=False):
        super().__init__(daemon=True)
        self.url = url
        self.values = values
        self.status = status
        self.dump_all = dump_all
        self.received = {}
        self.seen_channels = set()
        self.matched = set()
        self.stopped = threading.Event()

    def run(self):
        try:
            asyncio.run(self._recv())
        except Exception as e:  # noqa: BLE001
            self.status[0] = "reader died: %s" % e
            print("[reader] died:", e)

    async def _recv(self):
        import websockets
        n_want = len(TOPICS) + 1
        while not self.stopped.is_set():
            try:
                async with websockets.connect(
                    self.url, subprotocols=list(SUBPROTOCOLS),
                    open_timeout=3, close_timeout=1,
                    max_size=None, max_queue=64,
                ) as ws:
                    self.status[0] = "connected"
                    print("[reader] connected: %s" % self.url)
                    sub2key = {}
                    next_id = 1
                    while not self.stopped.is_set():
                        try:
                            pkt = await asyncio.wait_for(ws.recv(), timeout=1.0)
                        except asyncio.TimeoutError:
                            continue

                        # 文本帧: 控制消息 (advertise 等)
                        if isinstance(pkt, str):
                            ev = json.loads(pkt)
                            if ev.get("op") == "advertise":
                                subs = []
                                for ch in ev["channels"]:
                                    topic = ch.get("topic")
                                    if self.dump_all:
                                        print("[adv] %s enc=%s schema=%s" % (
                                            topic, ch.get("encoding"), ch.get("schemaName")))
                                    if topic in TOPICS:
                                        key = TOPICS[topic]
                                    elif topic == IMU_TOPIC:
                                        key = "__imu__"
                                    else:
                                        continue
                                    if ch["id"] in self.seen_channels:
                                        continue
                                    self.seen_channels.add(ch["id"])
                                    sid = next_id
                                    next_id += 1
                                    sub2key[sid] = (key, ch.get("encoding"))
                                    self.matched.add(topic)
                                    subs.append({"id": sid, "channelId": ch["id"]})
                                if subs:
                                    await ws.send(json.dumps(
                                        {"op": "subscribe", "subscriptions": subs}))
                                    print("[reader] subscribed %d/%d: %s" % (
                                        len(self.matched), n_want, sorted(self.matched)))

                        # 二进制帧: opcode 1 = 消息数据
                        # <I subscriptionId> <Q timestamp> <payload...>
                        elif pkt and pkt[0] == 1:
                            sid, _ts = struct.unpack_from("<IQ", pkt, 1)
                            got = sub2key.get(sid)
                            if got is None:
                                continue
                            key, enc = got
                            try:
                                if key == "__imu__":
                                    if enc == "json":
                                        m = json.loads(pkt[13:])
                                        o, g = m["orientation"], m["angular_velocity"]
                                        quat = (o["x"], o["y"], o["z"], o["w"])
                                        gyro = (g["x"], g["y"], g["z"])
                                    else:
                                        quat, gyro = parse_imu_cdr(pkt)
                                    self.values["imu"] = (quat, gyro)
                                    self.received["imu"] = self.received.get("imu", 0) + 1
                                else:
                                    if enc == "json":
                                        val = float(json.loads(pkt[13:]))
                                    else:
                                        # CDR: 4B 封装头 + 8B double
                                        val = struct.unpack_from("<d", pkt, 17)[0]
                                    self.values[key] = val
                                    self.received[key] = self.received.get(key, 0) + 1
                                self.status[0] = "receiving"
                            except Exception:  # noqa: BLE001
                                continue
            except Exception as e:  # noqa: BLE001
                self.status[0] = "disconnected: %s" % e
                print("[reader] %s" % self.status[0])
                await asyncio.sleep(1.0)

    def close(self):
        self.stopped.set()
        self.join(timeout=5)


def disable_skybox(m):
    """尽力关闭天空盒纹理(纯装饰); 部分 mujoco 版本不支持则跳过。"""
    try:
        if not hasattr(m, "tex_type"):
            print("[skybox] 该 mujoco 版本无 tex_type, 跳过")
            return
        role = int(mujoco.mjtTexture.mjTEXTURE_SKYBOX)
        n = 0
        for i in range(m.ntex):
            if int(m.tex_type[i]) == role:
                m.tex_type[i] = int(mujoco.mjtTexture.mjTEXTURE_2D)
                n += 1
        print("[skybox] disabled %d skybox texture(s)" % n)
    except Exception as e:  # noqa: BLE001
        print("[skybox] skip:", e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=URL_DEFAULT)
    ap.add_argument("--model", required=True)
    ap.add_argument("--offset", type=float, default=1.6614)
    ap.add_argument("--l1", type=float, default=0.175)
    ap.add_argument("--l2", type=float, default=0.208)
    # 位形镜像/反了就改这四个符号(先全 +1)
    ap.add_argument("--sgn-lf0", type=float, default=1.0)
    ap.add_argument("--sgn-l20", type=float, default=1.0)
    ap.add_argument("--sgn-rf0", type=float, default=1.0)
    ap.add_argument("--sgn-r20", type=float, default=1.0)
    # IMU 系 -> 机体系 的固定安装修正 (单位: 度); 装正了就是全 0
    ap.add_argument("--imu-roll-deg", type=float, default=0.0)
    ap.add_argument("--imu-pitch-deg", type=float, default=0.0)
    ap.add_argument("--imu-yaw-deg", type=float, default=0.0)
    ap.add_argument("--no-skybox", action="store_true")
    ap.add_argument("--dump-all", action="store_true",
                    help="打印 bridge 广播的所有话题")
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(args.model)
    d = mujoco.MjData(m)
    if args.no_skybox:
        disable_skybox(m)

    qa = {}
    for name in JOINTS:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise SystemExit("[FATAL] joint %r not found in model" % name)
        qa[name] = int(m.jnt_qposadr[jid])
    print("[model] %s" % args.model)

    # 浮动底座: 找模型里的 free joint
    free_adr = None
    for j in range(m.njnt):
        if int(m.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE):
            free_adr = int(m.jnt_qposadr[j])
            print("[imu] base free joint: %s (qposadr=%d)" % (
                mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j), free_adr))
            break
    if free_adr is None:
        print("[imu] [WARN] 模型里没有 free joint, 无法用 IMU 驱动底座姿态(只显示关节)")

    q_mount = euler_to_quat(
        math.radians(args.imu_roll_deg),
        math.radians(args.imu_pitch_deg),
        math.radians(args.imu_yaw_deg))
    if q_mount != (1.0, 0.0, 0.0, 0.0):
        print("[imu] mount correction rpy = (%.1f, %.1f, %.1f) deg" % (
            args.imu_roll_deg, args.imu_pitch_deg, args.imu_yaw_deg))

    kin = LegLinkage(args.l1, args.l2, args.offset)
    values = {k: 0.0 for k in TOPICS.values()}
    values["imu"] = None
    status = ["waiting"]
    reader = FoxgloveReader(args.url, values, status, dump_all=args.dump_all)
    reader.start()

    last = 0.0
    try:
        with mujoco.viewer.launch_passive(m, d) as v:
            while v.is_running():
                lf0 = args.sgn_lf0 * values["lf0"]
                l20 = args.sgn_l20 * values["l20"]
                rf0 = args.sgn_rf0 * values["rf0"]
                r20 = args.sgn_r20 * values["r20"]

                lf1 = kin.left(lf0, l20)
                rf1 = kin.right(rf0, r20)

                d.qpos[qa["lf0_Joint"]]     = lf0
                d.qpos[qa["lf1_Joint"]]     = lf1
                d.qpos[qa["l_wheel_Joint"]] = values["lw"]
                d.qpos[qa["rf0_Joint"]]     = rf0
                d.qpos[qa["rf1_Joint"]]     = rf1
                d.qpos[qa["r_wheel_Joint"]] = values["rw"]

                # IMU (q_WB, body->world) -> 浮动底座姿态
                imu = values["imu"]
                if free_adr is not None and imu is not None:
                    qx, qy, qz, qw = imu[0]
                    q = quat_normalize((qw, qx, qy, qz))
                    if q_mount != (1.0, 0.0, 0.0, 0.0):
                        q = quat_mul(q, q_mount)
                    d.qpos[free_adr + 3] = q[0]
                    d.qpos[free_adr + 4] = q[1]
                    d.qpos[free_adr + 5] = q[2]
                    d.qpos[free_adr + 6] = q[3]

                mujoco.mj_forward(m, d)
                v.sync()

                if time.monotonic() - last > 1.0:
                    last = time.monotonic()
                    if imu is None:
                        imu_txt = "none"
                    else:
                        imu_txt = "q=(%.3f,%.3f,%.3f,%.3f) g=(%.2f,%.2f,%.2f)" % (
                            imu[0][0], imu[0][1], imu[0][2], imu[0][3],
                            imu[1][0], imu[1][1], imu[1][2])
                    print("[status] %s  imu=%s  %s" % (
                        status[0], imu_txt,
                        {k: round(x, 3) for k, x in values.items()
                         if isinstance(x, float)}))
    finally:
        reader.close()


if __name__ == "__main__":
    main()
