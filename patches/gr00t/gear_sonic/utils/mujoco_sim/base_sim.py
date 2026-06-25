"""MuJoCo simulation environment and loop for the G1 (and H1) humanoid robots.

DefaultEnv owns the MuJoCo model/data, computes PD torques from Unitree SDK
commands, steps physics, and publishes observations back via the SDK bridge.
BaseSimulator wraps DefaultEnv with rate-limiting and viewer/image update loops.
"""

import os
import pathlib
from pathlib import Path
import pickle
import tempfile
from threading import Lock, Thread, Event
import time
from typing import Dict
import xml.etree.ElementTree as ET

import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from gear_sonic.utils.mujoco_sim.metric_utils import check_contact, check_height
from gear_sonic.utils.mujoco_sim.sim_utils import get_subtree_body_names
from gear_sonic.utils.mujoco_sim.unitree_sdk2py_bridge import ElasticBand, UnitreeSdk2Bridge
from gear_sonic.utils.mujoco_sim.robot import Robot
try:
    from gear_sonic.utils.mujoco_sim.robot_visibility import check_robot_in_user_view as _check_visibility
except ImportError:
    _check_visibility = None

GEAR_SONIC_ROOT = Path(os.path.abspath(__file__)).parent.parent.parent.parent


def _detect_wsl():
    """WSL 上で実行されているかを判定する。

    WSL では GPU 描画が d3d12 半仮想化層を経由して非常に重く、制御ループや
    マウス操作に影響する。そのため WSL のときだけ各種パフォーマンス最適化を
    既定で有効にし、ネイティブ Linux では元の挙動（画質そのまま）を維持する。
    """
    try:
        if os.path.exists("/dev/dxg"):
            return True
        with open("/proc/version") as f:
            return "microsoft" in f.read().lower()
    except Exception:
        return False


_IS_WSL = _detect_wsl()


class DefaultEnv:
    """Base environment class that handles simulation environment setup and step"""

    def __init__(
        self,
        config: Dict[str, any],
        env_name: str = "default",
        camera_configs: Dict[str, any] = {},
        onscreen: bool = False,
        offscreen: bool = False,
        enable_image_publish: bool = False,
    ):
        self.config = config
        self.env_name = env_name
        self.robot = Robot(self.config)
        self.num_body_dof = self.robot.NUM_JOINTS
        self.num_hand_dof = self.robot.NUM_HAND_JOINTS
        self.sim_dt = self.config["SIMULATE_DT"]
        self.obs = None
        self.torques = np.zeros(self.num_body_dof + self.num_hand_dof * 2)
        self.torque_limit = np.array(self.robot.MOTOR_EFFORT_LIMIT_LIST)
        self.camera_configs = camera_configs

        self.reward_lock = Lock()
        self.unitree_bridge = None
        self.onscreen = onscreen

        self.init_scene()
        self.last_reward = 0
        self._camera_index = 0
        self._user_yaw = 2  # 0=+y(N) 1=-x(W) 2=-y(S,初期) 3=+x(E)
        self._active_stream  = "user_eye"
        self._viewer_proc    = None
        self._viewer_last_t  = 0.0

        self.offscreen = offscreen
        self._enable_image_publish = enable_image_publish
        # 離屏レンダリングを制御ループから切り離すためのレンダースレッド用ステート。
        # _render_lock は mj_step とレンダー用スナップショットを排他する（保持は数μs）。
        self._render_lock = Lock()
        self._render_stop = Event()
        self._render_thread = None
        # 離屏レンダリングを別スレッドへ分離するか。既定は全プラットフォームで有効。
        # これは「同期レンダリングが制御ループ(200Hz)を塞ぐ」という通信レート低下バグの
        # 本質的な修正であり、出力画像は一切変わらない（透過的なリアルタイム性改善）。
        # 描画が遅くなる状況は WSL(d3d12) に限らず、Linux サーバの
        # ヘッドレス/リモート/ソフトウェア(osmesa,llvmpipe)レンダリングでも起きるため、
        # プラットフォームで門限せず常に分離する。GPU が速い環境ではオーバーヘッドは無視できる。
        # SIM_RENDER_THREAD=0 で元の同期パスに戻せる。
        self._use_render_thread = (
            enable_image_publish
            and bool(self.camera_configs)
            and os.environ.get("SIM_RENDER_THREAD", "1") == "1"
        )
        if self.offscreen and not self._use_render_thread:
            # 同期レンダリングパス（publishなし、または Linux 既定）は self.renderers を使う
            self.init_renderers()
        # 離屏レンダリング周期。SIM_IMAGE_DT で上書き可（大きくすると離屏fpsが下がり、
        # ウィンドウ側に GPU が回ってマウス操作が滑らかになる。例: SIM_IMAGE_DT=0.1 → 10Hz）。
        self.image_dt = float(os.environ.get("SIM_IMAGE_DT", self.config.get("IMAGE_DT", 0.033333)))
        self.image_publish_process = None

    def start_image_publish_subprocess(self, start_method: str = "spawn", camera_port: int = 5555):
        from gear_sonic.utils.mujoco_sim.image_publish_utils import ImagePublishProcess

        if len(self.camera_configs) == 0:
            print(
                "Warning: No camera configs provided, image publishing subprocess will not be started"
            )
            return
        start_method = self.config.get("MP_START_METHOD", "spawn")
        self.image_publish_process = ImagePublishProcess(
            camera_configs=self.camera_configs,
            image_dt=self.image_dt,
            zmq_port=camera_port,
            start_method=start_method,
            verbose=self.config.get("verbose", False),
        )
        self.image_publish_process.start_process()
        # 離屏レンダリングを専用スレッドへ。制御ループ(200Hz)を render() で塞がないため。
        self.start_render_thread()

    def _get_dof_indices_by_class(self):
        with tempfile.NamedTemporaryFile(mode="w+", delete=False, suffix=".xml") as f:
            mujoco.mj_saveLastXML(f.name, self.mj_model)
            temp_xml_path = f.name

        try:
            tree = ET.parse(temp_xml_path)
            root = tree.getroot()

            joint_class_map = {}
            for joint_element in root.findall(".//joint[@class]"):
                joint_name = joint_element.get("name")
                joint_class = joint_element.get("class")
                if joint_name and joint_class:
                    joint_id = mujoco.mj_name2id(
                        self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
                    )
                    if joint_id != -1:
                        dof_adr = self.mj_model.jnt_dofadr[joint_id]
                        if joint_class not in joint_class_map:
                            joint_class_map[joint_class] = []
                        joint_class_map[joint_class].append(dof_adr)
        finally:
            os.remove(temp_xml_path)

        return joint_class_map

    def _get_default_dof_properties(self):
        with tempfile.NamedTemporaryFile(mode="w+", delete=False, suffix=".xml") as f:
            mujoco.mj_saveLastXML(f.name, self.mj_model)
            temp_xml_path = f.name

        try:
            tree = ET.parse(temp_xml_path)
            root = tree.getroot()

            default_dof_properties = {}
            for default_element in root.findall(".//default/default[@class]"):
                class_name = default_element.get("class")
                joint_element = default_element.find("joint")
                if class_name and joint_element is not None:
                    properties = {}
                    if "damping" in joint_element.attrib:
                        properties["damping"] = float(joint_element.get("damping"))
                    if "armature" in joint_element.attrib:
                        properties["armature"] = float(joint_element.get("armature"))
                    if "frictionloss" in joint_element.attrib:
                        properties["frictionloss"] = float(joint_element.get("frictionloss"))

                    if properties:
                        default_dof_properties[class_name] = properties
        finally:
            os.remove(temp_xml_path)

        return default_dof_properties

    def init_scene(self):
        """Initialize the default robot scene"""
        xml_path = str(pathlib.Path(GEAR_SONIC_ROOT) / self.config["ROBOT_SCENE"])
        self.mj_model = mujoco.MjModel.from_xml_path(xml_path)
        self.mj_data = mujoco.MjData(self.mj_model)
        self.mj_model.opt.timestep = self.sim_dt
        self._apply_fast_visuals()
        self.torso_index = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
        self.root_body = "pelvis"
        self.root_body_id = self.mj_model.body(self.root_body).id

        self.joint_class_map = self._get_dof_indices_by_class()

        self.perform_sysid_search = self.config.get("perform_sysid_search", False)

        # Check for static root link (fixed base)
        self.use_floating_root_link = "floating_base_joint" in [
            self.mj_model.joint(i).name for i in range(self.mj_model.njnt)
        ]
        self.use_constrained_root_link = "constrained_base_joint" in [
            self.mj_model.joint(i).name for i in range(self.mj_model.njnt)
        ]

        # MuJoCo qpos/qvel arrays start with root DOFs before joint DOFs:
        # floating base has 7 qpos (pos + quat) and 6 qvel (lin + ang velocity)
        if self.use_floating_root_link:
            self.qpos_offset = 7
            self.qvel_offset = 6
        else:
            if self.use_constrained_root_link:
                self.qpos_offset = 1
                self.qvel_offset = 1
            else:
                raise ValueError(
                    "No root link found --"
                    "The absolute static root will make the simulation unstable."
                )

        # Enable the elastic band
        if self.config["ENABLE_ELASTIC_BAND"] and self.use_floating_root_link:
            self.elastic_band = ElasticBand()
            # Sync band target xy to robot's initial position; z=1 is the hang point
            self.elastic_band.point = np.array([
                self.mj_model.qpos0[0],
                self.mj_model.qpos0[1],
                1.0,
            ])
            # Patch Advance so angular correction is relative to the initial quat,
            # not identity — prevents massive torque when robot starts rotated.
            _init_q_wxyz = self.mj_model.qpos0[3:7].copy()  # [w,x,y,z]
            _init_rot = Rotation.from_quat([_init_q_wxyz[1], _init_q_wxyz[2],
                                            _init_q_wxyz[3], _init_q_wxyz[0]])
            _eb = self.elastic_band
            def _advance(pose, _eb=_eb, _init_rot=_init_rot):
                pos, lin_vel, ang_vel = pose[0:3], pose[7:10], pose[10:13]
                dx = _eb.point - pos
                f = _eb.kp_pos * (dx + np.array([0, 0, _eb.length])) + _eb.kd_pos * (0 - lin_vel)
                q = pose[3:7]
                cur_rot = Rotation.from_quat([q[1], q[2], q[3], q[0]])
                rotvec = (cur_rot * _init_rot.inv()).as_rotvec()
                torque = -_eb.kp_ang * rotvec - _eb.kd_ang * ang_vel
                return np.concatenate([f, torque])
            self.elastic_band.Advance = _advance
            if "g1" in self.config["ROBOT_TYPE"]:
                if self.config["enable_waist"]:
                    self.band_attached_link = self.mj_model.body("pelvis").id
                else:
                    self.band_attached_link = self.mj_model.body("torso_link").id
            elif "h1" in self.config["ROBOT_TYPE"]:
                self.band_attached_link = self.mj_model.body("torso_link").id
            else:
                self.band_attached_link = self.mj_model.body("base_link").id

            if self.onscreen:
                _orig_cb = self.elastic_band.MujuocoKeyCallback
                def _key_cb(key):
                    if key == ord('C') or key == ord('c'):
                        self.cycle_camera()
                    elif key in (262, 263, 264, 265, 266, 267):
                        self.move_user(key)
                    elif key == 32:
                        self.toggle_user_facing()
                    elif key == 92:  # ]
                        self.toggle_stream_camera()
                    elif key == ord('p') or key == ord('P'):
                        self.launch_viewer()
                    elif key in (262, 263, 264, 265, 266, 267):
                        self.move_user(key)
                    elif key == 32:
                        self.toggle_user_facing()
                    elif key == 92:  # ]
                        self.toggle_stream_camera()
                    elif key == ord('p') or key == ord('P'):
                        self.launch_viewer()
                    else:
                        _orig_cb(key)
                self.viewer = mujoco.viewer.launch_passive(
                    self.mj_model,
                    self.mj_data,
                    key_callback=_key_cb,
                    show_left_ui=False,
                    show_right_ui=False,
                )
            else:
                mujoco.mj_forward(self.mj_model, self.mj_data)
                self.viewer = None
        else:
            if self.onscreen:
                def _key_cb(key):
                    if key == ord('C') or key == ord('c'):
                        self.cycle_camera()
                    elif key in (262, 263, 264, 265, 266, 267):
                        self.move_user(key)
                    elif key == 32:
                        self.toggle_user_facing()
                    elif key == 92:  # ]
                        self.toggle_stream_camera()
                    elif key == ord('p') or key == ord('P'):
                        self.launch_viewer()
                self.viewer = mujoco.viewer.launch_passive(
                    self.mj_model, self.mj_data,
                    key_callback=_key_cb,
                    show_left_ui=False, show_right_ui=False
                )
            else:
                mujoco.mj_forward(self.mj_model, self.mj_data)
                self.viewer = None

        if self.viewer:
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.viewer.cam.azimuth = 210    # front-right of entrance looking into store
            self.viewer.cam.elevation = -25
            self.viewer.cam.distance = 3.5
            self.viewer.cam.lookat = np.array([0.0, 0.0, 0.8])  # robot start position

        self.body_joint_index = []
        self.left_hand_index = []
        self.right_hand_index = []
        for i in range(self.mj_model.njnt):
            name = self.mj_model.joint(i).name
            if any(
                [
                    part_name in name
                    for part_name in ["hip", "knee", "ankle", "waist", "shoulder", "elbow", "wrist"]
                ]
            ):
                self.body_joint_index.append(i)
            elif "left_hand" in name:
                self.left_hand_index.append(i)
            elif "right_hand" in name:
                self.right_hand_index.append(i)

        assert len(self.body_joint_index) == self.robot.NUM_JOINTS
        assert len(self.left_hand_index) == self.robot.NUM_HAND_JOINTS
        assert len(self.right_hand_index) == self.robot.NUM_HAND_JOINTS

        self.body_joint_index = np.array(self.body_joint_index)
        self.left_hand_index = np.array(self.left_hand_index)
        self.right_hand_index = np.array(self.right_hand_index)

    def _apply_fast_visuals(self):
        """オンスクリーンの操作性（特にマウスでの視点操作）を滑らかにするため、
        モデルレベルで重い描画機能を無効化する。

        WSL の d3d12 では影(4K shadowmap)＋反射が描画の約2/3を占め、ウィンドウが ~8fps
        に落ちてマウス操作がカクつく。これらを切ると ~21fps まで上がる。
        scene flag は viewer が毎フレーム再構築する際にリセットされるため、ここでは
        モデルレベル(light_castshadow / mat_reflectance)で無効化する（リセットされない）。

        既定は WSL のときだけ fast（影・反射オフ）。ネイティブ Linux では full（元の画質）。
        SIM_VIEWER_QUALITY=fast/full でプラットフォームに関係なく強制できる。
        SIM_DISABLE_MSAA=1 で追加でアンチエイリアス(MSAA)も無効化（さらに少し軽くなる）。
        """
        if os.environ.get("SIM_VIEWER_QUALITY", "fast" if _IS_WSL else "full") == "full":
            return
        try:
            if self.mj_model.nlight > 0:
                self.mj_model.light_castshadow[:] = 0          # 影パスを無効化（最大の負荷）
            if self.mj_model.nmat > 0:
                self.mj_model.mat_reflectance[:] = 0           # 反射パスを無効化
            if os.environ.get("SIM_DISABLE_MSAA", "0") == "1":
                self.mj_model.vis.quality.offsamples = 0       # MSAA 無効
            print("[viewer] fast visuals: shadow/reflection disabled "
                  "(SIM_VIEWER_QUALITY=full で戻せる)")
        except Exception as e:
            print(f"[viewer] _apply_fast_visuals skipped: {e}")

    def _apply_offscreen_render_flags(self, renderer):
        """離屏カメラの描画品質を調整する。

        WSL の d3d12 では影(SHADOW)と反射(REFLECTION)のパスが非常に高コスト
        （合計 ~85ms/frame、全体の約2/3）。ロボットの知覚・対話用カメラには不要なので
        既定で無効化し、8fps → ~21fps に引き上げる。
        離屏カメラは gear-sonic に渡す「機能的な知覚画像」であり、影・反射は不要。
        そこで全プラットフォームで既定オフにする（描画が軽くなり、WSL に限らず
        Linux のヘッドレス/リモート/ソフトウェアレンダリング環境でもカメラ Hz が上がる）。
        ウィンドウ(人間が見る)の画質には影響しない（別 Renderer のため）。
        SIM_OFFSCREEN_QUALITY=full で従来通り（影・反射あり）に戻せる。
        """
        if os.environ.get("SIM_OFFSCREEN_QUALITY", "fast") == "full":
            return
        F = mujoco.mjtRndFlag
        renderer.scene.flags[int(F.mjRND_SHADOW)] = 0
        renderer.scene.flags[int(F.mjRND_REFLECTION)] = 0

    def init_renderers(self):
        self.renderers = {}
        for camera_name, camera_config in self.camera_configs.items():
            renderer = mujoco.Renderer(
                self.mj_model, height=camera_config["height"], width=camera_config["width"]
            )
            self._apply_offscreen_render_flags(renderer)
            self.renderers[camera_name] = renderer

    def compute_body_torques(self) -> np.ndarray:
        # PD control: tau = tau_ff + kp * (q_des - q) + kd * (dq_des - dq)
        body_torques = np.zeros(self.num_body_dof)
        if self.unitree_bridge is not None and self.unitree_bridge.low_cmd:
            for i in range(self.unitree_bridge.num_body_motor):
                if self.unitree_bridge.use_sensor:
                    body_torques[i] = (
                        self.unitree_bridge.low_cmd.motor_cmd[i].tau
                        + self.unitree_bridge.low_cmd.motor_cmd[i].kp
                        * (self.unitree_bridge.low_cmd.motor_cmd[i].q - self.mj_data.sensordata[i])
                        + self.unitree_bridge.low_cmd.motor_cmd[i].kd
                        * (
                            self.unitree_bridge.low_cmd.motor_cmd[i].dq
                            - self.mj_data.sensordata[i + self.unitree_bridge.num_body_motor]
                        )
                    )
                else:
                    body_torques[i] = (
                        self.unitree_bridge.low_cmd.motor_cmd[i].tau
                        + self.unitree_bridge.low_cmd.motor_cmd[i].kp
                        * (
                            self.unitree_bridge.low_cmd.motor_cmd[i].q
                            - self.mj_data.qpos[self.body_joint_index[i] + self.qpos_offset - 1]
                        )
                        + self.unitree_bridge.low_cmd.motor_cmd[i].kd
                        * (
                            self.unitree_bridge.low_cmd.motor_cmd[i].dq
                            - self.mj_data.qvel[self.body_joint_index[i] + self.qvel_offset - 1]
                        )
                    )
        return body_torques

    def get_head_pose(self) -> np.ndarray:
        root_pos = self.mj_data.body("torso_link").xpos.copy()
        # Reorder quaternion from MuJoCo [w,x,y,z] to scipy [x,y,z,w]
        root_quat = self.mj_data.body("torso_link").xquat.copy()[[1, 2, 3, 0]]
        head_pos = root_pos + Rotation.from_quat(root_quat).apply(np.array([0.0, 0.0, -0.044]))
        return np.concatenate((head_pos, root_quat))

    def get_root_vel(self) -> np.ndarray:
        return self.mj_data.qvel[:6]

    def compute_hand_torques(self) -> np.ndarray:
        left_hand_torques = np.zeros(self.num_hand_dof)
        right_hand_torques = np.zeros(self.num_hand_dof)
        if self.unitree_bridge is not None and self.unitree_bridge.low_cmd:
            for i in range(self.unitree_bridge.num_hand_motor):
                left_hand_torques[i] = (
                    self.unitree_bridge.left_hand_cmd.motor_cmd[i].tau
                    + self.unitree_bridge.left_hand_cmd.motor_cmd[i].kp
                    * (
                        self.unitree_bridge.left_hand_cmd.motor_cmd[i].q
                        - self.mj_data.qpos[self.left_hand_index[i] + self.qpos_offset - 1]
                    )
                    + self.unitree_bridge.left_hand_cmd.motor_cmd[i].kd
                    * (
                        self.unitree_bridge.left_hand_cmd.motor_cmd[i].dq
                        - self.mj_data.qvel[self.left_hand_index[i] + self.qvel_offset - 1]
                    )
                )
                right_hand_torques[i] = (
                    self.unitree_bridge.right_hand_cmd.motor_cmd[i].tau
                    + self.unitree_bridge.right_hand_cmd.motor_cmd[i].kp
                    * (
                        self.unitree_bridge.right_hand_cmd.motor_cmd[i].q
                        - self.mj_data.qpos[self.right_hand_index[i] + self.qpos_offset - 1]
                    )
                    + self.unitree_bridge.right_hand_cmd.motor_cmd[i].kd
                    * (
                        self.unitree_bridge.right_hand_cmd.motor_cmd[i].dq
                        - self.mj_data.qvel[self.right_hand_index[i] + self.qvel_offset - 1]
                    )
                )
        return np.concatenate((left_hand_torques, right_hand_torques))

    def compute_body_qpos(self) -> np.ndarray:
        body_qpos = np.zeros(self.num_body_dof)
        if self.unitree_bridge is not None and self.unitree_bridge.low_cmd:
            for i in range(self.unitree_bridge.num_body_motor):
                body_qpos[i] = self.unitree_bridge.low_cmd.motor_cmd[i].q
        return body_qpos

    def compute_hand_qpos(self) -> np.ndarray:
        hand_qpos = np.zeros(self.num_hand_dof * 2)
        if self.unitree_bridge is not None and self.unitree_bridge.low_cmd:
            for i in range(self.unitree_bridge.num_hand_motor):
                hand_qpos[i] = self.unitree_bridge.left_hand_cmd.motor_cmd[i].q
                hand_qpos[i + self.num_hand_dof] = self.unitree_bridge.right_hand_cmd.motor_cmd[i].q
        return hand_qpos

    def prepare_obs(self) -> Dict[str, any]:
        obs = {}
        if self.use_floating_root_link:
            obs["floating_base_pose"] = self.mj_data.qpos[:7]
            obs["floating_base_vel"] = self.mj_data.qvel[:6]
            obs["floating_base_acc"] = self.mj_data.qacc[:6]
        else:
            obs["floating_base_pose"] = np.zeros(7)
            obs["floating_base_vel"] = np.zeros(6)
            obs["floating_base_acc"] = np.zeros(6)

        obs["secondary_imu_quat"] = self.mj_data.xquat[self.torso_index]

        pose = np.zeros(13)
        torso_link = self.mj_model.body("torso_link").id
        # mj_objectVelocity returns [ang_vel, lin_vel]; swap to [lin_vel, ang_vel]
        mujoco.mj_objectVelocity(
            self.mj_model, self.mj_data, mujoco.mjtObj.mjOBJ_BODY, torso_link, pose[7:13], 1
        )
        pose[7:10], pose[10:13] = (
            pose[10:13],
            pose[7:10].copy(),
        )
        obs["secondary_imu_vel"] = pose[7:13]

        obs["body_q"] = self.mj_data.qpos[self.body_joint_index + 7 - 1]
        obs["body_dq"] = self.mj_data.qvel[self.body_joint_index + 6 - 1]
        obs["body_ddq"] = self.mj_data.qacc[self.body_joint_index + 6 - 1]
        obs["body_tau_est"] = self.mj_data.actuator_force[self.body_joint_index - 1]
        if self.num_hand_dof > 0:
            obs["left_hand_q"] = self.mj_data.qpos[self.left_hand_index + self.qpos_offset - 1]
            obs["left_hand_dq"] = self.mj_data.qvel[self.left_hand_index + self.qvel_offset - 1]
            obs["left_hand_ddq"] = self.mj_data.qacc[self.left_hand_index + self.qvel_offset - 1]
            obs["left_hand_tau_est"] = self.mj_data.actuator_force[self.left_hand_index - 1]
            obs["right_hand_q"] = self.mj_data.qpos[self.right_hand_index + self.qpos_offset - 1]
            obs["right_hand_dq"] = self.mj_data.qvel[self.right_hand_index + self.qvel_offset - 1]
            obs["right_hand_ddq"] = self.mj_data.qacc[self.right_hand_index + self.qvel_offset - 1]
            obs["right_hand_tau_est"] = self.mj_data.actuator_force[self.right_hand_index - 1]
        obs["time"] = self.mj_data.time
        return obs

    def sim_step(self):
        self.obs = self.prepare_obs()
        self.unitree_bridge.PublishLowState(self.obs)
        if self.unitree_bridge.joystick:
            self.unitree_bridge.PublishWirelessController()
        if self.elastic_band:
            if self.elastic_band.enable and self.use_floating_root_link:
                pose = np.concatenate(
                    [
                        self.mj_data.xpos[self.band_attached_link],
                        self.mj_data.xquat[self.band_attached_link],
                        np.zeros(6),
                    ]
                )
                mujoco.mj_objectVelocity(
                    self.mj_model,
                    self.mj_data,
                    mujoco.mjtObj.mjOBJ_BODY,
                    self.band_attached_link,
                    pose[7:13],
                    0,
                )
                pose[7:10], pose[10:13] = pose[10:13], pose[7:10].copy()
                self.mj_data.xfrc_applied[self.band_attached_link] = self.elastic_band.Advance(pose)
            else:
                self.mj_data.xfrc_applied[self.band_attached_link] = np.zeros(6)
        body_torques = self.compute_body_torques()
        hand_torques = self.compute_hand_torques()
        # -1: actuator array is 0-based while joint indices from the model are 1-based
        self.torques[self.body_joint_index - 1] = body_torques
        if self.num_hand_dof > 0:
            self.torques[self.left_hand_index - 1] = hand_torques[: self.num_hand_dof]
            self.torques[self.right_hand_index - 1] = hand_torques[self.num_hand_dof :]

        self.torques = np.clip(self.torques, -self.torque_limit, self.torque_limit)

        if self.config["FREE_BASE"]:
            # Prepend 6 zeros for the floating-base root DOF actuators
            self.mj_data.ctrl = np.concatenate((np.zeros(6), self.torques))
        else:
            self.mj_data.ctrl = self.torques
        # レンダースレッドが mj_data をスナップショットする瞬間と排他（保持は数μs）。
        with self._render_lock:
            mujoco.mj_step(self.mj_model, self.mj_data)

        self.check_fall()

    def apply_perturbation(self, key):
        perturbation_x_body = 0.0
        perturbation_y_body = 0.0
        if key == "up":
            perturbation_x_body = 1.0
        elif key == "down":
            perturbation_x_body = -1.0
        elif key == "left":
            perturbation_y_body = 1.0
        elif key == "right":
            perturbation_y_body = -1.0

        vel_body = np.array([perturbation_x_body, perturbation_y_body, 0.0])
        vel_world = np.zeros(3)
        base_quat = self.mj_data.qpos[3:7]
        mujoco.mju_rotVecQuat(vel_world, vel_body, base_quat)

        self.mj_data.qvel[0] += vel_world[0]
        self.mj_data.qvel[1] += vel_world[1]
        mujoco.mj_forward(self.mj_model, self.mj_data)

    def update_viewer(self):
        if self.viewer is not None:
            self.viewer.sync()

    def update_viewer_camera(self):
        if self.viewer is not None:
            if self.viewer.cam.type == mujoco.mjtCamera.mjCAMERA_TRACKING:
                self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            else:
                self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING

    # C キーで切り替えるカメラの順序（固定）
    _CYCLE_CAMS = ["(free)", "ego_view", "user_eye", "counter"]

    def cycle_camera(self):
        """C キーでカメラを順番に切り替える: 自由 → robot 視点 → user 視点 → カウンター"""
        if self.viewer is None:
            return
        self._camera_index = (self._camera_index + 1) % len(self._CYCLE_CAMS)
        name = self._CYCLE_CAMS[self._camera_index]
        if name == "(free)":
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            print("[Camera] 自由視角")
        else:
            try:
                self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                self.viewer.cam.fixedcamid = self.mj_model.camera(name).id
                labels = {"ego_view": "robot 第一視点", "user_eye": "user 視点", "counter": "カウンター視点"}
                print(f"[Camera] {labels.get(name, name)}")
            except Exception as e:
                print(f"[Camera] {name} 取得失敗: {e}")


    # (quat[w,x,y,z], forward_axis, forward_sign, label)
    # yaw index: 0=北(+y)  1=西(-x)  2=南(-y)  3=東(+x)
    _USER_DIRS = [
        ([0.70711, 0, 0,  0.70711], 1, +1, "北(+y)"),
        ([0,       0, 0,  1      ], 0, -1, "西(-x)"),
        ([0.70711, 0, 0, -0.70711], 1, -1, "南(-y)"),
        ([1,       0, 0,  0      ], 0, +1, "東(+x)"),
    ]

    def _set_user_yaw(self, mid, yaw):
        self._user_yaw = yaw % 4
        quat, _, _, label = self._USER_DIRS[self._user_yaw]
        self.mj_data.mocap_quat[mid] = quat
        print(f"[User] 向き: {label}")

    def move_user(self, key):
        user_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, "user")
        if user_id < 0: return
        mid = self.mj_model.body_mocapid[user_id]
        if mid < 0: return
        S = 0.15
        p = self.mj_data.mocap_pos[mid].copy()
        _, axis, sign, _ = self._USER_DIRS[self._user_yaw]
        if   key == 265: p[axis] += S * sign   # ↑ 前進
        elif key == 264: p[axis] -= S * sign   # ↓ 後退
        elif key == 263: self._set_user_yaw(mid, self._user_yaw + 1)  # ← 左転向 (CCW)
        elif key == 262: self._set_user_yaw(mid, self._user_yaw - 1)  # → 右転向 (CW)
        elif key == 266: p[2] += S * 0.5
        elif key == 267: p[2] -= S * 0.5
        self.mj_data.mocap_pos[mid] = p

    def toggle_user_facing(self):
        """Space: 現在の向きから 180° 反転"""
        user_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, "user")
        if user_id < 0: return
        mid = self.mj_model.body_mocapid[user_id]
        if mid < 0: return
        self._set_user_yaw(mid, self._user_yaw + 2)

    def toggle_stream_camera(self):
        cams = list(self.camera_configs.keys())
        if not cams: return
        idx = cams.index(self._active_stream) if self._active_stream in cams else 0
        self._active_stream = cams[(idx + 1) % len(cams)]
        print(f"[Stream] → {self._active_stream}")

    def launch_viewer(self):
        import os, subprocess, sys, time
        now = time.time()
        if now - self._viewer_last_t < 1.0: return
        self._viewer_last_t = now
        if self._viewer_proc and self._viewer_proc.poll() is None:
            self._viewer_proc.terminate()
            self._viewer_proc = None
            print("[viewer] closed")
            return
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'user_eye_viewer.py')
        env = os.environ.copy()
        self._viewer_proc = subprocess.Popen(
            [sys.executable, script], env=env
        )
        print(f"[viewer] launched pid={self._viewer_proc.pid}")

    def update_reward(self):
        with self.reward_lock:
            self.last_reward = 0

    def get_reward(self):
        with self.reward_lock:
            return self.last_reward

    def set_unitree_bridge(self, unitree_bridge):
        self.unitree_bridge = unitree_bridge

    def get_privileged_obs(self):
        return {}

    def update_render_caches(self):
        render_caches = {}
        active = getattr(self, "_active_stream", None)
        target = {active: self.camera_configs[active]} if (
            active and active in self.camera_configs
        ) else self.camera_configs
        #print(f"[render] active={active}")
        for camera_name, camera_config in target.items():
            renderer = self.renderers[camera_name]
            if "params" in camera_config:
                renderer.update_scene(self.mj_data, camera=camera_config["params"])
            else:
                renderer.update_scene(self.mj_data, camera=camera_name)
            render_caches[camera_name + "_image"] = renderer.render()
        if self.image_publish_process is not None:
            self.image_publish_process.update_shared_memory(render_caches)
        return render_caches

    # ------------------------------------------------------------------
    # 離屏レンダリングを制御ループから切り離す専用スレッド
    # ------------------------------------------------------------------
    def start_render_thread(self):
        """制御ループとは別スレッドで離屏カメラをレンダリングする。

        GL コンテキストはスレッド固有なので、renderer はこのスレッド内で生成する。
        制御スレッドは mj_data のスナップショット中だけロックを取る（数μs）ので、
        重い render()+読み戻し(glReadPixels) は制御の 200Hz ループを一切塞がない。
        """
        if self._render_thread is not None:
            return
        if not self._use_render_thread:
            # 同期パス（Linux 既定 or SIM_RENDER_THREAD=0）。self.renderers は __init__ で生成済み。
            print("[render-thread] disabled (synchronous render in control loop)")
            return
        self._render_stop.clear()
        self._render_thread = Thread(target=self._render_loop, name="offscreen-render", daemon=True)
        self._render_thread.start()

    def stop_render_thread(self):
        if self._render_thread is None:
            return
        self._render_stop.set()
        self._render_thread.join(timeout=5)
        self._render_thread = None

    def _render_snapshot(self, render_data):
        """ライブ mj_data の動的状態をロック下で render_data へコピーし、mj_forward で派生量を再計算。

        ロックを握るのはコピーの一瞬だけ。mj_forward（軽い）と render（重い）はロック外で実行する。
        """
        with self._render_lock:
            render_data.qpos[:] = self.mj_data.qpos
            render_data.qvel[:] = self.mj_data.qvel
            render_data.ctrl[:] = self.mj_data.ctrl
            render_data.time = self.mj_data.time
            if self.mj_model.nmocap > 0:
                render_data.mocap_pos[:] = self.mj_data.mocap_pos
                render_data.mocap_quat[:] = self.mj_data.mocap_quat
            if self.mj_model.na > 0:
                render_data.act[:] = self.mj_data.act
            render_data.xfrc_applied[:] = self.mj_data.xfrc_applied
        # ロック外: 派生量（xpos/cam_xpos 等）を再計算。制御スレッドには影響しない。
        mujoco.mj_forward(self.mj_model, render_data)

    def _render_loop(self):
        try:
            renderers = {}
            for camera_name, camera_config in self.camera_configs.items():
                renderer = mujoco.Renderer(
                    self.mj_model,
                    height=camera_config["height"],
                    width=camera_config["width"],
                )
                self._apply_offscreen_render_flags(renderer)
                renderers[camera_name] = renderer
        except Exception as e:
            print(f"[render-thread] failed to init renderers: {e}")
            self._render_thread = None
            return

        render_data = mujoco.MjData(self.mj_model)
        print(f"[render-thread] started ({len(renderers)} cam, image_dt={self.image_dt})")

        while not self._render_stop.is_set():
            t0 = time.monotonic()
            try:
                self._render_snapshot(render_data)

                active = getattr(self, "_active_stream", None)
                target = {active: self.camera_configs[active]} if (
                    active and active in self.camera_configs
                ) else self.camera_configs

                render_caches = {}
                for camera_name, camera_config in target.items():
                    renderer = renderers[camera_name]
                    if "params" in camera_config:
                        renderer.update_scene(render_data, camera=camera_config["params"])
                    else:
                        renderer.update_scene(render_data, camera=camera_name)
                    render_caches[camera_name + "_image"] = renderer.render()

                if self.image_publish_process is not None:
                    if _check_visibility is not None:
                        self.image_publish_process._robot_vis_arr[0] = (
                            1 if _check_visibility(self.mj_model, render_data) else 0
                        )
                    self.image_publish_process.update_shared_memory(render_caches)
            except Exception as e:
                print(f"[render-thread] render error: {e}")

            # image_dt 周期を維持（render が遅くても制御ループとは無関係）
            sleep_t = self.image_dt - (time.monotonic() - t0)
            if sleep_t > 0:
                self._render_stop.wait(sleep_t)

        for r in renderers.values():
            try:
                r.close()
            except Exception:
                pass
        print("[render-thread] stopped")

    def handle_keyboard_button(self, key):
        if self.elastic_band:
            self.elastic_band.handle_keyboard_button(key)

        if key == "backspace":
            self.reset()
        if key == "v":
            self.update_viewer_camera()
        if key in ["up", "down", "left", "right"]:
            self.apply_perturbation(key)

    def check_fall(self):
        self.fall = False
        if self.mj_data.qpos[2] < 0.2:
            self.fall = True
            print(f"Warning: Robot has fallen, height: {self.mj_data.qpos[2]:.3f} m")

        if self.fall:
            self.reset()

    def check_self_collision(self):
        robot_bodies = get_subtree_body_names(self.mj_model, self.mj_model.body(self.root_body).id)
        self_collision, contact_bodies = check_contact(
            self.mj_model, self.mj_data, robot_bodies, robot_bodies, return_all_contact_bodies=True
        )
        if self_collision:
            print(f"Warning: Self-collision detected: {contact_bodies}")
        return self_collision

    def reset(self):
        mujoco.mj_resetData(self.mj_model, self.mj_data)


class BaseSimulator:
    """Base simulator class that handles initialization and running of simulations"""

    def __init__(
        self, config: Dict[str, any], env_name: str = "default", redis_client=None, **kwargs
    ):
        self.config = config
        self.env_name = env_name
        self.redis_client = redis_client
        if self.redis_client is not None:
            self.redis_client.set("push_left_hand", "false")
            self.redis_client.set("push_right_hand", "false")
            self.redis_client.set("push_torso", "false")

        # Create rate objects
        self.sim_dt = self.config["SIMULATE_DT"]
        self.reward_dt = self.config.get("REWARD_DT", 0.02)
        self.image_dt = self.config.get("IMAGE_DT", 0.033333)
        self.viewer_dt = self.config.get("VIEWER_DT", 0.02)
        self._running = True

        self.robot = Robot(self.config)

        # Create the environment
        if env_name == "default":
            self.sim_env = DefaultEnv(config, env_name, **kwargs)
        else:
            raise ValueError(
                f"Invalid environment name: {env_name}. "
                f"Only 'default' is supported in this minimal build."
            )

        try:
            if self.config.get("INTERFACE", None):
                ChannelFactoryInitialize(self.config["DOMAIN_ID"], self.config["INTERFACE"])
            else:
                ChannelFactoryInitialize(self.config["DOMAIN_ID"])
        except Exception as e:
            print(f"Note: Channel factory initialization attempt: {e}")

        self.init_unitree_bridge()
        self.sim_env.set_unitree_bridge(self.unitree_bridge)

        self.init_subscriber()
        self.init_publisher()

        self.sim_thread = None

    def start_as_thread(self):
        self.sim_thread = Thread(target=self.start)
        self.sim_thread.start()

    def start_image_publish_subprocess(self, start_method: str = "spawn", camera_port: int = 5555):
        self.sim_env.start_image_publish_subprocess(start_method, camera_port)

    def init_subscriber(self):
        pass

    def init_publisher(self):
        pass

    def init_unitree_bridge(self):
        self.unitree_bridge = UnitreeSdk2Bridge(self.config)
        if self.config["USE_JOYSTICK"]:
            self.unitree_bridge.SetupJoystick(
                device_id=self.config["JOYSTICK_DEVICE"], js_type=self.config["JOYSTICK_TYPE"]
            )

    def start(self):
        """Main simulation loop"""
        sim_cnt = 0
        ts = time.time()

        # 制御ループの実効周波数ログ（リアルタイム性の診断用）。SIM_LOG_HZ=1 で有効。
        _log_hz = os.environ.get("SIM_LOG_HZ", "0") == "1"
        _hz_t0 = time.monotonic()
        _hz_n0 = 0

        try:
            while self._running and (
                (self.sim_env.viewer and self.sim_env.viewer.is_running())
                or (self.sim_env.viewer is None)
            ):
                step_start = time.monotonic()

                self.sim_env.sim_step()
                now = time.time()
                if now - ts > 1 / 10.0 and self.redis_client is not None:
                    head_pose = self.sim_env.get_head_pose()
                    self.redis_client.set("head_pos", pickle.dumps(head_pose[:3]))
                    self.redis_client.set("head_quat", pickle.dumps(head_pose[3:]))
                    ts = now

                if sim_cnt % int(self.viewer_dt / self.sim_dt) == 0:
                    self.sim_env.update_viewer()

                if sim_cnt % int(self.reward_dt / self.sim_dt) == 0:
                    self.sim_env.update_reward()

                # レンダースレッドが動いている場合、制御ループ側では一切レンダリングしない
                # （render() は別スレッドに分離済み。ここで呼ぶと 200Hz 制御を塞ぐ）。
                if (self.sim_env._render_thread is None
                        and self.sim_env.offscreen
                        and sim_cnt % int(self.image_dt / self.sim_dt) == 0):
                    caches = self.sim_env.update_render_caches()
                    # robot_visibility は常に計算（active_stream に関係なく）
                    if _check_visibility is not None and self.sim_env.image_publish_process is not None:
                        self.sim_env.image_publish_process._robot_vis_arr[0] = 1 if _check_visibility(self.sim_env.mj_model, self.sim_env.mj_data) else 0

                # Simple rate limiter (replaces ROS rate)
                elapsed = time.monotonic() - step_start
                sleep_time = self.sim_dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

                sim_cnt += 1

                if _log_hz:
                    _dt = time.monotonic() - _hz_t0
                    if _dt >= 2.0:
                        hz = (sim_cnt - _hz_n0) / _dt
                        print(f"[ctrl-loop] {hz:.1f} Hz (target {1.0/self.sim_dt:.0f} Hz, "
                              f"RTF {hz*self.sim_dt:.2f})")
                        _hz_t0 = time.monotonic()
                        _hz_n0 = sim_cnt
        except KeyboardInterrupt:
            print("Simulator interrupted by user.")
        finally:
            self.close()

    def __del__(self):
        self.close()

    def reset(self):
        self.sim_env.reset()

    def close(self):
        self._running = False
        try:
            # 共有メモリを unlink する前に、まずレンダースレッドの書き込みを止める
            self.sim_env.stop_render_thread()
            if self.sim_env.image_publish_process is not None:
                self.sim_env.image_publish_process.stop()
            if self.sim_env.viewer is not None:
                self.sim_env.viewer.close()
        except Exception as e:
            print(f"Warning during close: {e}")

    def get_privileged_obs(self):
        return self.sim_env.get_privileged_obs()

    def handle_keyboard_button(self, key):
        self.sim_env.handle_keyboard_button(key)
