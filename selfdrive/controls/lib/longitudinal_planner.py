#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, N
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_speed_error, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params

#new
from openpilot.selfdrive.controls.lib.dec.longitudinal_planner import LongitudinalPlannerSP
from openpilot.selfdrive.carrot.config import UnifiedParams
#new: 红绿灯刹车/起步增强（独立模块，默认关=零影响）
from openpilot.selfdrive.carrot.traffic_light_brake import TrafficLightBrake
#new: 起步与跟车辅助（独立模块，三功能各自独立开关，默认关=零影响）
from openpilot.selfdrive.carrot.launch_assist import LaunchAssist
#new: 入弯预备减速（独立模块，默认关=零影响）
from openpilot.selfdrive.carrot.curve_anticipate import CurveAnticipate
#new: 丢目标缓冲（独立模块，默认关=零影响）：radar 丢小目标时用虚拟 lead 顶住 MPC
from openpilot.selfdrive.carrot.lead_buffer import LeadBuffer
#new: 幽灵刹车抑制（置信度阻尼，独立开关，默认关=零影响）

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MIN = -2.0 #-1.2
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.5
MIN_ALLOW_THROTTLE_SPEED = 2.5

# ===== e2e(视觉)兜底安全限幅（F+）=====
# 背景：blended(实验模式)分支原为 output = min(mpc, e2e) / shouldStop = e2e or mpc。
#   e2e 即 modelV2.action 的 desiredAcceleration / desiredVelocity / shouldStop。
# 风险：模型误报(幻影目标)会逐帧直接变成制动/停车指令。
# 依据（硬证据）：e2e 不参与 MPC 内部状态 —— v_desired_filter / prev_a / set_weights
#   均不读它；全仓库只有本文件 update() 末尾这几行消费它。
#   ⇒ 限幅后"误报持续 1 帧还是 100 帧"总伤害是同一个固定值，**不会累积**。
# 设计：方向全部单向更保守（只会更早减速/更易停），幅度设硬上限。
E2E_MAX_DELTA = 1.0     # m/s^2  视觉最多比 MPC 更负多少（约 0.10g）
E2E_V_MAX_DELTA = 3.0   # m/s    视觉最多把速度目标压低多少（约 10.8km/h）
E2E_SLOW_A_TH = -0.3    # m/s^2  认定"模型确实在减速"的加速度阈值
E2E_STOP_CONFIRM = 5    # 帧     视觉 shouldStop 需连续确认帧数（100Hz ⇒ 50ms）

# ===== F+ v3：停车意图解锁（2026-09-23）=====
# 实测（54618 帧）：行进中(v>5km/h)视觉 shouldStop 从未为 True（0 帧）——
#   模型表达"要停"靠的是 desiredVelocity 趋 0，不是 shouldStop。
#   而 v2 的速度门控最多只让 3 m/s ⇒ 实测巡航 50km/h 遇红灯时目标速度被卡在
#   43.8km/h，PID 误差仅 -3 m/s ⇒ 表现为"有刹车动作但力度极弱 / 刹不住 / 闯红灯"。
# 修复：识别到"停车意图"并连续确认后，解除两项限幅，让视觉目标速度直接生效。
E2E_PARK_V_TH = 2.0     # m/s   视觉期望速度低于此值 ⇒ 判为停车意图（7.2km/h）
E2E_PARK_A_TH = -1.2    # m/s^2 视觉期望加速度低于此值 ⇒ 判为停车意图
E2E_PARK_CONFIRM = 10   # 帧    停车意图需连续确认帧数（100Hz ⇒ 100ms，抗单帧误报）

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  steer_abs = abs(angle_steers)
  if v_ego > 20 or (v_ego > 25 and steer_abs < 3.0):
    return a_target
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner(LongitudinalPlannerSP): #new
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    #new
    self.mpc.mode = 'acc'
    LongitudinalPlannerSP.__init__(self, self.CP, self.mpc)
    #new
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_v_target_now = 0.0
    self.output_j_target_now = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

    self.vCluRatio = 1.0

    self.v_cruise_kph = 0.0

    self.sys_params = Params()
    #new
    self.params = UnifiedParams()
    self.frame = 0
    #new: 红绿灯刹车/起步增强控制器（开关关闭时整段 no-op）
    self.tlb = TrafficLightBrake()
    #new: 起步与跟车辅助控制器（三功能各自独立开关，关闭时整段 no-op）
    self.la = LaunchAssist()
    #new: 入弯预备减速控制器（独立开关，关闭时整段 no-op）
    self.ca = CurveAnticipate()
    #new: 丢目标缓冲控制器（独立开关，关闭时整段 no-op）
    self.lb = LeadBuffer()
    #new: 幽灵刹车抑制控制器（独立开关，关闭时整段 no-op）
    self.DynamicExperimentalSpeed = -1
    self.DynamicExperimentalLatA = 0.0
    self.UserExperimentalMode = False
    # ===== 动态实验模式迟滞参数 =====
    self._exp_latched = False  # 当前是否被动态逻辑锁定为实验模式
    self._exp_on_counter = 0  # 连续满足进入条件的帧数
    self._exp_off_counter = 0  # 连续满足退出条件的帧数
    self._exp_on_frames = 10  # 连续10帧才进入（比如 20Hz ≈ 0.5s）
    self._exp_off_frames = 20  # 连续20帧才退出（更保守）
    self._exp_hyst_speed = 5.0  # km/h 迟滞
    self._exp_hyst_latA = 0.5  # m/s^2 迟滞
    #new: e2e(视觉)兜底安全限幅 —— 视觉 shouldStop 连续确认计数器（F+）
    self._e2e_stop_cnt = 0
    self._e2e_park_cnt = 0   # F+ v3: 停车意图连续确认计数器
    #new

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm, carrot):
    #self.mpc.mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'
    #new
    if self.frame % 100 == 0:
      self.DynamicExperimentalSpeed = self.params.get_int("DynamicExperimentalSpeed")
      self.DynamicExperimentalLatA = self.params.get_float("DynamicExperimentalLatA")*0.1
    self.frame += 1

    modelData = sm['modelV2']
    orientation_rate = np.array(modelData.orientationRate.z)
    velocity = np.array(modelData.velocity.x)
    max_pred_lat_acc = np.amax(np.abs(orientation_rate) * velocity)
    v_ego = sm['carState'].vEgo
    v_ego_kph = v_ego * 3.6

    self.mpc.mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'

    # ===============================
    # 动态实验模式（进阶迟滞版）
    # ===============================

    # 条件开（进入实验模式）
    cond_speed_on = (
      self.DynamicExperimentalSpeed > 0 and
      5 < v_ego_kph < max(10, self.DynamicExperimentalSpeed)
    )

    cond_lat_on = (
      0 < self.DynamicExperimentalLatA < max_pred_lat_acc
    )

    # 条件关（退出实验模式，加迟滞）
    cond_speed_off = (
      self.DynamicExperimentalSpeed > 0 and
      (v_ego_kph < 2 or v_ego_kph > self.DynamicExperimentalSpeed + self._exp_hyst_speed)
    )

    cond_lat_off = (
      self.DynamicExperimentalLatA > 0 and
      max_pred_lat_acc <= max(0.3, self.DynamicExperimentalLatA - self._exp_hyst_latA)
    )

    enter_exp = cond_speed_on or cond_lat_on
    # 退出判据修正（F+）：未启用的通道（对应参数 <= 0 时其 cond_*_off 恒为 False）
    #   视为"不阻塞退出"。否则只设一个参数 ⇒ exit_exp 恒 False ⇒ 进去出不来
    #   （而写入的是持久化参数 ExperimentalMode ⇒ 断电重启仍处在实验模式）。
    #   安全性：两参数都为 0 时走上面的"动态实验模式"分支，不会落到这里，故无副作用。
    exit_exp = (cond_speed_off or self.DynamicExperimentalSpeed <= 0) and \
               (cond_lat_off or self.DynamicExperimentalLatA <= 0)

    # ========= 时间确认（防抖）=========
    if enter_exp:
      self._exp_on_counter += 1
    else:
      self._exp_on_counter = 0
    if exit_exp:
      self._exp_off_counter += 1
    else:
      self._exp_off_counter = 0

    # ========= 模式切换 =========
    # 设定值为0表示动态实验模式
    if self.DynamicExperimentalSpeed == 0 and self.DynamicExperimentalLatA == 0:
      LongitudinalPlannerSP.update(self, sm)
      if dec_mpc_mode := self.get_mpc_mode():
        self.mpc.mode = dec_mpc_mode

    # 设置值大于0表示条件实验模式
    elif self._exp_on_counter >= self._exp_on_frames and not self._exp_latched:
      # 进入实验模式
      #if not sm['selfdriveState'].experimentalMode and not self.UserExperimentalMode:
      #  self.sys_params.put_bool_nonblocking("ExperimentalMode", True)
      self.sys_params.put_bool_nonblocking("ExperimentalMode", True)

      self.mpc.mode = 'blended'
      self._exp_latched = True
      self.UserExperimentalMode = True
      self._exp_off_counter = 0

    elif self._exp_off_counter >= self._exp_off_frames and self._exp_latched:
      # 退出实验模式
      #if sm['selfdriveState'].experimentalMode:
      #  self.sys_params.put_bool_nonblocking("ExperimentalMode", False)
      self.sys_params.put_bool_nonblocking("ExperimentalMode", False)

      self._exp_latched = False
      self.UserExperimentalMode = False
      self._exp_on_counter = 0
    #new

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)

    self.v_cruise_kph = carrot.update(sm, v_cruise_kph, self.mpc.mode)
    self.mpc.mode = carrot.mode
    v_cruise = self.v_cruise_kph * CV.KPH_TO_MS

    vCluRatio = sm['carState'].vCluRatio
    if vCluRatio > 0.5:
      self.vCluRatio = vCluRatio
      v_cruise *= vCluRatio

    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel
    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized or carrot.soft_hold_active

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if self.mpc.mode == 'acc':
      #accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
      accel_limits = [A_CRUISE_MIN, carrot.get_carrot_accel(v_ego)]
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      accel_limits_turns = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_limits, self.CP)
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
      accel_limits_turns = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])

      self.mpc.prev_a = np.full(N+1, self.a_desired) ## carrot
      accel_limits_turns[0] = accel_limits_turns[0] = 0.0 ## carrot

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    if self.params.get_int("CommaLongAcc") > 0:
      self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED
    else:
      self.allow_throttle = True

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_limits_turns[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_limits_turns[1], clipped_accel_coast])
      accel_limits_turns[1] = min(accel_limits_turns[1], clipped_accel_coast_interp)

    if force_slow_decel:
      # 系统级强制减速（如车距极近）: 仍直接 v_cruise=0, 这是明确的停止意图
      v_cruise = 0.0
    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    # === 丢目标缓冲（独立模块，默认关=零影响）===
    # ★ 必须在 set_accel_limits 之前调用：缓冲期除"虚拟 lead"外还给加速度上限
    #   (accel_cap)，双保险地防住"radar 丢目标 ⇒ MPC 失去跟随约束 ⇒ 全速冲出去"。
    radar_state_for_mpc = self.lb.update(carrot, sm, sm['radarState'])
    if self.lb.accel_cap is not None:
      accel_limits_turns[1] = min(accel_limits_turns[1], float(self.lb.accel_cap))

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality, jerk_factor = carrot.jerk_factor_apply)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    # === 红绿灯刹车/起步增强（独立模块，默认关=零影响）===
    self.tlb.update(carrot, sm, v_ego, v_cruise)
    # === 起步与跟车辅助（独立模块，三功能各自独立开关，默认关=零影响）===
    self.la.update(carrot, sm, v_ego, v_cruise)
    # === 入弯预备减速（独立模块，默认关=零影响）===
    self.ca.update(carrot, sm, v_ego, v_cruise)
    # ★ 修复接线断点：三个辅助模块(tlb/la/ca)修改的是 carrot.v_cruise 属性，
    #   但 mpc.update 用的是 L270 已绑定的局部 v_cruise（旧值），属性改动不会反向刷新它。
    #   => assist 抬的起步/红绿灯/入弯目标速度根本没进 MPC 设定速度，功能等于没生效。
    #   重绑局部 v_cruise（单位同为 m/s，与 carrot.v_cruise 一致），让辅助模块真正驱动纵向。
    #   系统级强制减速(force_slow_decel)除外：明确停车意图，assist 不得覆盖（安全）。
    if not force_slow_decel:
      v_cruise = carrot.v_cruise
    # radar_state_for_mpc 已在上方由「丢目标缓冲」模块给出（模块关闭时即原对象，零影响）
    self.mpc.update(carrot, reset_state, radar_state_for_mpc, v_cruise, x, v, a, j, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    # --- TTC-based early FCW (carrot VRU tuning) ---
    # Independent of MPC: warn earlier when closing on any radar/model lead with low TTC.
    # Catches "seen but reacts too late" cases. Does NOT help if leadOne is fully missing
    # (model+radar both missed the target) -- that needs the vision VRU layer.
    ttc_fcw = False
    lead = sm['radarState'].leadOne
    v_ego = sm['carState'].vEgo
    if lead.status and not sm['carState'].standstill:
      v_closing = -lead.vRel  # >0 means we are approaching the lead
      if v_closing > 0.5 and v_ego > 3.0:
        ttc = lead.dRel / v_closing
        # night: 3.5s; day if too chatty: 2.5~3.0s
        ttc_fcw = (ttc < 2.0) and (lead.dRel < 80.0)
    self.fcw = ((self.mpc.crash_cnt > 1) or ttc_fcw) and not sm['carState'].standstill and not reset_state
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    longitudinalActuatorDelay = self.params.get_float("LongActuatorDelay")*0.01
    vEgoStopping = self.params.get_float("VEgoStopping") * 0.01
    action_t =  longitudinalActuatorDelay + DT_MDL

    output_a_target_mpc, output_should_stop_mpc, output_v_target_mpc, _ = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop
    output_v_target_now_e2e = sm['modelV2'].action.desiredVelocity

    if self.mpc.mode == 'acc':
      output_a_target = output_a_target_mpc
      output_v_target_now = output_v_target_mpc
      self.output_should_stop = output_should_stop_mpc
      self._e2e_stop_cnt = 0
      self._e2e_park_cnt = 0
    else:
      # ===== F+：e2e(视觉)兜底 + 安全限幅（全部落在 blended 分支 ⇒ 关实验模式即原样）=====
      # ① 停车意图识别（F+ v3，2026-09-23）：实测 54618 帧里"行进中(v>5km/h)"
      #    视觉 shouldStop 从未为 True（0 帧）—— 模型表达"要停"靠的是
      #    desiredVelocity 趋 0，而不是 shouldStop。因此只按 shouldStop 兜底不够：
      #    速度门控最多只让 3 m/s，实测巡航 50km/h 遇红灯时目标速度被卡在 43.8km/h
      #    ⇒ PID 误差仅 -3 m/s ⇒ "有刹车动作但力度极弱、刹不住、闯红灯"。
      park_intent = (output_v_target_now_e2e < E2E_PARK_V_TH) or (output_a_target_e2e < E2E_PARK_A_TH)
      self._e2e_park_cnt = min(self._e2e_park_cnt + 1, E2E_PARK_CONFIRM) if park_intent else 0
      park_confirmed = self._e2e_park_cnt >= E2E_PARK_CONFIRM
      if park_confirmed:
        # 停车意图已连续确认 100ms ⇒ 解除两项限幅，视觉目标速度/减速度直接生效。
        #   ★ 安全边界不变：MPC 的 a_min/a_max、CRASH_DISTANCE、DANGER_ZONE_COST
        #     以及 longcontrol 的 PID 限幅全部照旧；且 e2e 不参与 MPC 内部状态
        #     ⇒ 即使视觉误报，伤害上限固定、不随时间累积。
        output_a_target = min(output_a_target_mpc, output_a_target_e2e)
        output_v_target_now = min(output_v_target_mpc, output_v_target_now_e2e)
      else:
        # ② 加速度：保留"模型更早减速"的能力，但最多比 MPC 更负 E2E_MAX_DELTA。
        #    依据：e2e 不参与 MPC 内部状态 ⇒ 误报不随时间累积，总伤害上限固定。
        output_a_target = max(output_a_target_mpc - E2E_MAX_DELTA,
                              min(output_a_target_mpc, output_a_target_e2e))
        # ③ 速度门控（修"提速肉/卡 30"）：仅在"模型确实在减速意图"时才允许压低速度目标，
        #    且最多让 E2E_V_MAX_DELTA。实测本车 desiredVelocity P50=25.5km/h，
        #    且有 66.8% 的帧低于巡航设定 ⇒ 无条件 min() 会让车永远提不上速。
        if output_a_target_e2e < E2E_SLOW_A_TH and output_v_target_now_e2e < output_v_target_mpc:
          output_v_target_now = max(output_v_target_mpc - E2E_V_MAX_DELTA, output_v_target_now_e2e)
        else:
          output_v_target_now = output_v_target_mpc
      # ④ shouldStop：布尔量无法限幅，是唯一残留风险口 ⇒ 视觉侧需连续多帧确认
      self._e2e_stop_cnt = min(self._e2e_stop_cnt + 1, E2E_STOP_CONFIRM) if output_should_stop_e2e else 0
      self.output_should_stop = (self._e2e_stop_cnt >= E2E_STOP_CONFIRM) or output_should_stop_mpc

    #for idx in range(2):
    #  accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    #self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    #self.prev_accel_clip = accel_clip
    self.output_a_target = output_a_target
    self.output_v_target_now = output_v_target_now
    self.output_j_target_now = self.j_desired_trajectory[0]

  def publish(self, sm, pm, carrot):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.vTargetNow = float(self.output_v_target_now)
    longitudinalPlan.jTargetNow = float(self.output_j_target_now)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    longitudinalPlan.xState = carrot.xState.value
    longitudinalPlan.trafficState = carrot.trafficState.value
    longitudinalPlan.cruiseTarget = self.v_cruise_kph
    longitudinalPlan.tFollow = float(self.mpc.t_follow)
    longitudinalPlan.desiredDistance = float(self.mpc.desired_distance)
    longitudinalPlan.events = carrot.events.to_msg()
    longitudinalPlan.myDrivingMode = carrot.myDrivingMode.value

    pm.send('longitudinalPlan', plan_send)
