#!/usr/bin/env python3
"""
红绿灯刹车/起步增强（独立模块，不修改原红绿灯识别代码）

设计原则：
- 完全独立开关：TrafficLightBrakeMode = 0 时本模块整段 no-op，对原逻辑零影响。
- 复用原 carrot_functions 输出的逐帧识别结果（carrot.trafficState: red/green/off），
  在其之上做「多帧确认」+「力度接管」，不改动原识别文件本身。
- 红灯：多帧确认后，把停车瞄准点整体前移 margin 米，并把瞄准点收紧到「以 brake_decel
  停住」所需的距离（只收紧、不推远），避免原代码刹车力度不足导致车身过线 / 蠕动过线。
- 绿灯：多帧确认后强制起步，解决原代码置信度不足不起步的问题。
- 安全闸门：关 / 踩油门刹车 / 大幅转向 时一律不干预。

开关：
  TrafficLightBrakeMode   0=关 1=仅红灯加强刹车 2=红灯加强+绿灯强制起步
  TrafficLightBrakeMargin 提前停止距离(0.1m) 默认20 => 2.0m
  TrafficLightBrakeConfirm 确认帧数 默认8 (~0.4s @20Hz) 越大越稳
  TrafficLightBrakeDecel  红灯加强刹车的目标减速度(0.1m/s^2) 默认30 => 3.0，越小越柔和
"""
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.carrot.carrot_functions import XState, TrafficState

TLB_OFF = 0
TLB_RED_ONLY = 1
TLB_RED_GREEN = 2


class TrafficLightBrake:
  def __init__(self):
    self.params = Params()
    self.mode = TLB_OFF
    self.margin = 2.0
    self.confirm_frames = 8
    self.brake_decel = 3.0
    self._pc = 0
    self.red_streak = 0
    self.green_streak = 0
    self.confirmed_red = False
    self.confirmed_green = False

  def _read_params(self):
    # 每 ~0.5s 重读一次，跟随菜单实时调节
    self._pc += 1
    if self._pc % 10 != 0:
      return
    try:
      self.mode = self.params.get_int("TrafficLightBrakeMode")
      self.margin = max(0.0, float(self.params.get_int("TrafficLightBrakeMargin")) * 0.1)
      self.confirm_frames = max(1, self.params.get_int("TrafficLightBrakeConfirm"))
      self.brake_decel = max(0.0, float(self.params.get_int("TrafficLightBrakeDecel")) * 0.1)
    except Exception:
      # 未编译进 params_keys.h 前（理论上不会发生）一律当作关闭，零影响
      self.mode = TLB_OFF

  def _reset(self):
    self.red_streak = 0
    self.green_streak = 0
    self.confirmed_red = False
    self.confirmed_green = False

  def update(self, carrot, sm, v_ego, v_cruise):
    self._read_params()
    # 【仅实验模式】正常模式(acc)直接 no-op, 原逻辑零影响
    try:
      if not sm['selfdriveState'].experimentalMode:
        self._reset()
        return
    except Exception:
      self._reset()
      return
    if self.mode == TLB_OFF:
      self._reset()
      return

    cs = sm['carState']
    # 用户主动控速 / 大幅转向：完全不干预，原逻辑主导
    if cs.gasPressed or cs.brakePressed or abs(cs.steeringAngleDeg) > 20:
      self._reset()
      return

    ts = carrot.trafficState
    if ts == TrafficState.red:
      self.red_streak += 1
      self.green_streak = 0
    elif ts == TrafficState.green:
      self.green_streak += 1
      self.red_streak = 0
    else:
      # off：缓慢衰减，抗单帧抖动
      self.red_streak = max(0, self.red_streak - 1)
      self.green_streak = max(0, self.green_streak - 1)

    self.confirmed_red = self.red_streak >= self.confirm_frames
    self.confirmed_green = self.green_streak >= self.confirm_frames

    # —— 红灯：把停车瞄准点收到停止线内 + 保证足够的制动力 ——
    if self.mode >= TLB_RED_ONLY and self.confirmed_red:
      # 1) 提前停止距离：瞄准点整体前移 margin 米(保证车身不过线)
      d = max(0.0, carrot.stop_dist - self.margin)
      # 2) 保证最小制动减速度：把瞄准点收紧到「以 brake_decel 停住」所需的距离。
      #    只收紧不推远(d 已比 need 小 ⇒ 本来就要刹得更狠, 保持不动),
      #    绝不把停车点推到停止线之外 —— 那正是原实现「软绵绵 + 越线」的根因。
      if self.brake_decel > 0.0:
        need = (v_ego ** 2) / (2.0 * self.brake_decel)
        if d > need:
          d = need
      carrot.stop_dist = d
      carrot.v_cruise = 0.0
      carrot.mode = 'acc'
      # 已停稳时把停止点顶在提前量上, 防止蠕动过线
      if v_ego < 0.3 and carrot.stop_dist < self.margin:
        carrot.stop_dist = self.margin

    # —— 绿灯：多帧确认后强制起步 ——
    if self.mode == TLB_RED_GREEN and self.confirmed_green:
      if (carrot.xState in (XState.e2eStop, XState.e2eStopped) and
          not cs.leftBlinker and not carrot.carrot_stay_stop and
          carrot.trafficLightDetectMode == 2):
        carrot.xState = XState.e2eCruise
        carrot.stop_dist = 0.0
        carrot.v_cruise = v_cruise
        carrot.mode = 'acc'
        carrot.traffic_starting_count = 0
