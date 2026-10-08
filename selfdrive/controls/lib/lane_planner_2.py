import json
import math
import os
import numpy as np
from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
# from openpilot.common.swaglog import cloudlog
# from openpilot.common.logger import sLogger
from openpilot.common.params import Params

TRAJECTORY_SIZE = 33
# positive numbers go right
CAMERA_OFFSET = 0 #0.08
MIN_LANE_DISTANCE = 2.6
MAX_LANE_DISTANCE = 3.7
MAX_LANE_CENTERING_AWAY = 1.85
KEEP_MIN_DISTANCE_FROM_LANE = 1.35
KEEP_MIN_DISTANCE_FROM_EDGELANE = 1.15

# 无车道线时用两侧路沿几何中心替代模型偏向路径
# 总开关 EDGE_CENTERING_ENABLED(代码层); 用户开关由 UI 参数 EdgeCenteringEnabled 控制(默认开, 部署时预设=1)
EDGE_CENTERING_ENABLED = True

def clamp(num, min_value, max_value):
  # weird broken case, do something reasonable
  if min_value > num > max_value:
    return (min_value + max_value) * 0.5
  # ok, basic min/max below
  if num < min_value:
    return min_value
  if num > max_value:
    return max_value
  return num

def sigmoid(x, scale=1, offset=0):
  return (1 / (1 + math.exp(x*scale))) + offset

def lerp(start, end, t):
  t = clamp(t, 0.0, 1.0)
  return (start * (1.0 - t)) + (end * t)

def max_abs(a, b):
  return a if abs(a) > abs(b) else b

class LanePlanner:
  def __init__(self):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.le_y = np.zeros((TRAJECTORY_SIZE,))
    self.re_y = np.zeros((TRAJECTORY_SIZE,))
    #self.lane_width_estimate = FirstOrderFilter(3.2, 9.95, DT_MDL)
    self.lane_width_estimate = FirstOrderFilter(3.2, 3.0, DT_MDL)
    self.lane_width = 3.2
    self.lane_width_last = self.lane_width
    self.lane_change_multiplier = 1
    #self.lane_width_updated_count = 0

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.debugText = ""
    self.lane_width_left = 0.0
    self.lane_width_right = 0.0
    self.lane_width_left_filtered = FirstOrderFilter(1.0, 1.0, DT_MDL)
    self.lane_width_right_filtered = FirstOrderFilter(1.0, 1.0, DT_MDL)
    self.lane_offset_filtered = FirstOrderFilter(0.0, 2.0, DT_MDL)

    self.lanefull_mode = False
    self.d_prob_count = 0

    self.params = Params()

    # === 大路口动态偏移限制 (硬编码参数) ===
    self.curveOffsetLimit = 0.15       # 最大偏移限制(m), 0=关闭
    self.curveSpeedThreshold = 70      # curve_speed阈值, 超过此值视为大路口
    self.curvature = 0.0               # 本帧实测曲率(1/m); 由 lateral_planner 每帧写入,
                                       # 这里显式初始化以免首帧 getattr 落空(curve gate 判据要用)

    # === AutoCenter: 自动居中纠偏 (auto lane centering correction) ===
    # 0: off, 1: realtime correction only, 2: realtime + long-term learning (persisted)
    self.ac_enabled = 2
    self.ac_gain = 0.4                                        # P gain for realtime correction
    self.ac_error_filtered = FirstOrderFilter(0.0, 3.0, DT_MDL)  # 3s 低通 (2026-09-24 夜间居中: 1s->3s)
    self.ac_learned = 0.0                                     # slow-learned static offset (m)
    self.ac_learned_saved = 0.0
    self.ac_applied = 0.0                                     # rate-limited output (m)
    self.ac_param_ok = False                                  # params registered? else json file fallback
    self.ac_read_counter = 0
    self.ac_cmd_counter = 0
    self.ac_save_counter = 0
    self.ac_curve_gate = True                                 # 弯道暂停学习(默认开; 由 AutoLaneCorrectionCurveGate 覆盖)
    # --- 诊断读数 (2026-10-07 新增: 供 UI「自动居中纠正记录」弹窗显示, 不影响任何控制逻辑) ---
    self.ac_error_now = 0.0      # 本帧实测偏差(m): 正 = 车道中心在车右(即车偏左)
    self.ac_state = 0            # 状态码: 0关/1低速/2无线/3宽异常/4变道/5干预/6已居中/7正在纠正
    self.ac_learn_state = 0      # 学习码: 0未启用/1学习中/2弯道暂停/3无线暂停
    self.ac_meas_ok = False      # 本帧测量是否可信
    self.ac_saturated = False    # 学习值是否已顶到上限(提示用户存在未能补偿的持续偏差)
    # --- 学习控制 / 行程读数 (2026-10-07 新增, 供「清零」与弹窗) ---
    self.ac_trip_learn_min = 0.0 # 本次行程内学习值的最小/最大值(m), 供弹窗看出"在稳还是在漂"
    self.ac_trip_learn_max = 0.0
    self.ac_park_frames = 0      # 连续停车帧计数(用于切分行程)
    self.ac_sat_residual = 0.0   # 顶格期间的残差慢平均(m): 顶格后仍剩这么大偏差没补上 ⇒ 判断上限是否真卡住
    self.ac_sat_seen = False     # 本次行程内是否出现过顶格
    self.ac_sat_frames = 0       # 连续顶格帧数(顶格保护用)
    self.ac_releasing = False    # 正在"脱顶": 把学习值从上限缓慢拉回中性
    self._ac_load()

  AC_FILE = "/data/carrot_auto_center.json"
  AC_LEARN_LIMIT = 0.15      # max learned static offset (m)
  AC_P_LIMIT = 0.20          # max realtime correction (m)
  AC_TOTAL_LIMIT = 0.30      # max total correction (m)
  AC_LEARN_TAU = 20.0        # 学习时间常数(s): 2026-10-07 由 45 改为 20。
                             #   依据(离线实测, 非推断): 学习目标不是"固定偏置", 而是在「小时」尺度上
                             #   漂移——5天内 -9~+15cm 往返、同一天内就走完全程。45s 跟不上漂移速度。
  AC_RATE_LIMIT = 0.06       # max output slew rate (m/s)
  AC_DEADBAND = 0.015       # 死区(m): 2026-10-07 由 0.06 改为 0.015。
                            #   原 6cm 死区让"最后 6cm"完全没有实时修正(只剩慢积分兜底);
                            #   实测 63.5% 的可学习帧落在 |e|<6cm 内 ⇒ 改为 1.5cm, 只压 3s 低通后的噪声。
                            #   仅作用于 P 项输出(学习器输入早已不吃死区)。
  AC_CURVE_GATE = 0.0015    # 学习曲率门限(1/m): |curvature| 超过则暂停学习
                            #   原因: 弯道中模型车道中心存在系统偏差, 学进去会污染"静态偏置"(直道上反而歪)
  AC_WIDTH_TOL = 1.0        # 车道宽一致性容差(m): 与平滑估计值差超过此值视为误锁相邻车道线, 本帧弃用
  AC_SATURATED_RATIO = 0.97 # 学习值达到上限的该比例即视为"顶格"(UI 提示)
  AC_PARK_RESET_SEC = 60.0  # 连续停车超过该秒数视为"新行程", 复位行程学习值区间读数
  # --- 顶格保护 (2026-10-08 新增; 代码层常量, 不新增 UI 参数、不新增 [AC] 字段) ---
  AC_SAT_HYST = 0.90         # 顶格标志滞回: 进入用 AC_SATURATED_RATIO(0.97), 退出用本值
                             #   原单阈值在 0.97 边缘会反复翻转, 顶格标志(UI 的"顶"字)会抖
  AC_SAT_HOLD_SEC = 15.0     # 顶格持续超过该秒数才判定"该脱顶"(避免刚碰上限就被拉回)
  AC_SAT_RELEASE_TAU = 30.0  # 脱顶时学习值朝 0 回退的时间常数(s); 越大回得越慢
  AC_SAT_RELEASE_UNTIL = 0.6 # 回退到限幅的该比例即停止脱顶, 恢复正常积分
  AC_SAT_TAU = 10.0         # "顶格时残留偏差"观测用的时间常数(s)

  def _ac_read_config(self):
    # prefer Params (needs key registration + rebuild); fallback to json file (no rebuild needed)
    try:
      self.ac_enabled = self.params.get_int("AutoLaneCorrection")
      gain = self.params.get_int("AutoLaneCorrectionGain")
      self.ac_gain = clamp(gain, 10, 100) * 0.01 if gain > 0 else 0.4
      self.ac_param_ok = True
    except Exception:
      self.ac_param_ok = False
      try:
        if os.path.exists(self.AC_FILE):
          with open(self.AC_FILE) as f:
            d = json.load(f)
          self.ac_enabled = int(d.get("enabled", 2))
          self.ac_gain = clamp(float(d.get("gain", 40)), 10, 100) * 0.01
      except Exception:
        pass
    # 弯道暂停学习开关(独立读取; 读不到/未注册时保持默认开启 —— 更保守, 不会被弯道污染)
    try:
      self.ac_curve_gate = bool(self.params.get_bool("AutoLaneCorrectionCurveGate"))
    except Exception:
      self.ac_curve_gate = True

  def _ac_poll_cmd(self):
    # 一次性命令轮询(~1s 节拍, 由 update_auto_center 里的 ac_cmd_counter 驱动,
    #   比 _ac_read_config 的 5s 更快 —— 用户在设置里点完不该干等 5 秒)。
    #   为什么单独一个函数: 命令执行会改学习值, 不能让它在 __init__ 的 _ac_load 里被触发,
    #   否则写盘是异步的, _ac_load 紧接着读回旧值会把"清零"覆盖掉。
    try:
      ac_cmd = self.params.get_int("AutoLaneCorrectionCmd")
    except Exception:
      return
    if ac_cmd != 0:
      self._ac_run_cmd(ac_cmd)

  def _ac_run_cmd(self, cmd):
    # 命令码: 1 = 清零学习值(并复位积分/滤波), 2 = 立即保存当前学习值
    if cmd == 1:
      self.ac_learned = 0.0
      self.ac_learned_saved = 0.0
      self.ac_error_filtered.x = 0.0
      self.ac_applied = 0.0
      self.ac_sat_residual = 0.0
      self.ac_sat_seen = False
      self.ac_sat_frames = 0
      self.ac_releasing = False
      self.ac_trip_learn_min = 0.0
      self.ac_trip_learn_max = 0.0
      self._ac_save()
    elif cmd == 2:
      self._ac_save()
    # 执行完必须把命令位写回 0, 否则每 1s 会重复执行一次。
    #   用非阻塞写: 立刻写回旧值也没关系, cmd=1/2 都是幂等的。
    try:
      self.params.put_int_nonblocking("AutoLaneCorrectionCmd", 0)
    except Exception:
      pass

  def _ac_load(self):
    self._ac_read_config()
    try:
      if self.ac_param_ok:
        self.ac_learned = clamp(self.params.get_float("AutoLaneCorrectionLearned"), -self.AC_LEARN_LIMIT, self.AC_LEARN_LIMIT)
      elif os.path.exists(self.AC_FILE):
        with open(self.AC_FILE) as f:
          d = json.load(f)
        self.ac_learned = clamp(float(d.get("learned", 0.0)), -self.AC_LEARN_LIMIT, self.AC_LEARN_LIMIT)
    except Exception:
      self.ac_learned = 0.0
    self.ac_learned_saved = self.ac_learned
    # 行程区间读数从"当前学习值"起步, 而不是 0 —— 否则弹窗会显示"本次行程 0.0~Xcm",
    # 那个 0 是初始化默认值、不是真实最小值, 会让人误判"它从 0 一路学到 X"。
    self.ac_trip_learn_min = self.ac_learned
    self.ac_trip_learn_max = self.ac_learned

  def _ac_save(self):
    try:
      if self.ac_param_ok:
        self.params.put_float_nonblocking("AutoLaneCorrectionLearned", float(self.ac_learned))
      else:
        with open(self.AC_FILE, "w") as f:
          json.dump({"enabled": self.ac_enabled, "gain": round(self.ac_gain * 100), "learned": round(self.ac_learned, 4)}, f)
      self.ac_learned_saved = self.ac_learned
    except Exception:
      pass

  def update_auto_center(self, CS, v_ego):
    # 配置重读: ~5s 一次(100 帧; plannerd 跑 20Hz, DT_MDL=0.05, 故 1800 帧 = 90s 与保存节拍一致)
    self.ac_read_counter += 1
    if self.ac_read_counter >= 100:
      self.ac_read_counter = 0
      self._ac_read_config()
    # 一次性命令用更快的节拍(~1s): 用户在设置里点完不该干等 5 秒
    self.ac_cmd_counter += 1
    if self.ac_cmd_counter >= 20:
      self.ac_cmd_counter = 0
      self._ac_poll_cmd()

    # 学习饱和检测(供 UI 提示: 学习值已顶到上限 = 存在本功能补偿不完的持续偏差)
    #   2026-10-08: 加滞回 —— 进入 0.97 / 退出 0.90。单阈值在边缘来回翻转会让
    #   顶格标志(以及顶格保护的计时)反复起停, 脱顶刚启动就被判"不顶了"而中断。
    if self.ac_saturated:
      self.ac_saturated = abs(self.ac_learned) > self.AC_LEARN_LIMIT * self.AC_SAT_HYST
    else:
      self.ac_saturated = abs(self.ac_learned) >= self.AC_LEARN_LIMIT * self.AC_SATURATED_RATIO

    if self.ac_enabled <= 0:
      self.ac_error_filtered.x = 0.0
      self.ac_applied = 0.0
      self.ac_error_now = 0.0
      self.ac_state = 0
      self.ac_learn_state = 0
      self.ac_meas_ok = False
      return 0.0

    # --- 顶格计时(顶格保护): 顶格持续够久 ⇒ 从这一帧起进入"受控脱顶" ---
    #   放在 enabled<=0 早退之后: 功能关掉时不做任何计时/状态累积。
    #   只加计时不做动作 —— 动作放在下面"meas_ok"分支里, 与积分互斥。
    if self.ac_saturated:
      self.ac_sat_frames += 1
      if self.ac_sat_frames >= int(self.AC_SAT_HOLD_SEC / DT_MDL):
        self.ac_releasing = True
    elif not self.ac_releasing:
      self.ac_sat_frames = 0

    # --- 行程分割 + 学习值区间读数(纯诊断) ---
    #   连续停车 > AC_PARK_RESET_SEC 视为新行程一次, 复位区间与顶格观测。
    #   意义: 弹窗只看"当前值"分不清"在收敛"还是"在绕圈"; 有了本次行程的 min/max 就能一眼判断。
    if v_ego * 3.6 < 1.0:
      self.ac_park_frames += 1
      if self.ac_park_frames == int(self.AC_PARK_RESET_SEC / DT_MDL):
        self.ac_trip_learn_min = self.ac_learned
        self.ac_trip_learn_max = self.ac_learned
        self.ac_sat_residual = 0.0
        self.ac_sat_seen = False
        self.ac_sat_frames = 0
        self.ac_releasing = False
    else:
      self.ac_park_frames = 0
    if self.ac_learned < self.ac_trip_learn_min:
      self.ac_trip_learn_min = self.ac_learned
    if self.ac_learned > self.ac_trip_learn_max:
      self.ac_trip_learn_max = self.ac_learned

    # measurement gating: only trust confident, sane lane lines while driving straight-ish
    lane_width_now = self.rll_y[0] - self.lll_y[0]
    lines_ok = (self.lll_prob > 0.35 and self.rll_prob > 0.35 and   # 夜间居中: 0.5->0.35
                self.lll_std < 0.5 and self.rll_std < 0.5)          # 夜间居中: 0.3->0.5
    # 车道宽合理性: 绝对范围 + 与平滑估计值的一致性
    #   只靠绝对范围挡不住"把相邻车道线误当本车道"(那也可能落在 2.5~4.6m), 但与本车道上帧估计值差很大
    width_ok = 2.5 < lane_width_now < 4.6
    lane_width_ref = getattr(self, "lane_width", 0.0)
    if width_ok and 1.5 < lane_width_ref < 5.0:
      width_ok = abs(lane_width_now - lane_width_ref) < self.AC_WIDTH_TOL

    slow = v_ego * 3.6 <= 15.0
    steering = bool(CS.steeringPressed)
    changing = self.lane_change_multiplier <= 0.5

    # --- 状态码: 让 UI 说清"正在纠正什么 / 为什么没纠正" ---
    if slow:
      self.ac_state = 1
    elif changing:
      self.ac_state = 4
    elif steering:
      self.ac_state = 5
    elif not lines_ok:
      self.ac_state = 2
    elif not width_ok:
      self.ac_state = 3
    else:
      self.ac_state = 6            # 下面按误差大小细化为 6(已居中) / 7(正在纠正)

    meas_ok = (not slow) and (not changing) and (not steering) and lines_ok and width_ok
    self.ac_meas_ok = meas_ok

    if meas_ok:
      # e > 0: lane center is to the right of car -> shift path right (y positive = right in this fork)
      error = clamp((self.lll_y[0] + self.rll_y[0]) * 0.5, -0.6, 0.6)
      self.ac_error_now = error
      # 学习器改吃「未死区」的滤波值:
      #   旧实现先把 <6cm 的偏差归零再喂积分器 ⇒ 积分器在接近中心时输入恒为 0,
      #   永远学不到 6cm 以内的精细补偿, 居中精度被死区卡死在 ±6cm。
      #   滤波本身是 3s 低通(约 60 帧平均), 抗噪足够; 死区只保留在 P 项输出上。
      self.ac_error_filtered.update(error)
      # 曲率门限: 弯道中"模型预测的车道中心"有系统偏差(曲率越大越明显),
      #   学进去会把弯道特性污染成静态偏置, 结果直道上反而歪。只在近似直道时学习。
      cur = abs(getattr(self, "curvature", 0.0))
      curve_ok = (not self.ac_curve_gate) or (cur < self.AC_CURVE_GATE)
      if self.ac_enabled >= 2 and curve_ok and self.ac_releasing:
        # === 顶格保护 (2026-10-08): 顶格过久 ⇒ 受控"脱顶" ===
        #   背景(route 94 离线实测): 学习值在 ±15cm 内来回跑, 最后停在刻度顶端不动,
        #   停车 16 分钟也不退 ⇒ 下次上车先带着 15cm 的注入量起步。
        #   只靠 clamp 是"进得去、出不来": 想从中性重新学, 得先反向走完整刻度, 极慢。
        #   做法: 按 AC_SAT_RELEASE_TAU 把学习值朝 0 缓慢拉, 拉回限幅 60% 以内即恢复学习。
        #   与积分互斥(同一帧二选一): 否则"回退"会被"继续积分"当场抵消, 等于没保护。
        self.ac_learned -= self.ac_learned * (DT_MDL / self.AC_SAT_RELEASE_TAU)
        if abs(self.ac_learned) <= self.AC_LEARN_LIMIT * self.AC_SAT_RELEASE_UNTIL:
          self.ac_releasing = False
          self.ac_sat_frames = 0
        self.ac_learn_state = 1        # 沿用"学习中": 不新增 q 码, 免得读取端不认识
      elif self.ac_enabled >= 2 and curve_ok:
        # slow integrator removes steady-state bias (camera mount / steering bias)
        self.ac_learned = clamp(self.ac_learned + self.ac_error_filtered.x * (DT_MDL / self.AC_LEARN_TAU),
                                -self.AC_LEARN_LIMIT, self.AC_LEARN_LIMIT)
        self.ac_learn_state = 1
      elif self.ac_enabled >= 2:
        self.ac_learn_state = 2        # 学习暂停(弯道)
      else:
        self.ac_learn_state = 0
      # --- 顶格观测(纯诊断, 不参与任何控制) ---
      #   学习值已顶到上限、却仍有偏差 ⇒ "上限"很可能就是卡住它的东西。
      #   把这里的残留偏差做 10s 慢平均给 UI 看:
      #     接近 0    = 顶格无害(该补的已经补完了, 只是数值停在刻度顶端)
      #     明显非 0  = 上限真的不够(这就是本功能补偿不完的那个持续偏差)
      if self.ac_saturated:
        self.ac_sat_residual += (error - self.ac_sat_residual) * (DT_MDL / self.AC_SAT_TAU)
        self.ac_sat_seen = True
      if self.ac_state == 6 and abs(error) >= self.AC_DEADBAND:
        self.ac_state = 7              # 正在纠正
    else:
      self.ac_error_filtered.update(0.0)
      self.ac_error_now = 0.0
      self.ac_learn_state = 3 if self.ac_enabled >= 2 else 0

    # persist learned offset every ~90s when changed > 8mm
    #   旧值 600帧(30s)/5mm: 学习期间约每 30s 就写一次 params 文件, 长期白占 flash 写入; 放宽到 90s/8mm
    self.ac_save_counter += 1
    if self.ac_save_counter >= 1800:
      self.ac_save_counter = 0
      if abs(self.ac_learned - self.ac_learned_saved) > 0.008:
        self._ac_save()

    p_err = self.ac_error_filtered.x if abs(self.ac_error_filtered.x) >= self.AC_DEADBAND else 0.0
    p_term = clamp(p_err * self.ac_gain, -self.AC_P_LIMIT, self.AC_P_LIMIT)
    target = clamp(p_term + self.ac_learned, -self.AC_TOTAL_LIMIT, self.AC_TOTAL_LIMIT)
    max_step = self.AC_RATE_LIMIT * DT_MDL
    self.ac_applied = clamp(target, self.ac_applied - max_step, self.ac_applied + max_step)
    return self.ac_applied

  def parse_model(self, md):

    lane_lines = md.laneLines
    edges = md.roadEdges

    if len(lane_lines) >= 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      self.lll_y = np.array(lane_lines[1].y)
      self.rll_y = np.array(lane_lines[2].y)
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    if len(edges[0].t) == TRAJECTORY_SIZE:
      self.le_y = np.array(edges[0].y) + md.roadEdgeStds[0] * 0.4
      self.re_y = np.array(edges[1].y) - md.roadEdgeStds[1] * 0.4
    else:
      self.le_y = self.lll_y
      self.re_y = self.rll_y

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.Desire.laneChangeRight]

  def get_d_path(self, CS, v_ego, path_t, path_xyz, curve_speed):
    #if v_ego > 0.1:
    #  self.lane_width_updated_count = max(0, self.lane_width_updated_count - 1)
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = np.interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      #prob_mods.append(np.interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
      prob_mods.append(np.interp(width_at_t, [4.5, 6.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = np.interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = np.interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    self.l_prob, self.r_prob = l_prob, r_prob

    # Find current lanewidth
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])

    max_updated_count = 10.0 * DT_MDL
    both_lane_available = False
    #speed_lane_width = np.interp(v_ego*3.6, [0., 60.], [2.8, 3.5])
    if l_prob > 0.5 and r_prob > 0.5 and self.lane_change_multiplier > 0.5:
      both_lane_available = True
      #self.lane_width_updated_count = max_updated_count
      self.lane_width_estimate.update(current_lane_width)
      self.lane_width_last = self.lane_width_estimate.x
    #elif self.lane_width_updated_count <= 0 and v_ego > 0.1:   # 양쪽차선이 없을때.... 일정시간후(10초)부터 speed차선폭 적용함.
    #  self.lane_width_estimate.update(speed_lane_width)
    else:
      self.lane_width_estimate.update(self.lane_width_last)

    self.lane_width =  self.lane_width_estimate.x

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    # 가장 차선이 진한쪽으로 골라서..
    self.d_prob = max(l_prob, r_prob) if not both_lane_available else 1.0

    # 좌/우의 차선폭을 필터링.
    if self.lane_width_left > 0:
      self.lane_width_left_filtered.update(self.lane_width_left)
      #self.lane_width_left_filtered.x = self.lane_width_left #바로적용
    if self.lane_width_right > 0:
      self.lane_width_right_filtered.update(self.lane_width_right)
      #self.lane_width_right_filtered.x = self.lane_width_right #바로적용

    self.adjustLaneOffset = float(self.params.get_int("AdjustLaneOffset")) * 0.01
    self.adjustCurveOffset = float(self.params.get_int("AdjustCurveOffset")) * 0.01
    #self.adjustCurveOffset = self.adjustLaneOffset #float(self.params.get_int("AdjustCurveOffset")) * 0.01

    # === 大路口动态偏移限制 (硬编码参数) ===
    # self.curveOffsetLimit 和 self.curveSpeedThreshold 已在 __init__ 中初始化

    ADJUST_OFFSET_LIMIT = 0.4 #max(self.adjustLaneOffset, self.adjustCurveOffset)
    offset_curve = 0.0
    ## curve offset
    offset_curve = np.interp(abs(curve_speed), [50, 200], [self.adjustCurveOffset, 0.0]) * np.sign(curve_speed)

    offset_lane = 0.0
    if self.lane_width_left_filtered.x > 2.2 and self.lane_width_right_filtered.x > 2.2: #양쪽에 차로가 여유 있는경우
      offset_lane = 0.0
    elif self.lane_width_left_filtered.x < 2.0 and self.lane_width_right_filtered.x < 2.0: #양쪽에 차로가 여유 없는경우
      offset_lane = 0.0
    elif self.lane_width_left_filtered.x > self.lane_width_right_filtered.x:
      offset_lane = np.interp(self.lane_width, [2.5, 2.9], [0.0, self.adjustLaneOffset]) # 차선이 좁으면 안함..
    else:
      offset_lane = np.interp(self.lane_width, [2.5, 2.9], [0.0, -self.adjustLaneOffset]) # 차선이 좁으면 안함..

    #select lane path
    # 차선이 좁아지면, 도로경계쪽에 있는 차선 위주로 따라가도록함.
    if self.lane_width < 2.5:
      if r_prob > 0.5 and self.lane_width_right_filtered.x < self.lane_width_left_filtered.x:
        lane_path_y = path_from_right_lane
      elif l_prob > 0.5 and self.lane_width_left_filtered.x < 2.0:
        lane_path_y = path_from_left_lane
      else:
        lane_path_y = path_from_left_lane if l_prob > 0.5 or l_prob > r_prob else path_from_right_lane
    elif l_prob > 0.7 and r_prob > 0.7:
      lane_path_y = (path_from_left_lane + path_from_right_lane) / 2.
      # lane_width filtering에 의해서, 점점 줄어들때, 중앙선으로 붙어가는 현상이 생김..
      #if self.lane_width > 3.2:
      #  lane_path_y = path_from_right_lane
      #else:
      #  lane_path_y = (path_from_left_lane + path_from_right_lane) / 2.
    # 그외 진한차선을 따라가도록함.
    else:
      lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)

    use_laneless_center_adjust = False
    if use_laneless_center_adjust:
      ## 0.5초 앞의 중심을 보도록함.
      lane_path_y_center = np.interp(0.5, path_t, lane_path_y)
      path_xyz_y_center = np.interp(0.5, path_t, path_xyz[:,1])
      #lane_path_y_center = lane_path_y[0]
      #path_xyz_y_center = path_xyz[:,1][0]
      diff_center = (lane_path_y_center - path_xyz_y_center) if not self.lanefull_mode else 0.0
    else:
      diff_center = 0.0
    #print("center = {:.2f}={:.2f}-{:.2f}, lanefull={}".format(diff_center, lane_path_y_center, path_xyz_y_center, self.lanefull_mode))
    #diff_center = lane_path_y[5] - path_xyz[:,1][5] if not self.lanefull_mode else 0.0
    if offset_curve * offset_lane < 0:
      offset_total = np.clip(offset_curve + offset_lane + diff_center, - ADJUST_OFFSET_LIMIT, ADJUST_OFFSET_LIMIT)
    else:
      offset_total = np.clip(max(offset_curve, offset_lane, key=abs) + diff_center, - ADJUST_OFFSET_LIMIT, ADJUST_OFFSET_LIMIT)

    ## self.d_prob = 0 if lane_changing
    self.d_prob *= self.lane_change_multiplier  ## 차선변경중에는 꺼버림.
    if self.lane_change_multiplier < 0.5:
      #self.lane_offset_filtered.x = 0.0
      pass
    else:
      self.lane_offset_filtered.update(np.interp(self.d_prob, [0, 0.3], [0, offset_total]))

    # === 大路口动态偏移限制 ===
    # curve_speed 大 = 曲率小 = 大路口(弯道平缓)
    # 仅在 curveOffsetLimit > 0(功能开启) 且 curve_speed 超过阈值时生效
    if self.curveOffsetLimit > 0 and abs(curve_speed) > self.curveSpeedThreshold:
      if curve_speed > 0:    # 左转（大路口）：限制向左的最大偏移
        self.lane_offset_filtered.x = min(self.lane_offset_filtered.x, self.curveOffsetLimit)
      elif curve_speed < 0:  # 右转（大路口）：限制向右的最大偏移
        self.lane_offset_filtered.x = max(self.lane_offset_filtered.x, -self.curveOffsetLimit)

    ## laneless at lowspeed
    self.d_prob *= np.interp(v_ego*3.6, [5., 15.], [0.0, 1.0])

    #self.debugText = "OFFSET({:.2f}={:.2f}+{:.2f}+{:.2f}),Vc:{:.2f},dp:{:.1f},lf:{},lrw={:.1f}|{:.1f}|{:.1f}".format(
    #  self.lane_offset_filtered.x,
    #  diff_center, offset_lane, offset_curve,
    #  curve_speed,
    #  self.d_prob, self.lanefull_mode,
    #  self.lane_width_left_filtered.x, self.lane_width, self.lane_width_right_filtered.x)

    adjustLaneTime = self.params.get_float("LatMpcInputOffset") * 0.01 # 0.06
    laneline_active = False
    self.d_prob_count = self.d_prob_count + 1 if self.d_prob > 0.3 else 0
    if self.lanefull_mode and self.d_prob_count > int(1 / DT_MDL):
      laneline_active = True
      use_dist_mode = False  ## 아무리생각해봐도.. 같은 방법인듯...
      if use_dist_mode:
        lane_path_y_interp = np.interp(path_xyz[:,0] + v_ego * adjustLaneTime, self.ll_x, lane_path_y)
        path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
      else:
        safe_idxs = np.isfinite(self.ll_t)
        if safe_idxs[0]:
          lane_path_y_interp = np.interp(path_t * (1.0 + adjustLaneTime), self.ll_t[safe_idxs], lane_path_y[safe_idxs])
          path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]

    # AutoCenter: continuous auto correction towards lane center (works in lane mode AND laneless mode)
    ac_offset = self.update_auto_center(CS, v_ego)

    # === 路沿居中: 无车道线时用两侧路沿几何中心, 替代模型偏向的一侧 ===
    # 仅当车道线不可用时生效(有车道线路段 blend=0 完全不动), 不影响正常使用
    if EDGE_CENTERING_ENABLED and self.params.get_bool("EdgeCenteringEnabled"):
      lanelines_unavailable = (self.l_prob < 0.5) and (self.r_prob < 0.5)
      edge_width = abs(self.re_y[0] - self.le_y[0])
      edges_plausible = (1.5 < edge_width < 8.0)
      if lanelines_unavailable and edges_plausible:
        edge_center_traj = (self.le_y + self.re_y) * 0.5
        # 将路沿中心沿 x 轴对齐到 path 网格(与 lane_lines 同坐标系)
        if self.ll_x[-1] > self.ll_x[0]:
          edge_center_on_path = np.interp(path_xyz[:, 0], self.ll_x, edge_center_traj)
        else:
          edge_center_on_path = edge_center_traj
        # d_prob 越低(越无车道线) -> 越信任路沿中心; 有车道线时 blend=0 不动
        blend = clamp(1.0 - self.d_prob, 0.0, 1.0)
        path_xyz[:, 1] = (1.0 - blend) * path_xyz[:, 1] + blend * edge_center_on_path

    path_xyz[:, 1] += (CAMERA_OFFSET + self.lane_offset_filtered.x + ac_offset)

    self.offset_total = self.lane_offset_filtered.x + ac_offset

    return path_xyz, laneline_active

  def calculate_plan_yaw_and_yaw_rate(self, path_xyz):
    if path_xyz.shape[0] < 3:
        # 너무 짧으면 직진 가정
        N = path_xyz.shape[0]
        return np.zeros(N), np.zeros(N)

    # x, y 추출
    x = path_xyz[:, 0]
    y = path_xyz[:, 1]

    # 모두 동일한 점인지 확인
    if np.allclose(x, x[0]) and np.allclose(y, y[0]):
        return np.zeros(len(x)), np.zeros(len(x))

    # 안전한 diff 계산
    dx = np.diff(x)
    dy = np.diff(y)
    mask = (dx == 0) & (dy == 0)
    dx[mask] = 1e-4
    dy[mask] = 0.0

    yaw = np.arctan2(dy, dx)
    yaw = np.append(yaw, yaw[-1])  # N-1 → N
    yaw = np.unwrap(yaw)

    dx_full = np.clip(np.diff(x), 1e-4, None)
    yaw_rate = np.diff(yaw) / dx_full
    yaw_rate = np.append(yaw_rate, yaw_rate[-1])
    yaw_rate = np.append(yaw_rate, 0.0)

    # NaN/Inf 방어
    if np.any(np.isnan(yaw_rate)) or np.any(np.isinf(yaw_rate)):
        yaw_rate = np.zeros_like(yaw_rate)
    if np.any(np.isnan(yaw)) or np.any(np.isinf(yaw)):
        yaw = np.zeros_like(yaw)

    return yaw, yaw_rate