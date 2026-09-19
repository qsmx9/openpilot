#!/usr/bin/env python3
"""Drive stats accumulator.

Runs as a always-on background process. Subscribes to carState and accumulates
per-trip driving distance / duration / trip count into persistent params so the
offroad home UI can render a "driving data" panel.

Data is stored under the ``DriveStats`` param as JSON:
  {"today": {km, min, trips}, "week": {...}, "total": {...},
   "daily": {YYYY-MM-DD: km, ...}, "date": YYYY-MM-DD}

Historical data starts from zero (device has no pre-existing odometer feed).
"""

import json
import time
from datetime import date

from openpilot.common.params import Params
from openpilot.selfdrive.carrot.config import UnifiedParams
from openpilot.common.realtime import Ratekeeper
import cereal.messaging as messaging

PARAM = "DriveStats"
SAVE_INTERVAL = 30.0  # seconds
RATE = 20             # Hz


def _empty_bucket():
  return {"km": 0.0, "min": 0.0, "trips": 0}


def _empty_stats():
  return {"today": _empty_bucket(), "week": _empty_bucket(),
          "total": _empty_bucket(), "daily": {}, "date": ""}


def load_stats(params):
  try:
    raw = params.get(PARAM)
    if raw:
      d = json.loads(raw)
      for k in ("today", "week", "total"):
        if not isinstance(d.get(k), dict):
          d[k] = _empty_bucket()
      if not isinstance(d.get("daily"), dict):
        d["daily"] = {}
      d.setdefault("date", "")
      return d
  except Exception:
    pass
  return _empty_stats()


def save_stats(params, stats):
  try:
    params.put(PARAM, json.dumps(stats))
  except Exception:
    pass


def main():
  params = UnifiedParams()
  sm = messaging.SubMaster(['carState'])
  rk = Ratekeeper(RATE, print_delay_threshold=None)

  stats = load_stats(params)
  engaged_prev = False
  last_save = 0.0
  last_t = time.monotonic()
  last_date = date.today().isoformat()
  stats["date"] = last_date

  while True:
    sm.update()
    now = time.monotonic()
    dt = now - last_t
    last_t = now

    if sm.updated['carState']:
      cs = sm['carState']
      engaged = cs.controlsAllowed
      v = cs.vEgo  # m/s
      if engaged and v > 0.3:
        dist_km = v * dt / 1000.0
        for b in ("today", "week", "total"):
          stats[b]["km"] += dist_km
          stats[b]["min"] += dt / 60.0
        if not engaged_prev:
          for b in ("today", "week", "total"):
            stats[b]["trips"] += 1
        engaged_prev = True
      else:
        engaged_prev = False

    # midnight roll-over: archive today's km into daily[date], reset today
    today = date.today().isoformat()
    if today != last_date:
      if last_date:
        stats["daily"][last_date] = round(stats["today"]["km"], 2)
      stats["today"] = _empty_bucket()
      keys = sorted(stats["daily"].keys())
      for k in keys[:-7]:
        stats["daily"].pop(k, None)
      last_date = today
      stats["date"] = today

    if now - last_save > SAVE_INTERVAL:
      save_stats(params, stats)
      last_save = now

    rk.keep_time()


if __name__ == "__main__":
  main()
