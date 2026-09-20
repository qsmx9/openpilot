#!/usr/bin/env python3
"""Drive stats accumulator + software RTC fallback.

Runs as a always-on background process. Subscribes to carState and accumulates
per-trip driving distance / duration / trip count, then writes a JSON summary
that the offroad home UI renders as a "driving data" panel.

Data is stored as a plain JSON file (NOT a Params key, to avoid the compiled
whitelist). The UI reads the same file directly:
  {"today": {km, min, trips}, "week": {...}, "total": {...},
   "daily": {YYYY-MM-DD: km, ...}, "date": YYYY-MM-DD}

Historical data starts from zero (device has no pre-existing odometer feed).

Software RTC fallback (c3 has NO hardware RTC):
  We persist the last known wall-clock time to CLOCK_TS on /data (which
  survives power loss). On startup, if the system clock is clearly behind that
  record (e.g. just booted and reset to 1970), we restore it via `date -s`.
  While online, systemd-timesyncd keeps the clock accurate and overrides this.
  This keeps the device showing a sane date/time even offline, like a clock.
"""

import json
import os
import time
import subprocess
from datetime import date

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper

# NOTE: keep these OUTSIDE /data/params on purpose.
#   1) system/manager/manager.py calls Params.clearAll(CLEAR_ON_MANAGER_START) on
#      every boot, and common/params.cc deletes every file in the params dir that
#      is not a key in common/params_keys.h -> these JSON/timestamp files were
#      wiped on every reboot when they lived in the params dir.
#   2) /data/params/d* is a symlink to a per-install temp dir, so a hardcoded
#      /data/params/d_tmp no longer resolves to the active params dir on a fresh
#      install (the UI "reset calibration" action pointed there as well).
STATS_DIR = "/data/drive_stats"
STATS_FILE = os.path.join(STATS_DIR, "drive_stats.json")
CLOCK_TS = os.path.join(STATS_DIR, "clock.ts")
LEGACY_DIR = "/data/params/d_tmp"  # pre 2026-09-21 location, migrated once
# Timestamps below this are considered bogus (1970 epoch, or the AGNOS default
# bogus date ~2025-06-04). They must never be persisted/restored, or they would
# poison the software-RTC fallback. 1700000000 == 2023-11-14.
SANE_EPOCH = 1700000000.0
SAVE_INTERVAL = 30.0  # seconds
RATE = 20             # Hz


def _empty_bucket():
  return {"km": 0.0, "min": 0.0, "trips": 0}


def _empty_stats():
  return {"today": _empty_bucket(), "week": _empty_bucket(),
          "total": _empty_bucket(), "daily": {}, "date": ""}


def _iso_week(d):
  """Return (iso_year, iso_week) for an ISO-format date string."""
  return date.fromisoformat(d).isocalendar()[:2]


def load_stats():
  try:
    if os.path.exists(STATS_FILE):
      with open(STATS_FILE, "r") as f:
        d = json.load(f)
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


def save_stats(stats):
  try:
    d = os.path.dirname(STATS_FILE)
    if d:
      os.makedirs(d, exist_ok=True)
    tmp = STATS_FILE + ".tmp"
    with open(tmp, "w") as f:
      json.dump(stats, f)
    os.replace(tmp, STATS_FILE)
  except Exception:
    pass


def save_clock_ts():
  """Persist current wall-clock time so it survives power loss."""
  try:
    d = os.path.dirname(CLOCK_TS)
    if d:
      os.makedirs(d, exist_ok=True)
    with open(CLOCK_TS, "w") as f:
      f.write("%.3f" % time.time())
  except Exception:
    pass


def restore_clock_ts():
  """If the system clock is clearly behind our last persisted (sane) time,
  e.g. just booted with no RTC and reset to a bogus default, restore it via
  `date -s`. This process runs as a non-root user without CAP_SYS_TIME, so we
  use NOPASSWD sudo. Fails silently when offline/unset; systemd-timesyncd
  overrides this once NTP syncs."""
  try:
    if not os.path.exists(CLOCK_TS):
      return
    with open(CLOCK_TS) as f:
      saved = float(f.read().strip())
    if saved < SANE_EPOCH:
      return
    now = time.time()
    # Only restore when the clock is clearly behind the record
    if saved > now + 1.0:
      subprocess.run(["sudo", "-n", "/bin/date", "-s", "@%d" % int(saved)],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     check=False)
  except Exception:
    pass


def migrate_legacy():
  """One-time best-effort move of stats/clock files out of the params dir.

  Files sitting in the params dir but not registered in common/params_keys.h get
  unlinked on every boot, so on most devices there is nothing left to migrate;
  this only rescues devices whose params dir was not the active one."""
  try:
    os.makedirs(STATS_DIR, exist_ok=True)
    for name in ("drive_stats.json", "clock.ts"):
      old = os.path.join(LEGACY_DIR, name)
      new = os.path.join(STATS_DIR, name)
      if not os.path.exists(new) and os.path.exists(old):
        with open(old, "rb") as s, open(new, "wb") as d:
          d.write(s.read())
  except Exception:
    pass


def main():
  sm = messaging.SubMaster(['carState'])
  rk = Ratekeeper(RATE, print_delay_threshold=None)

  migrate_legacy()
  stats = load_stats()
  engaged_prev = False
  last_save = 0.0
  last_t = time.monotonic()
  last_date = date.today().isoformat()
  last_week = _iso_week(last_date) if last_date else None
  stats["date"] = last_date

  # Software RTC: bring the clock back from persisted time if it reset
  restore_clock_ts()
  save_clock_ts()  # persist immediately so a fresh boot always has a record

  while True:
    sm.update()
    now = time.monotonic()
    dt = now - last_t
    last_t = now

    if sm.updated['carState']:
      cs = sm['carState']
      engaged = cs.controlsAllowed
      v = cs.vEgo  # m/s
      # distance: only count actual movement while engaged
      if engaged and v > 0.3:
        dist_km = v * dt / 1000.0
        for b in ("today", "week", "total"):
          stats[b]["km"] += dist_km
          stats[b]["min"] += dt / 60.0
      # trip: count once per engagement session (rising edge of controlsAllowed)
      if engaged and not engaged_prev:
        for b in ("today", "week", "total"):
          stats[b]["trips"] += 1
      engaged_prev = engaged

    # midnight roll-over: archive today's km into daily[date], reset today,
    # and reset week on ISO week boundary (Monday)
    today = date.today().isoformat()
    if today != last_date:
      if last_date:
        stats["daily"][last_date] = round(stats["today"]["km"], 2)
        cur_week = _iso_week(today)
        if last_week is not None and cur_week != last_week:
          stats["week"] = _empty_bucket()
        last_week = cur_week
      stats["today"] = _empty_bucket()
      keys = sorted(stats["daily"].keys())
      for k in keys[:-7]:
        stats["daily"].pop(k, None)
      last_date = today
      stats["date"] = today

    if now - last_save > SAVE_INTERVAL:
      save_stats(stats)
      save_clock_ts()
      last_save = now

    rk.keep_time()


if __name__ == "__main__":
  main()
