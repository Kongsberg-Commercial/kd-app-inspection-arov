#!/usr/bin/env python3
# gen_srt.py

import csv
from datetime import datetime, timezone, timedelta

INPUT_CSV  = "rov_positions.csv"
OUTPUT_SRT = "telemetry.srt"
LAST_SEG_DUR = 3  # seconds for the last subtitle

def to_offset(ts, base):
    """Convert ISO timestamp to SRT-style offset relative to `base`."""
    dt = datetime.fromisoformat(ts)
    delta = dt - base
    total = delta.total_seconds()
    h = int(total // 3600)
    m = int((total % 3600) // 60)
    s, ms = divmod(total, 60)
    return f"{h:02d}:{m:02d}:{int(s):02d},{int(ms*1000)%1000:03d}"

# Read all rows, determine base time
with open(INPUT_CSV) as f:
    reader = csv.DictReader(f)
    rows = list(reader)

if not rows:
    raise SystemExit("No data in CSV!")

base = datetime.fromisoformat(rows[0]["timestamp_utc"])

with open(OUTPUT_SRT, "w", newline="\n") as o:
    for i, r in enumerate(rows):
        # start
        start = to_offset(r["timestamp_utc"], base)

        # end: next row’s time, or +LAST_SEG_DUR seconds for the last row
        if i + 1 < len(rows):
            end = to_offset(rows[i+1]["timestamp_utc"], base)
        else:
            dt = datetime.fromisoformat(r["timestamp_utc"])
            end_dt = dt + timedelta(seconds=LAST_SEG_DUR)
            end = to_offset(end_dt.isoformat(), base)

        # subtitle text
        lat = float(r["lat"])
        lng = float(r["lng"])
        down = float(r.get("down", 0.0))
        text = f"lat={lat:.6f} lon={lng:.6f} down={down:.2f}"

        # write cue
        o.write(f"{i+1}\n{start} --> {end}\n{text}\n\n")
