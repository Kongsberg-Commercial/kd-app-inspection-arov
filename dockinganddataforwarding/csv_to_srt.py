#!/usr/bin/env python3
# gen_srt.py

import csv
from datetime import datetime, timedelta
import os

INPUT_CSV  = "rov_positions.csv"
OUTPUT_SRT = "telemetry.srt"
LAST_SEG_DUR = 3  # seconds for last subtitle

def parse_iso_timestamp(iso_str):
    """Ensure parsing works across platforms by removing 'Z' or '+00:00'."""
    return datetime.fromisoformat(iso_str.replace("Z", "").replace("+00:00", ""))

def to_offset(ts, base):
    """Convert timestamp to SRT offset relative to base."""
    delta = ts - base
    total = delta.total_seconds()
    h = int(total // 3600)
    m = int((total % 3600) // 60)
    s, ms = divmod(total, 60)
    return f"{h:02d}:{m:02d}:{int(s):02d},{int(ms * 1000):03d}"

# Load rows from CSV
with open(INPUT_CSV, newline='', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    rows = list(reader)

if not rows:
    raise SystemExit("ERROR: No data in CSV.")

# Parse all timestamps ahead of time
for row in rows:
    row["dt"] = parse_iso_timestamp(row["timestamp_utc"])

base = rows[0]["dt"]

# Write SRT file
with open(OUTPUT_SRT, "w", encoding="utf-8", newline="\n") as out:
    for i, r in enumerate(rows):
        start = to_offset(r["dt"], base)

        if i + 1 < len(rows):
            end = to_offset(rows[i + 1]["dt"], base)
        else:
            end = to_offset(r["dt"] + timedelta(seconds=LAST_SEG_DUR), base)

        lat = float(r["lat"])
        lng = float(r["lng"])
        down = float(r["down"])
        text = f"lat={lat:.6f} lon={lng:.6f} down={down:.2f}"

        out.write(f"{i+1}\n{start} --> {end}\n{text}\n\n")

print(f"✅ SRT generated: {OUTPUT_SRT}")
