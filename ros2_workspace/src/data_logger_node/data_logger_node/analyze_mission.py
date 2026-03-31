#!/usr/bin/env python3
"""Post-mission analysis for detection counts and odometry drift proxy."""

from __future__ import annotations

import argparse
import math
import sqlite3


def analyze(db_path: str) -> str:
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    cur.execute("SELECT COUNT(*) FROM detections")
    detection_count = cur.fetchone()[0]

    cur.execute("SELECT AVG(confidence) FROM detections")
    avg_conf = cur.fetchone()[0] or 0.0

    cur.execute("SELECT x, y FROM odometry ORDER BY ts")
    points = cur.fetchall()
    drift = 0.0
    if len(points) > 1:
        start = points[0]
        end = points[-1]
        drift = math.hypot(end[0] - start[0], end[1] - start[1])

    conn.close()

    return (
        f"Detections: {detection_count}\n"
        f"Average confidence: {avg_conf:.3f}\n"
        f"Odometry endpoint drift: {drift:.3f} m\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("db", help="Path to mission_events.db")
    args = parser.parse_args()
    print(analyze(args.db))


if __name__ == "__main__":
    main()
