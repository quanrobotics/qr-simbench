# Metrics (v0.1)

All metrics are computed from the recorded MCAP via the canonical `NavigationRun` model
(`src/qrsimbench/analysis/`). Times are seconds since the goal was sent.

| Metric | Definition | Notes |
|---|---|---|
| success | Nav2 action result == SUCCEEDED | primary truth |
| time_to_goal | goal sent → terminal result | only when succeeded |
| path_length | Σ hypot(Δx, Δy) over `/odom` positions, steps > 0.5 m dropped | odom is smooth; map-frame poses contain AMCL corrections |
| straight_line_distance | start → goal | |
| path_efficiency | straight_line_distance / path_length, capped at 1 | ignores obstacles; later: planned reference length |
| goal_error | final true position → goal | true position = Gazebo model pose (PosePublisher, `/sim/ground_truth`); falls back to odometry anchored at the spawn pose |
| localization_error_mean / _max | AMCL estimate (odom ∘ map→odom TF) vs Gazebo true pose | P1 metric, informational |
| mean_speed / max_speed | \|linear velocity\| from `/odom` twist during the mission | |
| stopped_time | time with \|v\| < 0.02 m/s and \|ω\| < 0.03 rad/s, after a 2 s warm-up | |
| stop_episodes | contiguous stops ≥ 1 s | |
| min_clearance (LiDAR clearance proxy) | min finite `/scan` range within sensor limits, only while \|v\| > 0.05 m/s | engineering proxy, not a safety metric |
| clearance_p5 | 5th percentile of the above | |
| time_below_threshold | time with clearance below the WARN threshold | |

## Verdict (Engineering Validation Verdict)

- Nav2 result not SUCCEEDED (FAILED / CANCELED / TIMEOUT) → **FAIL**
- `min_clearance ≤ gates.min_clearance.greater_than_m` → **FAIL**
- `min_clearance < gates.min_clearance.warn_below_m` → **WARN**
- stopped_time > 20 s → **WARN** (`PROLONGED_STOP`)
- otherwise → **PASS**

This is not a safety certification and does not predict real-world performance.
