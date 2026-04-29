# NC Debugging in Lichtblick

This note describes the intended debugging workflow for the no-at-fault collision
(NC) metric.

The goal is not to display every NC internal scalar as a separate topic. The useful
debugging view is:

1. jump to planner output times whose 4 s trajectory horizon contains an NC collision,
2. inspect the full future ego/object polygon horizon at that planner output time,
3. visually identify the polygon overlap and the selected NC reason.

## Time Semantics

For each evaluated trajectory, let:

```text
t0 = planner output timestamp
dt = relative time inside the evaluated trajectory horizon
```

If NC finds a collision at `dt = 2.3 s`, the event happened at:

```text
event_time = t0 + 2.3 s
```

However, debug markers are stamped at `t0`, not at `event_time`.

That is intentional. In Lichtblick, pause or seek to the planner output time `t0`.
At that moment, the map should show the full predicted 4 s collision evidence for
the trajectory that was generated at `t0`.

## Debug Topics

The NC debug output is intentionally compact.

| Topic | Message type | Purpose |
|---|---|---|
| `/debug/epdms/nc/collision_summary` | `std_msgs/msg/String` | JSON summary for trajectories that contain NC collision events. This is the topic a custom Lichtblick panel should use for the clickable collision list. |
| `/debug/epdms/nc/horizon_markers` | `visualization_msgs/msg/MarkerArray` | Combined full-horizon map markers: ego footprints, engaged-object footprints, highlighted overlap outlines, and labels. |

The official metric topics still carry the scalar metric result:

| Topic | Message type | Purpose |
|---|---|---|
| `/open_loop/metrics/<variant>/no_at_fault_collision` | `std_msgs/msg/Float64` | Final NC score. |
| `/open_loop/metrics/<variant>/time_to_at_fault_collision_s` | `std_msgs/msg/Float64` | Relative time `dt` of the selected worst at-fault event. |
| `/open_loop/metrics/<variant>/no_at_fault_collision_available` | `std_msgs/msg/Bool` | Metric availability. |
| `/open_loop/metrics/<variant>/no_at_fault_collision_reason` | `std_msgs/msg/String` | Final NC reason. |

## Removed Debug Topics

The earlier debug design had many separate topics such as:

```text
/debug/epdms/nc/score
/debug/epdms/nc/time_to_at_fault_collision_s
/debug/epdms/nc/event_count
/debug/epdms/nc/at_fault_event_count
/debug/epdms/nc/reason
/debug/epdms/nc/worst_event/...
/debug/epdms/nc/events
/debug/epdms/nc/ego_footprints
/debug/epdms/nc/object_footprints
/debug/epdms/nc/collision_pairs
/debug/epdms/nc/front_bumper
/debug/epdms/nc/bad_area
/debug/epdms/nc/horizon_ego_footprints
/debug/epdms/nc/horizon_object_footprints
/debug/epdms/nc/horizon_overlap_areas
/debug/epdms/nc/horizon_labels
```

Those are no longer the preferred output.

Reason:

- score, reason, and time already exist as official metric topics,
- separate worst-event scalar topics are hard to use in the map panel,
- event-only polygons are less useful than the full 4 s horizon,
- object footprints can be heavy if published for every object.

The new design keeps only:

```text
collision_summary + one combined full-horizon MarkerArray
```

The full-horizon markers are combined into one topic so a single `DELETEALL`
marker clears the previous NC visualization and then all new ego/object/overlap/label
markers are added in the same message. This avoids separate MarkerArray topics
deleting each other at the same timestamp.

## Collision Summary JSON

`/debug/epdms/nc/collision_summary` is published only for evaluated trajectories that contain
at least one NC collision event.

Example:

```json
{
  "trajectory_stamp_sec": 1776838192.42,
  "score": 0.0,
  "reason": "at_fault_collision_with_agent",
  "worst_time_s": 2.3,
  "worst_event_stamp_sec": 1776838194.72,
  "event_count": 4,
  "at_fault_event_count": 1,
  "worst_object_id": "abc123",
  "worst_object_label": "CAR",
  "worst_collision_type": "ACTIVE_FRONT",
  "objects": [
    {
      "object_id": "abc123",
      "label": "CAR",
      "collision_type": "ACTIVE_FRONT",
      "first_collision_time_s": 2.3
    }
  ],
  "events": [
    {
      "trajectory_stamp_sec": 1776838192.42,
      "event_stamp_sec": 1776838194.72,
      "time_s": 2.3,
      "object_id": "abc123",
      "object_label": "CAR",
      "collision_type": "ACTIVE_FRONT",
      "reason": "at_fault_collision_with_agent",
      "agent": true,
      "at_fault": true,
      "score": 0.0,
      "ego_stopped": false,
      "track_stopped": false,
      "behind": false,
      "front_hit": true,
      "multiple_lanes": false,
      "non_drivable_area": false
    }
  ]
}
```

The Lichtblick plugin should use `trajectory_stamp_sec` for click-to-seek:

```ts
context.seekPlayback?.(trajectory_stamp_sec);
```

## MarkerArray Display

All NC debug markers use:

```text
header.frame_id = "map"
header.stamp = t0
```

The marker timestamp is the planner output time, not the future collision time.

Recommended interpretation:

| Marker namespace inside `/debug/epdms/nc/horizon_markers` | Meaning |
|---|---|
| `nc_horizon_ego_footprints` | The ego footprint at each trajectory sample in the 4 s horizon. |
| `nc_horizon_object_footprints` | The engaged object's logged footprint at each matching future sample. |
| `nc_horizon_overlap_areas` | The intersection polygon between ego and object footprints at overlap samples. |
| `nc_horizon_labels` | Human-readable event labels. |

Suggested visual semantics:

| Case | Color |
|---|---|
| ego horizon, no overlap | transparent cyan |
| object horizon, no overlap | transparent orange |
| overlap but not at-fault | thick yellow/orange outline |
| at-fault overlap | thick red/magenta outline |
| selected/worst event label | text marker near ego footprint |

## Lichtblick Workflow

The Lichtblick plugin provides a panel named:

```text
NC Collision Timeline
```

It subscribes to:

```text
/debug/epdms/nc/collision_summary
```

Panel behavior:

1. Subscribe to `/debug/epdms/nc/collision_summary`.
2. Build a table of collision-producing trajectory timestamps.
3. Show columns:

```text
t0 | score | reason | worst dt | object label | collision type | event count
```

4. On row click, call:

```ts
context.seekPlayback?.(trajectory_stamp_sec);
```

5. Use the `Previous` / `Next` buttons to step through collision-producing
   planner timestamps.
6. Use the score and reason filters to reduce the list.
7. In the 3D map panel, enable:

```text
/debug/epdms/nc/horizon_markers
```

Then each click jumps to the planner output time where that collision-producing
trajectory was generated, and the 3D view shows the full future collision evidence
for that `t0`.

# DAC Debugging in Lichtblick

This note describes the intended debugging workflow for the drivable-area compliance
(DAC) metric.

The useful DAC view is:

1. jump to planner output times whose evaluated trajectory scored `DAC = 0`,
2. inspect the full future ego-footprint horizon generated at that planner output time,
3. inspect the admissible road and parking polygons used for the first failing timestep,
4. identify which ego corner left the admissible area and when that happened.

## Time Semantics

For each evaluated trajectory, let:

```text
t0 = planner output timestamp
dt = relative time inside the evaluated trajectory horizon
```

If DAC first fails at `dt = 1.8 s`, the first failing sample happened at:

```text
failure_time = t0 + 1.8 s
```

Like NC, DAC debug markers are stamped at `t0`, not at `failure_time`.

That is intentional. In Lichtblick, pause or seek to `t0` and inspect the horizon that
was evaluated for the planner output generated at that timestamp.

## Debug Topics

The DAC debug output is intentionally split into a compact summary topic plus a few
3D marker topics:

| Topic | Message type | Purpose |
|---|---|---|
| `/debug/epdms/dac/violation_summary` | `std_msgs/msg/String` | JSON summary for trajectories whose DAC score is below `1.0`. This is the topic a custom Lichtblick panel should use for the clickable DAC failure list. |
| `/debug/epdms/dac/ego_footprints` | `visualization_msgs/msg/MarkerArray` | Full trajectory-horizon ego footprint outlines. Non-drivable samples are highlighted. |
| `/debug/epdms/dac/admissible_road_areas` | `visualization_msgs/msg/MarkerArray` | Road lanelet polygons used by DAC at the first failing timestep. |
| `/debug/epdms/dac/admissible_parking_areas` | `visualization_msgs/msg/MarkerArray` | `parking_lot` polygons used by DAC at the first failing timestep. |
| `/debug/epdms/dac/failing_corners` | `visualization_msgs/msg/MarkerArray` | Highlighted markers for the ego corners that fell outside the admissible set. |
| `/debug/epdms/dac/labels` | `visualization_msgs/msg/MarkerArray` | Human-readable DAC labels such as the first failing `dt` and inside-corner count. |

The official metric topics still carry the scalar metric result:

| Topic | Message type | Purpose |
|---|---|---|
| `/open_loop/metrics/<variant>/drivable_area_compliance` | `std_msgs/msg/Float64` | Final DAC score. |
| `/open_loop/metrics/<variant>/drivable_area_compliance_available` | `std_msgs/msg/Bool` | Metric availability. |
| `/open_loop/metrics/<variant>/drivable_area_compliance_reason` | `std_msgs/msg/String` | Final DAC reason. |

## DAC Summary JSON

`/debug/epdms/dac/violation_summary` is published only for evaluated trajectories whose DAC
score is below `1.0`.

Example:

```json
{
  "trajectory_stamp_sec": 1776838192.42,
  "score": 0.0,
  "reason": "non_compliant_corner_outside_drivable_area",
  "first_failure_time_s": 1.8,
  "failure_stamp_sec": 1776838194.22,
  "failing_corner_indices": [1, 2],
  "corner_count_inside": 2,
  "route_candidate_count": 4,
  "road_candidate_count": 6,
  "parking_candidate_count": 0
}
```

The Lichtblick plugin should use `trajectory_stamp_sec` for click-to-seek:

```ts
context.seekPlayback?.(trajectory_stamp_sec + 0.005);
```

## MarkerArray Display

All DAC debug markers use:

```text
header.frame_id = "map"
header.stamp = t0
```

Recommended interpretation:

| Topic | Meaning |
|---|---|
| `/debug/epdms/dac/ego_footprints` | The ego footprint at each trajectory sample in the evaluated horizon. |
| `/debug/epdms/dac/admissible_road_areas` | The road lanelet polygons used to judge the first failing DAC sample. |
| `/debug/epdms/dac/admissible_parking_areas` | The parking-lot polygons used to judge the first failing DAC sample. |
| `/debug/epdms/dac/failing_corners` | The ego corners that were outside all admissible road and parking polygons. |
| `/debug/epdms/dac/labels` | Human-readable DAC labels. |

Suggested visual semantics:

| Case | Color |
|---|---|
| ego horizon, drivable | transparent cyan |
| ego horizon, non-drivable | orange |
| admissible road polygons | cyan/blue |
| admissible parking polygons | green |
| failing corners | magenta |
| DAC label | red text |

## Lichtblick Workflow

The Lichtblick plugin provides a panel named:

```text
DAC Violation Timeline
```

It subscribes to:

```text
/debug/epdms/dac/violation_summary
```

Panel behavior:

1. Subscribe to `/debug/epdms/dac/violation_summary`.
2. Build a table of DAC-failing trajectory timestamps.
3. Show columns:

```text
t0 | score | first failing dt | reason | failing corners | inside corners | road/route/parking counts
```

4. On row click, call:

```ts
context.seekPlayback?.(trajectory_stamp_sec + 0.005);
```

5. Use the `Previous` / `Next` buttons to step through DAC-failing planner timestamps.
6. In the 3D map panel, enable:

```text
/debug/epdms/dac/ego_footprints
/debug/epdms/dac/admissible_road_areas
/debug/epdms/dac/admissible_parking_areas
/debug/epdms/dac/failing_corners
/debug/epdms/dac/labels
```

Then each click jumps to the planner output time whose selected trajectory first left
the admissible drivable area, and the 3D view shows exactly which corner left which
candidate area set.
trajectory was generated, and the map shows the full 4 s future polygon evidence.

# DDC Debugging

## Intent

DDC debugging should answer:

1. Which evaluated trajectory had non-perfect DDC?
2. Which local route-lane polygons were considered on-route near ego center?
3. Which local intersection polygons suppressed wrong-way accumulation?
4. Which centerline segments actually counted toward the 1.0 s wrong-way window?

## Debug Topics

| Topic | Message type | Purpose |
|---|---|---|
| `/debug/epdms/ddc/violation_summary` | `std_msgs/msg/String` | JSON summary for trajectories whose DDC score is below `1.0`. This is the topic a custom Lichtblick panel should use for the clickable DDC violation list. |
| `/debug/epdms/ddc/ego_centers` | `visualization_msgs/msg/MarkerArray` | Full trajectory-horizon ego-center polyline for the evaluated rollout. |
| `/debug/epdms/ddc/oncoming_segments` | `visualization_msgs/msg/MarkerArray` | The ego-center segments whose progress counted toward wrong-way accumulation. |
| `/debug/epdms/ddc/route_lane_polygons` | `visualization_msgs/msg/MarkerArray` | Nearby route-consistent admissible lane polygons used by the DDC center-point oncoming check inside the worst 1.0 s window. This historical topic name now also covers same-direction neighboring lanes and adjacent shoulders when they are part of the local admissible corridor. The published outlines include the current DDC soft admissible lane margin and are emitted per-sample within the worst window instead of being deduplicated to unique lane IDs. |
| `/debug/epdms/ddc/intersection_lane_polygons` | `visualization_msgs/msg/MarkerArray` | Nearby `intersection_area` polygons used by the DDC intersection leniency check inside the worst 1.0 s window. The topic name is historical, but the markers now represent map intersection-area polygons rather than only turn-direction-tagged lanelets. |
| `/debug/epdms/ddc/labels` | `visualization_msgs/msg/MarkerArray` | Human-readable DDC labels such as score, max wrong-way progress, and worst-window bounds. |

The official metric topics still carry the scalar result:

| Topic | Message type | Purpose |
|---|---|---|
| `/open_loop/metrics/<variant>/driving_direction_compliance` | `std_msgs/msg/Float64` | Final DDC score. |
| `/open_loop/metrics/<variant>/max_oncoming_progress_m` | `std_msgs/msg/Float64` | Maximum accumulated wrong-way progress over the worst 1.0 s window. |
| `/open_loop/metrics/<variant>/driving_direction_compliance_available` | `std_msgs/msg/Bool` | Metric availability. |
| `/open_loop/metrics/<variant>/driving_direction_compliance_reason` | `std_msgs/msg/String` | Final DDC reason. |

## DDC Summary JSON

`/debug/epdms/ddc/violation_summary` is published only for evaluated trajectories whose DDC
score is below `1.0`.

Example:

```json
{
  "trajectory_stamp_sec": 1776838192.42,
  "score": 0.5,
  "reason": "minor_oncoming_progress",
  "max_oncoming_progress_m": 3.4,
  "worst_window_start_s": 1.2,
  "worst_window_end_s": 2.0,
  "window_progress_m": 3.4,
  "sample_count": 9
}
```

The Lichtblick plugin should use `trajectory_stamp_sec` for click-to-seek:

```ts
context.seekPlayback?.(trajectory_stamp_sec + 0.005);
```

## MarkerArray Display

All DDC debug markers use:

```text
header.frame_id = "map"
header.stamp = t0
```

Recommended interpretation:

| Topic | Meaning |
|---|---|
| `/debug/epdms/ddc/ego_centers` | Ego-center path over the full evaluated horizon. |
| `/debug/epdms/ddc/oncoming_segments` | Only the centerline segments that contributed to wrong-way accumulation because `Oncoming && !Intersection` held there. |
| `/debug/epdms/ddc/route_lane_polygons` | Nearby route-consistent admissible lane polygons that counted as not-oncoming in the worst 1.0 s window. This can include same-direction neighboring lanes and adjacent shoulders. The outlines include the current DDC soft lane margin and are emitted for each sample in the worst window so the local boundary under the current ego position remains visible. |
| `/debug/epdms/ddc/intersection_lane_polygons` | Nearby `intersection_area` polygons that granted intersection leniency in the worst 1.0 s window. |
| `/debug/epdms/ddc/labels` | Human-readable DDC summary label. |

Suggested visual semantics:

| Case | Color |
|---|---|
| ego-center horizon | transparent cyan |
| counted oncoming segments | orange |
| nearby route-lane polygons | cyan/blue |
| nearby intersection polygons | green |
| DDC label | red text |

## Lichtblick Workflow

The Lichtblick plugin should provide a panel named:

```text
DDC Violation Timeline
```

It should subscribe to:

```text
/debug/epdms/ddc/violation_summary
```

Panel behavior:

1. Subscribe to `/debug/epdms/ddc/violation_summary`.
2. Build a table of DDC-non-perfect trajectory timestamps.
3. Show columns:

```text
t0 | score | max wrong-way progress | window start | window end | reason | sample count
```

4. On row click, call:

```ts
context.seekPlayback?.(trajectory_stamp_sec + 0.005);
```

5. In the 3D map panel, enable:

```text
/debug/epdms/ddc/ego_centers
/debug/epdms/ddc/oncoming_segments
/debug/epdms/ddc/route_lane_polygons
/debug/epdms/ddc/intersection_lane_polygons
/debug/epdms/ddc/labels
```

Then each click jumps to the planner output time and shows:

- the ego-center path over the evaluated horizon
- the local route-consistent admissible lane polygons, including the current soft lane margin, used to decide whether ego center was oncoming
- the local `intersection_area` polygons used to suppress accumulation
- the exact centerline segments that counted toward the worst 1.0 s wrong-way window

## TLC Debugging

TLC debug output is intended to answer:

- which trajectory first crossed a selected stop line while that movement was stop/red
- which stop line and regulatory element were used for the decision
- which movement and selected route lanelets were used to choose that stop line

### Topics

When TLC is enabled and a trajectory scores below `1.0`, the analyzer writes:

```text
/debug/epdms/tlc/violation_summary
/debug/epdms/tlc/ego_footprints
/debug/epdms/tlc/stop_lines
/debug/epdms/tlc/labels
```

### Summary Payload

`/debug/epdms/tlc/violation_summary` is a `std_msgs/msg/String` JSON message.

Expected shape:

```json
{
  "trajectory_stamp_sec": 1776838192.42,
  "score": 0.0,
  "reason": "red_light_stop_line_crossed",
  "first_failure_time_s": 1.6,
  "failure_stamp_sec": 1776838194.02,
  "intended_movement": "right",
  "regulatory_element_ids": [12034],
  "selected_lane_ids": [54021],
  "stop_line_ids": [9102],
  "selected_stop_line_count": 1
}
```

The Lichtblick plugin should use `trajectory_stamp_sec` for click-to-seek:

```ts
context.seekPlayback?.(trajectory_stamp_sec + 0.005);
```

### MarkerArray Display

All TLC debug markers use:

```text
header.frame_id = "map"
header.stamp = t0
```

Recommended interpretation:

| Topic | Meaning |
|---|---|
| `/debug/epdms/tlc/ego_footprints` | Ego footprint path over the full evaluated horizon. The failing sample is highlighted more strongly. |
| `/debug/epdms/tlc/stop_lines` | Stop line belonging to the selected movement-compatible traffic-light regulatory element. This is the scoring primitive. |
| `/debug/epdms/tlc/labels` | Human-readable TLC summary label. |

Suggested visual semantics:

| Case | Color |
|---|---|
| ego-footprint horizon | transparent cyan |
| failing ego footprint | orange |
| stop line | yellow/orange |
| TLC label | red text |

### Lichtblick Workflow

The Lichtblick plugin should provide a panel named:

```text
TLC Violation Timeline
```

It should subscribe to:

```text
/debug/epdms/tlc/violation_summary
```

Panel behavior:

1. Subscribe to `/debug/epdms/tlc/violation_summary`.
2. Build a table of TLC-non-perfect trajectory timestamps.
3. Show columns:

```text
t0 | score | dt | reason | movement | stop lines | selected lanes | regulatory elements
```

4. On row click, call:

```ts
context.seekPlayback?.(trajectory_stamp_sec + 0.005);
```

5. In the 3D map panel, enable:

```text
/debug/epdms/tlc/ego_footprints
/debug/epdms/tlc/stop_lines
/debug/epdms/tlc/labels
```

Then each click shows:

- the ego footprint horizon
- the selected stop line actually used by TLC
- the intended movement and selected lane IDs in the summary payload
- the exact first stop-line crossing sample that caused the violation

### Signal Association Note

The current stop-line TLC first tries to match a live signal group by the traffic-light
regulatory-element id. If that fails, it falls back to the selected movement-compatible
route lanelet ids, and then to the broader route lanelet ids under the same regulatory
element. A missing signal group now becomes `unavailable_missing_signal_group` only when
the ego actually reaches that selected stop line and the signal is required to judge
compliance.

## TTC Debugging

TTC debug output is intended to answer:

- which trajectory first produced a TTC failure
- which logged object and future offset caused it
- whether the failure came from `Ahead` directly or from the
  `BadOrIntersection && !Behind` branch

### Topics

When TTC is enabled and a trajectory scores below `1.0`, the analyzer writes:

```text
/debug/epdms/ttc/violation_summary
/debug/epdms/ttc/ego_footprints
/debug/epdms/ttc/object_footprints
/debug/epdms/ttc/overlap_areas
/debug/epdms/ttc/labels
```

### Summary Payload

`/debug/epdms/ttc/violation_summary` is a `std_msgs/msg/String` JSON message.

Expected shape:

```json
{
  "trajectory_stamp_sec": 1776838192.42,
  "score": 0.0,
  "reason": "collision_within_bound",
  "first_failure_time_s": 1.2,
  "failure_stamp_sec": 1776838193.62,
  "future_offset_s": 0.6,
  "object_id": "abc123",
  "object_label": "CAR",
  "ahead": false,
  "behind": false,
  "multiple_lanes": false,
  "non_drivable_area": true,
  "intersection": false,
  "bad_or_intersection": true
}
```

The Lichtblick plugin should use `trajectory_stamp_sec` for click-to-seek:

```ts
context.seekPlayback?.(trajectory_stamp_sec + 0.005);
```

### MarkerArray Display

All TTC debug markers use:

```text
header.frame_id = "map"
header.stamp = t0
```

Recommended interpretation:

| Topic | Meaning |
|---|---|
| `/debug/epdms/ttc/ego_footprints` | The full trajectory prefix up to the TTC base sample is shown first in dim cyan, then the selected TTC sample's projected ego footprints at checked offsets `0.0`, `0.3`, `0.6`, and `0.9 s` are drawn on top. Overlap offsets are highlighted more strongly. These polygons are anchored to the ego road-surface z and then lifted only by a small marker offset. |
| `/debug/epdms/ttc/object_footprints` | The same object's matched prefix footprints are shown in dim orange up to the TTC base sample, then the same logged object's queried future footprints at the checked TTC offsets are drawn on top. Overlap offsets are highlighted more strongly. These polygons use the same local road-surface z as the paired ego footprint instead of the object's 3D center height. |
| `/debug/epdms/ttc/overlap_areas` | The overlap polygon(s) at the checked offsets that actually overlap. The selected failing offset is included in this set and is anchored to the same local road-surface z as the paired TTC footprint markers. |
| `/debug/epdms/ttc/labels` | Human-readable TTC summary label. |

Suggested visual semantics:

| Case | Color |
|---|---|
| TTC ego prefix before TTC base sample | pale cyan |
| TTC ego checked TTC offsets, no overlap | deep blue |
| TTC ego footprint, overlap offset | orange |
| TTC object prefix before TTC base sample | pale orange |
| TTC object checked TTC offsets, no overlap | strong amber |
| TTC object footprint, overlap offset | yellow/orange |
| TTC overlap | magenta |
| TTC label | red text |

### Lichtblick Workflow

The Lichtblick plugin should provide a panel named:

```text
TTC Violation Timeline
```

It should subscribe to:

```text
/debug/epdms/ttc/violation_summary
```

Panel behavior:

1. Subscribe to `/debug/epdms/ttc/violation_summary`.
2. Build a table of TTC-producing trajectory timestamps.
3. Show columns:

```text
t0 | score | t_fail | delta | condition | reason | object label | ahead | bad/intersection
```

4. On row click, call:

```ts
context.seekPlayback?.(trajectory_stamp_sec + 0.005);
```

5. In the 3D map panel, enable:

```text
/debug/epdms/ttc/ego_footprints
/debug/epdms/ttc/object_footprints
/debug/epdms/ttc/overlap_areas
/debug/epdms/ttc/labels
```

## LK Debugging

LK debug output is intended to answer:

- where the selected 4 s center path stayed normal
- where intersection relaxation suppressed a would-be lane-keeping penalty
- which continuous over-threshold run actually caused `LK=0`
- which reference centerlines were used for the deviation check

### Topics

When LK is enabled and a trajectory scores below `1.0`, the analyzer writes:

```text
/debug/epdms/lk/violation_summary
/debug/epdms/lk/ego_center_path
/debug/epdms/lk/reference_centerlines
/debug/epdms/lk/labels
```

### Summary Payload

`/debug/epdms/lk/violation_summary` is a `std_msgs/msg/String` JSON message.

Expected shape:

```json
{
  "trajectory_stamp_sec": 1776838192.42,
  "score": 0.0,
  "reason": "available",
  "first_failure_time_s": 2.6,
  "failure_run_start_s": 0.5,
  "failure_run_end_s": 2.6,
  "max_continuous_violation_time_s": 2.1,
  "peak_abs_lateral_deviation_m": 0.9,
  "sample_count": 41
}
```

The Lichtblick plugin should use `trajectory_stamp_sec` for click-to-seek:

```ts
context.seekPlayback?.(trajectory_stamp_sec + 0.005);
```

### MarkerArray Display

All LK debug markers use:

```text
header.frame_id = "map"
header.stamp = t0
```

Recommended interpretation:

| Topic | Meaning |
|---|---|
| `/debug/epdms/lk/ego_center_path` | Full 4 s ego-center horizon, emitted as state-colored line segments. Normal segments are cyan, intersection-relaxed segments are green, ordinary over-threshold non-intersection segments are orange, and the failure-causing continuous run is red. |
| `/debug/epdms/lk/reference_centerlines` | The unique reference lanelet centerlines actually used for LK deviation measurement over the failing horizon. |
| `/debug/epdms/lk/labels` | Human-readable LK summary label (`LK`, max run, peak deviation). |

### Lichtblick Workflow

The Lichtblick plugin should provide a panel named:

```text
LK Violation Timeline
```

It should subscribe to:

```text
/debug/epdms/lk/violation_summary
```

Panel behavior:

1. Subscribe to `/debug/epdms/lk/violation_summary`.
2. Build a table of LK-failing trajectory timestamps.
3. Show columns:

```text
t0 | score | t_fail | run_start | run_end | max_run | peak_dev | reason
```

4. On row click, call:

```ts
context.seekPlayback?.(trajectory_stamp_sec + 0.005);
```

5. In the 3D map panel, enable:

```text
/debug/epdms/lk/ego_center_path
/debug/epdms/lk/reference_centerlines
/debug/epdms/lk/labels
```

This TTC horizon is not the full 4-second trajectory horizon. It is the local TTC
check horizon for the selected failing trajectory sample:

```text
delta = 0.0, 0.3, 0.6, 0.9 s
```

So the ego/object footprint topics should appear as:

1. a dim full prefix from trajectory start up to the TTC base sample, then
2. the short four-step TTC local horizon for that sampled state, with
3. the overlapping offset highlighted.

## 4s Trajectory Horizon Debugging

To inspect the exact evaluated 4-second horizon in 3D, the analyzer also writes:

```text
/debug/epdms/trajectory/planned_horizon_4s
/debug/epdms/trajectory/gt_horizon_4s
```

Both are `visualization_msgs/msg/MarkerArray` topics rendered as filled footprint polygons
generated from the exact ego footprint at each trajectory sample using `VehicleInfo`.
This is intended to show the occupied vehicle footprint over the 4-second horizon rather
than a thin centerline or empty outline rectangles.

Recommended interpretation:

| Topic | Meaning |
|---|---|
| `/debug/epdms/trajectory/planned_horizon_4s` | Evaluated planner trajectory truncated to 4.0 s and drawn as exact ego-footprint rectangles at each sample. |
| `/debug/epdms/trajectory/gt_horizon_4s` | Evaluated GT trajectory truncated to 4.0 s and drawn as exact ego-footprint rectangles at each sample. |

Suggested visual semantics:

| Case | Color |
|---|---|
| planned 4 s horizon | orange |
| GT 4 s horizon | cyan |

## 2D Camera Overlay

The first implementation should use the 3D map.

2D camera overlay is possible, but it requires projecting map-frame polygons into
image coordinates using TF and camera intrinsics. That should be a separate plugin
feature after the 3D workflow is validated.

## NC-Only Analyzer Run

For NC-focused debugging, calculate only NC:

```bash
ros2 run autoware_planning_data_analyzer autoware_planning_data_analyzer_node --ros-args \
  -p input_bag_path:=/path/to/eval_bag \
  -p output_dir:=/path/to/eval_json \
  -p trajectory_topic:=/diffusion_planner/output/trajectory \
  -p open_loop.metric_variant:=raw \
  -p open_loop.gt_source_mode:=gt_trajectory \
  -p open_loop.gt_trajectory_topic:=/diffusion_planner/output/gt_trajectory \
  -p open_loop.trajectory_evaluation_horizon:=4.0 \
  -p open_loop.enabled_metrics:="['nc']"
```

This keeps metric computation and debug output focused on NC and avoids producing
heavy topics for unrelated subscores.
