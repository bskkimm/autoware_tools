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
| `/debug/nc/collision_summary` | `std_msgs/msg/String` | JSON summary for trajectories that contain NC collision events. This is the topic a custom Lichtblick panel should use for the clickable collision list. |
| `/debug/nc/horizon_markers` | `visualization_msgs/msg/MarkerArray` | Combined full-horizon map markers: ego footprints, engaged-object footprints, highlighted overlap outlines, and labels. |

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
/debug/nc/score
/debug/nc/time_to_at_fault_collision_s
/debug/nc/event_count
/debug/nc/at_fault_event_count
/debug/nc/reason
/debug/nc/worst_event/...
/debug/nc/events
/debug/nc/ego_footprints
/debug/nc/object_footprints
/debug/nc/collision_pairs
/debug/nc/front_bumper
/debug/nc/bad_area
/debug/nc/horizon_ego_footprints
/debug/nc/horizon_object_footprints
/debug/nc/horizon_overlap_areas
/debug/nc/horizon_labels
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

`/debug/nc/collision_summary` is published only for evaluated trajectories that contain
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

| Marker namespace inside `/debug/nc/horizon_markers` | Meaning |
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
/debug/nc/collision_summary
```

Panel behavior:

1. Subscribe to `/debug/nc/collision_summary`.
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
/debug/nc/horizon_markers
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
| `/debug/dac/violation_summary` | `std_msgs/msg/String` | JSON summary for trajectories whose DAC score is below `1.0`. This is the topic a custom Lichtblick panel should use for the clickable DAC failure list. |
| `/debug/dac/ego_footprints` | `visualization_msgs/msg/MarkerArray` | Full trajectory-horizon ego footprint outlines. Non-drivable samples are highlighted. |
| `/debug/dac/admissible_road_areas` | `visualization_msgs/msg/MarkerArray` | Road lanelet polygons used by DAC at the first failing timestep. |
| `/debug/dac/admissible_parking_areas` | `visualization_msgs/msg/MarkerArray` | `parking_lot` polygons used by DAC at the first failing timestep. |
| `/debug/dac/failing_corners` | `visualization_msgs/msg/MarkerArray` | Highlighted markers for the ego corners that fell outside the admissible set. |
| `/debug/dac/labels` | `visualization_msgs/msg/MarkerArray` | Human-readable DAC labels such as the first failing `dt` and inside-corner count. |

The official metric topics still carry the scalar metric result:

| Topic | Message type | Purpose |
|---|---|---|
| `/open_loop/metrics/<variant>/drivable_area_compliance` | `std_msgs/msg/Float64` | Final DAC score. |
| `/open_loop/metrics/<variant>/drivable_area_compliance_available` | `std_msgs/msg/Bool` | Metric availability. |
| `/open_loop/metrics/<variant>/drivable_area_compliance_reason` | `std_msgs/msg/String` | Final DAC reason. |

## DAC Summary JSON

`/debug/dac/violation_summary` is published only for evaluated trajectories whose DAC
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
| `/debug/dac/ego_footprints` | The ego footprint at each trajectory sample in the evaluated horizon. |
| `/debug/dac/admissible_road_areas` | The road lanelet polygons used to judge the first failing DAC sample. |
| `/debug/dac/admissible_parking_areas` | The parking-lot polygons used to judge the first failing DAC sample. |
| `/debug/dac/failing_corners` | The ego corners that were outside all admissible road and parking polygons. |
| `/debug/dac/labels` | Human-readable DAC labels. |

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
/debug/dac/violation_summary
```

Panel behavior:

1. Subscribe to `/debug/dac/violation_summary`.
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
/debug/dac/ego_footprints
/debug/dac/admissible_road_areas
/debug/dac/admissible_parking_areas
/debug/dac/failing_corners
/debug/dac/labels
```

Then each click jumps to the planner output time whose selected trajectory first left
the admissible drivable area, and the 3D view shows exactly which corner left which
candidate area set.
trajectory was generated, and the map shows the full 4 s future polygon evidence.

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
