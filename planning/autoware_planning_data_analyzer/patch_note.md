# NAVSIM vs Autoware Unmatched Implementation Notes

This file records only mismatches we explicitly confirm together while reviewing
`implementation_report.md`, NAVSIM, and the migrated Autoware implementation.

New findings should be appended gradually as they are confirmed.

## 2026-04-24 - DAC May Be Too Strict Near Road Border

### What We Confirmed

The current migrated DAC uses route-designated lanelets, nearby road lanelets, pose-
intersecting road lanelets, and parking-lot polygons as the admissible area for the
corner-based drivable-area check.

This is already broader than the older route-lanelet-only proxy, but it can still be
too strict near road-border space when the physically drivable paved area extends a bit
beyond the lane polygon boundary.

In those cases, a trajectory can receive:

```text
DAC = 0
```

even though the tire is only slightly outside the lane polygon and still on practical
road surface near the border.

### Why This Matters

Lane-boundary-only or lane-polygon-dominant admissible area can over-penalize:

```text
slight tire ride on road-border-adjacent paved area
```

when that area is still reasonably drivable but is not represented inside the selected
lane polygons.

So the mismatch is:

```text
lane polygon boundary
vs
practical drivable road surface near the border
```

### Current Judgement

This does **not** automatically mean DAC should accept arbitrary space outside lanes.
The better direction is:

```text
include genuine map-supported drivable border area
do not include clearly non-drivable curb / sidewalk / median / exclusion space
```

So the likely future refinement is:

```text
lane + road-border-adjacent drivable area
```

rather than:

```text
lane only
```

or

```text
everything near the lane boundary
```

### Potential Fix Direction

When refining DAC, evaluate whether the admissible region should additionally include:

- shoulder lanelets that are genuinely drivable in the target semantics
- explicit extra drivable-area polygons from the map, if available
- another map-backed road-border representation that captures paved drivable margin

Avoid adding a broad geometric buffer unless the map semantics are unavailable and the
heuristic is explicitly documented.

## 2026-04-21 - NC/TTC Object Source Mismatch

### What We Confirmed

For NAVSIM non-reactive evaluation, other objects do **not** come from an
Autoware-style prediction message with `predicted_paths`.

NAVSIM uses scenario future tracked objects:

```text
object state at T
object state at T + 0.5
object state at T + 1.0
...
```

Then NAVSIM interpolates those tracked-object states to the scorer grid:

```text
T, T + 0.1, T + 0.2, ..., T + 4.0
```

So for NAVSIM:

```text
ego = submitted/planned/simulated ego trajectory
other objects = scenario future tracked objects, interpolated over time
```

For the migrated Autoware implementation, NC/TTC use:

```text
/perception/object_recognition/objects
autoware_perception_msgs/msg/PredictedObjects
```

At one evaluation timestamp, each object has:

```text
initial_pose = current object pose at that message time
predicted_paths = predicted future paths from that message time
```

The migrated code uses:

```text
t = 0:
  object initial_pose

t > 0:
  highest-confidence predicted_path, interpolated to query time
```

### Why This Is Unmatched

The two implementations both produce an object footprint at future horizon time `t`,
but the source is different:

```text
NAVSIM O_{t,o}
  = scenario/recorded/interpolated object footprint at horizon time t

Autoware O^{aw}_{t,o}
  = perception-predicted object footprint from the current PredictedObjects message
```

So Autoware's object future is prediction-based, while NAVSIM's non-reactive object
future is scenario/recorded-track based.

### Evidence We Checked

NAVSIM:

- `metric_cache_processor.py::_interpolate_gt_observation()`
  - samples scenario tracked objects with `scenario.get_tracked_objects_at_iteration(...)`
  - interpolates them to the proposal sampling interval
- `metric_cache_processor.py::_build_pdm_observation()`
  - builds `PDMObservation` from those interpolated detection tracks
- `pdm_scorer.py::_calculate_no_at_fault_collision()`
  - queries `self._observation[time_idx]`

Autoware:

- `src/metrics/no_at_fault_collision.cpp::build_object_caches()`
  - selects `highest_confidence_path(object)`
- `src/metrics/no_at_fault_collision.cpp::interpolate_object_state()`
  - uses `initial_pose` for `query_time_s <= 0`
  - uses predicted-path interpolation for `query_time_s > 0`
- `src/metrics/ttc_within_bound.cpp`
  - uses the same predicted-object cache pattern

Rosbag:

- Bag path:
  - `/home/beomseokkim2/rosbag/x2_takanawa/input_bag`
- Topic:
  - `/perception/object_recognition/objects`
- Type:
  - `autoware_perception_msgs/msg/PredictedObjects`
- Count:
  - `9054` messages

### Report Update Needed

In `implementation_report.md`, NC/TTC object-source wording should distinguish:

```text
NAVSIM:
  future tracked objects from the scenario, interpolated to the scorer grid

Autoware:
  current PredictedObjects message plus predicted_paths
```

It should not imply that Autoware `PredictedObjects.predicted_paths` are semantically
equivalent to NAVSIM non-reactive future object tracks.

## 2026-04-21 - NC Collided-Track Memory Missing

### What We Confirmed

NAVSIM NC keeps memory of objects that already collided with the ego. After an
object has already overlapped the ego, later overlaps with the same object are not
treated as fresh collision candidates again.

Conceptually:

```text
CollidedTracks_i = empty set

for each horizon time t:
  for each object o:
    if ego/object overlap is detected:
      if o is already in CollidedTracks_i:
        skip this overlap

      add o to CollidedTracks_i
      classify the collision
```

This prevents one sustained physical overlap from being re-counted or reclassified
across later timesteps.

The migrated Autoware NC does not keep equivalent collided-object memory. It checks
each trajectory time against each object state and returns immediately on the first
penalized collision, but non-penalized overlaps are not remembered.

### Why This Is Unmatched

For many cases, the missing memory does not affect the final Autoware score because
the code exits as soon as it finds a penalized collision.

The semantic difference can matter when the first overlap with an object is not
penalized, but a later continued overlap with the same object is reclassified as a
penalized collision.

Example:

```text
t = 1.0:
  ego overlaps object o
  collision type = STOPPED_EGO or ACTIVE_REAR
  no NC penalty

t = 1.1:
  ego is still overlapping the same object o
  collision type becomes ACTIVE_FRONT / STOPPED_TRACK / ACTIVE_LATERAL
  migrated Autoware can penalize this later overlap
```

NAVSIM's collided-track memory is designed to avoid treating the same sustained
overlap as a new independent collision candidate.

### Evidence We Checked

Autoware:

- `src/metrics/no_at_fault_collision.cpp::calculate_no_at_fault_collision()`
  - loops over trajectory points and predicted-object caches
  - checks `bg::intersects(ego_polygon, object_state->polygon)`
  - classifies each overlap with `classify_collision(...)`
  - does not maintain a set keyed by `PredictedObject.object_id`

### Potential Autoware Fix Direction

Use `PredictedObject.object_id` as the object identity and keep a collided-object set
inside NC evaluation:

```text
CollidedObjectIds = empty set

for each trajectory time t:
  for each object o:
    if ego/object overlap is detected:
      if object_id(o) in CollidedObjectIds:
        continue

      add object_id(o) to CollidedObjectIds
      classify collision
      penalize if the classified collision is at fault
```

## 2026-04-21 - NC Behind Predicate Is Too Broad

### What We Confirmed

NAVSIM delegates rear-position reasoning to nuPlan's `is_agent_behind()` helper.
The helper computes the relative angle between the ego heading vector and the
ego-to-object vector, then checks whether that angle is greater than 150 degrees:

```text
Behind_navsim = [relative_angle(ego_heading, ego_to_object_vector) > 150 deg]
```

This means NAVSIM only treats an object as "behind" when it is in a relatively narrow
rear cone.

The migrated Autoware helper uses a simpler half-plane test:

```text
forward_offset = cos(ego_yaw) * (object_x - ego_x)
               + sin(ego_yaw) * (object_y - ego_y)

Behind_aw = [forward_offset < 0]
```

This is equivalent to treating objects with relative angle greater than 90 degrees as
behind.

### Why This Is Unmatched

Autoware's condition is broader than NAVSIM's:

```text
NAVSIM behind:
  relative angle > 150 deg

Autoware behind:
  relative angle > 90 deg
```

So an object in the rear-side region can be classified differently.

Example:

```text
object relative angle = 120 deg

NAVSIM:
  120 deg <= 150 deg
  Behind = false

Autoware:
  120 deg > 90 deg
  Behind = true
```

This can change NC collision type classification. A collision that NAVSIM would treat
as not-behind, possibly `ACTIVE_LATERAL` or `ACTIVE_FRONT`, can become `ACTIVE_REAR`
in the migrated Autoware implementation. Since `ACTIVE_REAR` is not penalized as an
ego at-fault collision in NC, this can make the migrated implementation more forgiving
for rear-side overlaps.

### Evidence We Checked

NAVSIM / nuPlan:

- `/home/beomseokkim2/workspace/nuplan-devkit/nuplan/planning/simulation/observation/idm/utils.py`
  - `is_agent_behind(..., angle_tolerance=150)`
  - returns `get_agent_relative_angle(...) > np.deg2rad(angle_tolerance)`

Autoware:

- `src/metrics/metric_utils.cpp::forward_offset_in_ego_frame()`
  - computes the object forward offset in the ego frame
- `src/metrics/metric_utils.cpp::is_agent_behind()`
  - returns `forward_offset_in_ego_frame(...) < 0.0`

### Potential Autoware Fix Direction

Replace the half-plane test with a nuPlan-style relative-angle threshold:

```text
ego_heading = (cos(ego_yaw), sin(ego_yaw))
ego_to_object = normalize((object_x - ego_x, object_y - ego_y))
relative_angle = arccos(dot(ego_heading, ego_to_object))

Behind_aw_navsim_style = [relative_angle > 150 deg]
```

## 2026-04-22 - NC Multiple-Lanes Detection Is Seeded From Ego Pose

### What We Confirmed

NAVSIM's multiple-lanes flag is based on ego footprint/corner positions against lane
polygons. This lets NAVSIM detect cases where the ego body straddles more than one
lane, even if the ego reference pose is still inside only one lane.

The migrated Autoware NC lateral-fault branch instead starts from:

```text
route_handler->getRoadLaneletsAtPose(pose)
```

The actual Autoware `RouteHandler` implementation queries lanelets containing the
ego pose point:

```text
p = (pose.x, pose.y)
candidate_lanelets = laneletLayer.search(BoundingBox2d(p))

for each candidate:
  if lanelet::geometry::inside(candidate, p)
     and isRoadLanelet(candidate):
       keep candidate
```

Then the migrated NC code filters those lanelets to route lanelets and only then
checks whether the ego polygon intersects them.

Conceptually:

```text
R_t = road lanelets containing pose(A_t), filtered to route lanelets
n_t_route = number of lanelets in R_t intersected by ego polygon

MultipleLanes_aw = [n_t_route > 1]
NonDrivableArea_aw = [n_t_route = 0]
```

### Why This Is Unmatched

In normal road geometry, a single ego pose point usually belongs to one lanelet.
During a normal lane change, the ego footprint can overlap two lanes while the
`base_link` / rear-axle pose point is still inside only one lane.

Example:

```text
ego footprint overlaps lane A and lane B
ego base_link pose is still inside lane A

getRoadLaneletsAtPose(pose) -> {lane A}
R_t -> {lane A}
n_t_route -> 1
MultipleLanes_aw -> false
```

NAVSIM's footprint/corner-based logic can detect the same lane-straddling case as
multiple-lanes. Therefore the migrated Autoware implementation likely under-detects
`MultipleLanes` in ordinary lane-change or lane-straddling cases.

This matters for NC because lateral collisions are penalized only when the lateral
collision happens in a bad area:

```text
ACTIVE_LATERAL and (MultipleLanes or NonDrivableArea)
```

Missing `MultipleLanes` can therefore make some lateral collisions non-penalized.

### Evidence We Checked

Autoware:

- `RouteHandler::getRoadLaneletsAtPose(const Pose & pose)`
  - searches `laneletLayer` with a point bounding box
  - keeps lanelets whose polygon contains the pose point
  - keeps only road lanelets
- `src/metrics/no_at_fault_collision.cpp::compute_ego_area_flags()`
  - calls `getRoadLaneletsAtPose(pose)`
  - filters with `isRouteLanelet(lanelet)`
  - counts only those route lanelets intersecting the ego polygon

### Most Likely Fix Direction

Do not seed multiple-lanes detection from only the ego pose point. Use the ego
footprint polygon to find nearby candidate lanelets, then apply a NAVSIM-style
corner-in-lane test.

Recommended shape:

```text
candidate_lanelets = laneletLayer.search(bounding_box(P_t_aw))

for each candidate lanelet:
  if not road/lane/lane_connector:
    continue
  if route filtering is required and not isRouteLanelet(candidate):
    continue

  corner_count = number of ego footprint corners inside candidate lanelet polygon
  if corner_count > 0:
    touched_lane_count += 1

MultipleLanes_aw_navsim_style = [touched_lane_count > 1]
```

This is closer to NAVSIM because the result depends on where the ego footprint
corners are, not only on which lanelet contains the ego reference pose.

## 2026-04-22 - NC First At-Fault Collision Return Can Miss Later Worse Collision

### What We Confirmed

The migrated Autoware NC implementation returns immediately on the first at-fault
collision it encounters in trajectory/object loop order.

Current behavior:

```text
first at-fault collision wins
```

This differs from minimum-over-events bookkeeping, where the final NC score should be
the worst at-fault collision score observed over the evaluated horizon.

Example:

```text
t = 1.0:
  at-fault collision with non-agent object
  candidate score = 0.5
  migrated Autoware returns immediately

t = 2.0:
  at-fault collision with agent object
  candidate score = 0.0
  never evaluated

final migrated NC = 0.5
minimum-over-events NC = 0.0
```

### Why This Is Unmatched

NC score severity depends on collided object type:

```text
agent collision     -> 0.0
non-agent collision -> 0.5
no at-fault event   -> 1.0
```

Therefore, returning on the first at-fault event can produce a less severe final
score than the worst event in the horizon.

This is especially visible when an earlier non-agent at-fault collision is followed
by a later agent at-fault collision. The later `0.0` event should be able to reduce
the final score, but the current early return prevents that.

### Evidence We Checked

Autoware:

- `src/metrics/no_at_fault_collision.cpp::calculate_no_at_fault_collision()`
  - in the `front_or_stopped_track` branch:
    - sets `result.score` to `0.0` for agent or `0.5` for non-agent
    - returns immediately
  - in the penalized lateral-collision branch:
    - sets `result.score` to `0.0` for agent or `0.5` for non-agent
    - returns immediately

### Potential Autoware Fix Direction

Continue scanning after a `0.5` event and keep the minimum score found so far:

```text
result.score = 1.0

for each trajectory time t:
  for each object o:
    if at-fault collision:
      event_score = AgentObject(o) ? 0.0 : 0.5

      if event_score < result.score:
        result.score = event_score
        result.reason = event reason
        result.infraction_time_s = t

      if result.score == 0.0:
        return result
```

Returning on `0.0` is safe because no later event can reduce the score further.
Returning on `0.5` is the problematic case.

## 2026-04-22 - NC Stopped-Track Velocity Source And Non-Agent Rule Differ

### What We Confirmed

Both NAVSIM and the migrated Autoware implementation use the same basic stopped-speed
threshold shape:

```text
StoppedTrack = [object speed <= 0.05 m/s]
```

However, the velocity source and non-agent handling are not identical.

NAVSIM delegates stopped-track classification to nuPlan's `is_track_stopped()` helper:

```text
if tracked object is not a nuPlan Agent:
  StoppedTrack_navsim = true

if tracked object is a nuPlan Agent:
  StoppedTrack_navsim = [||tracked_object_velocity|| <= 0.05]
```

The migrated Autoware NC computes an object speed inside `interpolate_object_state()`:

```text
t = 0:
  v_aw = hypot(initial_twist.linear.x, initial_twist.linear.y)

t > 0:
  v_aw = distance(predicted_path[m+1], predicted_path[m]) / path_time_step
```

Then collision typing checks:

```text
StoppedTrack_aw = [v_aw <= 0.05]
```

This stopped-track check is applied to the computed object state, independent of the
later `is_agent_type()` score-severity check.

### Why This Is Unmatched

The threshold form is similar, but the meaning of the speed is different:

```text
NAVSIM:
  speed comes from the scenario/tracked-object state

Autoware:
  speed comes from current PredictedObject twist at t=0,
  then from predicted-path segment displacement for t>0
```

Also, NAVSIM treats non-agent tracked objects as stopped by definition, while migrated
Autoware treats a non-agent as stopped only if its computed speed is below the
threshold. In practice many static non-agent objects may still have near-zero speed,
but the rule is not identical.

This can affect NC collision type classification:

```text
STOPPED_TRACK
vs
ACTIVE_FRONT / ACTIVE_REAR / ACTIVE_LATERAL
```

Since `STOPPED_TRACK` is an at-fault collision type, different stopped-track
classification can change the final NC outcome.

### Evidence We Checked

NAVSIM / nuPlan:

- `/home/beomseokkim2/workspace/nuplan-devkit/nuplan/planning/simulation/observation/idm/utils.py`
  - `is_track_stopped(...)`
  - returns `true` for non-agent tracked objects
  - thresholds agent velocity magnitude at `0.05 m/s`

Autoware:

- `src/metrics/no_at_fault_collision.cpp::interpolate_object_state()`
  - uses `initial_twist` for `query_time_s <= 0`
  - derives future speed from adjacent predicted-path poses for `query_time_s > 0`
- `src/metrics/no_at_fault_collision.cpp::classify_collision()`
  - classifies `StoppedTrack` when `object_state.speed_mps <= 0.05`

### Potential Autoware Fix Direction

For closer NAVSIM behavior:

```text
if object is non-agent:
  StoppedTrack_aw_navsim_style = true
else:
  StoppedTrack_aw_navsim_style = [object_speed <= 0.05]
```

Separately, if NC/TTC are changed to use logged future object states instead of
`PredictedObjects.predicted_paths`, stopped-track speed should come from that same
logged future object state source rather than from predicted-path segment speed.
