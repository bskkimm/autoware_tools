# EPDMS Migration Implementation Report

## Scope and sources

This report reviews the migrated EPDMS-related implementation in this workspace against the canonical NAVSIM implementation in `~/workspace/navsim`.

Primary sources:

- `~/workspace/navsim/EPDMS_report.md`
- `~/workspace/navsim/docs/metrics.md`
- `~/workspace/navsim/navsim/planning/simulation/planner/pdm_planner/scoring/pdm_scorer.py`
- `~/workspace/navsim/navsim/planning/simulation/planner/pdm_planner/scoring/pdm_comfort_metrics.py`
- `~/workspace/navsim/navsim/planning/simulation/planner/pdm_planner/scoring/pdm_scorer_utils.py`
- `~/workspace/navsim/navsim/planning/simulation/planner/pdm_planner/scoring/scene_aggregator.py`
- `~/workspace/navsim/navsim/evaluate/pdm_score.py`
- `~/workspace/navsim/navsim/planning/script/run_pdm_score.py`
- `https://arxiv.org/abs/2406.15349` (NAVSIM)
- `https://arxiv.org/abs/2506.04218` (pseudo-simulation / NAVSIM v2 context)
- `planning/autoware_planning_data_analyzer/src/metrics/trajectory_metrics.cpp`
- `planning/autoware_planning_data_analyzer/src/metrics/epdms/*.cpp`
- `planning/autoware_planning_data_analyzer/src/metrics/geometry/*.cpp`
- `planning/autoware_planning_data_analyzer/src/open_loop_evaluator.cpp`

The EPDMS metric implementation is organized so `trajectory_metrics.cpp` acts as the
per-trajectory orchestrator. Shared EPDMS artifacts that would otherwise be recomputed
across subscores are built by `src/metrics/epdms/epdms_context.cpp`: route-relevant
lanelets, trajectory footprint / semantic drivable-area evaluations, and logged object
tracks. Geometry, lanelet, object-track, and comfort-signal helpers live under
`src/metrics/geometry/`.

In the ownership subsections below, the NAVSIM ownership wording follows the "Exact subscore ownership" section of `~/workspace/navsim/EPDMS_report.md`.

## Executive summary

1. The migrated Autoware code reproduces the **final single-trajectory 16-weight EPDMS composition**:
   $$
   (\mathrm{NC}\cdot \mathrm{DAC}\cdot \mathrm{DDC}\cdot \mathrm{TLC}) \cdot
   \frac{5\mathrm{EP}+5\mathrm{TTC}+2\mathrm{LK}+2\mathrm{HC}+2\mathrm{EC}}{16}.
   $$
2. The migration is **not a literal semantic port** for several subscores. The largest deviations are in **HC, EC, EP, TTC, TLC, and DAC**.
3. The migrated **human-filtered** path differs materially from NAVSIM code: **NAVSIM filters EP but not EC**, while the migrated code **filters EC but effectively never filters EP**.
4. NAVSIM's **pseudo closed-loop second-stage aggregation** is not implemented in this project. In easy terms: NAVSIM does **not** stop after scoring one trajectory once. It first scores the current scene, then also scores multiple follow-up scenes that represent slightly different future states, weights those follow-up scores by how close they are to the planner's likely endpoint, and combines them into one final published benchmark score. This local project does **not** perform that second-stage weighting-and-combination step. Instead, it only produces a per-trajectory local score (`synthetic_epdms_raw` or `synthetic_epdms_human_filtered`) for each evaluated sample. So the local output is best understood as "**the EPDMS of this one trajectory sample**," not "**the full NAVSIM final leaderboard-style score after pseudo closed-loop aggregation**."

## Comparison summary

| Item | Formula match | Input semantics match | Main verdict |
| --- | --- | --- | --- |
| NC | Mostly same | Partial | Close logic, different scene/object source |
| DAC | Different implementation | Partial | Same intent, different drivable-area construction |
| DDC | Same | Close | Near-port with route-handler substitutions |
| TLC | Different | No | Same intent, different violation primitive |
| EP | Different | No | Important migration deviation |
| TTC | Similar but not same | No | Important migration deviation |
| LK | Same core rule | Close | Near-port with lanelet-based centerline substitution |
| HC | Same core rule | Close | Close port with Autoware kinematic-history source |
| EC | Same core rule | Partial | Overlap/RMS port with Autoware trajectory source |
| Aggregation (raw EPDMS) | Same final 16-weight form | Partial | Formula ported, inputs not fully ported |
| Human-filtered | Different | Partial | Code behavior differs materially from NAVSIM |

---

## NC - No at-fault collision

### Definition and semantic meaning

NC is the **primary collision-responsibility safety subscore**. In NAVSIM / EPDMS semantics, it does not ask only whether contact happens; it asks whether the ego trajectory causes an **at-fault** collision. The subscore therefore distinguishes between excusable contact and safety-critical ego-caused contact, and it penalizes collisions with dynamic agents more strongly than collisions with non-agent objects.

Semantically, NC represents the most direct "did this planned behavior crash in an ego-responsible way?" component of PDMS/EPDMS. Because it is a multiplicative term, a severe at-fault collision can collapse the overall score regardless of progress or comfort.

### Ownership

- **NAVSIM primary owner:** `pdm_scorer.py::_calculate_no_at_fault_collision()`
- **NAVSIM outsourced helper logic:** `pdm_scorer_utils.py::get_collision_type()`
- **NAVSIM external state / cached dependency:** `self._observation`, `self._ego_polygons`, `self._ego_areas`, `self._states`, `self._collision_time_idcs`
- **Migrated owner:** `src/metrics/epdms/no_at_fault_collision.cpp::calculate_no_at_fault_collision()`
- **Migrated local helpers:** `classify_collision()`, `compute_ego_area_flags()`, `build_logged_object_tracks()`, `interpolate_logged_object_state()`, `is_agent_behind()`

### Required inputs: same availability or replacement?

- **Same NAVSIM inputs available?** **No**

| NC ingredient | NAVSIM input / logic | Migrated Autoware input / logic | Semantic match? | Judgement / impact |
| :--- | :--- | :--- | :--- | :--- |
| **Ego polygon** | $P_{i,t}=\mathrm{footprint}(S_{i,t})$ from simulated proposal state. NAVSIM evaluates every proposal $i$ at every scorer time $t$. | $P_t^{aw}$ from selected `TrajectoryPoint.pose` plus `VehicleInfo.createFootprint(0.0)`. The pose is expected to be the future `base_link` / rear-axle-center pose. | **Mostly matched** | Same per-time footprint concept. Main difference is source: NAVSIM simulated proposal state vs one selected Autoware trajectory. |
| **Object polygon** | $O_{t,o}$ from NAVSIM non-reactive scenario future tracked objects, interpolated to the scorer grid. | $O_{t,o}^{aw}$ from recorded `/perception/object_recognition/tracking/objects` `TrackedObjects`: each message contributes current tracked pose/twist/shape/class/object ID, object tracks are keyed by UUID, and object pose is interpolated at the NC query time. Objects whose highest-probability label is `UNKNOWN` are excluded before collision testing. | **Close replacement with Autoware artifact filter** | This intentionally uses recorded tracked object states rather than perception `PredictedObjects.predicted_paths`, so the object source is much closer to NAVSIM's logged future observations. The `UNKNOWN` exclusion suppresses tiny short-lived Autoware tracking artifacts observed in x2_odaiba; a real but unclassified obstacle would also be excluded. |
| **Collision candidate** | Valid candidate requires polygon overlap, no red-light pseudo-object token, and not already in proposal-local collided-track memory: $\mathrm{Overlap}(P_{i,t}, O_{t,o})$, `red_light_token not in o`, and $\neg(o\in C_{i,t})$. | Valid candidate is polygon overlap against a non-`UNKNOWN` interpolated tracked-object polygon: $\mathrm{Overlap}(P_t^{aw}, O_{t,o}^{aw})$. There is no NAVSIM red-light pseudo-object in tracked objects; repeated contacts are suppressed by collided object ID within the evaluated trajectory. | **Mostly matched** | Red-light exclusion is not needed for Autoware NC because those pseudo objects are absent. Object-ID collision memory is now kept locally for the evaluated trajectory. The extra `UNKNOWN` filter is an Autoware-side perception-artifact guard, not a NAVSIM rule. |
| **Ego stopped flag** | $\mathrm{StoppedEgo}_{i,t}=[\|\mathbf{v}^{ego}_{i,t}\|_2\le0.05]$. | $\mathrm{StoppedEgo}_{t}^{aw}=[\|\mathbf{v}^{ego}_{t}\|_2\le0.05]$ from trajectory longitudinal/lateral velocity. | **Mostly matched** | Same threshold form and same ego-stopped meaning. |
| **Tracked-object stopped flag** | `is_track_stopped(o)`: non-agent nuPlan tracked objects are stopped by definition; agent objects use velocity threshold $\|\mathbf{v}_o\|_2\le0.05$. | $\mathrm{StoppedTrack}_{t,o}^{aw}=[v_{t,o}^{aw}\le0.05]$, where $v_{t,o}^{aw}$ comes from tracked-object twist when available, otherwise from neighboring recorded tracked poses during interpolation. | **Mostly matched** | Same speed-threshold form for dynamic objects. The non-agent-by-definition stopped rule remains a semantic gap. |
| **Behind judgement** | nuPlan `is_agent_behind()`: relative angle between ego heading and ego-to-object vector is greater than $150^\circ$. | Local helper: forward offset in ego frame is negative, equivalent to relative angle greater than $90^\circ$. | **Unmatched** | Autoware's behind region is much broader, so rear-side objects can become `ACTIVE_REAR` instead of front/lateral. |
| **Front-bumper intersection** | $F_{i,t}=\mathrm{LineString}(P_{i,t}^{ext}[0],P_{i,t}^{ext}[3])$, then $\mathrm{Overlap}(F_{i,t}, O_{t,o})$. | $F_t^{aw}=\mathrm{LineString}(T_t(x_f,y_{min}),T_t(x_f,y_{max}))$ from `VehicleInfo` front and lateral offsets, then $\mathrm{Overlap}(F_t^{aw}, O_{t,o}^{aw})$. | **Mostly matched** | Same concept: line segment across the ego front bumper intersected with object polygon. Autoware constructs it explicitly from vehicle offsets. |
| **Collision type** | Priority order after overlap: `STOPPED_EGO`, `STOPPED_TRACK`, `ACTIVE_REAR`, `ACTIVE_FRONT`, otherwise `ACTIVE_LATERAL`. | Same priority order in `classify_collision()`. | **Structurally matched, input-unmatched** | The decision tree shape matches, but `StoppedTrack`, `Behind`, object pose/source, and collision-memory behavior differ. |
| **Bad-area flag** | $\mathrm{BadArea}_{i,t}=\mathrm{MultipleLanes}_{i,t}\lor\mathrm{NonDrivableArea}_{i,t}$ from NAVSIM cached ego-area flags. Multiple-lanes and non-drivable-area are corner/map-polygon checks. | $\mathrm{BadArea}_{t}^{aw}=\mathrm{MultipleLanes}_{t}^{aw}\lor\mathrm{NonDrivableArea}_{t}^{aw}$ from `RouteHandler::getRoadLaneletsAtPose(pose)`, route filtering, and ego-polygon intersection count. | **Unmatched** | Autoware seeds the check from the ego pose point, so ordinary footprint lane-straddling can be missed. NAVSIM is footprint/corner based. |
| **At-fault judgement** | At fault if collision type is `ACTIVE_FRONT` or `STOPPED_TRACK`, or if collision type is `ACTIVE_LATERAL` while bad-area is true. | Same logical rule using migrated collision type and migrated bad-area flag. | **Structurally matched, input-unmatched** | Formula shape is same, but the predicates feeding it differ. |
| **Per-collision score** | $\phi_{i,t,o}=0.0$ for at-fault agent collision, $0.5$ for at-fault non-agent collision, $1.0$ otherwise. | Same score set using `is_agent_type()` from highest-probability Autoware object label: vehicle, pedestrian, or animal are agents. | **Partially unmatched** | Score values match, but Autoware returns on first at-fault event, so an earlier $0.5$ can hide a later $0.0$. |
| **Full NAVSIM equation** | $\mathrm{NC}_{nav,i}=\min(1.0,\phi_{i,t,o}\text{ over }(t,o)\in K_i)$, with $K_i=\mathrm{ValidCollisionPairs}_i$. | Conceptual Autoware analogue is the same minimum form, but current C++ returns immediately on the first at-fault collision. | **Partially unmatched** | To match the minimum-over-events behavior, Autoware should keep scanning after a $0.5$ event and only safely early-return on $0.0$. |

- **Semantic replacement meaning:** NAVSIM scores against simulated ego proposals and scenario future tracked objects; the migrated code scores one selected Autoware trajectory against recorded Autoware tracked-object states from `/perception/object_recognition/tracking/objects`, interpolated across the bag timeline.
- **Autoware object-validity guard:** In x2_odaiba, NC failures were dominated by `UNKNOWN`
  polygon tracks with only 2-4 messages, zero velocity, and very small footprints. The
  migrated NC therefore excludes highest-probability `UNKNOWN` labels before overlap testing.
  This intentionally trades recall for stability against Autoware perception artifacts.

### Platform deviations and impact

- NAVSIM ignores red-light occupancy objects inside NC because TLC handles them separately. The migrated code does not have NAVSIM-style red-light observation tokens at all.
- NAVSIM uses cached `collided_track_ids` to suppress repeated penalization of already-accounted contacts. The migrated code returns on the first at-fault contact and does not carry NAVSIM's collision bookkeeping structure.
- NAVSIM lateral-fault assessment uses precomputed ego-area flags from the drivable-area map; the migrated code recomputes a weaker approximation from route-lanelet overlap at the current pose.
- **Impact:** **Moderate.** The fault taxonomy is close, but the environment model is different enough that score parity is not guaranteed.

### Equation comparison

**NAVSIM**

The following notation describes `pdm_scorer.py::_calculate_no_at_fault_collision()` and
`pdm_scorer_utils.py::get_collision_type()` at the input-equation level.

Let:

$$
i = \text{proposal index},\qquad
t = \text{time index},\qquad
o = \text{tracked-object token}.
$$

Let $S_{i,t}$ be the simulated ego state, $P_{i,t}$ be the ego footprint polygon, and
$O_{t,o}$ be the object polygon at time $t$.

#### NAVSIM NC input equations

**Per-time ego polygon**

NAVSIM NC does not use one accumulated swept polygon for the whole horizon. It uses the
single ego footprint at proposal $i$ and time $t$:

$$
P_{i,t} = \mathrm{footprint}(S_{i,t}).
$$

At each fixed time step, the implementation queries all proposal footprints from
`self._ego_polygons[:, t]`. In notation:

$$
\mathcal{P}_t=\{P_{i,t}\mid i\in\mathcal{I}\}.
$$

**Collision candidate set**

The observation spatial index returns object intersections at the same time step:

$$
I_t =
\{(i,o)\;|\;\mathrm{Overlap}(P_{i,t}, O_{t,o})\}.
$$

Red-light tokens and already-accounted collided track IDs are skipped:

$$
\mathrm{ValidCollision}_{i,t,o}
=
\left[\mathrm{Overlap}(P_{i,t}, O_{t,o})\right]
\land
\left[\neg(\mathrm{red\_light\_token}\in o)\right]
\land
\left[\neg(o\in C_{i,t})\right],
$$

where $C_{i,t}$ is the proposal-local collided-track memory. It is initialized from
`self._observation.collided_track_ids` and extended when a collision is classified as
not at fault.

**Ego stopped flag**

From the simulated state velocity:

$$
v_{i,t}^{ego} =
\sqrt{(v^{ego}_{x,i,t})^2 + (v^{ego}_{y,i,t})^2},
\qquad
\mathrm{StoppedEgo}_{i,t} =
\left[v_{i,t}^{ego}\le\tau_{stop}\right],
$$

with NAVSIM's local default:

$$
\tau_{stop}=0.05\;\mathrm{m/s}.
$$

**Tracked-object stopped flag**

NAVSIM delegates this to nuPlan's `is_track_stopped()` in
`~/workspace/nuplan-devkit/nuplan/planning/simulation/observation/idm/utils.py`.
The helper first checks whether the tracked object is a nuPlan `Agent`. Non-agent
tracked objects are treated as stopped by definition. For agents, it thresholds the
2D velocity-vector magnitude.

The object passed to this helper is `self._observation.unique_objects[o]`. That object
is keyed by token, so the helper value itself is not recomputed from $O_{t,o}$'s
polygon geometry. It is consumed at time $t$ during collision typing, which is why the
metric-level flag is written as $\mathrm{StoppedTrack}_{t,o}$ below.

Let:

$$
\mathrm{IsAgentClass}_o =
\left[\mathrm{unique\_objects}[o] \text{ is an instance of nuPlan } \mathrm{Agent}\right],
$$

and, for agent objects:

$$
\mathbf{v}_o = (v_{x,o}, v_{y,o}),\qquad
\|\mathbf{v}_o\|_2 = \sqrt{v_{x,o}^2 + v_{y,o}^2}.
$$

With nuPlan's default:

$$
\tau_{track\_stop}=0.05\;\mathrm{m/s},
$$

the actual helper logic is:

$$
\mathrm{StoppedTrack}_{t,o}
=
\begin{cases}
\mathrm{true}, & \neg\mathrm{IsAgentClass}_o \\
\left[\|\mathbf{v}_o\|_2 \le \tau_{track\_stop}\right], & \mathrm{IsAgentClass}_o.
\end{cases}
$$

So the stopped-track branch is broader than a pure speed check: static/non-agent
objects such as barriers are always classified as stopped tracks, while dynamic agents
are classified by velocity magnitude.

**Tracked-object pose state**

The object state used by collision typing is built from the object polygon centroid and
the tracked-object box heading:

$$
O^{pose}_{t,o} =
\left(
\mathrm{centroid}_x(O_{t,o}),
\mathrm{centroid}_y(O_{t,o}),
\psi_o
\right).
$$

**Ego rear-axle state**

The ego pose used by the helper predicates is extracted from the simulated ego state:

$$
E_{i,t}=(x_{i,t}^{ego},y_{i,t}^{ego},\psi_{i,t}^{ego}).
$$

**Behind predicate**

NAVSIM delegates rear-collision reasoning to nuPlan's `is_agent_behind()`, which
uses `get_agent_relative_angle()`. The helper computes the angle between the ego
heading vector and the vector from ego to object:

$$
\mathrm{Behind}_{i,t,o}
=
\mathrm{is\_agent\_behind}(E_{i,t},O^{pose}_{t,o}).
$$

Define the ego heading unit vector:

$$
\mathbf{h}_{i,t} =
\left(\cos\psi_{i,t}^{ego},\;\sin\psi_{i,t}^{ego}\right),
$$

and the ego-to-object displacement:

$$
\mathbf{d}_{i,t,o} =
\left(
\mathrm{centroid}_x(O_{t,o}) - x_{i,t}^{ego},
\mathrm{centroid}_y(O_{t,o}) - y_{i,t}^{ego}
\right).
$$

The relative angle is:

$$
\alpha_{i,t,o}
=
\arccos\left(
\frac{\mathbf{h}_{i,t}\cdot \mathbf{d}_{i,t,o}}
{\|\mathbf{d}_{i,t,o}\|_2}
\right).
$$

With nuPlan's default behind-angle tolerance:

$$
\theta_{behind}=150^\circ = \frac{5\pi}{6}\;\mathrm{rad},
$$

the actual helper logic is:

$$
\mathrm{Behind}_{i,t,o} =
\left[\alpha_{i,t,o} > \theta_{behind}\right].
$$

Thus, an object is "behind" if the vector from ego to object is more than 150 degrees
away from the ego's forward heading direction.

**Front-bumper intersection**

The front bumper is represented by a line segment from two ego polygon exterior points:

$$
F_{i,t} =
\mathrm{LineString}
\left(P_{i,t}^{ext}[0], P_{i,t}^{ext}[3]\right).
$$

Then:

$$
\mathrm{FrontHit}_{i,t,o} =
\left[\mathrm{Overlap}(F_{i,t}, O_{t,o})\right].
$$

**Collision type**

Collision type is evaluated only after an ego/object overlap has already been found by
the occupancy-map query. In other words, the following piecewise function is conditioned
on a valid collision candidate:

$$
(t,o)\in K_i
\quad\Rightarrow\quad
\mathrm{Overlap}(P_{i,t}, O_{t,o}).
$$

Under that precondition, collision type is a priority-ordered function:

$$
\mathrm{CollisionType}_{i,t,o} =
\begin{cases}
\mathrm{STOPPED\_EGO}, & \mathrm{StoppedEgo}_{i,t} \\
\mathrm{STOPPED\_TRACK}, & \neg\mathrm{StoppedEgo}_{i,t}\land \mathrm{StoppedTrack}_{t,o} \\
\mathrm{ACTIVE\_REAR}, & \neg\mathrm{StoppedEgo}_{i,t}\land\neg\mathrm{StoppedTrack}_{t,o}\land \mathrm{Behind}_{i,t,o} \\
\mathrm{ACTIVE\_FRONT}, & \neg\mathrm{StoppedEgo}_{i,t}\land\neg\mathrm{StoppedTrack}_{t,o}\land\neg\mathrm{Behind}_{i,t,o}\land \mathrm{FrontHit}_{i,t,o} \\
\mathrm{ACTIVE\_LATERAL}, & \text{otherwise}.
\end{cases}
$$

**Multiple-lanes flag**

Let $X_{i,t,k}$ be an ego footprint coordinate, where the four corner coordinates are
used for this flag and the center coordinate is ignored. The coordinate index follows
NAVSIM's `BBCoordsIndex` convention:

$$
k=0:\mathrm{FRONT\_LEFT},\quad
k=1:\mathrm{REAR\_LEFT},\quad
k=2:\mathrm{REAR\_RIGHT},\quad
k=3:\mathrm{FRONT\_RIGHT},\quad
k=4:\mathrm{CENTER}.
$$

For the multiple-lanes flag:

$$
k\in\mathrm{corners}=\{0,1,2,3\}.
$$

Let $M_p$ be the geometry of map polygon $p$ in NAVSIM's drivable-area map. In this
subsection, $p$ ranges over lane-like polygons only:

$$
M_p \in \{\text{polygons with semantic layer } \mathrm{LANE}
\text{ or } \mathrm{LANE\_CONNECTOR}\}.
$$

Then:

$$
\mathrm{InPoly}_{i,t,p,k} =
\mathbf{1}\left[X_{i,t,k}\in M_p\right],
$$

where $\mathrm{InPoly}_{i,t,p,k}=1$ means ego corner $k$ at proposal $i$ and time $t$
is inside lane polygon $M_p$, and $0$ means it is outside. Let $\mathcal{L}$ be this
set of lane-like polygon indices.

For a lane polygon $p$:

$$
\mathrm{CornerCount}_{i,t,p}
=
\sum_{k\in\mathrm{corners}}\mathrm{InPoly}_{i,t,p,k}.
$$

Thus $\mathrm{CornerCount}_{i,t,p}$ is the number of ego corners inside lane polygon
$M_p$. Its value is in $\{0,1,2,3,4\}$; for example, `4` means all ego corners are
inside that one lane polygon, while `0` means none are.

The two code conditions are:

$$
\mathrm{TouchesMoreThanOneLane}_{i,t}
=
\left[
\sum_{p\in\mathcal{L}}
\mathbf{1}\left(\mathrm{CornerCount}_{i,t,p}>0\right)
>1
\right],
$$

$$
\mathrm{NoSingleLaneContainsAllCorners}_{i,t}
=
\left[
\forall p\in\mathcal{L},\;
\mathrm{CornerCount}_{i,t,p}\ne 4
\right].
$$

Thus:

$$
\mathrm{MultipleLanes}_{i,t}
=
\mathrm{TouchesMoreThanOneLane}_{i,t}
\land
\mathrm{NoSingleLaneContainsAllCorners}_{i,t}.
$$

**Non-drivable-area flag**

Let $\mathcal{D}$ be the set of drivable-area polygons with semantic layers
`ROADBLOCK`, `INTERSECTION`, `DRIVABLE_AREA`, and `CARPARK_AREA`.

For each ego corner:

$$
\mathrm{CornerDrivable}_{i,t,k}
=
\left[
\sum_{p\in\mathcal{D}}\mathrm{InPoly}_{i,t,p,k}>0
\right].
$$

Then:

$$
\mathrm{NonDrivableArea}_{i,t}
=
\left[
\sum_{k\in\mathrm{corners}}
\mathbf{1}\left(\mathrm{CornerDrivable}_{i,t,k}\right)
<4
\right].
$$

**Bad-area flag used by NC**

NC only uses the multiple-lanes and non-drivable-area parts of `_ego_areas`:

$$
\mathrm{BadArea}_{i,t}
=
\mathrm{MultipleLanes}_{i,t}
\lor
\mathrm{NonDrivableArea}_{i,t}.
$$

**At-fault predicate**

Given the collision type and bad-area flag:

$$
\mathrm{AtFault}_{i,t,o}
=
\left[
\mathrm{CollisionType}_{i,t,o}
\in
\{\mathrm{ACTIVE\_FRONT},\mathrm{STOPPED\_TRACK}\}
\right]
\lor
\left[
\mathrm{BadArea}_{i,t}
\land
\mathrm{CollisionType}_{i,t,o}=\mathrm{ACTIVE\_LATERAL}
\right].
$$

**Agent-object predicate**

Let:

$$
\mathrm{AgentObject}_{o}
=
\left[
\mathrm{tracked\_object\_type}(o)\in\mathrm{AGENT\_TYPES}
\right].
$$

**Per-collision score**

For a valid collision candidate:

$$
\phi_{i,t,o} =
\begin{cases}
0.0, & \mathrm{AtFault}_{i,t,o}\land \mathrm{AgentObject}_{o} \\
0.5, & \mathrm{AtFault}_{i,t,o}\land \neg\mathrm{AgentObject}_{o} \\
1.0, & \text{otherwise}.
\end{cases}
$$

#### Full NAVSIM NC equation from the defined inputs

Let:

$$
K_i = \{(t,o)\;|\;\mathrm{ValidCollision}_{i,t,o}\}.
$$

Then the NAVSIM no-at-fault-collision score for proposal $i$ is:

$$
\mathrm{NC}_{nav,i}
=
\min\left(
\{1.0\}
\cup
\{\phi_{i,t,o}\;|\;(t,o)\in K_i\}
\right).
$$

Equivalently, for non-empty $K_i$, after expanding $\phi_{i,t,o}$:

$$
\mathrm{NC}_{nav,i}
=
\min_{(t,o)\in K_i}
\begin{cases}
0.0, &
\mathrm{AgentObject}_{o}
\land
\left(
\mathrm{CollisionType}_{i,t,o}\in\{\mathrm{ACTIVE\_FRONT},\mathrm{STOPPED\_TRACK}\}
\lor
\left(
(\mathrm{MultipleLanes}_{i,t}\lor\mathrm{NonDrivableArea}_{i,t})
\land
\mathrm{CollisionType}_{i,t,o}=\mathrm{ACTIVE\_LATERAL}
\right)
\right) \\
0.5, &
\neg\mathrm{AgentObject}_{o}
\land
\left(
\mathrm{CollisionType}_{i,t,o}\in\{\mathrm{ACTIVE\_FRONT},\mathrm{STOPPED\_TRACK}\}
\lor
\left(
(\mathrm{MultipleLanes}_{i,t}\lor\mathrm{NonDrivableArea}_{i,t})
\land
\mathrm{CollisionType}_{i,t,o}=\mathrm{ACTIVE\_LATERAL}
\right)
\right) \\
1.0, & \text{otherwise}.
\end{cases}
$$

If $K_i$ is empty, the score remains the initialized value:

$$
\mathrm{NC}_{nav,i}=1.0.
$$

The implementation is time-order dependent for the non-at-fault collided-token memory
$C_{i,t}$, because non-at-fault collision tokens are appended and skipped in later time
steps for the same proposal.

Compactly, the same rule can be summarized as:

$$
\mathrm{NC}_{nav} = \min_t \; \phi_{nav}(t),
$$

where an intersecting object at time $t$ produces

$$
\phi_{nav}(t)=
\begin{cases}
0.0, & \text{at-fault collision with agent type} \\
0.5, & \text{at-fault collision with non-agent} \\
1.0, & \text{otherwise}
\end{cases}
$$

and "at-fault" means

$$
\mathrm{ACTIVE\_FRONT}
\;\lor\;
\mathrm{STOPPED\_TRACK}
\;\lor\;
\bigl(\mathrm{ACTIVE\_LATERAL} \land (\mathrm{MULTIPLE\_LANES}\lor \mathrm{NON\_DRIVABLE})\bigr).
$$

Where:
- `ACTIVE_FRONT`: Collision at the front with ego moving forward.
- `STOPPED_TRACK`: Collision with a stationary object.
- `ACTIVE_LATERAL`: Collision at the side with ego moving forward.
- `MULTIPLE_LANES`: Ego touching multiple lanelets (drifting/lane-changing).
- `NON_DRIVABLE`: Ego driving outside the legal route lanelets.
- **The Set:** $t \in \{t_0, t_1, ... t_{end}\}$ over the scorer's proposal sampling grid.
- **Sampling:** The computer checks the collision status at each trajectory point


#### Migrated Autoware algorithmic explanation

The following describes the local Autoware implementation, not NAVSIM's Python scorer.

In `src/metrics/epdms/no_at_fault_collision.cpp`, the process looks like this:
1.  **Initialization:** Start with a perfect score of 1.0.
2.  **The Loop:** For every point in the `Trajectory`:
    *   **Step A:** Calculate the Ego Bumper position at that time.
    *   **Step B:** Check for intersection with surrounding objects.
    *   **Step C:** If there is an intersection, apply the At-Fault Logic (`ACTIVE_FRONT`, etc.).
    *   **Step D:** If it is at-fault, determine if the object is an Agent (0.0) or Non-Agent (0.5).
3.  **Early Exit:** The current C++ code returns immediately on the first at-fault
    collision it encounters, whether that event scores 0.0 or 0.5.

Collision typing itself is delegated to `get_collision_type()`.

**Migrated Autoware**

$$
\mathrm{NC}_{aw}^{conceptual} = \min_t \; \phi_{aw}(t),
$$

with the same score set $\{1.0, 0.5, 0.0\}$, but the contact is generated from
Autoware predicted objects, and the lateral-fault condition is

$$
\mathrm{ACTIVE\_LATERAL} \land (\mathrm{MULTIPLE\_LANES}\lor \mathrm{NON\_DRIVABLE}),
$$

where `MULTIPLE_LANES` and `NON_DRIVABLE` are recomputed from `RouteHandler` lanelet overlap rather than NAVSIM's cached drivable-area map.

The displayed minimum is the intended conceptual score. The actual C++ loop currently
uses first-at-fault return behavior, which can differ when a 0.5 non-agent collision is
encountered before a later 0.0 agent collision.

#### Migrated Autoware NC input equations

The migrated implementation evaluates one selected Autoware trajectory, not a batch of
NAVSIM proposal trajectories. Let:

$$
t = \text{trajectory sample time},\qquad
o = \text{Autoware predicted object index}.
$$

Let $A_t$ be the selected Autoware trajectory point at sample time $t$, $P_t^{aw}$ be
the ego footprint polygon at that sample, and $O_{t,o}^{aw}$ be the predicted object
polygon for object $o$ at that sample time.

**Ego footprint**

The local footprint is created from `VehicleInfo`, then transformed by the selected
trajectory point pose. `VehicleInfo` is loaded from ROS vehicle parameters
(`vehicle_info.param.yaml` through `VehicleInfoUtils`), not from the rosbag message
stream. It contains the ego vehicle geometry:

$$
\mathrm{VehicleInfo}
=
(\mathrm{wheel\_base},\mathrm{wheel\_tread},
\mathrm{front\_overhang},\mathrm{rear\_overhang},
\mathrm{left\_overhang},\mathrm{right\_overhang},\ldots).
$$

Autoware derives local footprint offsets from these parameters. The local reference
origin is the vehicle base frame origin, defined by `VehicleInfo` as the point on the
ground below the middle of the rear axle. The rear axle is the line connecting the two
rear wheels; its middle is halfway between the left and right rear wheels. "On the
ground below" means the 2D map-frame reference point obtained by projecting that rear
axle midpoint down to the road surface. Autoware's frame convention uses `base_link`
for this rear-axle-center frame, so a planned trajectory pose is expected to be the
future `base_link` pose, not the geometric center pose. Thus, the local footprint is
not just a centered length/width rectangle; it is expressed relative to that rear-axle
base frame:

$$
x_{\min}=-\mathrm{rear\_overhang},\qquad
x_{\max}=\mathrm{wheel\_base}+\mathrm{front\_overhang},
$$

$$
y_{\min}=-(\mathrm{wheel\_tread}/2+\mathrm{right\_overhang}),\qquad
y_{\max}=\mathrm{wheel\_tread}/2+\mathrm{left\_overhang}.
$$

The implementation calls:

$$
\mathrm{VehicleInfoFootprint}
=
\mathrm{VehicleInfo.createFootprint}(0.0).
$$

This footprint is a local polygon/ring. For each local footprint vertex
$u_k=(x_k^{local},y_k^{local})$, the selected trajectory point pose
$\mathrm{pose}(A_t)=(x_t^{ego},y_t^{ego},\psi_t^{ego})$ maps it to the world/map frame:

$$
x_{t,k}^{world}
=
x_t^{ego}
+\cos\psi_t^{ego}\,x_k^{local}
-\sin\psi_t^{ego}\,y_k^{local},
$$

$$
y_{t,k}^{world}
=
y_t^{ego}
+\sin\psi_t^{ego}\,x_k^{local}
+\cos\psi_t^{ego}\,y_k^{local}.
$$

The transformed footprint polygon is:

$$
P_t^{aw} =
\mathrm{Polygon}\left(
\{(x_{t,k}^{world},y_{t,k}^{world})\}_k
\right).
$$

In code, this is `create_pose_footprint(point.pose, local_footprint)`, implemented as
`transform_vector(local_footprint, pose2transform(point.pose))`.

**Object prediction path selection**

For each predicted object $o$, the migrated code selects the highest-confidence
predicted path:

$$
\pi_o^{*} =
\arg\max_{\pi\in\Pi_o}\mathrm{confidence}(\pi),
$$

where $\Pi_o$ is the set of predicted paths attached to object $o$.

If no path with at least two poses and positive time step is available, future-time
object states are unavailable except for the initial-time branch.

**Object state interpolation**

For a query time $t$, the migrated code forms an object state:

$$
Z_{t,o}^{aw} =
\left(
\mathrm{pose}_{t,o}^{aw},
v_{t,o}^{aw},
O_{t,o}^{aw}
\right).
$$

At $t\le 0$, this is taken from the object's initial pose and twist:

$$
\mathrm{pose}_{t,o}^{aw} =
\mathrm{initial\_pose}_o,
\qquad
v_{t,o}^{aw}
=
\sqrt{(v_{x,o}^{init})^2 + (v_{y,o}^{init})^2},
$$

and

$$
O_{t,o}^{aw}
=
\mathrm{to\_polygon2d}
\left(\mathrm{initial\_pose}_o,\mathrm{shape}_o\right).
$$

For $t>0$, let the selected path time step be $\Delta t_o$, and let
$m=\left\lfloor t/\Delta t_o\right\rfloor$ clamped so that both $m$ and $m+1$ exist.
Let:

$$
\lambda =
\mathrm{clamp}\left(
\frac{t - m\Delta t_o}{\Delta t_o},\;0,\;1
\right).
$$

The pose is interpolated between adjacent predicted-path poses:

$$
\mathrm{pose}_{t,o}^{aw}
=
\mathrm{interp}
\left(
\pi_o^{*}[m],\;\pi_o^{*}[m+1],\;\lambda
\right).
$$

The object speed is derived from the path segment length:

$$
v_{t,o}^{aw}
=
\frac{
\left\|
\mathrm{position}(\pi_o^{*}[m+1])
-
\mathrm{position}(\pi_o^{*}[m])
\right\|_2
}{\Delta t_o}.
$$

The object polygon is:

$$
O_{t,o}^{aw}
=
\mathrm{to\_polygon2d}
\left(\mathrm{pose}_{t,o}^{aw},\mathrm{shape}_o\right).
$$

**Collision candidate**

Autoware NC processes a collision candidate when the selected ego footprint intersects
the predicted object polygon:

$$
\mathrm{ValidCollision}_{t,o}^{aw}
=
\left[\mathrm{Overlap}(P_t^{aw}, O_{t,o}^{aw})\right].
$$

Unlike NAVSIM, there is no red-light pseudo-object filter and no collided-track memory
in this migrated path.

**Ego stopped flag**

The migrated code computes ego speed from the selected trajectory point's longitudinal
and lateral velocities:

$$
v_t^{ego,aw}
=
\sqrt{
\left(v_{long,t}^{ego}\right)^2
+
\left(v_{lat,t}^{ego}\right)^2
},
$$

with:

$$
\tau_{stop}^{aw}=0.05\;\mathrm{m/s}.
$$

Then:

$$
\mathrm{StoppedEgo}_{t}^{aw}
=
\left[v_t^{ego,aw}\le\tau_{stop}^{aw}\right].
$$

**Object stopped flag**

The migrated code uses the interpolated/derived object speed:

$$
\mathrm{StoppedTrack}_{t,o}^{aw}
=
\left[v_{t,o}^{aw}\le\tau_{stop}^{aw}\right].
$$

This differs from NAVSIM's nuPlan helper: non-agent objects are not automatically
classified as stopped by type; the migrated code uses the computed object-state speed.

**Behind predicate**

Autoware's local helper computes the object forward offset in the ego frame. Let:

$$
\Delta x_{t,o}=x_{t,o}^{aw}-x_t^{ego},\qquad
\Delta y_{t,o}=y_{t,o}^{aw}-y_t^{ego}.
$$

The forward offset is:

$$
f_{t,o}^{aw}
=
\cos(\psi_t^{ego})\Delta x_{t,o}
+
\sin(\psi_t^{ego})\Delta y_{t,o}.
$$

Then:

$$
\mathrm{Behind}_{t,o}^{aw}
=
\left[f_{t,o}^{aw}<0\right].
$$

So the migrated code uses a half-plane test behind the ego, not NAVSIM's
angle-greater-than-150-degrees test.

**Front-bumper intersection**

The front bumper is constructed from `VehicleInfo` offsets in the ego frame. Let:

$$
x_f=\mathrm{max\_longitudinal\_offset},\qquad
y_{min}=\mathrm{min\_lateral\_offset},\qquad
y_{max}=\mathrm{max\_lateral\_offset}.
$$

For ego pose $(x_t^{ego},y_t^{ego},\psi_t^{ego})$, define the transform:

$$
T_t(x,y)=
\left(
x_t^{ego}+\cos\psi_t^{ego}\,x-\sin\psi_t^{ego}\,y,\;
y_t^{ego}+\sin\psi_t^{ego}\,x+\cos\psi_t^{ego}\,y
\right).
$$

Then the migrated front-bumper line is:

$$
F_t^{aw}
=
\mathrm{LineString}
\left(
T_t(x_f,y_{min}),\;
T_t(x_f,y_{max})
\right).
$$

And:

$$
\mathrm{FrontHit}_{t,o}^{aw}
=
\left[\mathrm{Overlap}(F_t^{aw}, O_{t,o}^{aw})\right].
$$

**Collision type**

As in NAVSIM, collision type is evaluated only after an ego/object overlap has already
been found:

$$
\mathrm{ValidCollision}_{t,o}^{aw}
\Rightarrow
\mathrm{Overlap}(P_t^{aw}, O_{t,o}^{aw}).
$$

Under that precondition:

$$
\mathrm{CollisionType}_{t,o}^{aw}
=
\begin{cases}
\mathrm{STOPPED\_EGO}, & \mathrm{StoppedEgo}_{t}^{aw} \\
\mathrm{STOPPED\_TRACK}, & \neg\mathrm{StoppedEgo}_{t}^{aw}\land \mathrm{StoppedTrack}_{t,o}^{aw} \\
\mathrm{ACTIVE\_REAR}, & \neg\mathrm{StoppedEgo}_{t}^{aw}\land\neg\mathrm{StoppedTrack}_{t,o}^{aw}\land \mathrm{Behind}_{t,o}^{aw} \\
\mathrm{ACTIVE\_FRONT}, & \neg\mathrm{StoppedEgo}_{t}^{aw}\land\neg\mathrm{StoppedTrack}_{t,o}^{aw}\land\neg\mathrm{Behind}_{t,o}^{aw}\land \mathrm{FrontHit}_{t,o}^{aw} \\
\mathrm{ACTIVE\_LATERAL}, & \text{otherwise}.
\end{cases}
$$

**Route-lanelet bad-area flags**

The migrated lateral-fault branch recomputes area flags from `RouteHandler` only when
the collision type is `ACTIVE_LATERAL`.

Let $\mathcal{R}_t$ be the road lanelets returned by `getRoadLaneletsAtPose()` at the
ego pose, filtered to route lanelets:

$$
\mathcal{R}_t
=
\mathrm{RouteLaneletsAtPose}(\mathrm{pose}(A_t)).
$$

Let:

$$
\mathrm{IntersectsRouteLanelet}_{t,\ell}
=
\left[\mathrm{Overlap}(P_t^{aw}, \mathrm{polygon}(\ell))\right].
$$

The count used by the implementation is:

$$
n_t^{route}
=
\sum_{\ell\in\mathcal{R}_t}
\mathbf{1}\left(\mathrm{IntersectsRouteLanelet}_{t,\ell}\right).
$$

Then:

$$
\mathrm{MultipleLanes}_{t}^{aw}
=
\left[n_t^{route}>1\right],
\qquad
\mathrm{NonDrivableArea}_{t}^{aw}
=
\left[n_t^{route}=0\right].
$$

and:

$$
\mathrm{BadArea}_{t}^{aw}
=
\mathrm{MultipleLanes}_{t}^{aw}
\lor
\mathrm{NonDrivableArea}_{t}^{aw}.
$$

This is a route-lanelet overlap proxy, not NAVSIM's corner-in-drivable-area-map test.

**Agent-object predicate**

The migrated code classifies a predicted object as an agent when the highest-probability
classification label is a vehicle, pedestrian, or animal:

$$
\mathrm{AgentObject}_{o}^{aw}
=
\left[
\mathrm{isVehicle}(\mathrm{label}_o)
\lor
\mathrm{label}_o=\mathrm{PEDESTRIAN}
\lor
\mathrm{label}_o=\mathrm{ANIMAL}
\right],
$$

where $\mathrm{label}_o$ is the highest-probability classification label.

**At-fault predicate**

For a valid migrated collision candidate:

$$
\mathrm{AtFault}_{t,o}^{aw}
=
\left[
\mathrm{CollisionType}_{t,o}^{aw}
\in
\{\mathrm{ACTIVE\_FRONT},\mathrm{STOPPED\_TRACK}\}
\right]
\lor
\left[
\mathrm{BadArea}_{t}^{aw}
\land
\mathrm{CollisionType}_{t,o}^{aw}=\mathrm{ACTIVE\_LATERAL}
\right].
$$

**Per-collision score**

$$
\phi_{t,o}^{aw}
=
\begin{cases}
0.0, & \mathrm{AtFault}_{t,o}^{aw}\land \mathrm{AgentObject}_{o}^{aw} \\
0.5, & \mathrm{AtFault}_{t,o}^{aw}\land \neg\mathrm{AgentObject}_{o}^{aw} \\
1.0, & \text{otherwise}.
\end{cases}
$$

**Full migrated Autoware NC equation**

Let:

$$
K^{aw} = \{(t,o)\;|\;\mathrm{ValidCollision}_{t,o}^{aw}\}.
$$

If all inputs are available, the conceptual score is:

$$
\mathrm{NC}_{aw}
=
\min\left(
\{1.0\}
\cup
\{\phi_{t,o}^{aw}\;|\;(t,o)\in K^{aw}\}
\right).
$$

The actual C++ implementation returns immediately on the first at-fault collision it
encounters in trajectory/object loop order. This is not identical to the conceptual
minimum above or NAVSIM's minimum-over-events bookkeeping: an earlier at-fault non-agent
collision would return $0.5$ even if a later at-fault agent collision would have reduced
the conceptual minimum to $0.0$.

### Assessment

The migrated NC is a **reasonable structural imitation**, but not an exact semantic port because the observation model and ego-area source are different.

---

## DAC - Drivable area compliance

### Definition and semantic meaning

DAC is the **map-compliance safety subscore** for staying inside the drivable region. In NAVSIM semantics, it checks whether the ego footprint remains within the area that is legally and geometrically drivable, rather than drifting into sidewalks, medians, or other non-drivable map space.

### Ownership

- **NAVSIM primary owner:** `pdm_scorer.py::_calculate_drivable_area_compliance()`
- **NAVSIM outsourced helper logic:** no dedicated helper; depends on `_calculate_ego_area()`
- **NAVSIM external state / cached dependency:** `self._ego_areas`, `self._drivable_area_map`
- **Migrated owner:** `src/metrics/epdms/drivable_area_compliance.cpp::calculate_drivable_area_compliance()`
- **Migrated outsourced helper logic:** `src/metrics/geometry/metric_utils.cpp::compute_ego_area_flags()`
- **Migrated local helper dependencies:** `collect_route_relevant_lanelets()`, `create_pose_footprint()`, `collect_candidate_road_lanelets()`, `collect_candidate_parking_lots()`, `detect_non_drivable_area()`

### Required inputs: same availability or replacement?

- **Same NAVSIM inputs available?** **No**

| Required Input (NAVSIM) | Semantic Meaning (NAVSIM) | Autoware Replacement | Semantic Meaning (AW) | Judgement / Impact |
| :--- | :--- | :--- | :--- | :--- |
| **Drivable Area Map** | Definition of all legally/geometrically drivable space. | `RouteHandler` map + route state | Local-global semantic union of nearby road lanelets, road-shoulder lanelets, `intersection_area` polygons, `hatched_road_markings` polygons, `parking_lot` polygons, plus a per-corner closest-`road_border` side test for small boundary leakage. | **Moderate.** AW now approximates NAVSIM's semantic drivable union more closely, but still lacks a globally cached drivable-area layer. |
| **Simulated Ego Polygons** | The ego vehicle footprints over the trajectory. | `autoware_planning_msgs::msg::Trajectory` + `VehicleInfo` | The planned rollout footprints. | **Equivalent.** |
| **Ego-Area Classification** | Mask determining "ROAD" vs "NON-ROAD" status. | `compute_ego_area_flags()` | Per-corner classification against selected lanelet/polygon candidates around the footprint. | **Close.** Corner logic now matches NAVSIM style better than the previous footprint-within-union proxy. |

- **Semantic replacement meaning:** the migration now builds a per-timestep admissible set from map-supported vehicle-drivable polygons around the actual ego footprint, then classifies each ego corner against that set.

### Platform deviations and impact

- NAVSIM's drivable-area mask includes multiple polygon layers (`ROADBLOCK`, `INTERSECTION`, `DRIVABLE_AREA`, `CARPARK_AREA`) through the cached drivable map.
- The migrated code does **not** have a single equivalent semantic cache. Instead it combines:
  - all nearby road lanelets queried directly from the raw lanelet map around each ego footprint with a local-global search margin,
  - nearby `road_shoulder` lanelets,
  - nearby `intersection_area` polygons,
  - nearby `hatched_road_markings` polygons,
  - nearby `parking_lot` polygons from the lanelet map polygon layer.
  - a per-corner closest-`road_border` side test only when the semantic union rejects a corner.
- This is more permissive than the previous route-union proxy because non-route road surface, neighboring lanes, oncoming lanes, shoulders, and intersection-area polygons near the actual ego footprint can still count as physically drivable.
- It is still **not** equivalent to NAVSIM's full semantic drivable-area map because the map layer is built locally per sample rather than from one cached global `DRIVABLE_AREA` layer.
- **Impact:** **Moderate.** The corner-based failure condition now matches NAVSIM much more closely, but the admissible map region is still an Autoware-specific approximation.

### Equation comparison

#### NAVSIM DAC

**NAVSIM inputs.** Let $P_{i,t}$ be the ego footprint polygon for proposal $i$ at
time $t$, and let $X_{i,t,k}$ be ego footprint coordinate $k$, where the four corner
indices are used. $X_{i,t,k}$ is a world/map coordinate from the simulated ego
footprint, not a route-centerline point.

Let $\mathcal{D}$ be NAVSIM's drivable-area polygon index set:

$$
\mathcal{D}
=
\{\mathrm{ROADBLOCK},\mathrm{INTERSECTION},\mathrm{DRIVABLE\_AREA},\mathrm{CARPARK\_AREA}\}.
$$

Define:

$$
\mathrm{CornerDrivable}_{i,t,k}
=
\left[
\sum_{p\in\mathcal{D}}\mathbf{1}(X_{i,t,k}\in M_p)>0
\right].
$$

Then:

$$
\mathrm{NonDrivableArea}_{i,t}
=
\left[
\sum_{k\in\mathrm{corners}}
\mathbf{1}(\mathrm{CornerDrivable}_{i,t,k})<4
\right],
$$

and the full NAVSIM DAC score is:

$$
\mathrm{DAC}_{nav,i}
=
\mathbf{1}
\left[
\forall t,\;\neg \mathrm{NonDrivableArea}_{i,t}
\right].
$$

#### Migrated Autoware DAC

The migrated Autoware DAC is a **NAVSIM-style semantic drivable-area corner check**
over the Autoware lanelet map. It admits road lanelets, road-shoulder lanelets,
`intersection_area` polygons, `hatched_road_markings` polygons, `parking_lot`
polygons, and a conservative `road_border` closest-side fallback. This is closer to NAVSIM's
`ROADBLOCK`, `INTERSECTION`, `DRIVABLE_AREA`, and `CARPARK_AREA` union than the earlier
route-guided road-lanelet-only approximation. Road-border line strings are not converted
into a broad polygonal convex hull; they are used only as local one-corner boundary evidence.

The migrated Autoware DAC is now a **local-global semantic drivable-area check**.
The lanelet query is intentionally not route-direction-narrowed: opposite-direction
road lanelets are still physically drivable road surface for DAC, while DDC remains
responsible for wrong-way/oncoming progress.

First, for each selected trajectory point $A_t$, the ego footprint $P_t^{aw}$ is
constructed from `VehicleInfo` at that pose:

$$
P_t^{aw}
=
\mathrm{transform}
\left(
\mathrm{VehicleInfo.createFootprint}(0.0),\;
\mathrm{pose}(A_t)
\right).
$$

Next, the implementation collects route-designated road lanelets seen along the
selected trajectory only as additional candidates, not as a narrowing filter:

$$
\mathcal{L}^{route,aw}
=
\bigcup_t \mathcal{L}_t^{route,aw},
\qquad
\mathcal{L}_t^{route,aw}
=
\mathrm{RouteLaneletsAtPose}(\mathrm{pose}(A_t)).
$$

At each timestep, it adds all nearby road lanelets from raw map search around the
current ego footprint using a `15 m` local-global semantic search margin:

$$
\mathcal{L}_t^{bbox,aw}
=
\mathrm{RoadLaneletsFromBBoxSearch}(\mathrm{expandedBBox}_{15m}(P_t^{aw})).
$$

It further adds road lanelets reported at the current pose whose polygons actually
intersect the current ego footprint:

$$
\mathcal{L}_t^{pose,aw}
=
\mathrm{PoseIntersectingRoadLanelets}(P_t^{aw}, \mathrm{pose}(A_t)).
$$

The effective road candidate set is therefore:

$$
\mathcal{R}_t^{aw}
=
\mathcal{L}^{route,aw}
\cup
\mathcal{L}_t^{bbox,aw}
\cup
\mathcal{L}_t^{pose,aw}.
$$

Nearby `parking_lot` polygons are also added from the map polygon layer using the
same `15 m` local-global semantic search margin:

$$
\mathcal{P}_t^{aw}
=
\mathrm{NearbyParkingLotPolygons}(\mathrm{expandedBBox}_{15m}(P_t^{aw})).
$$

Nearby `road_shoulder` lanelets and `intersection_area` polygons are also added
with the same semantic search margin:

$$
\mathcal{S}_t^{aw}
=
\mathrm{RoadShoulderLaneletsFromBBoxSearch}(\mathrm{expandedBBox}_{15m}(P_t^{aw})),
\qquad
\mathcal{I}_t^{aw}
=
\mathrm{IntersectionAreaPolygons}(\mathrm{expandedBBox}_{15m}(P_t^{aw})).
$$

Nearby `hatched_road_markings` polygons are added as low-priority paved road-marking
space:

$$
\mathcal{H}_t^{aw}
=
\mathrm{HatchedRoadMarkingPolygons}(\mathrm{expandedBBox}_{15m}(P_t^{aw})).
$$

These polygon and lanelet candidates define the trusted semantic drivable union:

$$
\mathcal{U}_t^{sem,aw}
=
\mathcal{R}_t^{aw}
\cup
\mathcal{S}_t^{aw}
\cup
\mathcal{I}_t^{aw}
\cup
\mathcal{H}_t^{aw}
\cup
\mathcal{P}_t^{aw}.
$$

For each ego corner $X_{t,k}^{aw}$, the code first checks whether that corner lies
inside at least one trusted semantic drivable polygon:

$$
\mathrm{SemanticDrivable}_{t,k}^{aw}
=
\left[
\left(
\sum_{u\in\mathcal{U}_t^{sem,aw}}
\mathbf{1}\!\left(X_{t,k}^{aw}\in\mathrm{polygon}(u)\right)
\right)>0
\right].
$$

Only when this semantic check fails does the code use a `road_border` fallback.
Nearby `road_border` line strings are queried from a `5 m` footprint bbox:

$$
\mathcal{B}_t^{aw}
=
\mathrm{RoadBorderLines}(\mathrm{expandedBBox}_{5m}(P_t^{aw})).
$$

The fallback treats `road_border` as a final outside boundary, not as drivable area by
itself. For a semantically failed corner $X_{t,k}^{aw}$, the closest point on the
semantic drivable union boundary is:

$$
\Sigma_{t,k}^{aw}
=
\mathrm{ClosestSemanticBoundaryPoint}
\left(
X_{t,k}^{aw}, \mathcal{U}_t^{sem,aw}
\right).
$$

Each `road_border` line string is split into finite line segments. For each candidate
segment endpoint pair $(A_{t,k}^{aw}, B_{t,k}^{aw})$, let $Q_{t,k}^{aw}$ be the
closest point on that finite segment to the failed corner:

$$
Q_{t,k}^{aw}
=
\mathrm{ClosestPointOnSegment}
\left(
X_{t,k}^{aw}, A_{t,k}^{aw}, B_{t,k}^{aw}
\right).
$$

The segment direction and unit normal are:

$$
d_{t,k}^{aw}=B_{t,k}^{aw}-A_{t,k}^{aw},
\qquad
n_{t,k}^{aw}
=
\frac{(-d_y,\;d_x)}{\|d_{t,k}^{aw}\|}.
$$

The code probes both sides of the border from $Q_{t,k}^{aw}$ using an ordered distance sequence
`0.3 m`, `0.6 m`, `1.0 m`, `1.5 m`, `2.0 m`, `2.5 m`, `3.0 m`, and `4.0 m`
from the closest point. The first distance that puts exactly one side inside the
semantic union is used:

$$
Y_{t,k}^{+}(\rho)=Q_{t,k}^{aw}+\rho n_{t,k}^{aw},
\qquad
Y_{t,k}^{-}(\rho)=Q_{t,k}^{aw}-\rho n_{t,k}^{aw},
\qquad
\rho\in\{0.3,0.6,1.0,1.5,2.0,2.5,3.0,4.0\}.
$$

The road side is inferred only from actual semantic containment:

$$
\mathrm{PlusRoadSide}_{t,k}^{aw}
=
\left[
Y_{t,k}^{+}(\rho)\in\mathcal{U}_t^{sem,aw}
\right],
\qquad
\mathrm{MinusRoadSide}_{t,k}^{aw}
=
\left[
Y_{t,k}^{-}(\rho)\in\mathcal{U}_t^{sem,aw}
\right].
$$

The side test is valid only when exactly one side is semantic-drivable. If both sides
are semantic-drivable, or neither side is semantic-drivable, the border fallback is
rejected as ambiguous. The failed corner is accepted by the border fallback only when
all of the following are true:

- the closest segment exists,
- exactly one of $Y_{t,k}^{+}$ and $Y_{t,k}^{-}$ is inside the semantic drivable union,
- the failed corner lies on the same signed half-plane as the semantic-drivable side,
- the failed corner lies in the bounded gap between semantic boundary point
  $\Sigma_{t,k}^{aw}$ and border point $Q_{t,k}^{aw}$,
- the direction from $\Sigma_{t,k}^{aw}$ to $Q_{t,k}^{aw}$ is across the border rather
  than along it,
- the failed corner is within `3.0 m` of the border point,
- the semantic-boundary-to-border distance is at most `4.0 m`.

The signed half-plane check is:

$$
\mathrm{SameRoadSide}_{t,k}^{aw}
=
\left[
\left((X_{t,k}^{aw}-Q_{t,k}^{aw})\cdot n_{t,k}^{aw}\right)
\left((Y_{t,k}^{road}-Q_{t,k}^{aw})\cdot n_{t,k}^{aw}\right)
> 0
\right],
$$

where $Y_{t,k}^{road}$ is whichever of $Y_{t,k}^{+}$ or $Y_{t,k}^{-}$ is semantic-drivable
at the first unambiguous probe distance.

The bounded-gap test projects the failed corner onto the segment from the semantic
boundary point to the border point:

$$
\lambda_{t,k}^{aw}
=
\frac{
\left(X_{t,k}^{aw}-\Sigma_{t,k}^{aw}\right)\cdot
\left(Q_{t,k}^{aw}-\Sigma_{t,k}^{aw}\right)
}{
\left\|Q_{t,k}^{aw}-\Sigma_{t,k}^{aw}\right\|^2
}.
$$

The candidate is between the semantic boundary and the border only when:

$$
\mathrm{Between}_{t,k}^{aw}
=
\left[-0.05\le\lambda_{t,k}^{aw}\le1.05\right]
\land
\left[
\mathrm{dist}\left(
X_{t,k}^{aw},
\mathrm{line}\left(\Sigma_{t,k}^{aw}, Q_{t,k}^{aw}\right)
\right)\le0.75
\right].
$$

The across-road check rejects border segments that are merely close along the road
direction. With $\hat{e}_{SQ}$ as the unit vector from semantic boundary to border and
$\hat{t}_{AB}$ as the border segment tangent:

$$
\mathrm{AcrossBorder}_{t,k}^{aw}
=
\left[
\left|\hat{e}_{SQ}\cdot\hat{t}_{AB}\right|\le0.6
\right].
$$

Thus:

$$
\mathrm{RoadBorderFallback}_{t,k}^{aw}
=
\left[
\mathrm{ExactlyOneSemanticSide}_{t,k}^{aw}
\right]
\land
\left[
\mathrm{SameRoadSide}_{t,k}^{aw}
\right]
\land
\left[
\mathrm{Between}_{t,k}^{aw}
\right]
\land
\left[
\mathrm{AcrossBorder}_{t,k}^{aw}
\right]
\land
\left[
\mathrm{dist}\left(X_{t,k}^{aw},Q_{t,k}^{aw}\right)\le3.0
\right]
\land
\left[
\mathrm{dist}\left(\Sigma_{t,k}^{aw},Q_{t,k}^{aw}\right)\le4.0
\right].
$$

The final corner-drivable predicate is:

$$
\mathrm{CornerDrivable}_{t,k}^{aw}
=
\left[
\mathrm{SemanticDrivable}_{t,k}^{aw}
\lor
\mathrm{RoadBorderFallback}_{t,k}^{aw}
\right].
$$

The timestep is classified as non-drivable if fewer than all four ego corners are
drivable:

$$
\mathrm{NonDrivableArea}_t^{aw}
=
\left[
\sum_{k\in\mathrm{corners}}
\mathbf{1}\!\left(\mathrm{CornerDrivable}_{t,k}^{aw}\right) < 4
\right].
$$

Finally, DAC is `1` only if no timestep is non-drivable:

$$
\mathrm{DAC}_{aw}
=
\mathbf{1}
\left[
\forall t,\;\neg\mathrm{NonDrivableArea}_t^{aw}
\right].
$$

The implementation evaluates this corner-drivable state once per selected trajectory
sample and reuses the same per-timestep result for DAC scoring, DAC debug output, and
the NC lateral bad-area check, so the debug path does not introduce a separate second
DAC computation with different semantics.

If the trajectory is empty, the route handler is missing or not ready, the map is not
ready, or `VehicleInfo` cannot provide a valid footprint, the local metric is marked
unavailable rather than returning a NAVSIM-equivalent score.

**Main input gap.** NAVSIM asks whether ego corners remain inside a cached semantic
drivable-area map. The migrated Autoware code asks whether ego corners remain inside
the local-global union of nearby road lanelets, nearby `road_shoulder` lanelets,
nearby `intersection_area` polygons, nearby `hatched_road_markings` polygons, nearby
`parking_lot` polygons, and a strict per-corner closest-`road_border` side fallback.
This is closer than the previous proxy, but it is still not the same admissible map
layer set as NAVSIM because the semantic map is constructed from Autoware lanelet
entities at each sample rather than from NAVSIM's cached drivable-area layer.

### Assessment

The migrated DAC is **still not equation-identical** to NAVSIM, but it is now a
substantially closer structural imitation. The failure logic is corner-based like
NAVSIM; the remaining mismatch is mainly the admissible drivable-area source rather
than the compliance equation itself.

---

## DDC - Driving direction compliance

### Definition and semantic meaning

DDC is the **wrong-way / oncoming-traffic compliance subscore**. NAVSIM v2 introduced it to capture a traffic-rule violation that is not fully covered by collision checks alone: a trajectory can remain collision-free yet still make unsafe or illegal progress against the intended direction of travel.

Semantically, DDC measures how much meaningful forward motion the ego accumulates while driving in oncoming traffic, outside intersection leniency zones. It is therefore a graded rule-compliance metric: small incursions are penalized less than sustained wrong-way travel.

### Ownership

- **NAVSIM primary owner:** `pdm_scorer.py::_calculate_driving_direction_compliance()`
- **NAVSIM outsourced helper logic:** no dedicated helper; uses map queries on `self._drivable_area_map`
- **NAVSIM external state / cached dependency:** `self._ego_coords`, `self._ego_areas`, `self._drivable_area_map`, config thresholds
- **Migrated owner:** `src/metrics/epdms/driving_direction_compliance.cpp::calculate_driving_direction_compliance()`
- **Migrated call-site ownership:** `src/metrics/trajectory_metrics.cpp`

### Required inputs: same availability or replacement?

- **Same NAVSIM inputs available?** **Mostly yes, semantically**

| Required Input (NAVSIM) | Semantic Meaning (NAVSIM) | Autoware Replacement | Semantic Meaning (AW) | Judgement / Impact |
| :--- | :--- | :--- | :--- | :--- |
| **Oncoming Traffic Mask** | Center point not inside any on-route lane / lane-connector polygon. | Local nearby route-lane polygon membership from `RouteHandler` + raw map search | Ego center not inside any nearby route lane polygon. | **Approximate.** Preserves NAVSIM center-in-route-polygon logic, but scopes the route set locally around ego rather than over the whole mission route. |
| **Intersection Mask** | Center point inside an intersection polygon / lane connector leniency area. | Local nearby `is_intersection_lanelet()` polygon membership | Ego center inside a nearby intersection lanelet polygon. | **Approximate.** Preserves center-in-intersection logic with local lanelet polygons. |
| **Centerline Progress** | Accumulated forward distance. | `DrivingDirectionEvaluationPoint::progress_m` | Longitudinal distance along lanelet centerline. | **Equivalent.** |

- **Semantic replacement meaning:** oncoming traffic is approximated as "ego center not inside any nearby route lane polygon", and intersection is approximated as "ego center inside a nearby intersection lanelet polygon".

### Platform deviations and impact

- NAVSIM determines oncoming traffic from cached on-route lane / lane-connector polygons over the route set.
- The migrated code searches nearby lanelets around ego center and checks polygon containment only against nearby route lanelets.
- Thresholds are preserved exactly: horizon $1.0\,s$, compliance threshold $2.0\,m$, violation threshold $6.0\,m$.
- **Impact:** **Low to moderate.** The core rule is ported closely; the main difference is that the migrated implementation uses a local nearby route-lane subset instead of NAVSIM's full cached route polygon set.

### Equation comparison

#### NAVSIM DDC

**NAVSIM inputs.** Let $c_{i,t}$ be ego center position for proposal $i$ at time $t$.
Let the map-derived flags indicate whether this center point is in oncoming traffic
and whether it is in an intersection. The raw step progress is:

$$
\Delta s_{i,t}
=
\begin{cases}
\|c_{i,t}-c_{i,t-1}\|_2, & t>0 \\
0, & t=0.
\end{cases}
$$

Let:

$$
\mathrm{Oncoming}_{i,t}
=
\left[
c_{i,t}\text{ is not inside any on-route lane polygon}
\right],
$$

and:

$$
\mathrm{Intersection}_{i,t}
=
\left[
c_{i,t}\text{ is inside an intersection layer}
\right].
$$

Only non-intersection oncoming progress counts:

$$
\Delta s_{i,t}^{oncoming}
=
\begin{cases}
\Delta s_{i,t}, & \mathrm{Oncoming}_{i,t}\land\neg\mathrm{Intersection}_{i,t} \\
0, & \text{otherwise}.
\end{cases}
$$

For horizon $T_{DDC}=1.0\,s$:

$$
H_{i,t}
=
\sum_{\tau:\;0\le t-\tau\le T_{DDC}}
\Delta s_{i,\tau}^{oncoming},
\qquad
H_i^{max}=\max_t H_{i,t}.
$$

With thresholds $2.0\,m$ and $6.0\,m$:

$$
\mathrm{DDC}_{nav,i}
=
\begin{cases}
1.0, & H_i^{max}<2.0 \\
0.5, & 2.0\le H_i^{max}<6.0 \\
0.0, & H_i^{max}\ge 6.0.
\end{cases}
$$

#### Migrated Autoware DDC

**Migrated Autoware inputs.** For the selected trajectory, the implementation builds
evaluation points from trajectory poses and local map-polygon membership queries:

$$
e_t^{aw} =
\left(
t,\;\Delta s_t^{aw},\;\mathrm{Oncoming}_t^{aw},\;\mathrm{Intersection}_t^{aw}
\right),
$$

where:

$$
\Delta s_t^{aw}
=
\begin{cases}
\|p_t^{ego}-p_{t-1}^{ego}\|_2, & t>0 \\
0, & t=0,
\end{cases}
$$

where the oncoming flag is determined from local route-consistent drivable polygons
around ego center:

$$
\mathrm{Oncoming}_t^{aw}
=
\neg\mathrm{isPoseInRouteLanePolygon}(\mathrm{pose}_t,\mathrm{RouteHandler}),
$$

and the intersection flag is determined from nearby `intersection_area` polygons:

$$
\mathrm{Intersection}_t^{aw}
=
\mathrm{isPoseInIntersection}(\mathrm{pose}_t,\mathrm{RouteHandler}).
$$

Here, `isPoseInRouteLanePolygon(...)` searches a local region around the ego center,
starts from nearby route lanelets, expands to same-direction neighboring lanes, includes
adjacent shoulder lanelets when present, and returns true iff the ego center is covered by
at least one of those local admissible polygons. Likewise, `isPoseInIntersection(...)`
searches nearby map polygons whose type is `intersection_area` and returns true iff the
ego center is covered by one of those intersection-area polygons.

More concretely, let:

$$
\mathcal{L}_t^{route,seed,aw}
=
\mathrm{RouteLaneSeedsAtPose}(\mathrm{pose}_t),
$$

where `RouteLaneSeedsAtPose(...)` means the union of:

- route lanelets returned by `getRoadLaneletsAtPose(...)`
- the closest lanelet within route, if available

Let:

$$
\mathcal{L}_t^{road,near,aw}
=
\mathrm{NearbyRoadLanelets}(\mathrm{pose}_t),
\qquad
\mathcal{L}_t^{shoulder,near,aw}
=
\mathrm{NearbyShoulderLanelets}(\mathrm{pose}_t).
$$

The migrated implementation then uses the lane direction at the first route seed as a
local reference heading and keeps nearby polygons whose lane direction is sufficiently
aligned with that heading. The admissible not-oncoming lane set is therefore:

$$
\mathcal{L}_t^{ddc,aw}
=
\mathcal{L}_t^{road,route,aw}
\cup
\mathcal{L}_t^{road,same\_dir,aw}
\cup
\mathcal{L}_t^{shoulder,same\_dir,aw},
$$

where:

- $\mathcal{L}_t^{road,route,aw}$ are nearby road lanelets that `RouteHandler`
  already classifies as route lanelets
- $\mathcal{L}_t^{road,same\_dir,aw}$ are nearby non-route road lanelets whose
  local lane angle is close to the route-seed direction
- $\mathcal{L}_t^{shoulder,same\_dir,aw}$ are nearby `road_shoulder` lanelets whose
  local lane angle is close to the route-seed direction

Define:

$$
\mathrm{InAdmissibleLaneSet}_t^{aw}
=
\left[
\sum_{\ell \in \mathcal{L}_t^{ddc,aw}}
\mathbf{1}\left(p_t^{ego} \in \mathrm{polygon}(\ell)\right)
> 0
\right].
$$

Let the migrated DDC soft lane margin be:

$$
\varepsilon_{ddc}^{lane} = 0.35 \text{ m}.
$$

Define:

$$
\mathrm{NearAdmissibleLaneSet}_t^{aw}
=
\left[
\min_{\ell \in \mathcal{L}_t^{ddc,aw}}
\mathrm{dist}\left(p_t^{ego}, \mathrm{polygon}(\ell)\right)
\le
\varepsilon_{ddc}^{lane}
\right].
$$

Then:

$$
\mathrm{Oncoming}_t^{aw}
=
\neg
\left(
\mathrm{InAdmissibleLaneSet}_t^{aw}
\lor
\mathrm{NearAdmissibleLaneSet}_t^{aw}
\right).
$$

For intersection leniency, let:

$$
\mathcal{P}_t^{intersection,aw}
=
\mathrm{NearbyIntersectionAreaPolygons}(\mathrm{pose}_t),
$$

where the polygon type is `intersection_area`. Then:

$$
\mathrm{Intersection}_t^{aw}
=
\left[
\sum_{q \in \mathcal{P}_t^{intersection,aw}}
\mathbf{1}\left(p_t^{ego} \in \mathrm{polygon}(q)\right)
> 0
\right].
$$

**Currently included space.** The DDC admissible set currently includes:

- nearby on-route road lanelets
- nearby same-direction road lanelets, even if they are not tagged as route lanelets
- nearby same-direction `road_shoulder` lanelets
- a narrow `0.35 m` soft margin around those admitted road/shoulder polygons, so abrupt
  same-direction lane-crossing and boundary-touching cases are not over-penalized
- nearby `intersection_area` polygons for intersection leniency

**Currently excluded space.** The DDC admissible set currently does **not** include:

- wide painted separator / striped in-between space beyond the `0.35 m` soft margin, unless it is
  explicitly encoded by the map as an admitted polygon or shoulder lanelet
- arbitrary road-border-adjacent space from `road_border` lines alone
- opposite-direction lanelets, even if spatially close

Then:

$$
\Delta s_t^{aw,oncoming}
=
\begin{cases}
\max(0,\Delta s_t^{aw}), &
\mathrm{Oncoming}_t^{aw}\land\neg\mathrm{Intersection}_t^{aw} \\
0, & \text{otherwise},
\end{cases}
$$

$$
H_t^{aw}
=
\sum_{\tau:\;0\le t-\tau\le 1.0}
\Delta s_\tau^{aw,oncoming},
\qquad
H_{aw}^{max}=\max_t H_t^{aw}.
$$

The score uses the same thresholds:

$$
\mathrm{DDC}_{aw}
=
\begin{cases}
1.0, & H_{aw}^{max}<2.0 \\
0.5, & 2.0\le H_{aw}^{max}<6.0 \\
0.0, & H_{aw}^{max}\ge 6.0.
\end{cases}
$$

**Main input gap.** The thresholding shape is close to NAVSIM, but the migrated
implementation still uses local Autoware map queries around the selected trajectory poses
instead of NAVSIM's cached global polygon-index arrays. The semantic shape now matches
NAVSIM more closely because both `Oncoming` and `Intersection` are center-point membership
tests against polygonal admissible areas.

### Assessment

DDC is one of the **closest ports** in the migration. The math is the same; the remaining
deviation is mainly how the local Autoware admissible polygons are assembled around ego.

---

## TLC - Traffic light compliance

### Definition and semantic meaning

TLC is the **traffic-signal rule-compliance subscore**. In NAVSIM semantics, it penalizes trajectories that violate a red-light constraint, complementing collision and map metrics with an explicit traffic-control rule.

Semantically, TLC answers "did the ego commit a red-light violation?" rather than "did the ego collide?" This matters because a planner can violate traffic control even in the absence of a collision, and EPDMS treats that as multiplicatively disqualifying unsafe behavior.

### Ownership

- **NAVSIM primary owner:** `pdm_scorer.py::_calculate_traffic_light_compliance()`
- **NAVSIM outsourced helper logic:** no dedicated helper
- **NAVSIM external state / cached dependency:** `self._observation`, `self._ego_polygons`, `self._observation.red_light_token`
- **Migrated owner:** `src/metrics/epdms/traffic_light_compliance.cpp::calculate_traffic_light_compliance()`
- **Migrated local helpers:** `find_signal_group()`, `build_controlled_traffic_light_groups()`

### Required inputs: same availability or replacement?

- **Same NAVSIM inputs available?** **No**

| Required Input (NAVSIM) | Semantic Meaning (NAVSIM) | Autoware Replacement | Semantic Meaning (AW) | Judgement / Impact |
| :--- | :--- | :--- | :--- | :--- |
| **Red-light Tokens** | Objects representing a "locked" red light area. | `autoware_perception_msgs::msg::TrafficLightGroupArray` + route lanelets carrying the same `AutowareTrafficLight` regulatory element | Signal states tied to route-local movement-compatible stop lines. | **Moderate.** Still map-driven rather than observation-driven, and uses stop-line crossing rather than red pseudo-object polygons. |
| **Simulated Ego Polygons** | Footprint overlap with the red light area. | `autoware_planning_msgs::msg::Trajectory` + `VehicleInfo` | Ego footprint relative to the stop line. | **Equivalent.** |

- **Semantic replacement meaning:** the migrated code now selects route-local movement-compatible stop lines from route lanelets that share a traffic-light regulatory element and checks ego-stop-line crossing under stop/red.

### Platform deviations and impact

- NAVSIM detects violation when an ego polygon intersects an observation object whose token starts with `red_light_token`.
- The migrated code detects violation when the ego footprint intersects a movement-compatible route stop line whose traffic-light regulatory element is currently stop/red for that movement.
- The migrated code can still become unavailable when a signal group is missing exactly at a selected stop-line crossing opportunity; NAVSIM's scoring path does not have this exact availability mode because red-light occupancy is already encoded in the observation.
- **Impact:** **Moderate.** Same legal intent, but the migrated score uses stop-line crossing instead of red pseudo-object polygon overlap.

### Equation comparison

#### NAVSIM TLC

**NAVSIM inputs.** Let $R_t$ be the set of red-light pseudo-object polygons in the
observation at time $t$. Each red-light pseudo-object is a red-controlled on-route
lane connector polygon with token prefix `red_light`. Let $P_{i,t}$ be the simulated
ego footprint polygon.

For proposal $i$:

$$
\mathrm{RedIntersect}_{i,t}
=
\left[
\exists r\in R_t:\;\mathrm{Overlap}(P_{i,t}, r)
\right].
$$

The score is:

$$
\mathrm{TLC}_{nav,i}
=
\mathbf{1}
\left[
\forall t,\;\neg\mathrm{RedIntersect}_{i,t}
\right].
$$

#### Migrated Autoware TLC

**Migrated Autoware inputs.** The migrated implementation uses:

- route lanelets collected from the selected trajectory with `getRoadLaneletsAtPose()` and `isRouteLanelet()`
- `AutowareTrafficLight` regulatory elements attached to those route lanelets
- the current `TrafficLightGroupArray`
- the synchronized `TurnIndicatorsReport` when available
- the selected-trajectory ego footprint

For each traffic-light regulatory element `g`, the migrated implementation first
infers the intended movement `m(g)` in this order:

- turn indicator if it is currently left or right
- otherwise, the unique `turn_direction` found in the route lanelets under `g`
- otherwise, no explicit movement filter

Each regulatory element `g` also provides a stop line `s_g`. The migrated score uses
that stop line as the violation primitive.

For each trajectory sample `t`, let `P_t^{aw}` be the ego footprint polygon and let
`S_g` be the matching traffic-light signal group for regulatory element `g`.

Each route lanelet `\ell` under regulatory element `g` is considered only if it is
movement-compatible with `m(g)`. Such a lanelet is stop-controlled if:

$$
\mathrm{StopRequired}_{t,g,\ell}^{aw}
=
\mathrm{isTrafficSignalStop}(\ell, S_g).
$$

Regulatory element `g` is active stop/red at time `t` if any selected lanelet under
that movement requires stop:

$$
\mathrm{ActiveStopLine}_{t,g}^{aw}
=
\left[
\exists \ell \in \mathrm{SelectedLanelets}(g, m(g)):\;
\mathrm{StopRequired}_{t,g,\ell}^{aw}
\right].
$$

The migrated violation primitive is stop-line crossing:

$$
\mathrm{StopLineCrossed}_{t,g}^{aw}
=
\mathrm{ActiveStopLine}_{t,g}^{aw}
\land
\mathrm{Overlap}(P_t^{aw}, s_g).
$$

A TLC violation occurs if any relevant active stop line is crossed:

$$
\mathrm{RedViolation}_t^{aw}
=
\left[
\exists g:\;
\mathrm{StopLineCrossed}_{t,g}^{aw}
\right].
$$

Thus:

$$
\mathrm{TLC}_{aw}
=
\mathbf{1}
\left[
\forall t,\;\neg\mathrm{RedViolation}_t^{aw}
\right].
$$

Once a valid red-light stop-line crossing is found, the scorer stops checking later
horizon samples because the final TLC score is already `0`. The full ego horizon is
still retained for debug context.

Signal-group association is attempted in this order:

- direct match between `traffic_light_group_id` and the traffic-light regulatory-element id
- otherwise, fallback match against the selected movement-compatible route lanelet ids
- otherwise, fallback match against the broader route lanelet ids under the same regulatory
  element

If a relevant route traffic light still has no matching signal group after that association,
the local metric is marked unavailable only when the ego actually reaches that selected stop
line and the signal is needed to judge compliance.

**Lane / area inclusion detail.**

- Included:
  - route lanelets collected from selected-trajectory poses to infer which traffic
    lights are relevant
  - movement-compatible route lanelets under each relevant traffic-light regulatory
    element
  - the stop line attached to that relevant regulatory element
  - turn-indicator-assisted movement selection when the indicator is left or right
- Excluded:
  - polygon-overlap scoring against lane or connector polygons
  - adjacent incompatible movements, such as straight-red semantics while the ego is
    taking a right-turn-green-arrow movement
  - unrelated nearby non-route lanelets and their stop lines
  - traffic lights not attached to the route lane set
  - non-route intersection polygons and generic drivable-area polygons

**Main input gap.** NAVSIM's TLC checks overlap with red-light pseudo objects already
constructed in the observation. The migrated Autoware implementation reconstructs the
relevant red stop lines from route lanelets, turn-direction filtering, turn indicators,
and traffic-light regulatory elements. This is less NAVSIM-like geometrically, but more
stable in the current Autoware map and signal stack.

### Assessment

TLC is **less NAVSIM-like geometrically**, because the migrated score uses stop-line
crossing instead of red pseudo-object polygon overlap. It is currently the more robust
Autoware-side design because it avoids false `0` scores from unrelated straight/turn
movement polygons sharing the same traffic-light regulatory element.

---

## EP - Ego progress

### Definition and semantic meaning

EP is the **goal-directed efficiency / progress subscore**. In NAVSIM semantics, it measures how much useful forward progress a trajectory makes along the intended route centerline, relative to alternative proposals, instead of merely comparing against the human demonstration by displacement error.

Semantically, EP represents the "drive somewhere useful" part of PDMS/EPDMS. It prevents overly conservative behaviors from looking good simply because they avoid infractions: a safe trajectory that makes no route progress should not receive a top planning-quality score.

### Ownership

- **NAVSIM primary owner:** `pdm_scorer.py::_calculate_progress()`
- **NAVSIM outsourced helper logic:** no dedicated helper; uses `self._centerline.project(...)`
- **NAVSIM external state / cached dependency:** `self._ego_coords`, `self._centerline`, proposal batch state
- **Migrated owner:** `src/metrics/epdms/ego_progress.cpp::calculate_ego_progress()`
- **Migrated local helpers:** `calculate_raw_progress_m()`, `collect_route_relevant_lanelets()`

### Required inputs: same availability or replacement?

- **Same NAVSIM inputs available?** **No**

| Required Input (NAVSIM) | Semantic Meaning (NAVSIM) | Autoware Replacement | Semantic Meaning (AW) | Judgement / Impact |
| :--- | :--- | :--- | :--- | :--- |
| **Simulator Proposal Batch** | Standardized set of alternative paths for the car. | The same single evaluated Autoware trajectory used by NC/DAC/DDC/TLC. | Current selected planner output, treated as a one-proposal batch. | **Limited.** This makes the equation faithful for one proposal, but EP has no proposal-relative discrimination until full candidate-batch scoring is added. |
| **Centerline Map** | Global reference for measuring distance to goal. | `autoware::route_handler::RouteHandler` centerlines | Sequential lanelet centerlines in the active route. | **Equivalent.** |
| **Multiplicative validity** | Candidate-level $\mathrm{NC}_j\mathrm{DAC}_j\mathrm{DDC}_j\mathrm{TLC}_j$. | The already-computed selected-trajectory NC/DAC/DDC/TLC values. | Same selected-trajectory validity terms. | **Single-proposal equivalent.** Full NAVSIM equivalence still requires computing these terms for every candidate in the proposal batch. |

- **Semantic replacement meaning:** the migration uses the same evaluated Autoware trajectory as the one available proposal. This intentionally keeps EP aligned with the trajectory source used by the other subscores and leaves full candidate-batch EP as later work.

### Platform deviations and impact

- NAVSIM normalizes raw progress using the best **multiplicatively valid** proposal:
  $$
  \max_j \left(p_j \cdot M_j\right),
  \quad
  M_j = \mathrm{NC}_j \mathrm{DAC}_j \mathrm{DDC}_j \mathrm{TLC}_j.
  $$
- The migrated code now applies NAVSIM's multiplicative denominator form to the single evaluated trajectory:
  $$
  C_{EP}^{aw}=p^{aw}\mathrm{NC}^{aw}\mathrm{DAC}^{aw}\mathrm{DDC}^{aw}\mathrm{TLC}^{aw}.
  $$
- Because there is only one proposal in this mode, the score is expected to be `1.0` whenever EP is available: if the denominator is above $5.0\,m$, the ratio clips to one, and if it is at or below $5.0\,m$, NAVSIM's fallback also returns one.
- **Impact:** **Medium.** The formula is faithful for the current single evaluated trajectory source, but EP remains intentionally non-discriminative until full candidate-batch scoring is added.

### Equation comparison

#### NAVSIM EP

**NAVSIM inputs.** Let $c_{i,0}$ and $c_{i,T}$ be proposal $i$'s start and end ego
center positions. Let $s_i^{start}$ and $s_i^{end}$ be their projection arc lengths
on NAVSIM's cached centerline:

$$
s_i^{start}
=
\mathrm{project}_{centerline}(c_{i,0}),
\qquad
s_i^{end}
=
\mathrm{project}_{centerline}(c_{i,T}).
$$

Raw progress:

$$
p_i=\max(s_i^{end}-s_i^{start},0).
$$

Let:

$$
M_i = \mathrm{NC}_i\mathrm{DAC}_i\mathrm{DDC}_i\mathrm{TLC}_i.
$$

The progress denominator is the best multiplicatively-valid progress:

$$
C_{EP}=\max_j(p_jM_j).
$$

With NAVSIM threshold $\tau_{EP}=5.0\,m$:

$$
\mathrm{EP}_{nav,i}
=
\begin{cases}
\mathrm{clip}\left(\dfrac{p_i}{C_{EP}},0,1\right), & C_{EP}>\tau_{EP} \\
1.0, & C_{EP}\le\tau_{EP}.
\end{cases}
$$

#### Migrated Autoware EP

**Migrated Autoware inputs.** The migrated implementation receives the same selected
Autoware trajectory $q$ used by the other subscores. Route-lanelet progress is:

$$
p(q)
=
\max\left(
\mathrm{arc}_{route}(q_{end})
-
\mathrm{arc}_{route}(q_{start}),
0
\right).
$$

The selected trajectory's multiplicative validity is:

$$
M^{aw}=\mathrm{NC}^{aw}\mathrm{DAC}^{aw}\mathrm{DDC}^{aw}\mathrm{TLC}^{aw}.
$$

The single-proposal denominator is:

$$
C_{EP}^{aw}=p(q)M^{aw}.
$$

With the same threshold $5.0\,m$:

$$
\mathrm{EP}_{aw}
=
\begin{cases}
\mathrm{clip}\left(\dfrac{p(q)}{C_{EP}^{aw}},0,1\right), &
C_{EP}^{aw}>5.0 \\
1.0, & C_{EP}^{aw}\le 5.0.
\end{cases}
$$

The second branch uses NAVSIM's low-progress fallback. In this single-proposal mode,
this makes EP equal to `1.0` whenever the route projection and multiplicative inputs
are available.

**Main input gap.** NAVSIM normalizes by the best progress among all proposals that also
survive multiplicative subscores. Migrated Autoware currently uses one evaluated
proposal only. Later full fidelity requires evaluating NC/DAC/DDC/TLC for every
candidate in `/diffusion_planner/output/trajectories` and using
$\max_j(p_jM_j)$ as the denominator.

### Assessment

EP is a **material migration deviation**. The local score is proposal-relative, but not NAVSIM-proposal-relative in the same way.

---

## TTC - Time to collision within bound

### Definition and semantic meaning

TTC is the **proactive near-collision safety subscore**. In NAVSIM semantics, it looks ahead over a short future horizon and asks whether the current planned motion will place the ego in an imminent collision situation, even before an actual contact occurs.

Semantically, TTC is an anticipatory safety measure: whereas NC penalizes realized at-fault collisions, TTC penalizes trajectories that are already on a collision course within the bounded lookahead. This lets PDMS/EPDMS down-score dangerous plans before physical overlap occurs.

### Ownership

- **NAVSIM primary owner:** `pdm_scorer.py::_calculate_ttc()`
- **NAVSIM outsourced helper logic:** nuPlan helpers `is_agent_ahead`, `is_agent_behind`; shapely polygon creation
- **NAVSIM external state / cached dependency:** `self._observation`, `self._ego_coords`, `self._ego_areas`, `self._states`, `self._drivable_area_map`, `self._ttc_time_idcs`
- **Migrated owner:** `src/metrics/epdms/ttc_within_bound.cpp::calculate_ttc_within_bound()`
- **Migrated local helpers:** `build_logged_object_tracks()`, `interpolate_logged_object_state()`, `project_pose()`, `is_agent_ahead()`, `is_agent_behind()`

### Required inputs: same availability or replacement?

- **Same NAVSIM inputs available?** **No**

| Required Input (NAVSIM) | Semantic Meaning (NAVSIM) | Autoware Replacement | Semantic Meaning (AW) | Judgement / Impact |
| :--- | :--- | :--- | :--- | :--- |
| **Reactive Observation States** | Future object positions (Non-reactive; following logs). | `autoware_perception_msgs::msg::TrackedObjects` from `/perception/object_recognition/tracking/objects`; keyed by object UUID and interpolated at each TTC query time. Objects whose highest-probability label is `UNKNOWN` are excluded before TTC overlap testing. | Recorded tracked-object state sequence from the bag, with an Autoware-specific artifact guard. | **Close with artifact filter.** This avoids `PredictedObjects.predicted_paths` and uses recorded future tracked states, making the object source closer to NAVSIM's logged observation rollout. The `UNKNOWN` exclusion suppresses x2_odaiba tiny tracking artifacts but can ignore a real unclassified obstacle. |
| **Simulated Ego States** | Ego's projected kinematic rollout. | `autoware_planning_msgs::msg::Trajectory` (using `project_pose`) | Planning's kinematic intent. | **Equivalent.** Both use forward projections for lookahead. |
| **Drivable Area Map** | Used for "bad area" exception (off-road/intersections). | `autoware::route_handler::RouteHandler` (Intersection check) | Map-based identification of junctions. | **Low.** The migrated code simplifies this, dropping the "bad area" check. |

- **Semantic replacement meaning:** NAVSIM evaluates future collision against the simulated environment rollout; the migration evaluates it against recorded tracked-object states from the bag.
- **Autoware object-validity guard:** TTC uses the same `UNKNOWN` tracked-object exclusion as
  NC. This is not a NAVSIM semantic rule; it compensates for Autoware bags where tiny
  short-lived `UNKNOWN` polygon tracks otherwise dominate TTC failures.

### Platform deviations and impact

- Both implementations project ego forward under constant velocity over roughly the same horizon.
- NAVSIM also allows the "bad area" exception branch:
  `multiple lanes OR non-drivable area OR intersection`.
- The migrated code only keeps the `intersection` branch; it drops the `multiple-lanes/non-drivable-area` part from TTC itself.
- NAVSIM samples future indices from the scorer time grid; the migrated code hardcodes $(0.0, 0.3, 0.6, 0.9)$ seconds.
- **Impact:** **Moderate to high.** The shape of the rule is similar, but the scene source and one logical branch differ.

### Equation comparison

#### NAVSIM TTC

**NAVSIM inputs.** NAVSIM evaluates one proposal at one scorer timestep. The checked
future offsets are:

$$
\delta \in (0, 0.3, 0.6, 0.9).
$$

NAVSIM forward-projects ego under current velocity to a future ego polygon and
compares it with observation objects at the matching future query time. The objects
are the non-reactive scenario future tracked objects, interpolated to the scorer grid.

Let:

$$
\mathrm{TTCIntersect}_{i,t,\delta,o}
=
\left[
\mathrm{Overlap}(P_{i,t,\delta}^{proj}, O_{t+\delta,o})
\right].
$$

NAVSIM ignores red-light tokens, previously collided tokens, and stopped ego states.
For remaining intersections, define:

$$
\mathrm{BadOrIntersection}_{i,t}
=
\mathrm{MultipleLanes}_{i,t}
\lor
\mathrm{NonDrivableArea}_{i,t}
\lor
\mathrm{Intersection}_{i,t}.
$$

Let the checked-state ahead and behind predicates be the nuPlan relative-angle tests.
TTC fails if:

$$
\mathrm{TTCFail}_{i,t,\delta,o}^{nav}
=
\mathrm{TTCIntersect}_{i,t,\delta,o}
\land
\left[
\mathrm{Ahead}_{i,t,\delta,o}
\lor
\left(
\mathrm{BadOrIntersection}_{i,t}
\land
\neg\mathrm{Behind}_{i,t,\delta,o}
\right)
\right].
$$

The score is:

$$
\mathrm{TTC}_{nav,i}
=
1-
\mathbf{1}
\left[
\exists t,\delta,o:\;
\mathrm{TTCFail}_{i,t,\delta,o}^{nav}
\right].
$$

#### Migrated Autoware TTC

**Migrated Autoware inputs.** The migrated implementation uses the selected
trajectory, logged future objects reconstructed from `future_objects`, and
constant-velocity projection of ego. The checked future offsets are:

$$
\delta \in (0, 0.3, 0.6, 0.9).
$$

The migrated implementation projects the current ego footprint forward to the checked
future offset. Object pose is interpolated from the logged future object track at the
matching future query time, yielding the queried object polygon.

Intersection:

$$
\mathrm{TTCIntersect}_{t,\delta,o}^{aw}
=
\left[
\mathrm{Overlap}(P_{t,\delta}^{aw,proj}, O_{t+\delta,o}^{aw})
\right].
$$

The migrated ahead and behind predicates are the same nuPlan-style relative-angle
tests as NAVSIM, evaluated at the checked projected ego pose and queried object pose:

$$
\mathrm{Ahead}_{t,\delta,o}^{aw}
=
\left[\theta_{t,\delta,o}^{aw}<30^{\circ}\right],
\qquad
\mathrm{Behind}_{t,\delta,o}^{aw}
=
\left[\theta_{t,\delta,o}^{aw}>150^{\circ}\right].
$$

The migrated implementation also reuses the current-sample ego-area and route context:

$$
\mathrm{BadOrIntersection}_{t}^{aw}
=
\mathrm{MultipleLanes}_{t}^{aw}
\lor
\mathrm{NonDrivableArea}_{t}^{aw}
\lor
\mathrm{Intersection}_{t}^{aw}.
$$

The local TTC fail condition is:

$$
\mathrm{TTCFail}_{t,\delta,o}^{aw}
=
\mathrm{TTCIntersect}_{t,\delta,o}^{aw}
\land
\left[
\mathrm{Ahead}_{t,\delta,o}^{aw}
\lor
\left(
\mathrm{BadOrIntersection}_{t}^{aw}
\land
\neg\mathrm{Behind}_{t,\delta,o}^{aw}
\right)
\right].
$$

Then:

$$
\mathrm{TTC}_{aw}
=
1-
\mathbf{1}
\left[
\exists t,\delta,o:\;
\mathrm{TTCFail}_{t,\delta,o}^{aw}
\right].
$$

The migrated implementation also suppresses previously collided logged objects in later
TTC checks, matching the NAVSIM scorer shape more closely. It does **not** add
NAVSIM's red-light-token exclusion because the Autoware-side TTC object stream has no
red-light pseudo objects to exclude.

**Admitted / excluded road-space semantics.** TTC reuses the current selected-trajectory
sample's shared ego-area and route context:

- Included in `BadOrIntersection_t^{aw}`:
  - `MultipleLanes_t^{aw}` from the shared current-sample ego footprint evaluation
  - `NonDrivableArea_t^{aw}` from the same shared ego footprint evaluation
  - `Intersection_t^{aw}` from the local `intersection_area` / intersection-lanelet
    context already used by DDC
- Excluded:
  - red-light pseudo-token suppression
  - any future-offset-specific reclassification of multiple-lane or non-drivable status;
    those flags are taken from the current sample `t`, matching the NAVSIM equation

### Assessment
TTC is now a **close structural port**. The object source, angle predicates,
`BadOrIntersection` branch, and previously-collided-object suppression now follow the
NAVSIM rule shape closely. The remaining difference is mainly the absence of NAVSIM's
red-light pseudo tokens in the Autoware-side TTC object stream.

---

## LK - Lane keeping

### Definition and semantic meaning

LK is the **route-following lateral-discipline subscore**. NAVSIM v2 introduced it to penalize trajectories that drift too far from the route centerline for a sustained period, while deliberately relaxing the metric in intersections where centerline annotations are less reliable.

Semantically, LK captures a stability / lane-discipline property rather than a hard legality constraint. It reflects whether the planner stays consistently aligned with the intended lane corridor, without over-penalizing brief or intersection-related deviations.

### Ownership

- **NAVSIM primary owner:** `pdm_scorer.py::_calculate_lane_keeping()`
- **NAVSIM outsourced helper logic:** no dedicated helper; uses centerline geometry and map queries
- **NAVSIM external state / cached dependency:** `self._ego_coords`, `self._centerline`, `self._drivable_area_map`, config thresholds
- **Migrated owner:** `src/metrics/epdms/lane_keeping.cpp::calculate_lane_keeping_score()`
- **Migrated call-site ownership:** `src/metrics/trajectory_metrics.cpp`

### Required inputs: same availability or replacement?

- **Same NAVSIM inputs available?** **Close, but not identical**

| Required Input (NAVSIM) | Semantic Meaning (NAVSIM) | Autoware Replacement | Semantic Meaning (AW) | Judgement / Impact |
| :--- | :--- | :--- | :--- | :--- |
| **Cached Centerline** | Geometric target used for lateral deviation checks. | `autoware::route_handler::RouteHandler` centerlines | Center of the current road lanelet. | **Low.** Lanelet geometry is highly similar to NAVSIM centerlines. |
| **Ego Center Positions** | Kinematic rollout center-of-gravity. | `autoware_planning_msgs::msg::Trajectory` (x,y points) | Ego vehicle's planned center path. | **Equivalent.** |
| **Intersection Mask** | Used to relax LK scoring in junctions. | local `intersection_area` / route-intersection context | Point-in-intersection-area check, with route-intersection lanelets as fallback. | **Low.** Closer to the local intersection semantics already used by DDC/TTC. |

- **Semantic replacement meaning:** the lanelet centerline stands in for NAVSIM's cached `PDMPath` centerline.

### Platform deviations and impact

- Thresholds intentionally differ slightly from NAVSIM: the migrated deviation limit is relaxed
  from NAVSIM's $0.5\,m$ to $0.6\,m$ to reduce over-penalization from Autoware lanelet
  centerline placement, while the continuous violation duration remains $2.0\,s$.
- NAVSIM measures distance from the ego center to `self._centerline.linestring`.
- The migrated code measures distance to the route/reference lanelet centerline selected by `RouteHandler`.
- Intersection relaxation now reuses the shared local intersection context already used by DDC:
  local `intersection_area` polygons first, then route-intersection lanelets as fallback.
- The migrated code now also suppresses LK accumulation during:
  - explicit turn-indicator or hazard-light signal windows, expanded by `1.0 s` before signal
    activation and `1.0 s` after signal deactivation
  - low-speed / low-progress queue states
  - a short release-hysteresis window after queue motion resumes
- Missing reference lanelets become unavailable in the migrated code rather than silently behaving like NAVSIM's always-available cached centerline path.
- **Impact:** **Low to moderate.** The core rule is ported closely; deviations come from centerline selection and availability.

### Equation comparison

#### NAVSIM LK

**NAVSIM inputs.** Let $d_{i,t}$ be the distance from ego center $c_{i,t}$ to the
cached route centerline, and let $\mathrm{Intersection}_{i,t}$ indicate whether the
ego center is in an intersection layer.

Only non-intersection samples count:

$$
\mathrm{LKViolationSample}_{i,t}
=
\neg\mathrm{Intersection}_{i,t}
\land
\left[d_{i,t}>0.5\right].
$$

Let $\mathcal{R}_{i}$ be all consecutive runs of `LKViolationSample=true`. If
$\mathrm{duration}(r)$ is the duration of run $r$, then:

$$
\mathrm{LK}_{nav,i}
=
\mathbf{1}
\left[
\forall r\in\mathcal{R}_{i},\;
\mathrm{duration}(r)<2.0\,s
\right].
$$

#### Migrated Autoware LK

**Migrated Autoware inputs.** For each selected trajectory sample $t$, the
implementation chooses a reference route lanelet and computes lateral distance to
that lanelet centerline:

$$
d_t^{aw}
=
\mathrm{lateralDistanceToCenterline}
\left(\mathrm{referenceLanelet}_t,\mathrm{pose}_t\right).
$$

Let:

$$
\mathrm{Intersection}_t^{aw}
=
\mathrm{InIntersectionAreaOrRouteIntersectionLanelet}(\mathrm{pose}_t).
$$

Let $\mathcal{S}^{aw}$ be the union of explicit driver-intent signal intervals where
the turn indicator is left/right or the hazard light is enabled. Each interval is
expanded by one second before activation and one second after deactivation:

$$
\mathrm{LaneChangeExempt}_{t}^{aw}
=
\mathbf{1}
\left[
t \in
\bigcup_{[a,b]\in\mathcal{S}^{aw}}
[a-1.0,\;b+1.0]
\right].
$$

Then:

$$
\mathrm{LKViolationSample}_{t}^{aw}
=
\neg\mathrm{LaneChangeExempt}_{t}^{aw}
\land
\neg\mathrm{QueueExempt}_{t}^{aw}
\land
\neg\mathrm{QueueReleaseExempt}_{t}^{aw}
\land
\neg\mathrm{Intersection}_t^{aw}
\land
\left[|d_t^{aw}|>0.6\right].
$$

With the same continuous violation duration threshold:

$$
\mathrm{LK}_{aw}
=
\mathbf{1}
\left[
\forall r\in\mathcal{R}^{aw},\;
\mathrm{duration}(r)<2.0\,s
\right].
$$

Invalid or missing reference-lanelet samples reset the violation run in the migrated
implementation. If no finite lane-keeping sample exists at all, the metric is marked
unavailable instead of returning a binary score.

**Admitted / excluded lane-space semantics.** LK intentionally keeps the Autoware-side
per-sample centerline logic rather than NAVSIM's single cached route centerline:

- Included for centerline selection:
  - the per-sample `referenceLanelet_t` returned by the route handler
  - that lanelet's geometric centerline for `d_t^{aw}`
- Included for intersection relaxation:
  - local `intersection_area` polygons around the sample pose
  - route-intersection lanelets as fallback when no polygon directly contains the pose
- Included for lane-change relaxation:
  - turn-indicator active intervals (`ENABLE_LEFT` or `ENABLE_RIGHT`)
  - hazard-light active intervals (`ENABLE`)
  - fixed `1.0 s` pre/post grace around those explicit signal-active intervals
- Included for queue / constrained-traffic relaxation:
  - low-speed, low-progress samples
  - a short release-grace window immediately after that queue state ends
- Excluded:
  - a single globally cached route centerline shared across the whole rollout
  - geometry-only lane-change inference from `multiple_lanes` or reference-lanelet switching
  - GT-derived lane-change inference
  - a blanket queue exemption without low-speed / low-progress evidence
  - road-border or generic drivable-surface relaxation; LK remains a centerline-discipline
    metric rather than a DAC-style drivable-area metric
  - non-intersection resets other than missing / non-finite reference-lanelet samples

**Main input gap.** The score shape is close, but NAVSIM measures against its cached
centerline and intersection layers, while migrated Autoware measures against a
per-sample reference lanelet chosen from the route handler.

### Assessment

LK is a **close port**. The rule is the same; the measured centerline signal is the only meaningful platform substitution.

---

## HC - History comfort

### Definition and semantic meaning

HC is the **single-trajectory comfort-consistency subscore with motion-history awareness**. Relative to the earlier NAVSIM comfort metric, NAVSIM v2 defines History Comfort to check not only whether the planned rollout is dynamically comfortable on its own, but also whether it remains compatible with the vehicle's recent motion history.

Semantically, HC captures ride quality and control smoothness across the transition from past executed motion into the newly planned future. It is therefore meant to penalize abrupt accelerations, jerks, or yaw dynamics that would feel uncomfortable or dynamically implausible when continuing from the recent driving history.

### Ownership

- **NAVSIM primary owner:** `pdm_scorer.py::_calculate_history_comfort()`
- **NAVSIM outsourced helper logic:** `pdm_comfort_metrics.py::ego_is_comfortable()`, `ego_states_to_state_array(...)`
- **NAVSIM external state / cached dependency:** `self._human_past_trajectory`, `self._states`, `proposal_sampling.interval_length`
- **Migrated owner:** `src/metrics/epdms/history_comfort.cpp::calculate_history_comfort_metrics()`
- **Migrated storage owner:** `src/metrics/trajectory_metrics.cpp` and `src/open_loop_evaluator.cpp`

### Required inputs: same availability or replacement?

- **Same NAVSIM inputs available?** **Mostly yes**

| Required Input (NAVSIM) | Semantic Meaning (NAVSIM) | Autoware Replacement | Semantic Meaning (AW) | Judgement / Impact |
| :--- | :--- | :--- | :--- | :--- |
| **Human Past Trajectory** | Car's real history used for history-padded comfort. | `/localization/kinematic_state` plus `/localization/acceleration` history. | Recorded ego kinematic history before the trajectory stamp. | **Close.** Autoware now pads the proposal with a `1.5s` past history window sampled at `0.1s`. |
| **Simulated Proposal States** | The car's planned future rollout. | `autoware_planning_msgs::msg::Trajectory` | The planner's output points, including pose, velocity, and `acceleration_mps2`. | **Equivalent with platform field mapping.** |
| **Interval Length** | Standardized time-step for derivative extraction. | `HistoryComfortParameters::sample_interval_s` | NAVSIM-style `0.1s` padded-sequence sampling. | **Equivalent.** |

- **Semantic replacement meaning:** the migrated code now computes comfort from a padded
  sequence of recorded past ego kinematics and the current planned trajectory.

### Platform deviations and impact

- NAVSIM pads the planned rollout with human past states before evaluating comfort.
- The migrated Autoware code now pads the planned rollout with recorded ego kinematics before
  evaluating comfort.
- NAVSIM uses Savitzky-Golay smoothing and derivatives. The migrated code uses local
  polynomial regression with the same window and polynomial orders to avoid an extra runtime
  dependency while preserving the same smoothing/derivative semantics.
- Planned-horizon longitudinal acceleration is taken from `TrajectoryPoint::acceleration_mps2`.
  Past-horizon acceleration is taken from `/localization/acceleration`.
- Lateral acceleration is currently mapped from the available lateral acceleration source when
  present; with the current trajectory messages it is zero for the planned horizon because the
  trajectory has no lateral acceleration field.
- Threshold values are preserved exactly.
- **Impact:** **Low to medium.** The major NAVSIM history-padding gap is closed. Remaining
  differences are field-level platform mappings and the local C++ polynomial implementation
  rather than SciPy's exact Savitzky-Golay filter.

### Equation comparison

#### NAVSIM HC

**NAVSIM inputs.** NAVSIM builds a padded state sequence from past human states and
simulated proposal states:

$$
S_i^{HC}
=
\left[
S^{human\_past};
S_i^{proposal}
\right].
$$

Let `ego_is_comfortable()` extract smoothed dynamic signals:

$$
a_x(t),\;a_y(t),\;j(t),\;j_x(t),\;\dot{\psi}(t),\;\ddot{\psi}(t).
$$

The six binary checks are:

$$
C_{a_x}(t)=[-4.05<a_x(t)<2.40],
\quad
C_{a_y}(t)=[|a_y(t)|<4.89],
$$

$$
C_j(t)=[|j(t)|<8.37],
\quad
C_{j_x}(t)=[|j_x(t)|<4.13],
$$

$$
C_{\dot\psi}(t)=[|\dot\psi(t)|<0.95],
\quad
C_{\ddot\psi}(t)=[|\ddot\psi(t)|<1.93].
$$

Then:

$$
\mathrm{HC}_{nav,i}
=
\mathbf{1}
\left[
\forall t,\;
C_{a_x}(t)C_{a_y}(t)C_j(t)C_{j_x}(t)C_{\dot\psi}(t)C_{\ddot\psi}(t)=1
\right].
$$

If no past human trajectory is available, NAVSIM leaves HC initialized to `1.0`.

#### Migrated Autoware HC

**Migrated Autoware inputs.** The migrated code builds a padded state sequence:

$$
S^{HC}_{aw}
=
\left[
S^{ego\_past}_{kinematic};
S^{trajectory}_{proposal}
\right].
$$

The past segment uses recorded odometry and acceleration messages in
`[-1.5s, -0.1s]` relative to the trajectory stamp. The future segment uses the
planned trajectory in `[0.0s, 4.0s]`.

$$
a_x(t),\;a_y(t),\;j(t),\;j_x(t),\;\dot{\psi}(t),\;\ddot{\psi}(t)
$$

are computed over the full padded sequence using NAVSIM-style local polynomial
smoothing / derivatives:

$$
\mathrm{Smooth}_{8,2}(a_x),\quad
\mathrm{Smooth}_{8,2}(a_y),
$$

$$
\mathrm{Deriv}_{15,2}(a),\quad
\mathrm{Deriv}_{15,2}(a_x),\quad
\mathrm{Deriv}_{15,2}(\psi),\quad
\mathrm{Deriv}^{2}_{15,3}(\psi).
$$

The final score uses the NAVSIM threshold constants:

$$
\mathrm{HC}_{aw}
=
\mathbf{1}
\left[
\forall t,\;
-4.05<a_x(t)<2.40
\land
|a_y(t)|<4.89
\land
|j(t)|<8.37
\land
|j_x(t)|<4.13
\land
|\dot{\psi}(t)|<0.95
\land
|\ddot{\psi}(t)|<1.93
\right].
$$

If no padded states can be built, the implementation follows NAVSIM's default-pass
behavior and reports `HC=1.0` with a debug reason.

The migrated implementation also records each padded sample pose for debugging only.
This pose list does not enter the score equation. It is used to publish
`/debug/epdms/hc/horizon_footprints`, where normal samples are drawn as muted
blue-gray ego footprints, failed samples are colored by the component with the
largest threshold-severity ratio, and the worst peak sample is highlighted with a
white footprint outline. Component colors are orange-red for `ax`, yellow for `ay`,
magenta for jerk magnitude, pink for `jx`, cyan for yaw rate, and deep blue for yaw
acceleration.

### Assessment

HC is now a **close port**. The score equation, past-history padding, future horizon,
sample interval, and thresholds match NAVSIM. Remaining differences are Autoware's
available acceleration fields and the C++ local-polynomial implementation used in place
of SciPy's exact Savitzky-Golay helper.

---

## EC - Extended comfort

### Definition and semantic meaning

EC is the **cross-frame temporal consistency comfort subscore**. NAVSIM v2 introduced it to compare adjacent planner outputs across consecutive evaluation frames and to penalize planners whose successive plans imply inconsistent dynamic behavior, even if each single plan looks locally acceptable.

Semantically, EC captures planning smoothness over time at the evaluation-pipeline level: a good planner should not oscillate between materially different dynamic intents from one frame to the next. This makes EC an anti-flicker / anti-instability comfort metric rather than a per-trajectory comfort threshold.

### Ownership

- **NAVSIM primary owner:** `pdm_comfort_metrics.py::ego_is_two_frame_extended_comfort()`
- **NAVSIM used by:** `scene_aggregator.py::SceneAggregator._compute_two_frame_comfort()`
- **NAVSIM injected into final score by:** `run_pdm_score.py`, `run_pdm_score_one_stage.py`, `run_pdm_score_from_submission.py`
- **NAVSIM external state / cached dependency:** overlapping adjacent simulated trajectories, adjacency mapping / pseudo closed-loop grouping
- **Migrated owner:** `src/metrics/epdms/extended_comfort.cpp::calculate_extended_comfort()`
- **Migrated orchestration owner:** `src/open_loop_evaluator.cpp` phase-2 finalization

### Required inputs: same availability or replacement?

- **Same NAVSIM inputs available?** **No**

| Required Input (NAVSIM) | Semantic Meaning (NAVSIM) | Autoware Replacement | Semantic Meaning (AW) | Judgement / Impact |
| :--- | :--- | :--- | :--- | :--- |
| **Adjacent Simulated Trajectories** | Overlapping *simulated* car motion across two frames. | `previous_trajectory` and `current_trajectory` | Consistency between the raw *planner output* across two frames. | **High.** Comparing plans vs comparing simulated execution leads to different noise and consistency profiles. |
| **Adjacency Mapping** | Identification of consecutive scene tokens. | `SynchronizedData` temporal ordering | Temporal ordering of incoming ROS messages. | **Equivalent.** |

- **Semantic replacement meaning:** the migrated code compares published plans themselves, not the overlapping simulated rollouts used by NAVSIM.

### Platform deviations and impact

- NAVSIM extracts acceleration, jerk, yaw rate, and yaw acceleration from state arrays using the comfort helper stack and compares only the overlap region.
- The migrated code now extracts the same four signal categories with the shared HC/EC comfort-signal helper and compares the time-overlapped adjacent trajectories.
- Thresholds are preserved exactly: acceleration $0.7$, jerk $0.5$, yaw rate $0.1$, yaw acceleration $0.1$.
- **Impact:** **Medium.** The overlap and threshold semantics are now close to NAVSIM. The remaining gap is that Autoware compares published planner trajectories rather than NAVSIM's simulated `ego_simulated_states`.

### Equation comparison

#### NAVSIM EC

**NAVSIM inputs.** NAVSIM's EC is computed from two related frames/rollouts through
`ego_is_two_frame_extended_comfort()`. Let $S^{(1)}$ and $S^{(2)}$ be the two state
sequences being compared. For each dynamic feature:

$$
f\in
\{\mathrm{acceleration},\mathrm{jerk},\mathrm{yaw\_rate},\mathrm{yaw\_acceleration}\},
$$

define:

$$
\Delta f_t = f(S^{(1)})_t - f(S^{(2)})_t,
\qquad
\mathrm{RMS}_f =
\sqrt{
\frac{1}{N}
\sum_t(\Delta f_t)^2
}.
$$

With feature thresholds $\tau_f$:

$$
\mathrm{EC}_{nav}
=
\mathbf{1}
\left[
\forall f,\;\mathrm{RMS}_f\le\tau_f
\right].
$$

The NAVSIM thresholds in `pdm_comfort_metrics.py` are:

$$
\tau_a=0.7,\quad
\tau_j=0.5,\quad
\tau_{\dot\psi}=0.1,\quad
\tau_{\ddot\psi}=0.1.
$$

NAVSIM injects this EC into the weighted metric vector after base PDM scoring and then
recomputes the final EPDMS score.

#### Migrated Autoware EC

**Migrated Autoware inputs.** The migrated implementation compares consecutive
selected Autoware trajectories:

$$
Q^{prev},\quad Q^{curr}.
$$

Let:

$$
\Delta T = t(Q^{curr})-t(Q^{prev}),\qquad
\Delta q = \mathrm{dt}(Q^{curr}),\qquad
k = \mathrm{round}(\Delta T / \Delta q).
$$

If the adjacent Autoware trajectory header stamps are non-increasing at startup, the
implementation falls back to $\Delta T=\Delta q$ to avoid creating a non-semantic
unavailable comparison from duplicate initial stamps.

The compared overlap sequences are:

$$
S^{prev}_{aw}=Q^{prev}_{k:},\qquad
S^{curr}_{aw}=Q^{curr}_{:-k}.
$$

For 10 Hz Autoware planning with 0.1 s trajectory samples, this is normally:

$$
Q^{curr}_{0..38}\quad\mathrm{vs.}\quad Q^{prev}_{1..39}.
$$

For each signal:

$$
s\in\{a,j,\dot{\psi},\ddot{\psi}\},
$$

the RMS difference is:

$$
\mathrm{RMS}_s^{aw}
=
\sqrt{
\frac{1}{N_s}
\sum_{n=0}^{N_s-1}
\left(
s_n(S^{curr}_{aw})-s_n(S^{prev}_{aw})
\right)^2
}.
$$

The Autoware signal extraction uses the same local-polynomial smoothing and derivative
helper as HC. Planned-trajectory acceleration uses `TrajectoryPoint::acceleration_mps2`
as the longitudinal acceleration input and assumes zero lateral acceleration because
the Autoware trajectory message does not carry a lateral acceleration field. If either
trajectory has fewer than three points, the time alignment is invalid, or the overlap
has fewer than three samples, EC is marked unavailable.

The configured default thresholds are:

$$
\tau_a=0.7,\quad
\tau_j=0.5,\quad
\tau_{\dot\psi}=0.1,\quad
\tau_{\ddot\psi}=0.1.
$$

Then:

$$
\mathrm{EC}_{aw}
=
\mathbf{1}
\left[
\mathrm{RMS}_a^{aw}\le0.7
\land
\mathrm{RMS}_j^{aw}\le0.5
\land
\mathrm{RMS}_{\dot\psi}^{aw}\le0.1
\land
\mathrm{RMS}_{\ddot\psi}^{aw}\le0.1
\right].
$$

The first evaluated trajectory has no previous trajectory, so EC is marked unavailable
for that frame.

EC debug output is intentionally non-geometric. The implementation writes a JSON
comparison summary and aligned delta arrays:

$$
\Delta a,\quad \Delta j,\quad \Delta\dot{\psi},\quad \Delta\ddot{\psi}.
$$

**Main input gap.** NAVSIM EC is injected after base PDM scoring from NAVSIM's
two-frame extended-comfort path and compares simulated ego states. Migrated Autoware EC
compares consecutive selected trajectory messages in time, so the paired object is still
not identical even though overlap alignment and signal thresholds are now close.

### Assessment

EC is now a **close partial port**. The overlap alignment, smoothed signal extraction,
RMS gate, and thresholds follow NAVSIM. The remaining deviation is the state source:
published Autoware planner trajectories replace NAVSIM's simulated ego-state rollouts.

---

## Aggregation - raw EPDMS / `synthetic_epdms_raw`

### Ownership

- **NAVSIM base aggregation owner:** `pdm_scorer.py::_aggregate_pdm_scores()`
- **NAVSIM final EPDMS owner:** `run_pdm_score.py::compute_final_scores()` after EC injection
- **NAVSIM EC ownership feeding aggregation:** `scene_aggregator.py`
- **Migrated owner:** `src/metrics/epdms/epdms_aggregation.cpp::calculate_synthetic_epdms()`
- **Migrated orchestration owner:** `src/open_loop_evaluator.cpp`

### Required inputs: same availability or replacement?

- **Same subscore set available?** **Yes, nominally**
- **Same semantic inputs behind those subscores available?** **No, only partially**
- The migrated aggregator receives the nine local snapshots:
  HC, EC, EP, TTC, LK, DAC, NC, DDC, TLC.
- The local aggregator does **not** receive NAVSIM's pseudo closed-loop second-stage scene aggregation context.

### Platform deviations and impact

- At the **single-trajectory formula level**, the migrated raw aggregation matches NAVSIM's final EPDMS form.
- At the **pipeline level**, NAVSIM computes:
  1. base PDM without EC,
  2. scene-level EC,
  3. final EPDMS after EC injection,
  4. optional pseudo closed-loop second-stage weighting and group multiplication.
- The migrated project computes a single per-trajectory synthetic EPDMS directly in one local aggregation stage.
- **Impact:** **Low** if the target is only the final per-trajectory 16-weight formula; **high** if the target is NAVSIM's full published evaluation pipeline.

### Equation comparison

**NAVSIM final per-trajectory EPDMS**

$$
\mathrm{EPDMS}_{nav} =
\left(\mathrm{NC}\cdot\mathrm{DAC}\cdot\mathrm{DDC}\cdot\mathrm{TLC}\right)\cdot
\frac{
5\mathrm{EP}+5\mathrm{TTC}+2\mathrm{LK}+2\mathrm{HC}+2\mathrm{EC}
}{16}.
$$

This is assembled in two steps in NAVSIM:

$$
\mathrm{PDM}_{base} =
\left(\mathrm{NC}\cdot\mathrm{DAC}\cdot\mathrm{DDC}\cdot\mathrm{TLC}\right)\cdot
\frac{
5\mathrm{EP}+5\mathrm{TTC}+2\mathrm{LK}+2\mathrm{HC}
}{14},
$$

then EC is injected and the final score is recomputed.

**Migrated Autoware**

$$
\mathrm{synthetic\_epdms\_raw}_{aw} =
\left(\mathrm{NC}\cdot\mathrm{DAC}\cdot\mathrm{DDC}\cdot\mathrm{TLC}\right)\cdot
\frac{
5\mathrm{EP}+5\mathrm{TTC}+2\mathrm{LK}+2\mathrm{HC}+2\mathrm{EC}
}{16}.
$$

### Assessment

The migrated raw EPDMS aggregation is **formula-identical at the final single-trajectory level**, but not pipeline-identical to NAVSIM.

### Additional note: NAVSIM pseudo closed-loop published aggregation

NAVSIM also applies scene-level / group-level aggregation not present in this project. In `run_pdm_score.py`, stage-one and stage-two scores are grouped and multiplied, then averaged:

$$
\mathrm{groupScore} =
\frac{
\mathrm{stage1}_{group1}\cdot \mathrm{stage2}_{group1}

+\mathrm{stage1}_{group2}\cdot \mathrm{stage2}_{group2}
}{2}.
$$

This project has **no equivalent implementation**.

---

## Human-filtered - `synthetic_epdms_human_filtered`

### Ownership

- **NAVSIM owner of human-filter logic:** `navsim/evaluate/pdm_score.py`
- **NAVSIM owner of post-filter score recomputation:** `navsim/evaluate/pdm_score.py`
- **Migrated owner of human-reference construction:** `src/open_loop_evaluator.cpp::calculate_human_reference_snapshot()`
- **Migrated owner of human-filter logic:** `src/metrics/epdms/epdms_aggregation.cpp::calculate_human_filter_metrics()`
- **Migrated owner of filtered score aggregation:** `src/metrics/epdms/epdms_aggregation.cpp::calculate_synthetic_epdms()`

### Required inputs: same availability or replacement?

- **Same NAVSIM human reference inputs available?** **Partially**
- **NAVSIM requires:** a scored human trajectory through the same simulator / scorer stack on original scenes
- **Migrated replacement:** clone the synchronized sample and substitute the ground-truth trajectory for per-trajectory metrics.
- Human EP is computed by the same local single-proposal EP implementation on the ground-truth trajectory. This generally yields `1.0`, matching the practical NAVSIM human-only scoring behavior, but still keeps the filter path available if a human EP reference becomes `0`.

### Platform deviations and impact

- NAVSIM code filters columns in `PDMResults` when the human score is exactly zero, then recomputes multiplicative and weighted values.
- Because `PDMResults` does not store EC, NAVSIM code does **not** human-filter EC in that path. EC is injected later into the weighted vector.
- The migrated code now follows that behavior at the single-trajectory aggregation level:
  - filters `NC, DAC, DDC, TLC, EP, TTC, LK, HC`
  - does **not** filter `EC`
  - then recomputes the 16-weight final EPDMS using filtered base metrics plus unfiltered agent EC.
- **Impact:** **Low** for the final single-trajectory aggregation formula after this patch. The remaining gap is NAVSIM's pseudo closed-loop scene aggregation, which is still not implemented in this project.

### Equation comparison

**NAVSIM code**

For original scenes with `human_penalty_filter=True`, NAVSIM applies

$$
F_m(a_m,h_m) =
\begin{cases}
1, & h_m = 0 \\
a_m, & \text{otherwise}
\end{cases}
$$

for

$$
m \in \{\mathrm{NC},\mathrm{DAC},\mathrm{DDC},\mathrm{TLC},\mathrm{EP},\mathrm{TTC},\mathrm{LK},\mathrm{HC}\},
$$

then recomputes the product and weighted vector. EC is not part of this code path because it is injected later.

So the effective NAVSIM filtered single-trajectory score is

$$
\mathrm{EPDMS}^{HF}_{nav} =
\left(F_{NC}F_{DAC}F_{DDC}F_{TLC}\right)\cdot
\frac{
5F_{EP}+5F_{TTC}+2F_{LK}+2F_{HC}+2\mathrm{EC}
}{16}.
$$

**Migrated Autoware**

The migrated filter is

$$
F_m^{aw}(a_m,h_m) =
\begin{cases}
1, & a_m\text{ available } \land h_m\text{ available } \land |h_m|\le 10^{-9} \\
a_m, & \text{otherwise}
\end{cases}
$$

and the filtered score is

$$
\mathrm{synthetic\_epdms\_human\_filtered}_{aw} =
\left(F_{NC}^{aw}F_{DAC}^{aw}F_{DDC}^{aw}F_{TLC}^{aw}\right)\cdot
\frac{
5F_{EP}^{aw}+5F_{TTC}^{aw}+2F_{LK}^{aw}+2F_{HC}^{aw}+2\mathrm{EC}
}{16}.
$$

This matches the NAVSIM code path at the final single-trajectory formula level: EP is part of the filtered base metric vector, while EC remains unfiltered because NAVSIM injects EC after the human-filtered `PDMResults` columns have already been recomputed.

### Assessment

The migrated human-filtered implementation is **formula-equivalent to NAVSIM's single-trajectory code path** after this patch. It is still not equivalent to NAVSIM's full published pseudo closed-loop aggregation because this project does not construct or weight second-stage synthetic scenes.

---

## Evaluation frequency (rate) comparison

### Official NAVSIM

- **How often PDMS/EPDMS is computed:** effectively **once per scene token**, with adjacent scene frames spaced at **0.5 s**, i.e. **2 Hz**
- **Code basis:**
  - `navsim/planning/scenario_builder/navsim_scenario.py:52` sets `self._database_interval = 0.5`
  - `navsim/planning/scenario_builder/navsim_scenario.py:69-70` builds `TrajectorySampling(..., interval_length=0.5)`
  - `navsim/planning/simulation/planner/pdm_planner/scoring/scene_aggregator.py:60-64` computes `observation_interval` and asserts `0 < observation_interval < 0.55`, which matches adjacent scored scenes being about `0.5 s` apart
- **Interpretation:** NAVSIM computes one PDMS/EPDMS value per evaluation scene, and the canonical scene sequence is sampled on a 0.5-second grid.
- **Important distinction:** each score still evaluates a **4-second horizon**, but the **score emission frequency across scenes** is **2 Hz**.

### Migrated Autoware implementation

- **How often PDMS/EPDMS is computed:** by default, on **100 ms evaluation anchors**, i.e. **10 Hz**, subject to a synchronized trajectory being available at that anchor
- **Code basis:**
  - `planning/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml:8-10` sets `evaluation_interval_ms: 100.0`
  - `planning/autoware_planning_data_analyzer/src/base_evaluator.cpp:106-108` calls `get_kinematic_states_at_interval(topic_names.evaluation_interval_ms)`
  - `planning/autoware_planning_data_analyzer/src/bag_handler.hpp:288-313` samples odometry timestamps at `interval_ms` and requests the closest odometry within `interval_ms / 2`
  - `planning/autoware_planning_data_analyzer/src/base_evaluator.cpp:117-124` keeps only synchronized samples where `sync_data && sync_data->trajectory`
- **Interpretation:** the migrated evaluator attempts to score at a fixed 10 Hz synchronization grid, not at a NAVSIM-style 0.5-second scene cadence.
- **Practical caveat:** if the trajectory topic is slower than 10 Hz, the realized scoring rate is bounded by the availability of synchronized trajectory messages, but the evaluator is still designed around a **100 ms anchor interval**.

### Comparison takeaway

$$
\text{Official NAVSIM scoring rate} \approx 2~\text{Hz}
\qquad\text{vs.}\qquad
\text{Migrated default scoring rate} \approx 10~\text{Hz}.
$$

So the migrated implementation evaluates EPDMS/PDMS-like scores **much more frequently** than official NAVSIM by default: roughly **5 times as often** in wall-clock sampling cadence, even though both evaluate multi-second trajectories.

---

---

## Final conclusions

1. **Closest ports:** DDC and LK.
2. **Structurally close but platform-substituted:** NC and DAC.
3. **Material deviations:** HC, EC, EP, TTC, TLC.
4. **Raw aggregation:** final single-trajectory formula matches NAVSIM.
5. **Human-filtered aggregation:** code behavior does **not** match NAVSIM; EP and EC filtering differ in opposite directions.
6. **Evaluation rate:** the migrated implementation is sampled at **10 Hz by default**, versus NAVSIM's effective **2 Hz** scene cadence.
7. **Full evaluation pipeline:** NAVSIM's pseudo closed-loop stage aggregation is not present in this project, so the local outputs should not be treated as published NAVSIM EPDMS equivalents.
