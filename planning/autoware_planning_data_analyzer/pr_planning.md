# EPDMS Upstream PR Planning Runbook

This runbook is the source of truth for the remaining gradual EPDMS PR series into
`upstream/main` of `autowarefoundation/autoware_tools`.

PR #421 is already merged and is treated as the first accepted PR in this series. For the
remaining PRs, use the current `kim` implementation branch as the extraction reference:

```text
refactor/epdms-metric-topics
```

Do not use the older EPDMS-only source tree as the reference anymore. The current reference
branch contains the intended EPDMS source layout, runtime controls, aggregation/human-filter
logic, and EPDMS metric-topic namespace.

## Current Reference Layout

The complete target layout for the EPDMS-related analyzer code is:

```text
planning/autoware_planning_data_analyzer/src/
  metrics/
    metric_types.hpp
    trajectory_metrics.hpp
    trajectory_metrics.cpp

    geometry/
      comfort_signal.hpp
      comfort_signal.cpp
      ego_footprint.hpp
      ego_footprint.cpp
      lanelet_queries.hpp
      lanelet_queries.cpp
      metric_utils.hpp
      metric_utils.cpp
      object_tracks.hpp
      object_tracks.cpp

    epdms/
      context/
        epdms_context.hpp
        epdms_context.cpp
        epdms_types.hpp

      subscores/
        no_at_fault_collision.hpp
        no_at_fault_collision.cpp
        drivable_area_compliance.hpp
        drivable_area_compliance.cpp
        driving_direction_compliance.hpp
        driving_direction_compliance.cpp
        traffic_light_compliance.hpp
        traffic_light_compliance.cpp
        ttc_within_bound.hpp
        ttc_within_bound.cpp
        lane_keeping.hpp
        lane_keeping.cpp
        history_comfort.hpp
        history_comfort.cpp
        extended_comfort.hpp
        extended_comfort.cpp
        ego_progress.hpp
        ego_progress.cpp

      aggregation/
        epdms_aggregation.hpp
        epdms_aggregation.cpp
```

Keep `metrics/geometry/*` outside `metrics/epdms/*` because those helpers are general metric
geometry utilities and may be reused by non-EPDMS metrics later.

## Branch Strategy

- Start every upstream PR branch from latest `upstream/main`, unless it is intentionally stacked
  on another not-yet-merged PR.
- Do not open PRs directly from the long-lived `kim` branch. Port only the intended PR slice.
- Use signed commits:

```bash
git commit -s -m "refactor(planning_data_analyzer): ..."
```

- Avoid `migrate` in branch names and PR title prefixes. Prefer:
  - `refactor/planning-data-analyzer-epdms-context`
  - `feat/planning-data-analyzer-epdms-nc`
  - `fix/planning-data-analyzer-epdms-aggregation`
- Keep each PR reviewable: one infrastructure slice, one subscore slice, or one output-contract
  slice.
- If a PR depends on an unmerged previous PR, set the GitHub PR base to that previous PR branch
  so the diff does not duplicate earlier work.
- Before starting a new PR slice, commit and push the current slice and record validation paths.
- After opening each PR, update this runbook with any PR-specific lessons that affect the remaining
  EPDMS PR series.
- When modifying an existing PR in response to reviewer comments, update this runbook if the
  reviewer feedback changes the PR strategy, validation workflow, helper design, or recurring
  coding rules.

## Mandatory Duplication Audit Before Every PR Push

This is a hard gate. A general note to "avoid duplication" is not enough. Before opening or
updating every upstream PR, inspect the staged PR diff and actively remove repeated expressions
introduced by that PR.

Run and review:

```bash
git diff --name-only upstream/main..HEAD
git diff upstream/main..HEAD -- <changed-files>
```

If the PR is stacked on an unmerged PR, compare against the parent PR branch instead of
`upstream/main`.

During the audit, search specifically for these introduced patterns:

- Three or more near-identical blocks that differ only by metric name, topic name, enabled flag,
  message variable, or score field.
- Repeated score/available/reason topic declarations or writes.
- Repeated `if (enabled...) { ... writer.write(...) ... }` blocks.
- Repeated topic-prefix, topic-suffix, or alias-list construction.
- Repeated lanelet, polygon, footprint, object-track, or comfort-signal helper expressions.
- Repeated comments that explain the same behavior in multiple places.
- Local helper functions that duplicate existing Autoware utilities. Before adding a helper,
  search for an existing utility in the relevant dependency, especially conversion and geometry
  helpers such as ROS point conversion, lanelet-to-ROS conversion, angle normalization, and
  footprint conversion.

Required action:

- Replace repeated write/register blocks with a local lambda, small helper function, loop over a
  table, or a typed descriptor list.
- Replace repeated suffix/prefix handling with a helper, constant, or descriptor table.
- Replace repeated geometry/metric expressions with an existing shared helper before adding a new
  local helper.
- If an existing upstream utility is used but the `pilot-auto.x2` validation underlay lacks it,
  validate with a temporary compatibility patch and restore the upstream-intended code before
  committing.
- Keep behavior and review scope more important than abstraction. Do not introduce a broad
  cross-file framework only to remove one harmless two-line repeat.
- If repetition remains because abstraction would hide metric semantics or make the PR larger,
  state that explicitly in the PR description.

The PR description must include this line:

```markdown
- Duplication audit: performed; repeated introduced patterns handled by <helper/table/lambda/etc.>;
  remaining repetition is intentional because <reason or N/A>.
```

Do not push the PR branch until this duplication audit line is true. `pre-commit` passing is not a
substitute for this audit.

Reason strings are part of the output contract even when topic names and JSON keys are unchanged.
If a PR changes any `reason` value, explicitly state that in the PR description and include reason
counts in validation.

## Remaining PR Order

Recommended order after merged PR #421:

1. Shared metric source layout and reusable helpers.
   - Status: merged as PR #423.
   - Add reusable geometry helpers first, especially `metrics/geometry/comfort_signal.*`,
     `metrics/geometry/ego_footprint.*`, and `metrics/geometry/lanelet_queries.*`.
   - Do not pull `metrics/geometry/object_tracks.*` or `metrics/epdms/context/*` into this first
     helper PR unless the tracked-object/data-type changes they require are also intentionally in
     scope.
   - Keep existing score behavior unchanged. This helper PR may introduce preparatory EPDMS helper
     APIs for follow-up subscore PRs, but existing public helper functions used by current metrics
     must keep their previous semantics.
   - This PR should mainly reduce duplication and create the safe shared helper layout.

2. Runtime controls and output topic contract.
   - Status: merged as PR #425 from
     `bskkimm:feat/planning-data-analyzer-epdms-runtime-topics`.
   - Validation recorded:
     - Takanawa full run: `/home/beomseokkim2/rosbag/x2_takanawa/run_logs/20260515_215540`
     - `pre-commit run --all-files`: passed
     - Local `pilot-auto.x2` build/CTest: passed with temporary validation-only lanelet API
       compatibility patch restored before commit
   - Add default-all EPDMS metric selection.
   - Add `open_loop.debug_topics_enabled`, default `false`.
   - Move EPDMS score topics to `/open_loop/metrics/epdms/*`.
   - Keep diagnostic arrays excluded from the EPDMS namespace.

3. NC and shared object-track usage.
   - Status: opened as PR #426 from
     `bskkimm:feat/planning-data-analyzer-epdms-nc-prep`.
   - Port NAVSIM-faithful NC logic from the current reference branch.
   - Use recorded tracked objects from the selected object-track source.
   - Add `metrics/geometry/object_tracks.*` and `metrics/epdms/context/*` here, or in a
     immediately preceding object-track/context infrastructure PR, because those helpers depend on
     tracked-object data-type changes.
   - Preserve NC debug outputs behind `open_loop.debug_topics_enabled`.
   - Reviewer lesson from PR #426: NC object de-duplication must record a collided object ID only
     after the overlap is classified as at-fault. Ignored overlaps such as stopped ego, stopped
     track, or rear collisions must not mask a later at-fault collision with the same tracked
     object ID.
   - Reviewer lesson from PR #426: production scorer code should consume prepared shared helper
     outputs, e.g. `LoggedObjectTrack` and `TrajectoryFootprintEvaluation`; avoid optional pointer
     plus local fallback patterns in the main scorer.
   - Reviewer lesson from PR #426: if reviewer-requested bug fixes intentionally change a subscore
     compared with the baseline, record the changed count and reason. Example: the NC de-dup fix
     changed Takanawa NC non-1 count from 74 to 87 by exposing 13 previously masked at-fault
     lateral collisions.
   - Reviewer lesson from PR #426: before adding local hash/key helpers for common message types,
     search Autoware utilities first. For UUIDs, prefer the existing UUID conversion/hash path over
     custom byte-array keys.
   - Reviewer lesson from PR #426: avoid trivial local wrappers around existing Autoware utilities
     such as angle normalization or distance calculation. Use the existing utility directly when
     semantics match.
   - Reviewer lesson from PR #426: when slicing a sorted timeline, prefer standard algorithms
     (`lower_bound`/`upper_bound` plus `assign`) over manual loop-and-break code.
   - Reviewer lesson from PR #426: threshold constants must encode the measured quantity and unit,
     e.g. `VelocityThresholdMps`, and strict NAVSIM-style thresholds need a short rationale comment.
   - Reviewer lesson from PR #426: horizon/truncation helpers need edge-case tests for first point
     beyond horizon, single-point trajectory, and exact-horizon point.

4. DAC.
   - Status: opened as PR #427 from
     `bskkimm:feat/planning-data-analyzer-epdms-dac-prep`.
   - Port semantic drivable-area logic using road lanelets, road shoulder lanelets,
     `intersection_area`, `hatched_road_markings`, and `parking_lot`.
   - The PR activates the existing conservative road-border fallback already present in the shared
     ego-footprint context. Call this out explicitly in the PR description and review notes.
   - Document exactly which road-space categories are admitted/excluded.
   - Validation recorded:
     - Takanawa full run: `/home/beomseokkim2/rosbag/x2_takanawa/run_logs/20260519_165136`
       was compared, then deleted after recording the summary.
     - Evaluated trajectories: `8695`.
     - DAC exact match against `/home/beomseokkim2/rosbag/x2_takanawa/run_logs/20260506_155345`:
       `396` non-1, `0` unavailable, reason counts
       `compliant: 8299`, `non_compliant_corner_outside_drivable_area: 396`.
     - NC differs from the old baseline only by the accepted PR #426 de-dup delta:
       `74 -> 87` non-1.
     - Local build passed with temporary validation-only `pilot-auto.x2` lanelet API compatibility
       patch restored before push.
     - Direct gtests passed after reviewer-update patch: `test_metrics` `61/61`,
       `test_offline_evaluation` `20/20`.
     - `pre-commit run --all-files`: passed.
   - CI lesson from PR #427: Jazzy treats deprecated declarations as build errors. Test-only
     helpers using compatibility APIs such as `lanelet::utils::conversion::toBinMsg` need either
     the newer upstream API when available in both validation underlays, or a narrow diagnostic
     suppression around only that compatibility call.
   - Reviewer lesson from PR #427: include paths must use project-source-relative form
     (`metrics/...`), not relative traversal (`../../...`), and local conversion helpers must be
     deleted when an Autoware utility already exists.
   - Reviewer lesson from PR #427: unavailable results must not contain partial debug payload. Run
     all availability/validity checks before filling debug info.
   - Reviewer lesson from PR #427: changing only reason values still changes the output contract
     and must be documented in the PR description.

5. DDC.
   - Port wrong-way/oncoming progress logic.
   - Keep DAC-style generic non-drivable intrusion separate from DDC oncoming progress.
   - Reuse route/lanelet context where semantics match.

6. TLC.
   - Port stop-line based traffic-light compliance logic.
   - Include signal-group association and movement selection.
   - Preserve right/left arrow handling and turn-indicator movement inference.

7. TTC.
   - Port NAVSIM-style recorded-object TTC logic.
   - Reuse object-track preprocessing where possible.
   - Preserve `BadOrIntersection = MultipleLanes OR NonDrivableArea OR Intersection` semantics.

8. LK.
   - Port sample-wise centerline deviation logic.
   - Keep Autoware centerline selection as the intentional NAVSIM deviation.
   - Use turn-indicator/hazard-based lane-change exemption only.

9. HC and EC.
   - Port NAVSIM-style history comfort using past human states plus planned horizon.
   - Port two-frame extended comfort.
   - Keep comfort signal computation in shared `metrics/geometry/comfort_signal.*`.
   - Split HC and EC into separate PRs if the diff becomes hard to review.

10. EP.
    - Port NAVSIM-faithful ego progress logic using the current single selected trajectory topic.
    - Leave the future multi-candidate trajectory source as a documented follow-up.

11. Aggregation and human-filtered EPDMS.
    - Port NAVSIM-faithful synthetic EPDMS aggregation.
    - Human filter applies to NC, DAC, DDC, TLC, EP, TTC, LK, and HC.
    - Human filter does not apply to EC.
    - Use GT/human reference metrics for filtering where available.

12. Documentation and cleanup.
    - Align `implementation_report.md`.
    - Align `debugging_explanation.md`.
    - Remove stale topic names, stale labels, and obsolete transitional code.

## Preparatory Helper PR Policy

For infrastructure PRs that introduce shared helpers before the first subscore caller exists,
follow these rules:

- Existing helper APIs must remain behavior-preserving. Do not hide a semantic change behind an
  old function name.
- If a broader NAVSIM/EPDMS-specific interpretation is needed, introduce it under a new explicit
  helper name and switch callers only in the later subscore PR that intentionally changes the
  metric semantics.
- Example from PR #423:
  - `is_pose_in_intersection()` must keep the old closest-reference-route-lanelet behavior while
    the PR is presented as an infrastructure/refactor PR.
  - Broader logic using local route-consistent lanelets, `intersection_area` polygons, and lane
    margins should live in a separate EPDMS context helper and should be connected to TTC/DDC/DAC
    only in the later subscore PR where the score delta is documented and validated.
- Large helper APIs that are intentionally unused until follow-up PRs are acceptable only if the PR
  description and header comments say so clearly.
- Do not keep legacy or abandoned helper fields only for future possibility. If the latest reference
  branch no longer uses a concept, remove it from the preparatory PR. For example, do not keep a
  road-border polygon/envelope field if the accepted design uses line/segment/probe evidence rather
  than a road-border envelope polygon.
- Keep common headers lightweight. Do not make `metric_utils.hpp` include large helper headers such
  as `ego_footprint.hpp` or `lanelet_queries.hpp` just to preserve transitive includes. Add direct
  includes at call sites instead.
- Naming should distinguish stored probe/evaluation data from tests. Avoid names like `*Test` for
  runtime debug/evaluation records; prefer names such as `*Probe` or `*Evaluation`.
- Be strict about duplicated helper expressions in preparatory helper PRs. Before adding a new
  anonymous-namespace helper, check whether the same expression already exists in another geometry
  helper file or in Autoware utility libraries.
- If two helper files need the same lanelet/polygon conversion, point containment, unique-append,
  bounding-box, distance, or angle-normalization logic, prefer a small shared internal helper over
  duplicated local functions.
- Prefer existing Autoware utility functions over local reimplementations when semantics match, for
  example angle normalization and squared-distance helpers.
- Before adding a local hash, key conversion, normalization, distance, or slicing helper, search the
  existing Autoware utility headers and standard library algorithms. Reviewer feedback has repeatedly
  preferred established utilities over locally reimplemented equivalents.
- If an existing Autoware utility appears to accept the desired type, try the direct utility call
  first. If the required validation underlay fails to compile that direct form, keep the adapter
  minimal and add a short comment explaining the compatibility reason.
- Example from PR #423: `calc_squared_distance2d(Point2d, Point2d)` passed pre-commit but failed
  against the required `pilot-auto.x2` validation underlay because that dependency version routes
  through message-style `.x/.y/.z` fields. In that case, keeping a `Point2d` to
  `geometry_msgs::msg::Point` adapter is acceptable only with an explicit compatibility comment.
- Reviewer comments about duplicated helper expressions should be handled in the current PR when
  they are local to the touched files and can be fixed without changing metric semantics.

## Required Local Checks Before Every PR

Run these before opening or updating each PR:

```bash
git fetch upstream main
git status --short --branch
```

Build:

```bash
source /opt/ros/humble/setup.bash
source /home/beomseokkim2/workspace/pilot-auto.x2/install/setup.bash
colcon build --symlink-install --packages-select autoware_planning_data_analyzer \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Tests:

```bash
source /opt/ros/humble/setup.bash
source /home/beomseokkim2/workspace/pilot-auto.x2/install/setup.bash
colcon test --packages-select autoware_planning_data_analyzer --ctest-args --output-on-failure
```

Pre-commit:

```bash
pre-commit run --all-files
```

This is a hard gate. Do not open or update an upstream PR until `pre-commit run --all-files`
finishes successfully on the exact branch tip being pushed.

Important lessons from PR #423:

- `pre-commit-optional` passing in GitHub Actions is not enough. The required upstream status can
  still fail as `pre-commit.ci - pr`.
- Local build, CTest, and full analyzer validation do not exercise all pre-commit hooks.
- The `ros-include-guard` hook rewrites include guards based on the full path. New headers under
  `src/metrics/geometry/` must use guards like:

```cpp
#ifndef METRICS__GEOMETRY__COMFORT_SIGNAL_HPP_
#define METRICS__GEOMETRY__COMFORT_SIGNAL_HPP_
```

- If any hook modifies files, commit those modifications and rerun `pre-commit run --all-files`
  from the beginning. Passing only the previously failed hook is not sufficient for opening the PR.

If the broad pre-commit check is too slow during local iteration, first run pre-commit on changed
files, but `pre-commit run --all-files` must still pass before push/PR:

```bash
pre-commit run --files <changed-files>
pre-commit run --all-files
```

## Full Analyzer Validation

Before opening each PR, run the analyzer pipeline. Use the `pilot-auto.x2` underlay for local
validation:

```bash
source /opt/ros/humble/setup.bash
source /home/beomseokkim2/workspace/pilot-auto.x2/install/setup.bash
```

Do not modify files inside `/home/beomseokkim2/workspace/pilot-auto.x2`. If an upstream PR branch
temporarily needs the same dependency behavior as the `kim` reference branch to run against the
`pilot-auto.x2` underlay, patch only the PR branch, run validation, then restore the upstream
dependency state before committing and pushing.

The lanelet dependency mismatch is a known validation-only issue:

- `autowarefoundation/autoware_tools:main` can use newer
  `autoware::experimental::lanelet2_utils::*` APIs.
- The local `pilot-auto.x2` underlay used for reproducible full-run validation may expose older
  `lanelet::utils::*` wrappers instead.
- For full-run validation against `pilot-auto.x2`, it is acceptable to temporarily patch only the
  upstream PR branch to call the `pilot-auto.x2`-compatible lanelet APIs.
- After validation, restore the upstream-compatible lanelet calls before committing/pushing the PR.
- Never commit the temporary `pilot-auto.x2` compatibility patch unless the PR explicitly targets
  that dependency compatibility.
- If a reviewer requests use of a newer utility form, validate both the upstream-intended form and
  the `pilot-auto.x2` validation form. If they differ, use a temporary local compatibility patch
  for the validation run, then restore the upstream-intended code before commit/push.
- Before pushing, confirm with `git diff upstream/main..HEAD` and `git status --short` that no
  validation-only lanelet dependency edits remain.

Validate with cumulative EPDMS metrics:

```yaml
open_loop.enabled_metrics: ["nc", "dac", "ddc", "tlc", "ttc", "lk", "hc", "ec", "ep"]
```

Validation policy:

- Infrastructure-only PRs: score JSON and topic set should be behaviorally unchanged against the
  accepted baseline unless the PR explicitly changes output names.
- Subscore PRs are exact migration PRs. Port the same logic from the current reference branch
  (`kim/refactor/epdms-metric-topics`) into the target upstream PR branch. Do not approximate,
  simplify, or independently reimplement the subscore logic.
- Subscore PRs must reproduce the current `kim` reference artifact for that subscore exactly:
  evaluated count, available/unavailable counts, score mean, non-1 count, reason counts, and
  changed timestamp set must match. If exact reproduction is impossible, stop and diagnose before
  committing/pushing.
- A subscore PR may have score deltas only when the PR intentionally changes semantics beyond the
  current reference branch. That must be explicitly planned before implementation and documented in
  the PR description with timestamp-level evidence. Otherwise, any score delta is a blocker.
- Aggregation/human-filter PRs: compare raw subscore values, raw EPDMS, human references,
  filter-applied counts, and human-filtered EPDMS.
- Always check already-ported subscores for regressions.

Required Takanawa score baseline:

- Use `/home/beomseokkim2/rosbag/x2_takanawa/run_logs/20260506_155345` as the primary full-score
  baseline for EPDMS PR validation.
- This baseline contains all EPDMS subscore JSON fields, raw synthetic EPDMS, human references,
  human-filtered subscores, and aggregate reason/availability counts.
- Baseline evaluated trajectory count: `8695`.
- This baseline does not contain `/debug/epdms/*` topics. Use it for score/JSON regression, not as
  the debug-topic contract baseline.
- When a PR changes a subscore intentionally, compare that subscore against this baseline and
  explain each intentional delta. Already-ported unchanged subscores should remain unchanged unless
  the PR explicitly documents a dependency-driven delta.

Required bag:

- `x2_takanawa`: primary regression comparison against the accepted `kim` reference artifact.
- Do not run `x2_odaiba` during consecutive PR preparation unless the user explicitly requests it
  for that PR. The default required full-run validation for remaining EPDMS prep branches is
  Takanawa only.

For every full run, record:

- input bag path
- map path
- output artifact path
- exact command or script path
- evaluated trajectory count
- available/unavailable count per subscore
- non-1 count per subscore
- representative failure timestamps
- metric topics produced
- debug topics produced when `open_loop.debug_topics_enabled:=true`

Full-run artifacts are large. After the comparison against the baseline/reference artifact is
complete and the needed summary is recorded, delete the newly produced full-run artifact directory
unless it must be kept for reviewer inspection or explicit follow-up debugging. Do not delete the
required baseline artifact.

Do not open a PR if score deltas exist and the reason is not understood. For exact subscore
migration PRs, understanding the delta is not sufficient: the branch must be corrected until it
reproduces the reference artifact exactly, unless an intentional semantic delta was approved before
the work began.

## Output Topic Contract

EPDMS score, availability, reason, and aggregate score topics should use:

```text
/open_loop/metrics/epdms/*
```

Diagnostic trajectory arrays remain outside that namespace:

```text
/trajectory/raw/ttc_values
/trajectory/raw/lateral_deviations
/trajectory/raw/travel_distances
/trajectory/raw/longitudinal_accelerations
/trajectory/raw/lateral_accelerations
/trajectory/raw/lateral_jerks
/trajectory/raw/jerk_magnitudes
/trajectory/raw/longitudinal_jerks
/trajectory/raw/yaw_rates
/trajectory/raw/yaw_accelerations
```

Non-EPDMS raw metrics remain under:

```text
/open_loop/metrics/raw/*
```

Debug topics remain under:

```text
/debug/epdms/*
```

and must be disabled by default unless `open_loop.debug_topics_enabled:=true`.

## PR Description Format

Use the upstream PR template headings, filled with the concise analyzer-specific content below.
For score-logic migration PRs, include a pipeline-style `Logic Change` section. This is required
even when the code diff is large because reusable helper infrastructure is introduced. The goal is
to make the data flow understandable without reading the full implementation diff.

```markdown
## Description

### Scope

- PR type: infrastructure / subscore / runtime-control / output-topic / aggregation / docs.
- Target metric or subscore:
- Base branch:
- Parent PR/branch if stacked:
- This is PR N in the EPDMS patch PR series.

### What Changed

- Concise implementation bullets.
- Score semantics changed: yes/no.
- Debug topics changed: yes/no.
- Output topic or JSON schema changed: yes/no.
- Duplication audit: performed; repeated introduced patterns handled by <helper/table/lambda/etc.>;
  remaining repetition is intentional because <reason or N/A>.

### Expected Behavior

- Expected no score change / expected score change:
- Expected topic change:
- Known intentional deltas:

### Logic Change

As-is pipeline:

1. <Current data source / synchronized input>
2. <Current preprocessing>
3. <Current comparison/check>
4. <Current score decision>

To-be pipeline:

1. <New or migrated data source>
2. <New reusable helper/context construction>
3. <Per-sample/per-horizon computation>
4. <Shared geometry/object/lanelet/comfort helper reuse>
5. <Final score decision>

Line-count note:

- If additions are larger than deletions, explain whether this PR introduces reusable helper
  infrastructure that later EPDMS PRs will reuse instead of duplicating logic.

## How was this PR tested?

### Local Checks

- Build:
  - Command:
  - Result:
- Unit tests:
  - Command:
  - Result:
- Pre-commit:
  - Command:
  - Result:

### Full Analyzer Runs

- Takanawa:
  - Input:
  - Output artifact:
  - Command/script:
- Odaiba:
  - Not run unless explicitly requested.

### Metric Comparison

- Baseline artifact:
- PR artifact:
- Total evaluated trajectories:
- Availability comparison:
- Non-1/failure count comparison:
- Reason-count comparison:
- Representative changed timestamps:
- Explanation for intentional deltas:

### Topic Verification

- Metric topics produced:
- Availability/reason topics produced:
- Debug topics produced:
- Existing ported topics still produced:

## Notes for reviewers

- Review focus:
- Known limitations:
- Follow-up PRs:
- Deferred NAVSIM-faithfulness gaps:

## Effects on system behavior

- Runtime behavior:
- Score behavior:
- Debug/output behavior:
- Backward compatibility:
```

Do not leave validation fields blank. If a section is not applicable, write `N/A` with the reason.

## Upstream CI Expectations

Observed upstream checks include:

- `DCO`
- `semantic-pull-request / semantic-pull-request`
- `pre-commit.ci - pr`
- `pre-commit-optional`
- `spell-check-differential`
- `build-and-test-differential (humble)`
- `build-and-test-differential (jazzy)`
- Codecov patch/project checks
- Mergify checks

Use semantic PR titles, for example:

```text
refactor(planning_data_analyzer): add EPDMS shared metric context
feat(planning_data_analyzer): add NAVSIM-style NC metric
fix(planning_data_analyzer): align EPDMS human-filter aggregation
```

The differential build/test workflow may require the label:

```text
tag:run-build-and-test-differential
```

Use:

```text
component:planning
```

Common local failure risks:

- missing `Signed-off-by`
- `clang-format` drift
- cpplint include-order or line-length failures
- Jazzy `-Werror=deprecated-declarations`, especially for test-only lanelet map conversion helpers
- markdownlint failures in planning docs
- stale package dependencies if new dependencies are added
- missing differential build/test label
