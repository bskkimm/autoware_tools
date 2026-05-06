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

## Remaining PR Order

Recommended order after merged PR #421:

1. Shared metric source layout and reusable helpers.
   - Add `metrics/trajectory_metrics.*`, `metrics/metric_types.hpp`, `metrics/geometry/*`, and
     `metrics/epdms/context/*`.
   - Keep score behavior unchanged as much as possible.
   - This PR should mainly reduce duplication and create the final file layout.

2. Runtime controls and output topic contract.
   - Add default-all EPDMS metric selection.
   - Add `open_loop.debug_topics_enabled`, default `false`.
   - Move EPDMS score topics to `/open_loop/metrics/epdms/*`.
   - Keep diagnostic arrays excluded from the EPDMS namespace.

3. NC and shared object-track usage.
   - Port NAVSIM-faithful NC logic from the current reference branch.
   - Use recorded tracked objects from the selected object-track source.
   - Preserve NC debug outputs behind `open_loop.debug_topics_enabled`.

4. DAC.
   - Port semantic drivable-area logic using road lanelets, road shoulder lanelets,
     `intersection_area`, `hatched_road_markings`, and `parking_lot`.
   - Do not include the experimental road-border fallback unless it is explicitly revalidated
     and accepted for upstream.
   - Document exactly which road-space categories are admitted/excluded.

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

If the broad pre-commit check is too slow during local iteration, first run pre-commit on changed
files, then run `pre-commit run --all-files` before opening the PR.

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

Validate with cumulative EPDMS metrics:

```yaml
open_loop.enabled_metrics: ['nc', 'dac', 'ddc', 'tlc', 'ttc', 'lk', 'hc', 'ec', 'ep']
```

Validation policy:

- Infrastructure-only PRs: score JSON and topic set should be behaviorally unchanged against the
  accepted baseline unless the PR explicitly changes output names.
- Subscore PRs: compare the PR-branch result to the current `kim` reference artifact for the same
  subscore.
- Aggregation/human-filter PRs: compare raw subscore values, raw EPDMS, human references,
  filter-applied counts, and human-filtered EPDMS.
- Always check already-ported subscores for regressions.

Required bags:

- `x2_takanawa`: primary regression comparison against the accepted `kim` reference artifact.
- `x2_odaiba`: required when touching map semantics, object semantics, DAC, DDC, NC, TTC, TLC, or
  aggregation. Recommended for all remaining EPDMS PRs if runtime allows.

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

Do not open a PR if score deltas exist and the reason is not understood.

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

### Expected Behavior

- Expected no score change / expected score change:
- Expected topic change:
- Known intentional deltas:

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
  - Input:
  - Output artifact:
  - Command/script:

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
- markdownlint failures in planning docs
- stale package dependencies if new dependencies are added
- missing differential build/test label

