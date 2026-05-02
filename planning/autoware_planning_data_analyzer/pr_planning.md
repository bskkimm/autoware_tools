# EPDMS Upstream PR Planning Runbook

This runbook is mandatory for gradual EPDMS migration PRs from the `kim` remote into
`upstream/main` of `autowarefoundation/autoware_tools`.

## Branch Strategy

- Always start each upstream PR branch from the latest `upstream/main`, unless the PR is
  intentionally stacked on a previous not-yet-merged migration PR.
- Do not rebase or directly PR the long-lived `kim` implementation branch. Port only the
  intended PR slice.
- Keep PRs reviewable: one common infrastructure slice or one subscore migration slice per PR.
- If a PR depends on a previous PR, set the GitHub PR base to that previous PR branch to avoid
  duplicated diffs.
- Before starting the next slice, ensure the current slice is committed, pushed, and has a
  reproducible validation record.

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

If this is too broad for an intermediate local check, at minimum run pre-commit on changed files
first, then run `pre-commit run --all-files` before opening the PR.

## Required Full Analyzer Validation

Before opening each PR, run the full analyzer pipeline, not only unit tests.

- For infrastructure-only PRs, verify the result JSON and debug topic set are behaviorally
  unchanged against the latest accepted baseline for both bags.
- For PRs that migrate or change a subscore, compare the migrated subscore against the existing
  `kim` artifact for `x2_takanawa`.
- When the subscore is expected to change behavior, report the exact expected deltas:
  - total evaluated trajectories
  - available/unavailable count
  - number of non-1 scores
  - first/representative failure timestamps
  - reason-count changes
  - expected debug topics produced
- Also check that already-migrated subscores do not regress:
  - metric topics still exist
  - debug topics still exist
  - availability counts stay consistent unless intentionally changed
  - failure counts stay consistent unless intentionally changed

Use cumulative EPDMS metrics in the full run. The cumulative set grows as migration proceeds:

```yaml
open_loop.enabled_metrics: ['nc', 'dac', 'ddc', 'tlc', 'ttc', 'lk', 'hc', 'ec', 'ep']
```

For the current migration work, validate at least:

- `x2_takanawa`: primary regression comparison against existing `kim` artifacts.
- `x2_odaiba`: cross-map sanity check when the PR touches map/object/subscore semantics.

## Subscore PR Acceptance Checklist

For every actual subscore migration PR, record the following in the PR description:

- Baseline artifact path from `kim`.
- New PR-branch artifact path.
- Full run command or script path.
- Build result.
- Unit test result.
- Pre-commit result.
- Metric topic presence.
- Debug topic presence.
- Failure count comparison.
- Availability comparison.
- Representative timestamp diagnosis if counts differ.

Do not open the PR if the score changed and the reason is not understood.

## Upstream CI Expectations

Observed from upstream PR:

- Reference PR: <https://github.com/autowarefoundation/autoware_tools/pull/401>
- Required DCO check exists and passed there as `DCO`.
- Use signed commits for every commit:

```bash
git commit -s -m "feat(planning_data_analyzer): ..."
```

- PR title must satisfy semantic PR rules, e.g.:
  - `feat(planning_data_analyzer): migrate NC EPDMS metric`
  - `fix(planning_data_analyzer): align TTC object filtering`
  - `refactor(planning_data_analyzer): add EPDMS shared context`
- The differential build/test workflow runs only when label
  `tag:run-build-and-test-differential` is present.
- Use `component:planning` for analyzer PRs.

Checks observed on PR #401:

- `DCO`
- `semantic-pull-request / semantic-pull-request`
- `pre-commit.ci - pr`
- `pre-commit-optional`
- `spell-check-differential`
- `build-and-test-differential (humble)`
- `build-and-test-differential (jazzy)`
- Codecov patch/project checks
- Mergify checks

Local pre-commit config includes:

- markdown/yaml/json/xml checks
- markdownlint
- prettier
- yamllint
- package dependency checks
- ROS package formatting hooks
- shellcheck/shfmt
- black/isort
- `clang-format`
- `cpplint`

The common local failure risks for analyzer PRs are:

- missing `Signed-off-by`
- clang-format changes on C++ files
- cpplint include-order or line-length issues
- markdownlint on migration docs
- stale package dependencies if new dependencies are added
- differential build/test label missing on the PR

## PR Slice Order

Recommended order:

1. Common EPDMS infrastructure and shared helpers.
2. NC.
3. DAC.
4. DDC.
5. TLC.
6. TTC.
7. LK.
8. HC and EC, or split if the diff is large.
9. EP.
10. Aggregation, human-filtered outputs, docs, and cleanup.

Keep PRs stacked only where dependency is unavoidable. If a PR can be independent, branch it from
the latest `upstream/main`.
