# Analyzer Implementation Notes

This repository contains analyzer logic where multiple subscores may inspect the same
trajectory data, map context, object state, and derived geometry.

## Required engineering rule for analyzer work

When implementing or refactoring analyzer metrics, especially EPDMS-related logic:

- Do not duplicate computation across analyzer functions, metrics, or subscores unless it is
  genuinely unavoidable.
- Prefer computing shared derived artifacts once and reusing them when the inputs,
  sampling points, and semantics are the same.
- This applies not only to ego footprints, but also to any repeated map queries, route
  queries, trajectory-derived values, object-track preprocessing, polygon generation,
  candidate-set construction, and similar intermediate computation.
- Keep debug behavior intact. Removing duplicated computation must not remove or degrade
  debug outputs, debug timing, or debug-specific geometry when those are intentionally
  produced.
- Do not change intended metric semantics just to reduce computation.
- Any deduplication must preserve current observable behavior unless the change explicitly
  targets a verified bug.

## Practical guidance

- Reuse helper functions and cached shared artifacts before adding new per-metric loops that
  rebuild the same result.
- If a metric needs different sampling times, projection logic, or intentionally different
  semantics, separate computation is acceptable only to that extent.
- When deduplicating, verify that score outputs, infraction timing, availability logic, and
  debug artifacts remain consistent.
- When updating any subscore with the user, also update the corresponding migrated
  Autoware equation/explanation section in
  `planning/autoware_planning_data_analyzer/implementation_report.md`.
- Keep that report section aligned with the actual implementation and match the notation,
  display-math style, structure, and writing style already used by neighboring metric
  sections in the report.
- When a subscore logic update changes which lanelets, polygons, or road-space categories
  are admitted or excluded, explicitly document those admitted/excluded spaces in the
  corresponding `implementation_report.md` section rather than only describing the
  high-level score equation.
- For traffic-control metrics such as TLC, that documentation must also make the
  movement-selection rule explicit: which approach lanes, connector-like lanelets,
  turn-direction-specific polygons, and neighboring same-movement polygons are included
  or excluded.
- When an Autoware-side subscore implementation is modified and the corresponding
  debugging support is added or changed, run the full analyzer pipeline with
  `open_loop.enabled_metrics` expanded cumulatively to include all already-integrated
  subscores plus the new one, unless the user explicitly asks for isolated validation.
- The intended cumulative chain is additive over time. Example:
  - after NC: `['nc']`
  - after DAC: `['nc','dac']`
  - after DDC: `['nc','dac','ddc']`
  - after the next subscore: `['nc','dac','ddc','<next>']`
- After that accumulated run, verify that:
  - the newly added subscore result topics are produced,
  - the corresponding new debug topics are produced, and
  - the previously integrated subscores still produce consistent results and debug topics.
- The purpose of that accumulated run is to catch regressions where a new subscore or
  debug path breaks existing subscores.
- When patching equations in `implementation_report.md`, prefer the subset of math that
  actually renders cleanly in the user's Markdown preview. If preview rendering is mixed,
  do not broadly rewrite unrelated equations. Patch only the failing expressions and
  prefer preview-safe named predicates like `\mathrm{Overlap}(...)` over unsupported
  symbols such as `\cap`, `\ne\varnothing`, `\notin`, or `\widehat` when those display
  raw in preview.
- When adding or changing debugging methods, debug outputs, or debug-specific logic,
  also update
  `planning/autoware_planning_data_analyzer/debugging_explanation.md`
  so the debugging documentation stays aligned with the implementation.
- When implementing or discussing debugging workflows in Lichtblick, refer to the
  actual local Lichtblick plugin code under
  `~/workspace/AutowareLichtblickPlugins/src/` and name the concrete panels,
  converters, topics, and expected 3D/timeline behavior being used.
- When adding a new Lichtblick panel or related plugin-side materials for a subscore,
  do that work in the Lichtblick repo `~/workspace/AutowareLichtblickPlugins` on a
  dedicated branch. Commit and push that plugin branch to the `myrepo` remote before
  moving on.
- When refining DAC semantics, explicitly consider whether lane polygons alone are too
  strict near road-border-adjacent paved area. Prefer adding genuine map-supported
  drivable border area when justified, but do not broadly accept arbitrary space
  outside lane boundaries.
- For DDC semantics, do not conflate generic non-drivable side space with actual
  oncoming traffic. Sidewalk / pedestrian-side / curbside non-drivable intrusion should
  be handled by DAC-style drivable-area logic, while DDC should only count meaningful
  wrong-way progress in opposite-direction vehicle-travel space.
- NC future issue noted for later investigation:
  - If the planned future ego footprint overlaps a pedestrian or other object that
    remains effectively static over the evaluated future horizon, NC is expected to
    fail (`0`) because NC is a future time-matched collision metric, not only a
    current-time collision metric.
  - A visual overlap in the combined 4-second footprint ribbon alone is not sufficient
    evidence, but if the object is truly static and future time-matched sampled
    footprints also overlap, then `NC=1` is suspicious.
  - The concrete raised case is around `1774401659.95`, where TTC fails while NC stays
    `1.0` even though the user observed a static pedestrian apparently overlapped by
    the planned future horizon.
  - When revisiting NC, explicitly inspect whether the issue comes from object
    interpolation, object track continuity, sampled-time mismatch, collision
    classification filtering, or inconsistency between the 4-second debug horizon and
    the exact future samples NC checks.
- When moving on to a different subscore, aggregated full EPDMS work, or human-filtered
  work, first commit the current state to the corresponding subscore branch, then create
  the next working branch on top of that committed state. Push and manage those subscore
  branches on the `kim` remote.
