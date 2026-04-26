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
- When an Autoware-side subscore implementation is modified and the corresponding
  debugging support is added or changed, run the full analyzer pipeline with
  `open_loop.enabled_metrics` expanded cumulatively to include all already-integrated
  subscores plus the new one, unless the user explicitly asks for isolated validation.
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
- When moving on to a different subscore, aggregated full EPDMS work, or human-filtered
  work, first commit the current state to the corresponding subscore branch, then create
  the next working branch on top of that committed state. Push and manage those subscore
  branches on the `kim` remote.
