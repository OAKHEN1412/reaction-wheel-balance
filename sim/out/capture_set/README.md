# capture_set (Codex, 2026-09-22) — FAILED VALIDATION, not a deliverable

`campaign_validation.csv`: the model predicts "fell" for all 8 known launches,
including the 3 that balanced for 29–57 s. It scores 5/8 only by guessing one
class. `capture_grid.csv` (22,950 states) therefore describes a machine that
cannot recover from anything and must not be used to choose jump parameters.
Kept so the failure is inspectable. The later attempt, `../nano_replica/`,
failed in the opposite direction (predicts "held" for everything); see its
README for what it did establish.
