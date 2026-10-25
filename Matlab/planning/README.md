# Paths between IK solutions

[MATLAB setup and checks](../README.md)

`path_between_to_ik_solutions` searches for another IK configuration of the starting tool pose. Each attempt interpolates the arm joints and continues the initial wrist branch while preserving tool orientation. Intermediate tool positions may move. Validation checks the dense sample grid, not the intervals between samples.

```matlab
[path, goal, report] = path_between_to_ik_solutions(q_start, ...
    plot_results=false, verbose=false);
if isempty(path)
    disp(report(:,{'IK_index','Outcome','Status'}));
else
    plot(path.s, path.q.');
end
```

An unsuccessful search returns `path=[]` and `goal=[]`, together with the complete candidate report. It does not throw an exception. Invalid inputs and inadmissible starts still raise errors before IK enumeration. A failed search does not rule out a different arm path.

## Candidate report

Every row corresponds to one IK solution, in solver order. Existing `IK_index` and `Status` columns are retained.

| Column | Meaning |
| --- | --- |
| `IK_index` | Candidate index in the IK solution matrix. |
| `Status` | Human-readable outcome, including every applicable rejection reason. |
| `Outcome` | `"endpoint_rejected"`, `"path_rejected"`, or `"passed"`. |
| `ReasonCodes` | Cell containing a string column of rejection codes; empty for a passed path. |
| `Goal` | Six joint angles in radians, stored as one row per candidate. |
| `Attempt` | Cell containing the trace result; `[]` if endpoint screening prevented an attempt. |
| `Selected` | True for the first passed path that was returned; all false if none passed. |

Reason codes are `same_start`, `nonfinite_angles`, `joint_limits`, `wrist_singularity`, `opposite_wrist_sign`, `arm_sign`, `jacobian_margin`, and `goal_mismatch`. Multiple codes may apply. Use `Outcome` and `ReasonCodes` for programmatic decisions instead of parsing `Status`.

Attempt details are retained regardless of `plot_results` or `verbose`:

```matlab
i = find(report.Outcome == "path_rejected", 1);
if ~isempty(i)
    attempt = report.Attempt{i};
    s_failed = attempt.s_failed;
    valid_prefix = attempt.path.q;
    rejected_configuration = attempt.rejected_q;
end
```

`attempt.path` contains the completed admissible samples and their diagnostics, possibly an empty prefix. A rejected sample is never appended to it. A `goal_mismatch` retains the complete sampled path, with the mismatch in `attempt.path.goal_error`.

The attempt also contains `reason`, `reason_codes`, `s_failed`, `arm_failed`, `rejected_q`, `rejected_sigma`, `rejected_arm_det`, and `violated_joints`. `violated_joints` lists one-based joint indices for a limit violation. Unavailable numerical failure measurements are `NaN`; unavailable configurations and joint-index lists are empty. Passed attempts have empty reasons and `s_failed=NaN`.

## Console and figures

`verbose=true` prints a short success summary with margins, endpoint closure, and pose residuals. On failure it prints attempted-path rejection reasons, or endpoint reasons when no candidate was eligible. `verbose=false` suppresses planner text. The returned data is independent of verbosity.

`plot_results=true` shows the selected path in one figure, or valid prefixes of failed attempts when no path passed. Turning plotting off does not discard diagnostics.

## Checks

Run `check_path_planning` after `curo6r_setup`. It covers start validation, a regular transition, unsuccessful searches with and without path attempts, retained failure data, and quiet/formatted output. It is included in `run_checks`.
