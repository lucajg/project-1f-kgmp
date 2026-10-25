function [eligible_goals,candidate_status,candidate_codes] = screen_ik_goals(Q_ik,q_start,poe_model,settings)
% Screen IK endpoints whose target pose has already been checked by the solver.
% settings supplies joint limits, initial arm/wrist signs, Jacobian scale,
% and singular_tol, sigma_tol, goal_tol. q_start is a six-element column.
% Eligibility is a row mask; status is a column of all rejection reasons.
% Eligible endpoints have an empty status until their path attempt is evaluated.
% candidate_codes contains a string column of machine-readable reasons per goal.
n_goals = size(Q_ik,2);
eligible_goals = false(1,n_goals);
candidate_status = strings(n_goals,1);
candidate_codes = repmat({strings(0,1)},n_goals,1);

for i = 1:n_goals
    q_goal = Q_ik(:,i);
    % FK and Jacobian checks have no meaning for nonfinite configurations.
    if any(~isfinite(q_goal))
        candidate_status(i) = "Nonfinite joint angles";
        candidate_codes{i} = "nonfinite_angles";
        continue
    end

    % Keep every applicable reason, even when an earlier condition fails.
    reasons = strings(0,1);
    codes = strings(0,1);
    delta = atan2(sin(q_goal-q_start),cos(q_goal-q_start));
    if max(abs(delta)) <= settings.goal_tol
        reasons(end+1) = "Starting posture (modulo 2*pi)";
        codes(end+1,1) = "same_start";
    end
    if any(q_goal < settings.q_min | q_goal > settings.q_max)
        reasons(end+1) = "Joint-limit violation";
        codes(end+1,1) = "joint_limits";
    end
    wrist_sine = sin(q_goal(5));
    if abs(wrist_sine) <= settings.singular_tol
        reasons(end+1) = "Wrist singularity threshold violated";
        codes(end+1,1) = "wrist_singularity";
    end
    if sign(wrist_sine) == -settings.wrist_sign
        reasons(end+1) = "Opposite wrist sign";
        codes(end+1,1) = "opposite_wrist_sign";
    end

    % Only the 3x3 arm-position block is needed at the wrist centre.
    [~,joint_positions,joint_axes] = poe_fk(q_goal,poe_model);
    J_arm = cross(joint_axes(:,1:3),joint_positions(:,5)-joint_positions(:,1:3),1);
    J = point_jacobian(joint_positions,joint_axes,joint_positions(:,6),eye(3));
    if sign(det(J_arm)) ~= settings.arm_sign
        reasons(end+1) = "Opposite or singular arm sign";
        codes(end+1,1) = "arm_sign";
    end
    if min(svd(settings.jacobian_scale * J)) <= settings.sigma_tol
        reasons(end+1) = "Endpoint sigma_min at or below threshold";
        codes(end+1,1) = "jacobian_margin";
    end

    eligible_goals(i) = isempty(reasons);
    candidate_codes{i} = codes;
    if ~eligible_goals(i)
        candidate_status(i) = strjoin(reasons,"; ");
    end
end
end
