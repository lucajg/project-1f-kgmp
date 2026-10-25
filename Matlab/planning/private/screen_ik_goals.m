function [eligible_goals,candidate_status] = screen_ik_goals(Q_ik,q_start,poe_model,settings)
% Screen IK endpoints whose target pose has already been checked by the solver.
% settings supplies joint limits, initial arm/wrist signs, Jacobian scale,
% and singular_tol, sigma_tol, goal_tol. q_start is a six-element column.
% Eligibility is a row mask; status is a column of all rejection reasons.
% Eligible endpoints have an empty status until their path attempt is evaluated.
n_goals = size(Q_ik,2);
eligible_goals = false(1,n_goals);
candidate_status = strings(n_goals,1);

for i = 1:n_goals
    q_goal = Q_ik(:,i);
    % FK and Jacobian checks have no meaning for nonfinite configurations.
    if any(~isfinite(q_goal))
        candidate_status(i) = "Nonfinite joint angles";
        continue
    end

    % Keep every applicable reason, even when an earlier condition fails.
    reasons = strings(0,1);
    delta = atan2(sin(q_goal-q_start),cos(q_goal-q_start));
    if max(abs(delta)) <= settings.goal_tol
        reasons(end+1) = "Starting posture (modulo 2*pi)";
    end
    if any(q_goal < settings.q_min | q_goal > settings.q_max)
        reasons(end+1) = "Joint-limit violation";
    end
    wrist_sine = sin(q_goal(5));
    if abs(wrist_sine) <= settings.singular_tol
        reasons(end+1) = "Wrist singularity threshold violated";
    end
    if sign(wrist_sine) == -settings.wrist_sign
        reasons(end+1) = "Opposite wrist sign";
    end

    % Both endpoint Jacobians use the same joint origins and axes in the base frame.
    [~,joint_positions,joint_axes] = poe_fk(q_goal,poe_model);
    J_wrist = point_jacobian(joint_positions,joint_axes,joint_positions(:,5),eye(3));
    J = point_jacobian(joint_positions,joint_axes,joint_positions(:,6),eye(3));
    if sign(det(J_wrist(1:3,1:3))) ~= settings.arm_sign
        reasons(end+1) = "Opposite or singular arm sign";
    end
    if min(svd(settings.jacobian_scale * J)) <= settings.sigma_tol
        reasons(end+1) = "Endpoint sigma_min at or below threshold";
    end

    eligible_goals(i) = isempty(reasons);
    if ~eligible_goals(i)
        candidate_status(i) = strjoin(reasons,"; ");
    end
end
end
