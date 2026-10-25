function attempt = trace_arm_segment(q_start,q_goal,R_d,poe_model,settings)
% Trace one straight arm segment while preserving the initial wrist branch.
% settings supplies s_samples, joint limits, Jacobian scale, initial arm sign,
% tool offset, and the singularity/goal tolerances from the caller.
% attempt.path contains only completed, admissible samples (possibly none).
% An empty attempt.reason indicates success; failure details stay outside path.
% Validation applies to sampled configurations, not the intervals between them.

s_samples = settings.s_samples;
n_samples = numel(s_samples);
q_min = settings.q_min;
q_max = settings.q_max;
q_planned = zeros(6,n_samples);
p_planned = zeros(3,n_samples);
joint_margin = nan(1,n_samples);
sigma_min = zeros(1,n_samples);
arm_det = zeros(1,n_samples);
theta = zeros(1,n_samples);
error_position = zeros(1,n_samples);

% Regular wrist IK returns positive sin(q5) first, negative second. Fixing
% that sign leaves exactly one branch; only its angle representation changes.
wrist_branch = 1 + (sin(q_start(5)) < 0);
q_previous = q_start;
rejection_reason = "";
n_valid = 0;
goal_error = NaN;
rejected_q = zeros(6,0);
rejected_sigma = NaN;

for k = 1:n_samples
    s_k = s_samples(k);
    q_arm = (1-s_k)*q_start(1:3) + s_k*q_goal(1:3);
    [wrist_solutions, is_singular] = analytical_ik_wrist( ...
        q_arm, R_d, poe_model, settings.singular_tol);
    if is_singular
        rejection_reason = sprintf('Wrist singularity at s = %.6f.',s_k);
        break
    end

    % Unwrap the selected branch near the previous angles to remove 2*pi jumps.
    wrist = wrist_solutions(:,wrist_branch);
    delta = wrist - q_previous(4:6);
    wrist = q_previous(4:6) + atan2(sin(delta),cos(delta));
    q_current = [q_arm; wrist];
    outside = find(q_current < q_min | q_current > q_max);
    if ~isempty(outside)
        rejected_q = q_current;
        rejection_reason = sprintf('Joint limits violated at s = %.6f (%s).', ...
            s_k,strjoin(compose("q%d",outside),", "));
        break
    end

    % Preserve the initial arm sign and check the full Jacobian margin.
    J_wrist = poe_jacobian(q_current, 0, 5, poe_model);
    arm_det(k) = det(J_wrist(1:3,1:3));
    reasons = strings(0,1);
    if sign(arm_det(k)) ~= settings.arm_sign
        reasons(end+1) = sprintf('Arm determinant changed sign at s = %.6f.',s_k);
    end
    J = poe_jacobian(q_current, 0, 6, poe_model);
    sigma_min(k) = min(svd(settings.jacobian_scale * J));
    if sigma_min(k) <= settings.sigma_tol
        reasons(end+1) = sprintf('sigma_min = %.3g <= %.3g at s = %.6f.', ...
            sigma_min(k), settings.sigma_tol, s_k);
    end
    if ~isempty(reasons)
        rejected_q = q_current;
        rejected_sigma = sigma_min(k);
        rejection_reason = strjoin(reasons,"; ");
        break
    end

    joint_margin(k) = min([q_current-q_min; q_max-q_current]);
    q_planned(:,k) = q_current;
    q_previous = q_current;

    % Record pose diagnostics; these residuals are not rejection criteria.
    [T_k, joint_positions] = poe_fk(q_current, poe_model);
    R_error = R_d' * T_k(1:3,1:3);
    cos_theta = (trace(R_error)-1)/2;
    theta(k) = acos(min(max(cos_theta,-1),1));
    p_planned(:,k) = T_k(1:3,4);

    % This is a consistency residual, not an independent Cartesian path target.
    p_desired = joint_positions(:,5) + R_d * settings.tool_offset;
    error_position(k) = norm(p_planned(:,k)-p_desired);
    n_valid = k;
end

if strlength(rejection_reason) == 0
    % A 2*pi offset is distinct with bounded joints, even at the same pose.
    goal_error = norm(q_planned(:,end)-q_goal,inf);
    if goal_error > settings.goal_tol
        rejection_reason = sprintf('Path misses goal by %.3g rad.',goal_error);
    end
end

% Trim every diagnostic to the valid prefix, excluding the rejected sample.
valid = 1:n_valid;
path = struct('s',s_samples(valid),'q',q_planned(:,valid),'p',p_planned(:,valid), ...
    'joint_margin',joint_margin(valid),'sigma_min',sigma_min(valid), ...
    'arm_det',arm_det(valid),'theta',theta(valid), ...
    'error_position',error_position(valid),'goal_error',goal_error);
attempt = struct('path',path,'reason',rejection_reason, ...
    's_failed',NaN,'arm_failed',zeros(3,0), ...
    'rejected_q',rejected_q,'rejected_sigma',rejected_sigma);
if strlength(rejection_reason) > 0
    attempt.s_failed = s_k;
    attempt.arm_failed = q_arm;
end
end
