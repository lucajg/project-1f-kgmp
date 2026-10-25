function [selected_path, q_goal, candidate_report] = path_between_to_ik_solutions(q_start,options)
% Connect two IK configurations of the same tool pose.
% Interpolate the arm joints (q1:q3), then continue the wrist joints (q4:q6)
% while holding the tool orientation fixed. The path parameter s is not time.
% q_start is a six-element joint vector in radians (row or column).
% Return the selected trajectory and diagnostics, its goal, and candidate outcomes.
% Optional named inputs and defaults:
%   n_samples=19999, singular_tol=1e-6, sigma_tol=1e-3, goal_tol=1e-6,
%   plot_results=true, verbose=true.
% If every attempt fails, plot_results shows their valid prefixes before erroring.
% Example: path_between_to_ik_solutions(q_start,n_samples=5001,plot_results=false)

arguments
    q_start
    options.n_samples (1,1) double ...
        {mustBeReal,mustBeFinite,mustBeInteger,mustBeGreaterThanOrEqual(options.n_samples,2)} = 19999
    options.singular_tol (1,1) double {mustBeReal,mustBeFinite,mustBeNonnegative} = 1e-6
    options.sigma_tol (1,1) double {mustBeReal,mustBeFinite,mustBeNonnegative} = 1e-3
    options.goal_tol (1,1) double {mustBeReal,mustBeFinite,mustBeNonnegative} = 1e-6
    options.plot_results (1,1) logical = true
    options.verbose (1,1) logical = true
end

%% 1. Validate the start configuration and choose numerical settings
validateattributes(q_start, {'numeric'}, {'real','finite','vector','numel',6});
q_start = q_start(:);
n_samples = options.n_samples;
s_min = 0;
s_max = 1;
singular_tol = options.singular_tol; % Threshold on abs(sin(q5)).
sigma_tol = options.sigma_tol;       % Threshold for the scaled Jacobian (L = 2 m).
goal_tol = options.goal_tol;         % Maximum final joint error, in radians.
q_min = -pi * ones(6,1);
q_max =  pi * ones(6,1);
L = 2;                     % Characteristic length in metres.

% A regular wrist keeps the sign of sin(q5) along a continuous motion.
if abs(sin(q_start(5))) <= singular_tol
    error('The initial wrist is singular; no regular wrist sign is defined.');
end
wrist_sign = sign(sin(q_start(5)));

%% 2. Enumerate and screen candidate goals
poe_model = poe_data;
dh_model = dh_data;
T_d = poe_fk(q_start, poe_model);
R_d = T_d(1:3,1:3);         % Desired tool orientation throughout the path.

% The arm Jacobian maps q1:q3 velocities to wrist-centre linear velocity.
J_wrist_start = poe_jacobian(q_start, 0, 5, poe_model);
arm_det_start = det(J_wrist_start(1:3,1:3));
arm_sign = sign(arm_det_start);
if arm_sign == 0
    error('The initial arm Jacobian is singular; no regular arm sign is defined.');
end

Q_ik = analytical_ik_6r(T_d, poe_model, dh_model);

% Scale linear velocity by 1/L before comparing it with angular velocity.
jacobian_scale = [eye(3)/L, zeros(3,3); zeros(3,3), eye(3)];
% Exclude the starting posture (modulo 2*pi), invalid endpoints, and the
% opposite or singular wrist branch. The IK solver already verifies the pose.
delta = atan2(sin(Q_ik-q_start),cos(Q_ik-q_start));
same_start = max(abs(delta),[],1) <= goal_tol;
finite_angles = all(isfinite(Q_ik),1);
within_limits = finite_angles & all(Q_ik >= q_min & Q_ik <= q_max,1);
same_wrist_sign = sign(sin(Q_ik(5,:))) == wrist_sign;
regular_wrist = abs(sin(Q_ik(5,:))) > singular_tol;
eligible_goals = ~same_start & within_limits & same_wrist_sign & regular_wrist;
candidate_status = strings(size(Q_ik,2),1);
candidate_status(~finite_angles) = "Nonfinite joint angles";

% Collect every applicable endpoint reason, even after an earlier check fails.
% Nonfinite configurations cannot meaningfully be checked by FK/Jacobians.
for i = find(finite_angles)
    reasons = strings(0,1);
    if same_start(i)
        reasons(end+1) = "Starting posture (modulo 2*pi)";
    end
    if ~within_limits(i)
        reasons(end+1) = "Joint-limit violation";
    end
    if ~regular_wrist(i)
        reasons(end+1) = "Wrist singularity threshold violated";
    end
    if sign(sin(Q_ik(5,i))) == -wrist_sign
        reasons(end+1) = "Opposite wrist sign";
    end
    J_wrist = poe_jacobian(Q_ik(:,i),0,5,poe_model);
    J = poe_jacobian(Q_ik(:,i),0,6,poe_model);
    if sign(det(J_wrist(1:3,1:3))) ~= arm_sign
        reasons(end+1) = "Opposite or singular arm sign";
    end
    if min(svd(jacobian_scale * J)) <= sigma_tol
        reasons(end+1) = "Endpoint sigma_min at or below threshold";
    end
    if ~isempty(reasons)
        eligible_goals(i) = false;
        candidate_status(i) = strjoin(reasons,"; ");
    end
end

%% 3. Allocate the sampled path and its diagnostics
s_samples = linspace(s_min,s_max,n_samples);
q_planned = zeros(6,n_samples);       % Joint configurations, one per column.
p_planned = zeros(3,n_samples);       % Tool positions in the base frame.
joint_margin = nan(1,n_samples);      % Smallest distance to a joint limit.
sigma_min = zeros(1,n_samples);       % Smallest scaled Jacobian singular value.
arm_det = zeros(1,n_samples);         % Determinant of the wrist-centre arm Jacobian.
theta = zeros(1,n_samples);           % Tool orientation error, in radians.
error_position = zeros(1,n_samples);  % Tool position consistency error, in metres.

tool_offset = [dh_model.d(6); 0; 0];  % Wrist-to-tool offset in tool coordinates.

%% 4. Try the original wrist continuation for every eligible goal
% Reuse the sample buffers; only a complete, validated path can be selected.
% These checks validate sampled configurations, not the intervals between them.
goal_index = [];
failed_attempts = cell(1,size(Q_ik,2));
for i = find(eligible_goals)
    q_candidate = Q_ik(:,i);
    q_arm_at_s = @(s) (1-s)*q_start(1:3) + s*q_candidate(1:3);
    q_previous = q_start;
    rejection_reason = "";
    n_valid = 0;
    rejected_q = zeros(6,0);
    rejected_sigma = NaN;
    for k = 1:n_samples
        s_k = s_samples(k);
        q_arm = q_arm_at_s(s_k);

        % Solve orientation IK for this arm configuration.
        [wrist_solutions, is_singular] = analytical_ik_wrist( ...
            q_arm, R_d, poe_model, singular_tol);
        if is_singular
            rejection_reason = sprintf('Wrist singularity at s = %.6f.',s_k);
            break
        end

        % Express both solutions near the previous wrist angles. atan2(sin,cos)
        % wraps each angular difference to [-pi, pi], removing 2*pi jumps.
        delta = wrist_solutions - q_previous(4:6);
        wrist_candidates = q_previous(4:6) + atan2(sin(delta),cos(delta));
        q_candidates = [repmat(q_arm,1,size(wrist_candidates,2)); wrist_candidates];

        % Keep only candidates within all six joint limits and on the same
        % regular wrist branch as the start configuration.
        within_limits = all(q_candidates >= q_min & q_candidates <= q_max,1);
        same_wrist_sign = sign(sin(wrist_candidates(2,:))) == wrist_sign;
        valid = within_limits & same_wrist_sign;
        if ~any(valid)
            rejected_q = q_candidates; % Keep both unwrapped wrist candidates.
            wrist_reasons = strings(1,size(q_candidates,2));
            for branch = 1:size(q_candidates,2)
                reasons = strings(0,1);
                if ~within_limits(branch)
                    outside = find(q_candidates(:,branch) < q_min | q_candidates(:,branch) > q_max);
                    reasons(end+1) = "joint limits (" + strjoin(compose("q%d",outside),", ") + ")";
                end
                if ~same_wrist_sign(branch)
                    reasons(end+1) = "opposite wrist sign";
                end
                wrist_reasons(branch) = sprintf('Wrist candidate %d: %s',branch,strjoin(reasons,", "));
            end
            rejection_reason = sprintf('No admissible wrist candidate at s = %.6f; %s', ...
                s_k,strjoin(wrist_reasons,"; "));
            break
        end
        q_candidates = q_candidates(:,valid);

        % Choose the admissible wrist requiring the smallest joint-space step.
        wrist_distance = vecnorm(q_candidates(4:6,:)-q_previous(4:6),2,1);
        [~, nearest] = min(wrist_distance);
        q_current = q_candidates(:,nearest);

        % Both wrist candidates share this arm Jacobian. Its determinant must
        % keep the initial nonzero sign at every sampled configuration.
        J_wrist = poe_jacobian(q_current, 0, 5, poe_model);
        arm_det(k) = det(J_wrist(1:3,1:3));
        reasons = strings(0,1);
        if sign(arm_det(k)) ~= arm_sign
            reasons(end+1) = sprintf('Arm determinant changed sign at s = %.6f.',s_k);
        end

        % Check both Jacobian conditions before rejecting this configuration.
        J = poe_jacobian(q_current, 0, 6, poe_model);
        J_tilde = jacobian_scale * J;
        sigma_min(k) = min(svd(J_tilde));
        if sigma_min(k) <= sigma_tol
            reasons(end+1) = sprintf('sigma_min = %.3g <= %.3g at s = %.6f.', ...
                sigma_min(k), sigma_tol, s_k);
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

        % Verify the tool orientation and record its position using FK.
        [T_k, joint_positions] = poe_fk(q_planned(:,k), poe_model);
        R_k = T_k(1:3,1:3);
        R_error = R_d' * R_k;
        cos_theta = (trace(R_error)-1)/2;
        theta(k) = acos(min(max(cos_theta,-1),1));  % Clamp numerical roundoff.
        p_planned(:,k) = T_k(1:3,4);

        % Consistency with the current wrist centre and desired orientation;
        % this does not prescribe an independent Cartesian position path.
        p_desired = joint_positions(:,5) + R_d * tool_offset;
        error_position(k) = norm(p_planned(:,k)-p_desired);
        n_valid = k; % Advance only after this sample has completed every check.
    end

    if strlength(rejection_reason) == 0
        % Check actual joint angles: a 2*pi offset is distinct with bounded joints.
        goal_error = norm(q_planned(:,end)-q_candidate,inf);
        if goal_error > goal_tol
            rejection_reason = sprintf('Path misses goal by %.3g rad.',goal_error);
        end
    end
    if strlength(rejection_reason) > 0
        candidate_status(i) = "Path rejected: " + rejection_reason;
        if options.plot_results
            % Never retain unfilled columns or samples left over from another goal.
            failed_attempts{i} = struct('s',s_samples(1:n_valid), ...
                'q',q_planned(:,1:n_valid),'p',p_planned(:,1:n_valid), ...
                'sigma_min',sigma_min(1:n_valid),'reason',rejection_reason, ...
                's_failed',s_k,'arm_failed',q_arm, ...
                'rejected_q',rejected_q,'rejected_sigma',rejected_sigma);
        end
        continue
    end
    candidate_status(i) = "Path passed";
    % Keep the first success, while still testing all remaining candidates.
    if isempty(goal_index)
        goal_index = i;
        selected_path = struct('q',q_planned,'p',p_planned, ...
            'joint_margin',joint_margin,'sigma_min',sigma_min,'arm_det',arm_det, ...
            'theta',theta,'error_position',error_position,'goal_error',goal_error);
    end
end

%% 5. Select a validated path and report path closure
candidate_report = table((1:size(Q_ik,2)).',candidate_status, ...
    'VariableNames',{'IK_index','Status'});
if options.verbose
    disp(candidate_report);
end
if isempty(goal_index)
    if options.plot_results
        for i = find(~cellfun(@isempty,failed_attempts))
            plot_failed_path(failed_attempts{i},i,Q_ik,poe_model, ...
                q_min,q_max,wrist_sign,sigma_tol);
        end
        drawnow;
    end
    error('No candidate passed straight-arm-path validation. A different arm path may still exist.');
end
if options.verbose
    fprintf('Selected IK solution %d after sampled path validation.\n',goal_index);
end
q_goal = Q_ik(:,goal_index);
q_planned = selected_path.q;
p_planned = selected_path.p;
joint_margin = selected_path.joint_margin;
sigma_min = selected_path.sigma_min;
arm_det = selected_path.arm_det;
theta = selected_path.theta;
error_position = selected_path.error_position;
goal_error = selected_path.goal_error;

if options.verbose
    position_closure = norm(p_planned(:,end) - p_planned(:,1))
    joint_change = norm(q_planned(:,end) - q_planned(:,1))
end

%% 6. Plot the joint path, tool path, and singularity margin
if options.plot_results
    figure;
    subplot(1,2,1);
    plot(s_samples, q_planned.');
    xlabel('Path parameter s');
    ylabel('Joint angle (rad)');
    legend('q_1','q_2','q_3','q_4','q_5','q_6');
    title('Joint space');
    grid on;

    subplot(1,2,2);
    plot3(p_planned(1,:), p_planned(2,:), p_planned(3,:));
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
    title('Task space');
    axis equal;
    grid on;

    figure;
    plot(s_samples, sigma_min);
    yline(sigma_tol,'r--','Threshold');
    xlabel('Path parameter s');
    ylabel('$\sigma_{\min}(\tilde{J})$','Interpreter','latex');
    grid on;

    figure;
    plot(s_samples, joint_margin);
    yline(0,'r--');
    xlabel('Path parameter s');
    ylabel('Minimum joint-limit clearance (rad)');
    grid on;

    % Project all target-pose IK solutions and the selected path onto (q2,q3).
    ik_path_figure = plot_ik_path(Q_ik, q_planned, poe_model);
end

%% 7. Report the worst sampled singularity margin and pose errors
if options.verbose
    [sigma_worst, k_worst] = min(sigma_min);
    sigma_worst
    s_worst = s_samples(k_worst)
    max_orientation_error_angle = max(theta)
    max_discrepancy_desired_vs_planned_position = max(error_position)
end
end
