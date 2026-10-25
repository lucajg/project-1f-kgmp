function [selected_path, q_goal, candidate_report] = path_between_to_ik_solutions(q_start,options)
% Connect two IK configurations of the same tool pose.
% Interpolate the arm joints (q1:q3), then continue the wrist joints (q4:q6)
% while holding the tool orientation fixed. The path parameter s is not time.
% q_start is a six-element joint vector in radians (row or column).
% Return the selected trajectory and diagnostics, its goal, and candidate outcomes.
% selected_path.s contains the path parameter for each column of selected_path.q.
% Optional named inputs and defaults:
%   n_samples=10000, singular_tol=1e-6, sigma_tol=1e-3, goal_tol=1e-6,
%   plot_results=true, verbose=true.
% If every attempt fails, plot_results shows their valid prefixes before erroring.
% Example: path_between_to_ik_solutions(q_start,n_samples=5001,plot_results=false)

arguments
    q_start
    options.n_samples (1,1) double ...
        {mustBeReal,mustBeFinite,mustBeInteger,mustBeGreaterThanOrEqual(options.n_samples,2)} = 10000
    options.singular_tol (1,1) double {mustBeReal,mustBeFinite,mustBeNonnegative} = 1e-6
    options.sigma_tol (1,1) double {mustBeReal,mustBeFinite,mustBeNonnegative} = 1e-3
    options.goal_tol (1,1) double {mustBeReal,mustBeFinite,mustBeNonnegative} = 1e-6
    options.plot_results (1,1) logical = true
    options.verbose (1,1) logical = true
end

%% 1. Validate the start configuration and choose numerical settings
validateattributes(q_start, {'numeric'}, {'real','finite','vector','numel',6});
q_start = q_start(:);
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

%% 3. Trace a separate path attempt for every eligible goal
settings = struct('s_samples',linspace(0,1,options.n_samples), ...
    'q_min',q_min,'q_max',q_max,'arm_sign',arm_sign, ...
    'jacobian_scale',jacobian_scale,'tool_offset',[dh_model.d(6); 0; 0], ...
    'singular_tol',singular_tol,'sigma_tol',sigma_tol,'goal_tol',goal_tol);
goal_index = [];
failed_attempts = cell(1,size(Q_ik,2));
for i = find(eligible_goals)
    attempt = trace_arm_segment(q_start,Q_ik(:,i),R_d,poe_model,settings);
    if strlength(attempt.reason) > 0
        candidate_status(i) = "Path rejected: " + attempt.reason;
        if options.plot_results
            failed_attempts{i} = attempt;
        end
        continue
    end
    candidate_status(i) = "Path passed";
    % Keep the first success, while still testing all remaining candidates.
    if isempty(goal_index)
        goal_index = i;
        selected_path = attempt.path;
    end
end

%% 4. Select a validated path and report path closure
candidate_report = table((1:size(Q_ik,2)).',candidate_status, ...
    'VariableNames',{'IK_index','Status'});
if options.verbose
    disp(candidate_report);
end
if isempty(goal_index)
    if options.plot_results
        for i = find(~cellfun(@isempty,failed_attempts))
            plot_failed_path(failed_attempts{i},i,Q_ik,poe_model, ...
                q_min,q_max,sigma_tol);
        end
        drawnow;
    end
    error('No candidate passed straight-arm-path validation. A different arm path may still exist.');
end
if options.verbose
    fprintf('Selected IK solution %d after sampled path validation.\n',goal_index);
end
q_goal = Q_ik(:,goal_index);

if options.verbose
    position_closure = norm(selected_path.p(:,end) - selected_path.p(:,1))
    joint_change = norm(selected_path.q(:,end) - selected_path.q(:,1))
end

%% 5. Plot the joint path, tool path, and singularity margin
if options.plot_results
    plot_selected_path(selected_path,Q_ik,poe_model,sigma_tol);
end

%% 6. Report the worst sampled singularity margin and pose errors
if options.verbose
    [sigma_worst, k_worst] = min(selected_path.sigma_min);
    sigma_worst
    s_worst = selected_path.s(k_worst)
    max_orientation_error_angle = max(selected_path.theta)
    max_discrepancy_desired_vs_planned_position = max(selected_path.error_position)
end
end
