function [selected_path, q_goal, candidate_report] = path_between_to_ik_solutions(q_start,options)
% Connect two IK configurations of the same tool pose.
% Interpolate the arm joints (q1:q3), then continue the wrist joints (q4:q6)
% while holding the tool orientation fixed. The path parameter s is not time.
% q_start is a six-element joint vector in radians (row or column).
% Its joint limits and wrist/Jacobian margins are checked before goal enumeration.
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
% Scale linear velocity by 1/L before comparing it with angular velocity.
jacobian_scale = [eye(3)/L, zeros(3,3); zeros(3,3), eye(3)];

outside = find(q_start < q_min | q_start > q_max);
if ~isempty(outside)
    error('path_between_to_ik_solutions:StartJointLimits', ...
        'The initial configuration violates joint limits (%s).', ...
        strjoin(compose("q%d",outside),", "));
end

% A regular wrist keeps the sign of sin(q5) along a continuous motion.
wrist_sine = sin(q_start(5));
if abs(wrist_sine) <= singular_tol
    error('path_between_to_ik_solutions:StartWristSingularity', ...
        'The initial wrist has abs(sin(q5)) = %.3g <= singular_tol = %.3g.', ...
        abs(wrist_sine),singular_tol);
end
wrist_sign = sign(wrist_sine);

poe_model = poe_data;
dh_model = dh_data;
[T_d,joint_positions,joint_axes] = poe_fk(q_start,poe_model);
R_d = T_d(1:3,1:3);         % Desired tool orientation throughout the path.

% Use the same conditioning threshold as endpoint and sampled-path validation.
J_start = point_jacobian(joint_positions,joint_axes,joint_positions(:,6),eye(3));
sigma_start = min(svd(jacobian_scale * J_start));
if sigma_start <= sigma_tol
    error('path_between_to_ik_solutions:StartJacobianMargin', ...
        'The initial configuration has scaled sigma_min = %.3g <= sigma_tol = %.3g.', ...
        sigma_start,sigma_tol);
end

% The determinant labels the arm branch after the conditioning check.
J_wrist_start = point_jacobian(joint_positions,joint_axes,joint_positions(:,5),eye(3));
arm_det_start = det(J_wrist_start(1:3,1:3));
arm_sign = sign(arm_det_start);
if arm_sign == 0
    error('path_between_to_ik_solutions:StartArmSign', ...
        'A nonzero initial arm determinant sign could not be determined.');
end

%% 2. Enumerate and screen candidate goals
Q_ik = analytical_ik_6r(T_d, poe_model, dh_model);

settings = struct('s_samples',linspace(0,1,options.n_samples), ...
    'q_min',q_min,'q_max',q_max,'arm_sign',arm_sign,'wrist_sign',wrist_sign, ...
    'jacobian_scale',jacobian_scale,'tool_offset',[dh_model.d(6); 0; 0], ...
    'singular_tol',singular_tol,'sigma_tol',sigma_tol,'goal_tol',goal_tol);
[eligible_goals,candidate_status] = screen_ik_goals(Q_ik,q_start,poe_model,settings);

%% 3. Trace a separate path attempt for every eligible goal
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
