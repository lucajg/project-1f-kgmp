function [selected_path, q_goal, candidate_report] = path_between_to_ik_solutions(q_start,options)
% Connect two IK configurations of the same tool pose.
% Interpolate the arm joints (q1:q3), then continue the wrist joints (q4:q6)
% while holding the tool orientation fixed. The path parameter s is not time.
% q_start is a six-element joint vector in radians (row or column).
% Its joint limits and wrist/Jacobian margins are checked before goal enumeration.
% Return the selected trajectory and diagnostics, its goal, and candidate outcomes.
% selected_path.s contains the path parameter for each column of selected_path.q.
% If no path passes, selected_path and q_goal are []; candidate_report is returned.
% The report includes Outcome, ReasonCodes, Goal (one row per candidate), Attempt
% (a cell containing the trace result or []), and Selected, besides IK_index/Status.
% Attempt details are available even with plotting disabled. Invalid starts error.
% Optional named inputs and defaults:
%   n_samples=10000, singular_tol=1e-6, sigma_tol=1e-3, goal_tol=1e-6,
%   plot_results=true, verbose=true.
% If every attempt fails, plot_results shows their valid prefixes before returning.
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
J_arm_start = cross(joint_axes(:,1:3),joint_positions(:,5)-joint_positions(:,1:3),1);
arm_det_start = det(J_arm_start);
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
[eligible_goals,candidate_status,candidate_codes] = screen_ik_goals( ...
    Q_ik,q_start,poe_model,settings);

%% 3. Trace a separate path attempt for every eligible goal
selected_path = [];
q_goal = [];
goal_index = [];
n_goals = size(Q_ik,2);
candidate_outcome = repmat("endpoint_rejected",n_goals,1);
candidate_attempts = cell(n_goals,1);
for i = find(eligible_goals)
    attempt = trace_arm_segment(q_start,Q_ik(:,i),R_d,poe_model,settings);
    candidate_attempts{i} = attempt;
    candidate_codes{i} = attempt.reason_codes;
    if strlength(attempt.reason) > 0
        candidate_outcome(i) = "path_rejected";
        candidate_status(i) = "Path rejected: " + attempt.reason;
        continue
    end
    candidate_outcome(i) = "passed";
    candidate_status(i) = "Path passed";
    % Keep the first success, while still testing all remaining candidates.
    if isempty(goal_index)
        goal_index = i;
        selected_path = attempt.path;
    end
end

%% 4. Return all outcomes, including failed attempts independently of plotting
selected = false(n_goals,1);
if ~isempty(goal_index)
    q_goal = Q_ik(:,goal_index);
    selected(goal_index) = true;
end
candidate_report = table((1:n_goals).',candidate_status,candidate_outcome, ...
    candidate_codes,Q_ik.',candidate_attempts,selected, ...
    'VariableNames',{'IK_index','Status','Outcome','ReasonCodes','Goal','Attempt','Selected'});

%% 5. Present the result
if options.verbose
    print_planning_summary(selected_path,candidate_report);
end
if options.plot_results
    if isempty(selected_path)
        for i = find(candidate_outcome == "path_rejected").'
            plot_failed_path(candidate_attempts{i},i,Q_ik,poe_model, ...
                q_min,q_max,sigma_tol);
        end
        drawnow;
    else
        plot_selected_path(selected_path,Q_ik,poe_model,sigma_tol);
    end
end
end
