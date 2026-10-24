% Connect two IK configurations of the same tool pose.
% Interpolate the arm joints (q1:q3), then continue the wrist joints (q4:q6)
% while holding the tool orientation fixed. The path parameter s is not time.

%% 1. Choose the start configuration and numerical settings
q_start = pi/2 * [1; -1; 1; -1; 1; -1];
goal_index = 7;             % Column of the full-pose IK solution matrix.
n_samples = 19999;
s_min = 0;
s_max = 1;
singular_tol = 1e-6;        % Threshold on abs(sin(q5)).
sigma_tol = 1e-3;           % Rejection threshold for the scaled Jacobian (L = 2 m).
goal_tol = 1e-6;            % Maximum final joint error, in radians.
q_min = -pi * ones(6,1);
q_max =  pi * ones(6,1);
L = 2;                     % Characteristic length in metres.

% A regular wrist keeps the sign of sin(q5) along a continuous motion.
if abs(sin(q_start(5))) <= singular_tol
    error('The initial wrist is singular; no regular wrist sign is defined.');
end
wrist_sign = sign(sin(q_start(5)));

%% 2. Find the goal configuration and define the arm path
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
q_goal = Q_ik(:,goal_index);
q_arm_at_s = @(s) (1-s)*q_start(1:3) + s*q_goal(1:3);

%% 3. Allocate the sampled path and its diagnostics
s_samples = linspace(s_min,s_max,n_samples);
q_planned = zeros(6,n_samples);       % Joint configurations, one per column.
p_planned = zeros(3,n_samples);       % Tool positions in the base frame.
joint_margin = nan(1,n_samples);      % Smallest distance to a joint limit.
sigma_min = zeros(1,n_samples);       % Smallest scaled Jacobian singular value.
arm_det = zeros(1,n_samples);         % Determinant of the wrist-centre arm Jacobian.
theta = zeros(1,n_samples);           % Tool orientation error, in radians.
error_position = zeros(1,n_samples);  % Tool position consistency error, in metres.

% Scale linear velocity by 1/L before comparing it with angular velocity.
jacobian_scale = [eye(3)/L, zeros(3,3); zeros(3,3), eye(3)];
tool_offset = [dh_model.d(6); 0; 0];  % Wrist-to-tool offset in tool coordinates.
q_previous = q_start;

%% 4. Continue the wrist along the prescribed arm path
for k = 1:n_samples
    s_k = s_samples(k);
    q_arm = q_arm_at_s(s_k);

    % Solve orientation IK for this arm configuration.
    [wrist_solutions, is_singular] = analytical_ik_wrist( ...
        q_arm, R_d, poe_model, singular_tol);
    if is_singular
        error('Wrist singularity at s = %.6f.',s_k);
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
        error('No wrist candidate preserves the wrist sign and joint limits at s = %.6f.',s_k);
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
    if sign(arm_det(k)) ~= arm_sign
        error('Arm Jacobian determinant lost its initial sign at s = %.6f (det = %.3g, initial det = %.3g).', ...
            s_k, arm_det(k), arm_det_start);
    end

    joint_margin(k) = min([q_current-q_min; q_max-q_current]);
    q_planned(:,k) = q_current;
    q_previous = q_current;

    % Check the full robot Jacobian, including both arm and wrist effects.
    J = poe_jacobian(q_planned(:,k), 0, 6, poe_model);
    J_tilde = jacobian_scale * J;
    sigma_min(k) = min(svd(J_tilde));
    if sigma_min(k) <= sigma_tol
        error('Jacobian singularity margin too small at s = %.6f (sigma_min = %.3g <= %.3g).', ...
            s_k, sigma_min(k), sigma_tol);
    end

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
end

%% 5. Verify the requested endpoint and report path closure
% Check actual joint angles: a 2*pi offset is distinct with bounded joints.
goal_error = norm(q_planned(:,end)-q_goal,inf);
if goal_error > goal_tol
    error('Continued path misses q_goal (maximum joint error %.3g rad).',goal_error);
end

position_closure = norm(p_planned(:,end) - p_planned(:,1))
joint_change = norm(q_planned(:,end) - q_planned(:,1))

%% 6. Plot the joint path, tool path, and singularity margin
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
ylabel('\sigma_{min}(\tilde{J})');
grid on;

figure;
plot(s_samples, joint_margin);
yline(0,'r--');
xlabel('Path parameter s');
ylabel('Minimum joint-limit clearance (rad)');
grid on;

% Project all target-pose IK solutions and the selected path onto (q2,q3).
ik_path_figure = plot_ik_path(Q_ik, q_planned, poe_model);

%% 7. Report the worst sampled singularity margin and pose errors
[sigma_worst, k_worst] = min(sigma_min);
sigma_worst
s_worst = s_samples(k_worst)
max_orientation_error_angle = max(theta)
max_discrepancy_desired_vs_planned_position = max(error_position)
