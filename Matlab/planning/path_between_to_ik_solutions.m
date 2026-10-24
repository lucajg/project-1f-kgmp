q_start = pi/2*[1;-1;1;-1;1;-1];
singular_tol=1e-6;
if abs(sin(q_start(5))) <= singular_tol
    error('The initial wrist is singular; no regular wrist sign is defined.');
end
wrist_sign = sign(sin(q_start(5)));
poe_model = poe_data;
[T_d, p, w, F0] = poe_fk(q_start, poe_model);
R_d = T_d(1:3,1:3);
dh_model = dh_data;
Q_ik = analytical_ik_6r(T_d, poe_model, dh_model);

q_goal = Q_ik(:,7);

q_a = @(s) (1-s)*q_start(1:3) + s*q_goal(1:3);
n_samples = 19999;
s_max=1;
s_min=0;
s_samples = linspace(s_min,s_max,n_samples);
joint_margin = nan(1,n_samples);
q_planned=zeros(6,n_samples);
q_min = -pi*ones(6,1);
q_max =  pi*ones(6,1);
q_previous = q_start;
L=2; % characteristic length in metres
sigma_min=zeros(1,n_samples);
theta = zeros(1,n_samples);
error_position = zeros(1,n_samples);
p_planned = zeros(3,n_samples);
for k=1:n_samples
    s_k = s_samples(k);
    current_q_a = q_a(s_k);
    [wrist, is_singular] = analytical_ik_wrist(current_q_a, R_d, poe_model, singular_tol);
    if is_singular
        error('Wrist singularity at s = %.6f.',s_k);
    end
    % Unwrap both wrist solutions near the previous configuration.
    delta = wrist - q_previous(4:6);
    wrist = q_previous(4:6) + atan2(sin(delta),cos(delta));
    q_candidates = [repmat(current_q_a,1,size(wrist,2)); wrist];
    valid = all(q_candidates >= q_min & q_candidates <= q_max,1) ...
        & (sign(sin(wrist(2,:))) == wrist_sign);
    if ~any(valid)
        error('No wrist candidate preserves the wrist sign and joint limits at s = %.6f.',s_k);
    end
    q_candidates = q_candidates(:,valid);
    [~, nearest] = min(vecnorm(q_candidates(4:6,:)-q_previous(4:6),2,1));
    q_current = q_candidates(:,nearest);
    joint_margin(k) = min([q_current-q_min; q_max-q_current]);
    q_planned(:,k) = q_current;
    q_previous = q_current;

    J = poe_jacobian(q_planned(:,k), 0, 6, poe_model);
    J_tilde = [eye(3)/L, zeros(3,3); zeros(3,3), eye(3)]*J;
    sigma_min(k) = min(svd(J_tilde));

    [T_k, p_k, ~, ~] = poe_fk(q_planned(:,k), poe_model);
    R_k = T_k(1:3,1:3);
    E_k = R_d'*R_k;
    theta(k)=acos(min(max((trace(E_k)-1)/2, -1), 1));
    p_planned(:,k) = T_k(1:3,4);
    p_desired = (p_k(:,5)+R_d*[dh_model.d(6);0;0]);
    error_position(k)=norm(p_planned(:,k)-p_desired);
end

% Check actual joint angles: a 2*pi offset is distinct with bounded joints.
goal_error = norm(q_planned(:,end)-q_goal,inf);
if goal_error > 1e-6
    error('Continued path misses q_goal (maximum joint error %.3g rad).',goal_error);
end

position_closure = norm(p_planned(:,end) - p_planned(:,1))
joint_change = norm(q_planned(:,end) - q_planned(:,1))

% Joint-space trajectory and end-effector path in task space.
figure;
subplot(1,2,1);
plot(s_samples, q_planned.');
xlabel('Path parameter s'); ylabel('Joint angle (rad)');
legend('q_1','q_2','q_3','q_4','q_5','q_6');
title('Joint space'); grid on;

subplot(1,2,2);
plot3(p_planned(1,:), p_planned(2,:), p_planned(3,:));
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Task space'); axis equal; grid on;

figure;
plot(s_samples, sigma_min);
xlabel('Path parameter s');
ylabel('\sigma_{min}(\tilde{J})');
grid on;

[sigma_worst, k_worst] = min(sigma_min);
sigma_worst
s_worst = s_samples(k_worst)
max(theta)
max(error_position)
