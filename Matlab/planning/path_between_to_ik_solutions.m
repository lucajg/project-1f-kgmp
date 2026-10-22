q_start = pi/2*[1;-1;1;-1;1;-1];
poe_model = poe_data;
[T_d, p, w, F0] = poe_fk(q_start, poe_model);
R_d = T_d(1:3,1:3);
dh_model = dh_data;
Q_ik = analytical_ik_6r(T_d, poe_model, dh_model);

q_goal = Q_ik(:,7);

q_a = @(s) (1-s)*q_start(1:3) + s*q_goal(1:3);
n_samples = 10000;
s_max=1;
s_min=0;
s_samples = linspace(s_min,s_max,n_samples);
singular_tol=1e-6;
joint_margin = nan(1,n_samples);
q_planned=zeros(6,n_samples);
q_min = -pi*ones(6,1);
q_max =  pi*ones(6,1);
q_previous = q_start;
L=2;
sigma_min=zeros(1,n_samples);
theta = zeros(1,n_samples);
for k=1:n_samples
    s_k = s_samples(k);
    current_q_a = q_a(s_k);
    [wrist, is_singular] = analytical_ik_wrist(current_q_a, R_d, poe_model, singular_tol);
    if is_singular
        error('Wrist singularity at s = %.6f.',s_k);
    end
    q_w = wrist(:,1);
    delta = q_w - q_previous(4:6);
    q_w = q_previous(4:6) + atan2(sin(delta),cos(delta));
    q_current = [current_q_a; q_w];
    joint_margin(k) = min([q_current-q_min; q_max-q_current]);
    if joint_margin(k) < 0
        error('Joint limit reached at s = %.6f.',s_k);
    end
    q_planned(:,k) = q_current;
    q_previous = q_current;

    J = poe_jacobian(q_planned(:,k), 0, 6, poe_model);
    J_tilde = [eye(3)/L, zeros(3,3); zeros(3,3), eye(3)]*J;
    sigma_min(k) = min(svd(J_tilde));

    T_k = poe_fk(q_planned(:,k), poe_model);
    R_k = T_k(1:3,1:3);
    E_k = R_d'*R_k;
    theta(k)=acos(min(max((trace(E_k)-1)/2, -1), 1));
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

p_planned = zeros(3,n_samples);
for k = 1:n_samples
    T = poe_fk(q_planned(:,k), poe_model);
    p_planned(:,k) = T(1:3,4);
end
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