function plot_selected_path(selected_path,Q_ik,poe_model,sigma_tol)
% Plot a successful path in joint/task space, its margins, and the IK map.
q_planned = selected_path.q;
p_planned = selected_path.p;

figure;
subplot(1,2,1);
plot(selected_path.s, q_planned.');
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
plot(selected_path.s, selected_path.sigma_min);
yline(sigma_tol,'r--','Threshold');
xlabel('Path parameter s');
ylabel('$\sigma_{\min}(\tilde{J})$','Interpreter','latex');
grid on;

figure;
plot(selected_path.s, selected_path.joint_margin);
yline(0,'r--');
xlabel('Path parameter s');
ylabel('Minimum joint-limit clearance (rad)');
grid on;

% Project all target-pose IK solutions and the selected path onto (q2,q3).
plot_ik_path(Q_ik, q_planned, poe_model);
end
