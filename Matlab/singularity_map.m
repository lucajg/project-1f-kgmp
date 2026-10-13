% CuRo6R: sign of det(J) over (q2,q3), with its zero contour in black.
% Numerical evaluation reuses the exact geometric Jacobian; no finite
% differences or Symbolic Math Toolbox are needed for this grid.
%
% For this spherical-wrist robot, det(J) = D(q2,q3)*sin(q5), independent
% of q1, q4 and q6. Fix q5 = pi/2 to expose the arm singularities D = 0.
% At q5 = 0 or +/-pi the wrist is singular for EVERY (q2,q3) pair.

poe_model = poe_data();
angles = linspace(-pi,pi,301);  % Increase the sample count for a finer map.
[Q2,Q3] = meshgrid(angles,angles);
detJ = zeros(size(Q2));
q = [0;0;0;0;pi/2;0];

for i = 1:numel(Q2)
    q(2:3) = [Q2(i);Q3(i)];
    J = poe_jacobian(q,0,6,poe_model);  % Tool Jacobian, in the base frame.
    detJ(i) = det(J);
end

figure('Name','CuRo6R | Singularity map');
% Two colors encode sign only; exact zero samples belong to the contour.
imagesc(angles,angles,detJ > 0,'AlphaData',detJ ~= 0);
clim([0,1]);
axis xy equal tight;
hold on;
% Estimate zeros by interpolating detJ between opposite-sign grid samples.
% Use the determinant values here, not the binary color data above.
% A finite grid can miss zeros that touch zero without changing sign.
contour(Q2,Q3,detJ,[0,0],'k','LineWidth',1.5);
hold off;
colormap([0.2,0.4,0.8; 0.95,0.55,0.2]);  % Negative: blue; positive: orange.
cb = colorbar;
cb.Ticks = [0.25,0.75];
cb.TickLabels = {'det(J) < 0','det(J) > 0'};
xlabel('q_2 (rad)');
ylabel('q_3 (rad)');
title({'CuRo6R: sign of det(J), q_5 = \pi/2', 'Black curves: det(J) = 0'});
ticks = [-pi,-pi/2,0,pi/2,pi];
tick_labels = {'-\pi','-\pi/2','0','\pi/2','\pi'};
xticks(ticks);
yticks(ticks);
xticklabels(tick_labels);
yticklabels(tick_labels);
