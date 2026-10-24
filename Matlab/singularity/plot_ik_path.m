function fig = plot_ik_path(Q_ik, q_path, poe_model)
%PLOT_IK_PATH Show target-pose IK solutions and a path on the arm singularity map.
% Columns of Q_ik are numbered as returned by analytical_ik_6r.
% Labels + and - denote sign(sin(q5)); 0 denotes a singular wrist.

map = singularity_map_data(poe_model,241);
positive_color = [0 .45 .70];
negative_color = [.85 .33 .10];
arm_negative_color = [.90 .88 .97];
arm_positive_color = [1 .94 .80];
wrist_signs = sign(sin(Q_ik(5,:)));
wrist_signs(abs(sin(Q_ik(5,:))) <= 1e-10) = 0;

fig = figure('Name','IK solutions and connecting path','Color','w', ...
    'Position',[100 100 900 680]);
ax = axes(fig);
hold(ax,'on');

% Shade the arm regions using the same determinant convention as the planner.
% The cached map stores D = -det(J_arm), with q5 = pi/2.
arm_det_map = -map.detJ;
det_limit = max(abs(arm_det_map(:)));
contourf(ax,map.angles,map.angles,arm_det_map,[-det_limit 0 det_limit], ...
    'LineStyle','none','HandleVisibility','off');
colormap(ax,[arm_negative_color; arm_positive_color]);
caxis(ax,[-det_limit det_limit]);
patch(ax,nan,nan,arm_negative_color,'EdgeColor','none', ...
    'DisplayName','det(J_{arm}) < 0');
patch(ax,nan,nan,arm_positive_color,'EdgeColor','none', ...
    'DisplayName','det(J_{arm}) > 0');

plot(ax,map.arm(1,:),map.arm(2,:),'k','LineWidth',1.2, ...
    'DisplayName','Arm singularities');
plot(ax,q_path(2,:),q_path(3,:),'Color',[.2 .6 .3],'LineWidth',2, ...
    'DisplayName','Selected path');

% Wrist flips coincide in this projection: an outer circle and an inner
% square keep both signs visible without moving their actual positions.
positive = wrist_signs > 0;
negative = wrist_signs < 0;
plot(ax,Q_ik(2,positive),Q_ik(3,positive),'o','LineStyle','none', ...
    'Color',positive_color,'MarkerSize',11,'LineWidth',1.5, ...
    'DisplayName','Positive wrist (+)');
plot(ax,Q_ik(2,negative),Q_ik(3,negative),'s','LineStyle','none', ...
    'Color',negative_color,'MarkerFaceColor',negative_color,'MarkerSize',4, ...
    'DisplayName','Negative wrist (-)');
singular = wrist_signs == 0;
if any(singular)
    plot(ax,Q_ik(2,singular),Q_ik(3,singular),'kd','LineStyle','none', ...
        'MarkerSize',8,'DisplayName','Singular wrist (0)');
end

% Focus on the region containing every solution and the complete path.
points = [Q_ik(2:3,:), q_path(2:3,:)];
lower = min(points,[],2);
upper = max(points,[],2);
padding = max(.55,.2*(upper-lower));
axis(ax,'equal');
xlim(ax,[max(-pi,lower(1)-padding(1)), min(pi,upper(1)+padding(1))]);
ylim(ax,[max(-pi,lower(2)-padding(2)), min(pi,upper(2)+padding(2))]);

% Combine labels at coincident points, retaining every IK solution ID.
% Alternate label sides to separate nearby projected arm solutions.
remaining = 1:size(Q_ik,2);
group_number = 0;
while ~isempty(remaining)
    id = remaining(1);
    same_point = vecnorm(Q_ik(2:3,remaining)-Q_ik(2:3,id),2,1) < 1e-6;
    group = remaining(same_point);
    labels = cell(1,numel(group));
    for j = 1:numel(group)
        candidate = group(j);
        if wrist_signs(candidate) > 0
            suffix = '+';
        elseif wrist_signs(candidate) < 0
            suffix = '-';
        else
            suffix = '0';
        end
        labels{j} = sprintf('%d%s',candidate,suffix);
        if norm(Q_ik(:,candidate)-q_path(:,1),inf) < 1e-6
            labels{j} = [labels{j}, ' (start)'];
        end
        if norm(Q_ik(:,candidate)-q_path(:,end),inf) < 1e-6
            labels{j} = [labels{j}, ' (goal)'];
        end
    end
    group_number = group_number+1;
    direction = (-1)^group_number;
    alignment = 'left';
    if direction < 0, alignment = 'right'; end
    text(ax,Q_ik(2,id)+direction*.08,Q_ik(3,id),strjoin(labels,' / '), ...
        'HorizontalAlignment',alignment,'VerticalAlignment','bottom', ...
        'FontSize',10,'Interpreter','none','BackgroundColor','w','Margin',2);
    remaining = remaining(~same_point);
end

xlabel(ax,'q_2 (rad)');
ylabel(ax,'q_3 (rad)');
title(ax,{'IK solutions and connecting path', ...
    'Labels: solution ID and wrist sign = sign(sin(q_5))'});
grid(ax,'on');
set(ax,'Layer','top');
box(ax,'on');
legend(ax,'Location','northeastoutside');
end
