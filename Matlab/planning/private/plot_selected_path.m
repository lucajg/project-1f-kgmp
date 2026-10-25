function fig = plot_selected_path(selected_path,Q_ik,poe_model,sigma_tol)
% Show the successful path and its sampled margins in one coordinated figure.
s = selected_path.s;
q = selected_path.q;
p = selected_path.p;
path_color = [0 .45 .70];
threshold_color = [.75 .30 .12];
[sigma_worst,k_sigma] = min(selected_path.sigma_min);
[margin_worst,k_margin] = min(selected_path.joint_margin);

fig = figure('Name','IK transition','NumberTitle','off','Color','w', ...
    'Units','normalized','Position',[.04 .05 .92 .88]);
layout = tiledlayout(fig,6,2,'TileSpacing','compact','Padding','compact');
title(layout,'IK transition','FontSize',18,'FontWeight','bold');
subtitle(layout,sprintf('Fixed tool orientation  |  %d samples  |  Position closure %.2g m', ...
    numel(s),norm(p(:,end)-p(:,1))),'FontSize',10);

% The tool path closes at its start; a tangent arrow indicates direction.
tool_ax = nexttile(layout,2,[3 1]);
plot3(tool_ax,p(1,:),p(2,:),p(3,:),'Color',path_color,'LineWidth',1.8, ...
    'DisplayName','Tool path');
hold(tool_ax,'on');
plot3(tool_ax,p(1,1),p(2,1),p(3,1),'o','Color',path_color, ...
    'MarkerFaceColor','w','MarkerSize',7,'LineWidth',1.5,'DisplayName','Start / end');
k = max(1,floor(numel(s)/2));
direction = p(:,k+1)-p(:,k);
if norm(direction) > 0
    direction = direction/norm(direction)*.1*norm(max(p,[],2)-min(p,[],2));
    quiver3(tool_ax,p(1,k),p(2,k),p(3,k),direction(1),direction(2),direction(3),0, ...
        'Color',path_color,'LineWidth',1.5,'MaxHeadSize',.8,'HandleVisibility','off');
end
title(tool_ax,'Tool path');
xlabel(tool_ax,'x (m)'); ylabel(tool_ax,'y (m)'); zlabel(tool_ax,'z (m)');
axis(tool_ax,'equal');
view(tool_ax,3);
legend(tool_ax,'Location','southoutside','Orientation','horizontal','Box','off');

map_ax = nexttile(layout,8,[3 1]);
plot_ik_path(Q_ik,q,poe_model,true,map_ax);
axis(map_ax,'normal');
title(map_ax,'Arm path and IK branches');
subtitle(map_ax,'Labels: IK index and wrist sign (+ / -)','FontSize',9);
legend(map_ax,'Location','southoutside','NumColumns',3,'FontSize',8,'Box','off');

joint_ax = nexttile(layout,1,[2 1]);
colororder(joint_ax,lines(6));
plot(joint_ax,s,q.','LineWidth',1.3);
title(joint_ax,'Joint angles');
ylabel(joint_ax,'Angle (rad)');
legend(joint_ax,{'q_1','q_2','q_3','q_4','q_5','q_6'}, ...
    'Location','northwest','NumColumns',6,'Box','off');
% Leave space above the curves for the compact joint legend.
angle_range = [min(q,[],'all'),max(q,[],'all')];
angle_span = max(diff(angle_range),.1);
ylim(joint_ax,angle_range+[-.05 .4]*angle_span);

sigma_ax = nexttile(layout,5,[2 1]);
plot(sigma_ax,s,selected_path.sigma_min,'Color',path_color,'LineWidth',1.5);
hold(sigma_ax,'on');
yline(sigma_ax,sigma_tol,'--','Threshold','Color',threshold_color, ...
    'LabelHorizontalAlignment','left','FontSize',9);
plot(sigma_ax,s(k_sigma),sigma_worst,'o','Color',path_color, ...
    'MarkerFaceColor',path_color,'MarkerSize',5);
title(sigma_ax,sprintf('Jacobian margin  |  min %.3g',sigma_worst));
ylabel(sigma_ax,'$\sigma_{\min}(\tilde{J})$','Interpreter','latex');
ylim(sigma_ax,[0,1.15*max(selected_path.sigma_min)]);

margin_ax = nexttile(layout,9,[2 1]);
plot(margin_ax,s,selected_path.joint_margin,'Color',path_color,'LineWidth',1.5);
hold(margin_ax,'on');
yline(margin_ax,0,'--','Joint limit','Color',threshold_color, ...
    'LabelHorizontalAlignment','left','FontSize',9);
plot(margin_ax,s(k_margin),margin_worst,'o','Color',path_color, ...
    'MarkerFaceColor',path_color,'MarkerSize',5);
title(margin_ax,sprintf('Joint-limit clearance  |  min %.3g rad',margin_worst));
ylabel(margin_ax,'Clearance (rad)');
ylim(margin_ax,[0,max(.05,1.15*max(selected_path.joint_margin))]);

% All sample plots share s, so zooming one keeps the others aligned.
sample_axes = [joint_ax,sigma_ax,margin_ax];
linkaxes(sample_axes,'x');
xlim(joint_ax,[s(1),s(end)]);
for ax = sample_axes
    xlabel(ax,'Path parameter s');
end
for ax = [tool_ax,map_ax,sample_axes]
    grid(ax,'on');
    set(ax,'FontName','Helvetica','FontSize',10,'Box','off', ...
        'GridAlpha',.12,'LineWidth',.8,'TickDir','out');
end
end
