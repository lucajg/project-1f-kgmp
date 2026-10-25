function plot_failed_path(attempt,goal_index,Q_ik,poe_model,q_min,q_max,wrist_sign,sigma_tol)
% Plot a failed attempt without treating its rejected sample as valid motion.
heading = sprintf('Failed attempt - partial trajectory toward IK %d',goal_index);
if ~isempty(attempt.s) && attempt.s(end) == attempt.s_failed
    heading = sprintf('Failed attempt - endpoint mismatch for IK %d',goal_index);
end
% Put accumulated reasons on separate lines so the diagnostic title stays readable.
caption = [string(heading); split(string(attempt.reason),"; ")];
fig = figure('Name',heading,'Color','w','Position',[80 50 1100 950]);
same_sign = sign(sin(attempt.rejected_q(5,:))) == wrist_sign;

% Each joint gets its own limits. Rejected candidates are isolated markers,
% never appended to the valid prefix. A red cross can expose a limit crossing.
for j = 1:6
    subplot(4,2,j);
    plot(attempt.s,attempt.q(j,:),'LineWidth',1.3,'DisplayName','Valid prefix');
    hold on;
    yline(q_min(j),'k--','HandleVisibility','off');
    yline(q_max(j),'k--','HandleVisibility','off');
    xline(attempt.s_failed,'r:','Failure','HandleVisibility','off');
    if ~isempty(attempt.s)
        plot(attempt.s(end),attempt.q(j,end),'ko','HandleVisibility','off');
    end
    if any(same_sign)
        plot(repmat(attempt.s_failed,1,nnz(same_sign)),attempt.rejected_q(j,same_sign), ...
            'rx','MarkerSize',9,'LineWidth',1.5,'DisplayName','Rejected: same wrist sign');
    end
    if any(~same_sign)
        plot(repmat(attempt.s_failed,1,nnz(~same_sign)),attempt.rejected_q(j,~same_sign), ...
            '+','Color',[.5 .5 .5],'MarkerSize',8,'DisplayName','Other wrist sign');
    end
    if any(attempt.rejected_q(j,same_sign) < q_min(j) | attempt.rejected_q(j,same_sign) > q_max(j))
        title(sprintf('q_%d - joint limit violated',j),'Color','r');
    else
        title(sprintf('q_%d',j));
    end
    xlabel('Path parameter s');
    ylabel('Angle (rad)');
    xlim([0 1]);
    grid on;
    if j == 4, legend('Location','best','FontSize',8); end
end

subplot(4,2,7);
plot(attempt.s,attempt.sigma_min,'LineWidth',1.3);
hold on;
yline(sigma_tol,'r--','Threshold');
xline(attempt.s_failed,'r:','Failure');
if isfinite(attempt.rejected_sigma)
    plot(attempt.s_failed,attempt.rejected_sigma,'rx','MarkerSize',9,'LineWidth',1.5);
end
xlabel('Path parameter s');
ylabel('$\sigma_{\min}(\tilde{J})$','Interpreter','latex');
title('Singularity margin');
xlim([0 1]);
grid on;

subplot(4,2,8);
plot3(attempt.p(1,:),attempt.p(2,:),attempt.p(3,:),'LineWidth',1.5);
hold on;
if isempty(attempt.s)
    text(.5,.5,'No validated samples','Units','normalized','HorizontalAlignment','center');
else
    plot3(attempt.p(1,end),attempt.p(2,end),attempt.p(3,end),'ko');
end
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Task-space prefix (circle: last valid sample)');
axis equal;
grid on;
sgtitle(fig,caption,'Interpreter','none');

% Reuse the existing aspect map, but suppress its successful-goal labelling.
[map_figure,ax] = plot_ik_path(Q_ik,attempt.q,poe_model,false);
set(map_figure,'Name',heading);
if ~isempty(attempt.s)
    plot(ax,attempt.q(2,end),attempt.q(3,end),'ko','MarkerSize',8, ...
        'DisplayName','Last valid sample');
end
plot(ax,attempt.arm_failed(2),attempt.arm_failed(3),'rx','MarkerSize',10, ...
    'LineWidth',1.5,'DisplayName','Failed arm sample');
plot(ax,Q_ik(2,goal_index),Q_ik(3,goal_index),'kp','MarkerSize',12, ...
    'DisplayName',sprintf('Requested goal: IK %d',goal_index));
title(ax,caption,'Interpreter','none');
legend(ax,'Location','northeastoutside');
end
