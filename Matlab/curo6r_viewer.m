function fig = curo6r_viewer(model)
%CURO6R_VIEWER Six joint sliders driving the independent poe_fk function.
% Usage: run data.m, then curo6r_viewer(model). No Robotics Toolbox needed.
% Sliders show degrees; the kinematics function receives radians.

n = size(model.xi_hat,3);
q = zeros(n,1);
axisLength = 0.5;
fig = uifigure('Name','CuRo6R - Product of Exponentials', ...
    'Position',[100 100 1080 680], 'Visible','off');
layout = uigridlayout(fig,[1 2]);
layout.ColumnWidth = {'1x',300};

ax = uiaxes(layout);
ax.Layout.Column = 1;
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,35,25);
xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); zlabel(ax,'z (m)');
title(ax,'Joint origins and axes; endpoint frame at link_6');

% A conservative reach bound keeps the scale fixed while the robot moves.
home = [zeros(3,1), model.r, model.M(1:3,4)];
reach = sum(sqrt(sum(diff(home,1,2).^2,1))) + axisLength;
xlim(ax,[-reach reach]); ylim(ax,[-reach reach]); zlim(ax,[-reach reach]);

% Create graphics once. redraw() changes their data, never recreates axes.
links = plot3(ax,nan,nan,nan,'-o','LineWidth',2,'DisplayName','Links');
z = zeros(1,n);
axesArrows = quiver3(ax,z,z,z,z,z,z,0,'Color',[.5 .3 .6], ...
    'DisplayName','Joint axes');
frame = gobjects(3,1);
colors = [.85 .2 .2; .2 .65 .2; .2 .35 .9];
frameNames = {'x_E','y_E','z_E'};
for k = 1:3
    frame(k) = quiver3(ax,0,0,0,0,0,0,0,'Color',colors(k,:), ...
        'LineWidth',2,'DisplayName',frameNames{k});
end
legend(ax,'show','Location','northwest');

controls = uigridlayout(layout,[n+1 2]);
controls.Layout.Column = 2;
controls.ColumnWidth = {92,'1x'};
controls.RowHeight = [repmat({'1x'},1,n), {32}];
labels = gobjects(n,1);
sliders = gobjects(n,1);
for i = 1:n
    labels(i) = uilabel(controls,'Text',sprintf('q%d: 0 deg',i));
    labels(i).Layout.Row = i;
    labels(i).Layout.Column = 1;
    sliders(i) = uislider(controls,'Limits',[-180 180],'Value',0, ...
        'MajorTicks',[-180 0 180],'MinorTicks',[]);
    sliders(i).Layout.Row = i;
    sliders(i).Layout.Column = 2;
    sliders(i).ValueChangingFcn = @(~,event) moveJoint(i,event.Value);
    sliders(i).ValueChangedFcn  = @(~,event) moveJoint(i,event.Value);
end
homeButton = uibutton(controls,'Text','Home','ButtonPushedFcn',@goHome);
homeButton.Layout.Row = n+1;
homeButton.Layout.Column = [1 2];
redraw();
fig.Visible = 'on';

    function moveJoint(i, degrees)
        % event.Value follows the thumb during dragging; slider.Value lags.
        q(i) = degrees*pi/180;
        labels(i).Text = sprintf('q%d: %.0f deg',i,degrees);
        redraw();
    end

    function redraw()
        [T, p, w] = poe_fk(q, model);
        points = [zeros(3,1), p, T(1:3,4)];
        set(links,'XData',points(1,:),'YData',points(2,:),'ZData',points(3,:));
        set(axesArrows,'XData',p(1,:),'YData',p(2,:),'ZData',p(3,:), ...
            'UData',axisLength*w(1,:),'VData',axisLength*w(2,:), ...
            'WData',axisLength*w(3,:));
        for k = 1:3
            set(frame(k),'XData',T(1,4),'YData',T(2,4),'ZData',T(3,4), ...
                'UData',axisLength*T(1,k),'VData',axisLength*T(2,k), ...
                'WData',axisLength*T(3,k));
        end
        drawnow limitrate nocallbacks
    end

    function goHome(~,~)
        q(:) = 0;
        for j = 1:n
            sliders(j).Value = 0;
            labels(j).Text = sprintf('q%d: 0 deg',j);
        end
        redraw();
    end
end
