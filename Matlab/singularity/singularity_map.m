function fig = singularity_map
%SINGULARITY_MAP Explore CuRo6R wrist-center targets and full pose IK.
% Position controls place the wrist center in metres; tool roll/pitch/yaw
% are in degrees, with R = Rz(yaw)*Ry(pitch)*Rx(roll).
% Drag the star to change rho_w and z_w at fixed wrist azimuth/tool orientation.
% Colors/numbers follow nearest full joint solutions modulo 2*pi.
% At singularities branches can merge; continuous families have one representative.

poe = poe_data();
dh = dh_data();
map = singularity_map_data(poe,241);  % Cached: never recomputed while dragging.
colors = [0 .45 .70; .85 .33 .10; .35 .60 .10; .49 .18 .56; ...
          .75 .55 .05; .10 .65 .70; .65 .08 .18; .80 .30 .65];
memory = nan(6,8);
selected = 0;
azimuth = 0;
dragging = false;
dragDirection = [1;0];
dragBounds = zeros(2,2);
previousPointer = 'arrow';
initial = [3;0;3;0;0;0];  % Wrist center (m), tool RPY (deg); eight regular solutions.
pose = initial;  % [wrist x; wrist y; wrist z; tool roll; tool pitch; tool yaw].
reach = sum(vecnorm(diff([zeros(3,1),poe.r(:,1:5)],1,2)));
limits = [-reach reach; -reach reach; -reach reach; -180 180; -90 90; -180 180];

visibility = get(groot,'defaultFigureVisible');
screen = get(groot,'ScreenSize');
dpi = get(groot,'ScreenPixelsPerInch');
scale = min([dpi/96,.94*screen(3)/1240,.85*screen(4)/800]);
fontScale = scale*96/dpi;  % Respect display scaling, while fitting smaller screens.
fig = figure('Name','CuRo6R | Singularity and IK explorer','NumberTitle','off', ...
    'Tag','singularity-map','Position',[screen(1:2)+[30 50],scale*[1240 800]], ...
    'Color',[.97 .97 .97], ...
    'MenuBar','none','ToolBar','figure','Visible','off');
try
    set(fig,'DefaultUicontrolUnits','normalized','DefaultUicontrolFontSize',10*fontScale, ...
        'DefaultAxesFontSize',10*fontScale,'DefaultTextFontSize',10*fontScale, ...
        'DefaultUicontrolInterruptible','off','DefaultUicontrolBusyAction','cancel');
    armAxes = axes(fig,'Position',[.065 .43 .39 .44],'Tag','arm-map');
    imagesc(armAxes,map.angles,map.angles,map.detJ > 0);
    set(armAxes,'YDir','normal','CLim',[0 1]);
    colormap(armAxes,[.85 .85 1; 1 .90 .90]);  % Preserve the soft sign colors.
    hold(armAxes,'on');
    plot(armAxes,map.arm(1,:),map.arm(2,:),'k','LineWidth',1.2);
    axis(armAxes,'equal'); axis(armAxes,[-pi pi -pi pi]);
    ticks = [-pi -pi/2 0 pi/2 pi];
    labels = {'-\pi','-\pi/2','0','\pi/2','\pi'};
    set(armAxes,'XTick',ticks,'YTick',ticks,'XTickLabel',labels,'YTickLabel',labels);
    xlabel(armAxes,'q_2 (rad)'); ylabel(armAxes,'q_3 (rad)');
    title(armAxes,{'Arm factor: det(J) = D(q_2,q_3) sin(q_5)', ...
        'Blue: D < 0     Pink: D > 0'});
    marks = gobjects(1,8); numbers = gobjects(1,8);
    for marker = 1:8
        marks(marker) = plot(armAxes,nan,nan,'o','Color',colors(marker,:), ...
            'LineWidth',2,'MarkerSize',13*fontScale,'Tag',sprintf('solution-%d',marker));
        numbers(marker) = text(armAxes,0,0,'','FontWeight','bold','FontSize',11*fontScale, ...
            'VerticalAlignment','bottom','Clipping','on');
    end

    workspaceAxes = axes(fig,'Position',[.57 .43 .38 .44],'Tag','workspace-map');
    disableDefaultInteractivity(workspaceAxes);  % Let the star handle mouse dragging.
    hold(workspaceAxes,'on'); grid(workspaceAxes,'on'); box(workspaceAxes,'on');
    boundary = plot(workspaceAxes,map.wrist(1,:),map.wrist(2,:),'k', ...
        'LineWidth',1.3,'Tag','arm-boundary');
    target = plot(workspaceAxes,nan,nan,'p','MarkerSize',13*fontScale, ...
        'MarkerEdgeColor','k','MarkerFaceColor',[.95 .75 .15], ...
        'Tag','wrist-target','ButtonDownFcn',@startDrag);
    xlim(workspaceAxes,[0 reach]); ylim(workspaceAxes,[-reach reach]);
    xlabel(workspaceAxes,'\rho_w = \surd(x_w^2 + y_w^2) (m)'); ylabel(workspaceAxes,'z_w (m)');
    workspaceTitle = title(workspaceAxes,'');
    key = legend(workspaceAxes,[boundary,target],{'Arm singularities','Wrist center (drag)'}, ...
        'Orientation','horizontal');
    set(key,'Units','normalized','Position',[.58 .335 .37 .035]);

    names = {'wrist x (m)','wrist y (m)','wrist z (m)','roll (deg)','pitch (deg)','yaw (deg)'};
    sliders = gobjects(6,1); fields = gobjects(6,1); listeners = cell(6,1);
    for i = 1:6
        y = .295-(i-1)*.043;
        uicontrol(fig,'Style','text','String',names{i},'Position',[.025 y .085 .028], ...
            'HorizontalAlignment','left','BackgroundColor',fig.Color);
        fields(i) = uicontrol(fig,'Style','edit','String',sprintf('%.4f',pose(i)), ...
            'Position',[.11 y .075 .032],'BackgroundColor','white', ...
            'Tag',sprintf('pose-%d-field',i),'Callback',@(s,~) changePose(i,s));
        step = .01;
        if i > 3, step = 1; end
        % The relative page step sets the native thumb width; keep all six equal.
        sliders(i) = uicontrol(fig,'Style','slider','Min',limits(i,1),'Max',limits(i,2), ...
            'Value',pose(i),'SliderStep',[step/diff(limits(i,:)),1/36], ...
            'Position',[.20 y .24 .028],'Tag',sprintf('pose-%d-slider',i), ...
            'Callback',@(s,~) changePose(i,s));
        listeners{i} = addlistener(sliders(i),'ContinuousValueChange',@(s,~) changePose(i,s));
    end
    setappdata(fig,'sliderListeners',listeners);
    uicontrol(fig,'Style','pushbutton','String','Reset pose','Position',[.025 .025 .095 .035], ...
        'Tag','reset-pose','Callback',@(~,~) resetPose());
    uicontrol(fig,'Style','text','String','Tool R = Rz(yaw) Ry(pitch) Rx(roll)', ...
        'Position',[.135 .025 .31 .03],'BackgroundColor',fig.Color);
    uicontrol(fig,'Style','text','String','IK solutions (degrees) - paired rings share q2, q3', ...
        'Position',[.49 .303 .49 .03],'BackgroundColor',fig.Color,'FontWeight','bold');
    solutionTable = uitable(fig,'Units','normalized','Position',[.49 .055 .49 .245], ...
        'ColumnName',{'ID','q1','q2','q3','q4','q5','q6','State'},'RowName',[], ...
        'FontSize',10*fontScale, ...
        'BackgroundColor',.90+.10*colors,'ColumnEditable',false(1,8), ...
        'Tag','solutions','CellSelectionCallback',@selectSolution);
    fig.SizeChangedFcn = @(~,~) resizeTable();
    resizeTable();
    status = uicontrol(fig,'Style','text','Position',[.03 .955 .94 .03], ...
        'BackgroundColor',fig.Color,'FontWeight','bold','Tag','ik-status');
    redraw();
    fig.Visible = visibility;
catch exception
    delete(fig);
    rethrow(exception);
end

    function changePose(i,source)
        if strcmp(source.Style,'edit'), value = str2double(source.String);
        else, value = source.Value;
        end
        if ~isreal(value) || ~isfinite(value) || value < limits(i,1) || value > limits(i,2)
            fields(i).String = sprintf('%.4f',pose(i));
            status.String = sprintf('Enter %s between %.2f and %.2f.',names{i},limits(i,:));
            return;
        end
        % Release need not repeat the last drag update; a valid edit clears errors.
        if value == pose(i) && strcmp(source.Style,'slider'), return; end
        pose(i) = value;
        fields(i).String = sprintf('%.4f',value);
        sliders(i).Value = value;
        redraw();
        drawnow limitrate nocallbacks;
    end

    function startDrag(~,~)
        if ~strcmp(fig.SelectionType,'normal') || dragging, return; end
        dragging = true;
        dragDirection = [cos(azimuth);sin(azimuth)];
        % Freeze the view while dragging; keep Cartesian coordinates in slider range.
        dragBounds = [workspaceAxes.XLim;workspaceAxes.YLim];
        dragBounds(1,:) = max(0,min(dragBounds(1,:),reach/max(abs(dragDirection))));
        dragBounds(2,:) = max(-reach,min(dragBounds(2,:),reach));
        previousPointer = fig.Pointer;
        fig.Pointer = 'fleur';
        fig.WindowButtonMotionFcn = @dragTarget;
        fig.WindowButtonUpFcn = @stopDrag;
    end

    function dragTarget(~,~)
        if ~dragging, return; end
        point = workspaceAxes.CurrentPoint(1,1:2).';
        point = max(dragBounds(:,1),min(point,dragBounds(:,2)));
        position = [point(1)*dragDirection;point(2)];
        if isequal(position,pose(1:3)), return; end
        pose(1:3) = position;
        for k = 1:3
            sliders(k).Value = pose(k);
            fields(k).String = sprintf('%.4f',pose(k));
        end
        redraw();
        drawnow limitrate nocallbacks;
    end

    function stopDrag(~,~)
        if ~dragging, return; end
        dragTarget([],[]);  % Include the final mouse position on release.
        dragging = false;
        fig.WindowButtonMotionFcn = [];
        fig.WindowButtonUpFcn = [];
        fig.Pointer = previousPointer;
        xlim(workspaceAxes,[0,max(reach,1.05*hypot(pose(1),pose(2)))]);
    end

    function redraw()
        T = toolTransform(pose,dh.d(6));
        [Q,memory] = match_ik_solutions(analytical_ik_6r(T,poe,dh),memory);
        active = find(all(isfinite(Q),1));
        data = repmat({'--'},8,8);
        set(marks,'XData',nan,'YData',nan,'MarkerFaceColor','none','LineWidth',2);
        set(numbers,'String','');
        for id = 1:8
            hex = sprintf('#%02X%02X%02X',round(255*colors(id,:)));
            data{id,1} = sprintf('<html><font color="%s"><b>%d</b></font></html>',hex,id);
            if ~ismember(id,active), data{id,8} = 'Not present'; continue; end
            data(id,2:7) = arrayfun(@(v) sprintf('%.1f',v),Q(:,id)'*180/pi,'UniformOutput',false);
            J = poe_jacobian(Q(:,id),0,5,poe);
            s = svd(J(1:3,1:3));
            data{id,8} = 'Regular';
            if s(end) < 1e-6*s(1), data{id,8} = 'Near arm'; end
            if abs(sin(Q(5,id))) < 1e-6, data{id,8} = 'Near wrist'; end
            if abs(sin(Q(5,id))) < 1e-10, data{id,8} = 'Wrist family'; end
            marks(id).XData = Q(2,id); marks(id).YData = Q(3,id);
        end
        % Coincident wrist flips get concentric rings and a combined colored label.
        remaining = active;
        while ~isempty(remaining)
            id = remaining(1);
            delta = atan2(sin(Q(2:3,remaining)-Q(2:3,id)),cos(Q(2:3,remaining)-Q(2:3,id)));
            group = remaining(vecnorm(delta) < 1e-6);
            label = cell(1,numel(group));
            for j = 1:numel(group)
                k = group(j);
                marks(k).MarkerSize = fontScale*max(5,16-7*(j-1));
                label{j} = sprintf('\\color[rgb]{%.3f,%.3f,%.3f}%d',colors(k,:),k);
            end
            marks(group(end)).MarkerFaceColor = colors(group(end),:);
            numbers(id).Position = [min(Q(2,id)+.10,pi-.5),min(Q(3,id)+.10,pi-.2),0];
            numbers(id).String = strjoin(label,' / ');
            remaining = setdiff(remaining,group,'stable');
        end
        if ismember(selected,active), marks(selected).LineWidth = 4; end
        solutionTable.Data = data;
        rho = hypot(pose(1),pose(2));
        if rho > 1e-10, azimuth = atan2(pose(2),pose(1)); end
        target.XData = rho; target.YData = pose(3);
        if ~dragging  % Changing the scale during a drag would move the target under the pointer.
            xlim(workspaceAxes,[0,max(reach,1.05*rho)]);
        end
        workspaceTitle.String = {'Wrist-center workspace', ...
            sprintf('Target azimuth = %.1f deg; black: D = 0',azimuth*180/pi)};
        if isempty(active)
            status.String = 'Unreachable pose: no IK solutions. Move the pose back into reach.';
            target.MarkerFaceColor = [.85 .25 .2];
        else
            status.String = sprintf('%d IK solutions | IDs follow joint continuity | Wrist singularities are flagged in the list',numel(active));
            target.MarkerFaceColor = [.95 .75 .15];
        end
        % T is the tool pose; pose(1:3) and wristPosition are the wrist target.
        fig.UserData = struct('Q',Q,'T',T,'pose',pose, ...
            'wristPosition',pose(1:3),'azimuth',azimuth);
    end

    function resetPose()
        pose = initial; memory(:) = nan; selected = 0;
        for k = 1:6
            sliders(k).Value = pose(k); fields(k).String = sprintf('%.4f',pose(k));
        end
        redraw();
    end

    function selectSolution(~,event)
        if isempty(event.Indices), return; end
        selected = event.Indices(1,1);
        set(marks,'LineWidth',2);
        marks(selected).LineWidth = 4;
    end

    function resizeTable()
        bounds = getpixelposition(solutionTable);
        widths = max(8,bounds(3)-20*scale)*[.06,repmat(.11,1,6),.28];
        solutionTable.ColumnWidth = num2cell(floor(widths));
    end
end

function T = toolTransform(pose,d6)
% Convert wrist position and fixed-axis tool roll/pitch/yaw to a tool pose.
r = pose(4)*pi/180; p = pose(5)*pi/180; y = pose(6)*pi/180;
Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
Rz = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
R = Rz*Ry*Rx;
T = [R,pose(1:3)+d6*R(:,1); 0 0 0 1];
end
