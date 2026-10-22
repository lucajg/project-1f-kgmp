function fig = curo6r_viewer(varargin)
%CURO6R_VIEWER Simple joint-axis plots for one or two CuRo6R models.
% curo6r_viewer(poe_model), curo6r_viewer(dh_model), or both models together.
% Controls use degrees. FK uses radians. Sliders update on release.

narginchk(1,2);
models = cellfun(@describeModel,varargin,'UniformOutput',false);
models = [models{:}];
n = models(1).n;
assert(all([models.n] == n),'Models must have the same number of joints.');
count = numel(models);
q = zeros(n,1);
reach = max([models.reach]) + .65;
colors = lines(n);

% Classic graphics avoid the web-based uifigure layout and event machinery.
fig = figure('Name','CuRo6R | Joint axes','NumberTitle','off', ...
    'Tag','curo6r-viewer','Position',[80 100 850+400*(count-1) 650], ...
    'MenuBar','none','ToolBar','figure','Color',[.95 .95 .95],'Visible','off');
try
    set(fig,'DefaultUicontrolUnits','normalized', ...
        'DefaultUicontrolFontSize',10, ...
        'DefaultUicontrolInterruptible','off','DefaultUicontrolBusyAction','cancel');
    ax = gobjects(1,count);
    drawScene = cell(1,count);
    for k = 1:count
        ax(k) = axes('Parent',fig,'Units','normalized', ...
            'Position',[(k-1)/count+.06/count, .28, .85/count, .63], ...
            'Tag',sprintf('model-axes-%d',k));
        drawScene{k} = curo6r_scene(ax(k),n,reach,.65,models(k).chainStyle);
        title(ax(k),models(k).name);
        % Fixed bounds cover all joint configurations; no resize/FK callbacks.
        axis(ax(k),reach*[-1 1 -1 1 -1 1]);
    end

    sliders = gobjects(n,1);
    fields = gobjects(n,1);
    width = .94/n;
    for i = 1:n
        x = .03+(i-1)*width;
        uicontrol(fig,'Style','text','String',sprintf('q%d [°]',i), ...
            'Position',[x .17 .45*width .035],'ForegroundColor',.75*colors(i,:), ...
            'BackgroundColor',fig.Color,'FontWeight','bold');
        fields(i) = uicontrol(fig,'Style','edit','String','0', ...
            'Position',[x+.46*width .17 .43*width .04], ...
            'BackgroundColor','white','Tag',sprintf('q%d-field',i), ...
            'Callback',@(source,~) changeJoint(i,source));
        sliders(i) = uicontrol(fig,'Style','slider','Min',-180,'Max',180,'Value',0, ...
            'SliderStep',[1 10]/360,'Position',[x .12 .89*width .025], ...
            'Tag',sprintf('q%d-slider',i), ...
            'Callback',@(source,~) changeJoint(i,source));
    end
    uicontrol(fig,'Style','pushbutton','String','Home','Position',[.03 .04 .08 .045], ...
        'Tag','home','Callback',@(~,~) goHome());
    uicontrol(fig,'Style','pushbutton','String','Reset view','Position',[.12 .04 .1 .045], ...
        'Tag','reset-view','Callback',@(~,~) resetView());
    match = uicontrol(fig,'Style','pushbutton','String','Match views', ...
        'Position',[.23 .04 .1 .045],'Tag','match-views', ...
        'TooltipString','Rotate or zoom a plot, then copy its view to the other plot.', ...
        'Callback',@(~,~) matchViews());
    if count == 1, match.Enable = 'off'; end
    items = [{'All axes'},arrayfun(@(i) sprintf('Joint %d',i),1:n,'UniformOutput',false)];
    selection = uicontrol(fig,'Style','popupmenu','String',items,'Value',1, ...
        'Position',[.34 .04 .12 .045],'Tag','axis-selection','Callback',@(~,~) redraw());
    status = uicontrol(fig,'Style','text','Position',[.48 .025 .5 .06], ...
        'BackgroundColor',fig.Color,'HorizontalAlignment','left','Tag','status');
    redraw();
    fig.Visible = 'on';
catch exception
    % A failed launch must not leave an invisible, half-built viewer behind.
    delete(fig);
    rethrow(exception);
end

    function changeJoint(i,source)
        if strcmp(source.Style,'edit')
            degrees = str2double(source.String);
        else
            degrees = source.Value;
        end
        if ~isfinite(degrees) || ~isreal(degrees) || abs(degrees) > 180
            fields(i).String = sprintf('%g',q(i)*180/pi);
            status.String = 'Enter an angle between -180 and 180 degrees.';
            return;
        end
        q(i) = degrees*pi/180;
        fields(i).String = sprintf('%g',degrees);
        sliders(i).Value = degrees;
        redraw(); % MATLAB paints once the callback returns; no nested drawnow.
    end

    function redraw()
        states = cell(1,count);
        for j = 1:count
            [T,p,w] = models(j).fk(q);
            drawScene{j}(T,p,w,selection.Value-1);
            states{j} = struct('T',T,'p',p,'w',w);
        end
        if count == 2
            a = states{1}; b = states{2};
            errors = [norm(a.T-b.T,'fro'),norm(a.w-b.w,'fro'), ...
                norm(cross(b.p-a.p,a.w,1),'fro')];
            status.String = sprintf('Pose: %.1e | Direction: %.1e\nAxis line: %.1e m',errors);
        else
            status.String = sprintf('Endpoint E (m)\nx: %.3f   y: %.3f   z: %.3f', ...
                states{1}.T(1:3,4));
        end
    end

    function goHome()
        q(:) = 0;
        set(sliders,'Value',0);
        set(fields,'String','0');
        redraw();
    end

    function resetView()
        for j = 1:count
            axis(ax(j),reach*[-1 1 -1 1 -1 1]);
            camtarget(ax(j),'auto'); campos(ax(j),'auto'); camva(ax(j),'auto');
            camup(ax(j),'auto');
            view(ax(j),35,25);
        end
    end

    function matchViews()
        source = fig.CurrentAxes;
        if isempty(source) || ~any(source == ax), return; end
        properties = {'XLim','YLim','ZLim','CameraPosition','CameraTarget', ...
            'CameraUpVector','CameraViewAngle'};
        set(ax,properties,repmat(get(source,properties),count,1));
    end
end

function info = describeModel(model)
% Only this adapter knows which FK function and frame convention to use.
validateattributes(model,{'struct'},{'scalar'});
if all(isfield(model,{'xi_hat','r','M'}))
    info = struct('name','PoE: physical joint origins','n',size(model.xi_hat,3), ...
        'fk',@(q) poe_fk(q,model),'chainStyle','-');
elseif all(isfield(model,{'theta','d','a','alpha','tool_transform'}))
    info = struct('name','DH: frame origins (dashed chain)','n',numel(model.theta), ...
        'fk',@(q) dh_fk(q,model),'chainStyle','--');
else
    error('curo6r_viewer:UnknownModel','Use a model from poe_data or dh_data.');
end
[T,p] = info.fk(zeros(info.n,1));
% Distances between successive origins are fixed along this serial chain.
home = [zeros(3,1),p,T(1:3,4)];
info.reach = sum(vecnorm(diff(home,1,2)));
end
