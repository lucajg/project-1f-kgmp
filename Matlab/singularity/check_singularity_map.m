function check_singularity_map
% Check branch identities, wrist-center maps, and actual UI callbacks.
poe = poe_data(); dh = dh_data();
source = [-45;162;84;36;-124;-124]*pi/180;
T = poe_fk(source,poe);
raw = analytical_ik_6r(T,poe,dh);
assert(size(raw,2) == 8);
[first,memory] = match_ik_solutions(raw,nan(6,8));

% Root ordering, absent branches, and +/-pi wrapping must not change IDs.
[again,memory] = match_ik_solutions(raw(:,8:-1:1),memory);
assert(norm(first-again,'fro') < 1e-12);
[partial,memory] = match_ik_solutions(first(:,[2 5 8]),memory);
assert(isequal(find(all(isfinite(partial),1)),[2 5 8]));
[empty,memory] = match_ik_solutions(zeros(6,0),memory);
assert(all(isnan(empty(:))));
[again,memory] = match_ik_solutions(raw,memory);
assert(norm(first-again,'fro') < 1e-12);
wrapped = atan2(sin(first+.03),cos(first+.03));
[again,~] = match_ik_solutions(wrapped(:,[8 3 1 7 2 5 4 6]),memory);
assert(norm(again-wrapped,'fro') < 1e-12);

% Follow a physical motion, independently validating every solution with DH.
previous = first; memory = first; max_step = 0;
for a = linspace(0,.04,41)
    q = source+[a;-a/2;a/3;0;0;0];
    T = poe_fk(q,poe);
    [Q,memory] = match_ik_solutions(analytical_ik_6r(T,poe,dh),memory);
    assert(all(isfinite(Q(:))));
    delta = atan2(sin(Q-previous),cos(Q-previous));
    max_step = max(max_step,max(vecnorm(delta)));
    for id = 1:8, assert(norm(dh_fk(Q(:,id),dh)-T,'fro') < 1e-7); end
    previous = Q;
end
assert(max_step < .05,'Solution IDs jumped along a regular path.');

% Wrist-center critical values are independent of base angle and wrist joints.
map = singularity_map_data(poe,241);
valid = find(isfinite(map.arm(1,:)));
valid = valid(1:7:end);
max_det = 0; max_workspace = 0;
for q1 = [-2 0 .7 2]
    for k = valid
        q = [q1;map.arm(:,k);.3+q1;pi/2;-.4-q1];
        [~,p] = dh_fk(q,dh);
        max_workspace = max(max_workspace,norm([hypot(p(1,5),p(2,5));p(3,5)]-map.wrist(:,k)));
        max_det = max(max_det,abs(det(poe_jacobian(q,0,6,poe))));
    end
end
assert(max_workspace < 1e-10,'Wrist-center curves do not match independent DH kinematics.');
assert(max_det < .01,'Interpolated curve is too far from a singularity.');

visibility = get(groot,'defaultFigureVisible');
set(groot,'defaultFigureVisible','off');
restore = onCleanup(@() set(groot,'defaultFigureVisible',visibility));
fig = singularity_map;
cleanup = onCleanup(@() delete(fig));
initial = fig.UserData;
assert(nnz(all(isfinite(initial.Q),1)) == 8);
assert(isequal(initial.pose,[3;0;3;0;0;0]),'Unexpected startup target.');
assert(norm(initial.T-[eye(3),[3+dh.d(6);0;3];0 0 0 1],'fro') < 1e-12);
button = findobj(fig,'Tag','reset-pose');
boundary = findobj(fig,'Tag','arm-boundary');
target = findobj(fig,'Tag','wrist-target');
ax = findobj(fig,'Tag','workspace-map');
check_app_state(fig,map,dh);

% Position sliders translate the wrist and tool together at fixed tool orientation.
latency = 0;
for i = 1:3
    slider = findobj(fig,'Tag',sprintf('pose-%d-slider',i));
    slider.Value = slider.Value+.01;
    tic;
    notify(slider,'ContinuousValueChange');
    latency = max(latency,toc);
    displacement = zeros(3,1); displacement(i) = .01;
    assert(norm(fig.UserData.wristPosition-initial.wristPosition-displacement) < 1e-12);
    assert(norm(fig.UserData.T(1:3,4)-initial.T(1:3,4)-displacement) < 1e-12);
    assert(isequal(fig.UserData.T(1:3,1:3),initial.T(1:3,1:3)));
    check_app_state(fig,map,dh);
    button.Callback(button,[]);
end

% Orientation edits keep the wrist target and arm solutions fixed, retaining both flips.
for i = 4:6
    field = findobj(fig,'Tag',sprintf('pose-%d-field',i));
    field.String = sprintf('%.16g',initial.pose(i)-5);
    field.Callback(field,[]);
    assert(abs(fig.UserData.pose(i)-(initial.pose(i)-5)) < 1e-12);
    assert(isequal(fig.UserData.wristPosition,initial.wristPosition));
    assert(norm(fig.UserData.T(1:3,1:3)-initial.T(1:3,1:3),'fro') > .01);
    delta = atan2(sin(fig.UserData.Q(1:3,:)-initial.Q(1:3,:)), ...
        cos(fig.UserData.Q(1:3,:)-initial.Q(1:3,:)));
    assert(norm(delta,'fro') < 1e-7,'Changing tool orientation moved the arm solutions.');
    check_app_state(fig,map,dh);
    button.Callback(button,[]);
end

% Exercise the actual drag callbacks. A small view around the initial target
% bounds the current pointer to a nearby reachable point, even in a hidden figure.
oldLimits = [ax.XLim;ax.YLim];
dragLimits = [hypot(initial.pose(1),initial.pose(2));initial.pose(3)]+[-.01 .01];
ax.XLim = dragLimits(1,:); ax.YLim = dragLimits(2,:);
pointer = fig.Pointer;
target.ButtonDownFcn(target,[]);
assert(~isempty(fig.WindowButtonMotionFcn) && strcmp(fig.Pointer,'fleur'));
point = ax.CurrentPoint(1,1:2).';
point = max(dragLimits(:,1),min(point,dragLimits(:,2)));
expected = [point(1)*[cos(initial.azimuth);sin(initial.azimuth)];point(2)];
fig.WindowButtonMotionFcn(fig,[]);
assert(norm(fig.UserData.wristPosition-expected) < 1e-12);
assert(isequal(fig.UserData.T(1:3,1:3),initial.T(1:3,1:3)));
assert(isequal([ax.XLim;ax.YLim],dragLimits),'The axes changed during dragging.');
for i = 1:3
    slider = findobj(fig,'Tag',sprintf('pose-%d-slider',i));
    field = findobj(fig,'Tag',sprintf('pose-%d-field',i));
    assert(abs(slider.Value-expected(i)) < 1e-12);
    assert(abs(str2double(field.String)-expected(i)) < 5.1e-5);
end
check_app_state(fig,map,dh);
fig.WindowButtonUpFcn(fig,[]);
assert(isempty(fig.WindowButtonMotionFcn) && isempty(fig.WindowButtonUpFcn));
assert(strcmp(fig.Pointer,pointer));
ax.XLim = oldLimits(1,:); ax.YLim = oldLimits(2,:);
button.Callback(button,[]);

% An invalid edit must leave the target intact; a far corner is unreachable.
field = findobj(fig,'Tag','pose-2-field');
before = fig.UserData.T;
field.String = 'NaN'; field.Callback(field,[]);
assert(isequal(fig.UserData.T,before));
for i = 1:3
    slider = findobj(fig,'Tag',sprintf('pose-%d-slider',i));
    slider.Value = slider.Max; slider.Callback(slider,[]);
end
assert(all(isnan(fig.UserData.Q(:))));
assert(contains(findobj(fig,'Tag','ik-status').String,'Unreachable'));
assert(hypot(fig.UserData.pose(1),fig.UserData.pose(2)) < ax.XLim(2));
assert(isequaln([boundary.XData;boundary.YData],map.wrist));
button.Callback(button,[]);
assert(norm(fig.UserData.Q-initial.Q,'fro') < 1e-12);
assert(isequal(fig.UserData.pose,initial.pose),'Reset must restore the default target.');
table = findobj(fig,'Tag','solutions');
table.CellSelectionCallback(table,struct('Indices',[3 1]));
assert(findobj(fig,'Tag','solution-3').LineWidth == 4);

% Feed an exactly singular wrist pose through the six public input fields.
T = poe_fk([.4;-.4;.8;.7;0;-1.1],poe); R = T(1:3,1:3);
pose = [T(1:3,4)-dh.d(6)*R(:,1); atan2(R(3,2),R(3,3))*180/pi; ...
    atan2(-R(3,1),hypot(R(1,1),R(2,1)))*180/pi;atan2(R(2,1),R(1,1))*180/pi];
for i = 1:6
    field = findobj(fig,'Tag',sprintf('pose-%d-field',i));
    field.String = sprintf('%.16g',pose(i)); field.Callback(field,[]);
end
assert(any(strcmp(table.Data(:,8),'Wrist family')));
assert(norm(fig.UserData.T-T,'fro') < 1e-12);
assert(isequaln([boundary.XData;boundary.YData],map.wrist));
fprintf('Singularity app passed: stable IDs, FK, fixed wrist-center map, independent position/orientation, unreachable and singular targets.\n');
fprintf('Max tracking step: %.3g rad; workspace error: %.3g m; contour |det J|: %.3g; update: %.3f s.\n', ...
    max_step,max_workspace,max_det,latency);
end

function check_app_state(fig,map,dh)
% Check physical coordinates and all regular branches independently of the UI transform.
state = fig.UserData;
boundary = findobj(fig,'Tag','arm-boundary');
target = findobj(fig,'Tag','wrist-target');
assert(isequaln([boundary.XData;boundary.YData],map.wrist),'The workspace boundary moved.');
assert(isequal(state.pose(1:3),state.wristPosition));
assert(norm([target.XData;target.YData]-[hypot(state.pose(1),state.pose(2));state.pose(3)]) < 1e-12);
assert(all(isfinite(state.Q(:))),'Expected eight regular IK solutions.');
for id = 1:8
    [T,p] = dh_fk(state.Q(:,id),dh);
    assert(norm(T-state.T,'fro') < 1e-7);
    assert(norm(p(:,5)-state.wristPosition) < 1e-7,'IK does not place the wrist at its target.');
    delta = atan2(sin(state.Q(1:3,:)-state.Q(1:3,id)),cos(state.Q(1:3,:)-state.Q(1:3,id)));
    pair = find(vecnorm(delta) < 1e-7);
    assert(numel(pair) == 2 && prod(sin(state.Q(5,pair))) < 0,'An arm solution lost its wrist flip.');
end
end
