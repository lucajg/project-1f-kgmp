function check_singularity_map
% Check branch identities, workspace slices, and actual UI callbacks.
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

% Both intersections of a translated circle with a ray must be retained.
curve = singularity_workspace_curve([.5;2],[1;0;.3],0);
assert(norm(curve(:,[1 3])-[1.5 .5;2.3 2.3],'fro') < 1e-12);
curve = singularity_workspace_curve([.5;2],[1;0;.3],pi/2);
assert(all(isnan(curve(:))));

map = singularity_map_data(poe,241);
valid = find(isfinite(map.arm(1,:)));
valid = valid(1:7:end);
max_det = 0; max_slice = 0;
for phi = [-2 0 .7 2]
    offset = [1;.4;.7];
    wrist = map.wrist(:,valid);
    slice = singularity_workspace_curve(wrist,offset,phi);
    n = numel(valid);
    for k = 1:n
        arm = map.arm(:,valid(k));
        [~,base] = poe_fk([0;arm;0;pi/2;0],poe);
        for index = [k,n+1+k]
            if ~isfinite(slice(1,index)), continue; end
            tool = [slice(1,index)*[cos(phi);sin(phi)];slice(2,index)];
            pw = tool-offset;
            q1 = atan2(pw(2),pw(1))-atan2(base(2,5),base(1,5));
            q = [q1;arm;0;pi/2;0];
            [~,p] = poe_fk(q,poe);
            max_slice = max(max_slice,norm(p(:,5)+offset-tool));
            max_det = max(max_det,abs(det(poe_jacobian(q,0,6,poe))));
        end
    end
end
assert(max_slice < 1e-10,'Workspace curves do not match forward kinematics.');
assert(max_det < .01,'Interpolated curve is too far from a singularity.');

visibility = get(groot,'defaultFigureVisible');
set(groot,'defaultFigureVisible','off');
restore = onCleanup(@() set(groot,'defaultFigureVisible',visibility));
fig = singularity_map;
cleanup = onCleanup(@() delete(fig));
initial = fig.UserData;
assert(nnz(all(isfinite(initial.Q),1)) == 8);
slider = findobj(fig,'Tag','pose-1-slider');
slider.Value = slider.Value+.001;
tic;
notify(slider,'ContinuousValueChange');  % Exercise the live-drag listener.
latency = toc;
assert(abs(fig.UserData.pose(1)-slider.Value) < 1e-12);
for id = 1:8
    assert(norm(dh_fk(fig.UserData.Q(:,id),dh)-fig.UserData.T,'fro') < 1e-7);
end

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
ax = findobj(fig,'Tag','workspace-map');
assert(hypot(fig.UserData.pose(1),fig.UserData.pose(2)) < ax.XLim(2));
button = findobj(fig,'Tag','reset-pose'); button.Callback(button,[]);
assert(norm(fig.UserData.Q-initial.Q,'fro') < 1e-12);
table = findobj(fig,'Tag','solutions');
table.CellSelectionCallback(table,struct('Indices',[3 1]));
assert(findobj(fig,'Tag','solution-3').LineWidth == 4);

% Feed an exactly singular wrist pose through the six public input fields.
T = poe_fk([.4;-.4;.8;.7;0;-1.1],poe); R = T(1:3,1:3);
pose = [T(1:3,4); atan2(R(3,2),R(3,3))*180/pi; ...
    atan2(-R(3,1),hypot(R(1,1),R(2,1)))*180/pi;atan2(R(2,1),R(1,1))*180/pi];
for i = 1:6
    field = findobj(fig,'Tag',sprintf('pose-%d-field',i));
    field.String = sprintf('%.16g',pose(i)); field.Callback(field,[]);
end
assert(any(strcmp(table.Data(:,8),'Wrist family')));
fprintf('Singularity app passed: stable IDs, FK, workspace slices, live controls, unreachable and singular targets.\n');
fprintf('Max tracking step: %.3g rad; slice error: %.3g m; contour |det J|: %.3g; update: %.3f s.\n', ...
    max_step,max_slice,max_det,latency);
end
