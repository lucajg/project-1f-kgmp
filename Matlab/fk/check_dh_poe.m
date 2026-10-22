function check_dh_poe
%CHECK_DH_POE Compare DH, PoE and URDF joint axes and the link_6 pose.
% Run curo6r_setup first, then check_dh_poe. No Robotics Toolbox is needed.

poe_model = poe_data();
dh_model = dh_data();
urdf_data;
n = dh_model.n;
names = {'DH', 'PoE', 'URDF'};
models = {dh_model, poe_model, urdf_model};
fk = {@dh_fk, @poe_fk, @urdf_fk};
pairs = [1, 2; 1, 3; 2, 3];
measures = {'pose', 'axis direction', 'axis line'};
tolerance = 1e-10;

% A local stream keeps the samples repeatable without changing global RNG state.
stream = RandStream('mt19937ar', 'Seed', 42);

% Each column is one configuration: 1 + 6 + 6 + 1 + 500 = 514 total.
Q = [zeros(n,1), ...                       % Home: all joints zero.
     (pi/2)*eye(n), ...                    % Each joint alone at +90 degrees.
     -(pi/2)*eye(n), ...                   % Each joint alone at -90 degrees.
     [20; -35; 50; 15; -40; 60]*pi/180, ... % One mixed pose, degrees -> radians.
     -pi + 2*pi*rand(stream,n,500)];        % 500 random configurations.

% Third dimension selects the model; error rows select the model pair.
T = zeros(4,4,3);
p = zeros(3,n,3);
w = zeros(3,n,3);
max_error = zeros(3,3);
max_origin_error = 0;

for k = 1:size(Q,2)
    q = Q(:,k);
    for m = 1:3
        [T(:,:,m), p(:,:,m), w(:,:,m)] = fk{m}(q, models{m});
    end

    for j = 1:size(pairs,1)
        a = pairs(j,1);
        b = pairs(j,2);
        pose_error = norm(T(:,:,a) - T(:,:,b), 'fro');
        direction_error = norm(w(:,:,a) - w(:,:,b), 'fro');

        % Origins may slide along an axis; only perpendicular separation counts.
        axis_error = norm(cross(p(:,:,a) - p(:,:,b), w(:,:,b), 1), 'fro');
        max_error(j,:) = max(max_error(j,:), ...
            [pose_error, direction_error, axis_error]);
    end

    % PoE and URDF also use the same physical joint origins.
    max_origin_error = max(max_origin_error, norm(p(:,:,2) - p(:,:,3), 'fro'));
end

for j = 1:size(pairs,1)
    for e = 1:numel(measures)
        assert(max_error(j,e) < tolerance, ...
            '%s-%s %s error %.3e exceeds tolerance %.3e.', ...
            names{pairs(j,1)}, names{pairs(j,2)}, measures{e}, ...
            max_error(j,e), tolerance);
    end
end
assert(max_origin_error < tolerance, ...
    'PoE-URDF joint-origin error %.3e exceeds tolerance %.3e.', ...
    max_origin_error, tolerance);

for m = 1:3
    [T_row, p_row, w_row] = fk{m}(q.', models{m});
    assert(norm(T_row - T(:,:,m), 'fro') < tolerance ...
        && norm(p_row - p(:,:,m), 'fro') < tolerance ...
        && norm(w_row - w(:,:,m), 'fro') < tolerance, ...
        '%s row and column joint vectors should give the same result.', names{m});
end

check_poe_prefixes(poe_model,dh_model,urdf_model,Q(:,[1,14,15]));
fprintf('DH, PoE and URDF agree for %d configurations.\n', size(Q,2));
% Pose error mixes rotation and position; axis-line error is in metres.
fprintf('%-12s %12s %16s %14s\n', 'Pair', 'Max pose', 'Max direction', 'Max axis line');
for j = 1:size(pairs,1)
    pair_name = [names{pairs(j,1)}, '-', names{pairs(j,2)}];
    fprintf('%-12s %12.3e %16.3e %14.3e\n', pair_name, max_error(j,:));
end
fprintf('Maximum PoE-URDF joint-origin error: %.3e m\n', max_origin_error);
end

function check_poe_prefixes(poe,dh,urdf,Q)
% Active joint count must not choose or replace the endpoint home pose.
tool_offset = [0,-1,0,.2; 1,0,0,-.1; 0,0,1,.3; 0,0,0,1];
tool = poe;
tool.M = poe.M * tool_offset;
for sample = 1:size(Q,2)
    for n_active = 1:dh.n
        q = Q(1:n_active,sample);
        q_full = [q; zeros(dh.n-n_active,1)];
        [T,p,w,F0] = poe_fk(q,poe,n_active);
        [T_dh,~,~,T0] = dh_fk(q_full,dh);
        [~,p_urdf,w_urdf] = urdf_fk(q_full,urdf);
        assert(norm(T-T_dh,'fro') < 1e-10,'PoE prefix changed the endpoint.');
        assert(norm(p-p_urdf(:,1:n_active),'fro') < 1e-10);
        assert(norm(w-w_urdf(:,1:n_active),'fro') < 1e-10);
        assert(isequal(size(F0),[4,4,n_active+1]));
        assert(isequal(F0(:,:,1),eye(4)));
        assert(norm(poe_fk(q.',poe,n_active)-T,'fro') < 1e-10);
        T_tool = poe_fk(q,tool,n_active);
        assert(norm(T_tool-T_dh*tool_offset,'fro') < 1e-10, ...
            'PoE prefix replaced the custom endpoint home pose.');
        if n_active == 3
            F = F0(:,:,end);
            wrist = F(1:3,1:3)*poe.r(:,5) + F(1:3,4);
            assert(norm(wrist-T0(1:3,4,6)) < 1e-10, ...
                'PoE arm prefix disagrees with the DH wrist center.');
        end
    end
end
fprintf('PoE prefixes passed: tool poses, joint axes, row inputs, and wrist center.\n');
end
