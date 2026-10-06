function check_dh_poe
%CHECK_DH_POE Compare DH, PoE and URDF joint axes and the link_6 pose.
% Run check_dh_poe from the Matlab folder. No Robotics Toolbox is needed.

poe_data;
dh_data;
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

fprintf('DH, PoE and URDF agree for %d configurations.\n', size(Q,2));
% Pose error mixes rotation and position; axis-line error is in metres.
fprintf('%-12s %12s %16s %14s\n', 'Pair', 'Max pose', 'Max direction', 'Max axis line');
for j = 1:size(pairs,1)
    pair_name = [names{pairs(j,1)}, '-', names{pairs(j,2)}];
    fprintf('%-12s %12.3e %16.3e %14.3e\n', pair_name, max_error(j,:));
end
fprintf('Maximum PoE-URDF joint-origin error: %.3e m\n', max_origin_error);
end
