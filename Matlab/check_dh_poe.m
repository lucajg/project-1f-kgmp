function check_dh_poe
%CHECK_DH_POE Check that DH and PoE describe the same robot and endpoint.
% Run check_dh_poe from the Matlab folder. No Robotics Toolbox is needed.

poe_data;
dh_data;
n = dh_model.n;

% A local Mersenne Twister generator.
stream = RandStream('mt19937ar', 'Seed', 42);

% Each column of Q is one configuration (n joint angles in radians).
% Concatenating these blocks gives 1 + n + n + 1 + 100 = 114 columns.
% rand(stream,n,100) draws an n-by-100 array of values between 0 and 1;
% multiplying by 2*pi and subtracting pi gives angles between -pi and pi.
Q = [zeros(n,1), ...                       % Home: all joints zero.
     (pi/2)*eye(n), ...                    % Each joint alone at +90 degrees.
     -(pi/2)*eye(n), ...                   % Each joint alone at -90 degrees.
     [20; -35; 50; 15; -40; 60]*pi/180, ... % One mixed pose, degrees -> radians.
     -pi + 2*pi*rand(stream,n,500)];        % 100 random configurations.

% Largest errors seen so far across all configurations:
% max_error(1): endpoint pose matrix error (rotation and position combined).
% max_error(2): joint-axis direction error (dimensionless).
% max_error(3): joint-axis line-location error (metres).
% Each error is a Frobenius norm, combining all entries of its error matrix.
max_error = zeros(1,3);

for k = 1:size(Q,2)
    q = Q(:,k);
    [T_poe, p_poe, w_poe] = poe_fk(q, poe_model);
    [T_dh, p_dh, w_dh] = dh_fk(q, dh_model);

    pose_error = norm(T_dh - T_poe, 'fro');
    direction_error = norm(w_dh - w_poe, 'fro');

    % Origins may slide along an axis without changing the revolute joint.
    % Their perpendicular separation must vanish: (p_dh-p_poe) x w_poe = 0.
    axis_error = norm(cross(p_dh - p_poe, w_poe, 1), 'fro');
    
    % Element-by-element maximum: retain the worst value of each error type.
    max_error = max(max_error, [pose_error, direction_error, axis_error]);
end

tolerance = 1e-10;
assert(all(max_error < tolerance), 'DH and PoE models do not agree.');

% Both input vector orientations should give the same outputs.
[T_row, p_row, w_row] = dh_fk(q.', dh_model);
assert(norm(T_row - T_dh, 'fro') < tolerance ...
    && norm(p_row - p_dh, 'fro') < tolerance ...
    && norm(w_row - w_dh, 'fro') < tolerance, ...
    'Row and column joint vectors should give the same result.');

fprintf('DH and PoE agree for %d configurations.\n', size(Q,2));
fprintf('Maximum pose error:           %.3e\n', max_error(1));
fprintf('Maximum axis-direction error: %.3e\n', max_error(2));
fprintf('Maximum axis-line error:      %.3e\n', max_error(3));
end
