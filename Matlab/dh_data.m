% Denavit-Hartenberg data for CuRo6R.
% UNITS:
%   distances: [m];
%   angles: [rad].
% Run dh_data, then call dh_fk(q, dh_model).
% Base frame 0 is base_link; endpoint E is link_6, as in poe_data.
%
% Joint i rotates about z_(i-1). The transform from frame i to frame i-1 is
% A_i(q_i) = Rz(theta_i + q_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i).
% theta contains fixed offsets; q uses the same joint zeros/signs as PoE.

robot_dimensions; % Shared named dimensions in metres.

%         Joint:  1      2      3      4      5      6
theta = [        0,     0,  pi/2,     0,     0,     0];
d     = [    h1+h2,    w3,    h4, l4+l5,     0,    l6];
a     = [       l2,    l3,     0,     0,     0,     0];
alpha = [    -pi/3,  pi/2,  pi/2, -pi/2,  pi/2,     0];
n = numel(theta);

% DH origins need not coincide with the physical joint origins in PoE.

% Fixed pose of endpoint E in DH frame 6: ^6 T_E.
% The origins coincide, with x_E = z_6, y_E = x_6, z_E = y_6.
% Append this on the RIGHT: ^0 T_E = A_1 * ... * A_6 * ^6 T_E.
tool_transform = [0, 1, 0, 0;
                  0, 0, 1, 0;
                  1, 0, 0, 0;
                  0, 0, 0, 1];

dh_model = struct('n', n, 'theta', theta, 'd', d, 'a', a, ...
    'alpha', alpha, 'tool_transform', tool_transform);
