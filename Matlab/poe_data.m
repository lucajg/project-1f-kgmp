% CuRo6R home geometry in base_link. 
% UNITS: 
%   distances: [m];
%   angles: [rad].
% Run once to create model, then call poe_fk(q, poe_model) or curo6r_viewer(poe_model).
% Endpoint E is link_6.

robot_dimensions;            % Shared named dimensions in metres.
alpha = pi/6;                 % Use 0.52359877559 to match the URDF.
c = cos(alpha);
s = sin(alpha);

% Unit rotation axes at the home configuration, one joint per column.
omega = zeros(3,6);
omega(:,1) = [0; 0; 1];
omega(:,2) = [0; c; s];
omega(:,3) = [0; -s; c];
omega(:,4) = [1; 0; 0];
omega(:,5) = omega(:,3);
omega(:,6) = omega(:,4);

% Successive joint-origin offsets.
r = zeros(3,6);
r(:,1) = [0; 0; h1];
r(:,2) = r(:,1) + [l2; 0; h2];
r(:,3) = r(:,2) + l3*[1; 0; 0] + w3*[0; c; s];
r(:,4) = r(:,3) + l4*[1; 0; 0] + h4*[0; -s; c];
r(:,5) = r(:,4) + [l5; 0; 0];
r(:,6) = r(:,5) + [l6; 0; 0];

% Compute v = r x omega = -omega x r.
v = zeros(3,6);
xi_hat = zeros(4,4,6);
for i = 1:6
    v(:,i) = cross(r(:,i), omega(:,i));
    w = omega(:,i);
    omega_skew = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
    xi_hat(:,:,i) = [omega_skew v(:,i); 0 0 0 0];
end
S = [omega; v];

% Endpoint pose at q = 0.
R0 = [1 0 0; 0 c -s; 0 s c];
M = [R0 r(:,6); 0 0 0 1];

poe_model = struct('omega', omega, 'r', r, 'S', S, 'xi_hat', xi_hat, 'M', M);
