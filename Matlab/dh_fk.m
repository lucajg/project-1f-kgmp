function [T, p, w] = dh_fk(q, dh_model)
%DH_FK Forward kinematics using the standard DH convention.
% q: joint angles in radians.
% T: pose of endpoint E = link_6 in base_link.
% p(:,i): origin of DH frame i-1, a point on joint i's axis.
% w(:,i): unit direction of joint i's axis. Both are in base_link.
% p may differ from poe_fk's joint origin along the SAME axis line.

n = numel(dh_model.theta);
validateattributes(q, {'numeric'}, {'real','finite','vector','numel',n});
q = q(:);
p = zeros(3,n);
w = zeros(3,n);
F = eye(4);

% The four elementary transforms make the DH construction explicit.
Rz = @(t) [cos(t), -sin(t), 0, 0;
           sin(t),  cos(t), 0, 0;
                0,       0, 1, 0;
                0,       0, 0, 1];
Tz = @(d) [1, 0, 0, 0;
           0, 1, 0, 0;
           0, 0, 1, d;
           0, 0, 0, 1];
Tx = @(a) [1, 0, 0, a;
           0, 1, 0, 0;
           0, 0, 1, 0;
           0, 0, 0, 1];
Rx = @(b) [1,      0,       0, 0;
           0, cos(b), -sin(b), 0;
           0, sin(b),  cos(b), 0;
           0,      0,       0, 1];

for i = 1:n
    % Before A_i, F = ^0 T_(i-1). Its origin and z-axis locate joint i.
    p(:,i) = F(1:3,4);
    w(:,i) = F(1:3,3);

    theta = dh_model.theta(i) + q(i);
    A = Rz(theta) * Tz(dh_model.d(i)) * Tx(dh_model.a(i)) ...
        * Rx(dh_model.alpha(i));
    F = F * A; % Now F = ^0 T_i.
end

% Convert the final DH frame to the endpoint frame used by the PoE model.
T = F * dh_model.tool_transform;
end
