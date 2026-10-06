function [T, p, w] = urdf_fk(q, urdf_model)
%URDF_FK Compose fixed joint origins and local revolute motions.
% q: joint angles in radians; T: pose of link_6 in base_link.
% p(:,i), w(:,i): physical joint origin and unit axis in base_link.

n = urdf_model.n;
validateattributes(q, {'numeric'}, {'real','finite','vector','numel',n});
q = q(:);
p = zeros(3,n);
w = zeros(3,n);
F = eye(4);

Rx = @(t) [1, 0, 0; 0, cos(t), -sin(t); 0, sin(t), cos(t)];
Ry = @(t) [cos(t), 0, sin(t); 0, 1, 0; -sin(t), 0, cos(t)];
Rz = @(t) [cos(t), -sin(t), 0; sin(t), cos(t), 0; 0, 0, 1];

for i = 1:n
    % URDF uses fixed-axis roll, pitch, yaw: R = Rz(yaw)*Ry(pitch)*Rx(roll).
    rpy = urdf_model.rpy(:,i);
    R = Rz(rpy(3)) * Ry(rpy(2)) * Rx(rpy(1));
    origin = [R, urdf_model.xyz(:,i); 0, 0, 0, 1];
    F = F * origin;

    % Place the joint before applying its own motion.
    a = urdf_model.axis(:,i);
    p(:,i) = F(1:3,4);
    w(:,i) = F(1:3,1:3) * a;

    % Rodrigues' formula rotates by q about the local unit axis a.
    K = [0, -a(3), a(2); a(3), 0, -a(1); -a(2), a(1), 0];
    R = eye(3) + sin(q(i))*K + (1-cos(q(i)))*(K*K);
    F = F * [R, zeros(3,1); 0, 0, 0, 1];
end
T = F;
end
