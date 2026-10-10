function Q = analytical_ik_3r(T_d, poe_model, dh_model)
% Position IK for CuRo6R: each column of Q is [q1; q2; q3] in radians.
% T_d is the desired link_6 pose; only its wrist center is constrained.
% Solutions lie in [-pi, pi]; unreachable targets return zeros(3,0).
% A continuous family is represented by one solution. Use numeric models.

validateattributes(T_d, {'double'}, {'real','finite','size',[4,4]});
R_d = T_d(1:3,1:3);
if norm(R_d.'*R_d-eye(3),'fro') > 1e-8 || det(R_d) < 0 ...
        || norm(T_d(4,:)-[0,0,0,1]) > 1e-10
    error('analytical_ik_3r:InvalidPose', 'T_d must be a rigid transform.');
end

a1 = dh_model.a(1);
a2 = dh_model.a(2);
d1 = dh_model.d(1);
d2 = dh_model.d(2);
d3 = dh_model.d(3);
d4 = dh_model.d(4);
d6 = dh_model.d(6);
validateattributes([a1,a2,d1,d2,d3,d4,d6], {'double'}, {'real','finite'});
if a2 == 0 || d4 == 0
    error('analytical_ik_3r:DegenerateGeometry', ...
        'This derivation requires nonzero a2 and d4.');
end

p_w = T_d(1:3,4) - d6*R_d(:,1);
x = p_w(1);
y = p_w(2);
h = p_w(3) - d1;
c = sqrt(3)/2;
K = d2^2 - a2^2 - d3^2 - d4^2;
N0 = x^2 + y^2 + h^2 + a1^2;
position_tol = 1e-8*max([1, norm(p_w), abs(a1), abs(a2), abs(d4)]);
angle_tol = 1e-7;

% With t = tan(q1/2), Y = Y_tilde/D and N = N_tilde/D.
% Coefficients are in descending powers of t: [quadratic, linear, constant].
D = [1, 0, 1];
Y_tilde = [h/2-c*y, -2*c*x, h/2+c*y];
N_tilde = [N0+2*a1*x, -4*a1*y, N0-2*a1*x];
S = Y_tilde - d2*D;
C = N_tilde + K*D - 2*d2*Y_tilde;

% Clear denominators in sin(q3)^2 + cos(q3)^2 = 1; conv squares a polynomial.
polynomial = 4*a2^2*conv(S,S) + conv(C,C) - 4*a2^2*d4^2*conv(D,D);
scale = 4*a2^2*norm(S,1)^2 + norm(C,1)^2 + 4*a2^2*d4^2*norm(D,1)^2;
if norm(polynomial,inf) <= 128*eps*scale
    % The constraint is identically zero: choose one member of the family.
    q1_candidates = 0;
else
    t = roots(polynomial/norm(polynomial,inf));
    t = t(abs(imag(t)) <= 1e-6*(1+abs(real(t))));
    % q1 = pi is missing from the half-angle chart; FK will check it below.
    q1_candidates = [2*atan(real(t)); pi];
end

% Move the wrist-center home point, rather than the original tool endpoint.
arm_model = poe_model;
arm_model.M(1:3,4) = poe_model.r(:,5);
Q = zeros(3,0);
for q1 = q1_candidates.'
    % Undo joint 1, translate to the shoulder, then undo the fixed 30-degree tilt.
    u = x*cos(q1) + y*sin(q1);
    v = -x*sin(q1) + y*cos(q1);
    X = u - a1;
    Y = c*v + h/2;
    Z = -v/2 + c*h;
    N = X^2 + Y^2 + Z^2;

    s3 = (Y-d2)/d4;
    c3 = (N+K-2*d2*Y)/(2*a2*d4);
    q3 = atan2(s3,c3);
    A = a2 + d4*cos(q3);
    q2 = atan2(d3*X-A*Z, A*X+d3*Z);
    q = [q1; q2; q3];

    T_w = poe_fk(q,arm_model,3);
    if norm(T_w(1:3,4)-p_w) > position_tol
        continue
    end
    % Compare angles modulo 2*pi to merge repeated roots and the +/-pi boundary.
    difference = atan2(sin(Q-q),cos(Q-q));
    if all(vecnorm(difference,2,1) > angle_tol)
        Q(:,end+1) = q; %#ok<AGROW>
    end
end
end
