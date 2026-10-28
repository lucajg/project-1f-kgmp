function Q = analytical_ik_3r(T_d, poe_model, dh_model)
% Position IK from the q3 quartic in report/sections/analytical_ik.tex.
% Each column of Q is [q1; q2; q3] in radians.
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
if a1 == 0 || d3 == 0 || d4 == 0
    error('analytical_ik_3r:DegenerateGeometry', ...
        'This derivation requires nonzero a1, d3 and d4.');
end

p_w = T_d(1:3,4) - d6*R_d(:,1);
x = p_w(1);
y = p_w(2);
h = p_w(3) - d1;
alpha = pi/6;  % Fixed URDF shoulder tilt, not the first DH twist.
c_alpha = cos(alpha);
s_alpha = sin(alpha);
rho = hypot(x,y);
Rcal = rho^2 + h^2;
L = a1^2 + a2^2 + d2^2 + d3^2 + d4^2;
position_tol = 1e-8*max([1, norm(p_w), abs(a1), abs(a2), abs(d4)]);
axis_tol = 128*eps*max([1, norm(p_w), abs(a1), abs(a2), abs(d4)]);
angle_tol = 1e-7;

% With t = tan(q3/2), use the hatted numerators in eq. (quartic).
% Coefficients are in descending powers of t: [quadratic, linear, constant].
Delta = [1, 0, 1];
F1_hat = [a2-d4, 0, a2+d4];
B_hat = [d2, 2*d4, d2];
F3_hat = [L-2*a2*d4, 4*d2*d4, L+2*a2*d4];
U = Rcal*Delta - F3_hat;
V = h*Delta - s_alpha*B_hat;

% P = cos(alpha)^2*U^2 + 4*a1^2*V^2
%     - 4*a1^2*cos(alpha)^2*(F1_hat^2 + d3^2*Delta^2).
polynomial = c_alpha^2*conv(U,U) + 4*a1^2*conv(V,V) ...
    - 4*a1^2*c_alpha^2*(conv(F1_hat,F1_hat) + d3^2*conv(Delta,Delta));
scale = c_alpha^2*norm(U,1)^2 + 4*a1^2*norm(V,1)^2 ...
    + 4*a1^2*c_alpha^2*(norm(F1_hat,1)^2 + d3^2*norm(Delta,1)^2);
if rho <= axis_tol
    % Here A = Y = 0, hence B = sin(alpha)*h. Recover q3 directly to
    % avoid splitting a repeated quartic root at this continuous q1 family.
    s3 = (s_alpha*h-d2)/d4;
    if abs(s3) > 1+128*eps
        Q = zeros(3,0);
        return
    end
    q3 = asin(min(1,max(-1,s3)));
    q3_candidates = [q3; atan2(sin(q3),-cos(q3))];
elseif norm(polynomial,inf) <= 128*eps*scale
    q3_candidates = 0;  % Representative if special dimensions give P == 0.
else
    % Use bounded roots in two charts of the SAME quartic. Near q3 = pi,
    % solve u^4*P(1/u) with u = cot(q3/2) to avoid huge, ill-conditioned
    % tangent roots (especially repeated roots). Reversing coefficients
    % performs this change of variable; roots handles exact degree drops.
    p = polynomial/norm(polynomial,inf);
    t = bounded_real_roots(p);
    u = bounded_real_roots(fliplr(p));
    q3_candidates = [2*atan(t); atan2(2*u,u.^2-1); pi];
end

Q = zeros(3,0);
for q3 = q3_candidates.'
    F1 = a2 + d4*cos(q3);
    B = d2 + d4*sin(q3);
    F3 = L + 2*a2*d4*cos(q3) + 2*d2*d4*sin(q3);
    X = (Rcal-F3)/(2*a1);
    D = (h-s_alpha*B)/c_alpha;
    closure = X^2 + D^2 - F1^2 - d3^2;
    closure_scale = max([1, X^2+D^2, F1^2+d3^2]);
    closure_tol = 1e-8*closure_scale;
    if q3 == pi
        % Test the omitted point at roundoff precision, not the looser FK
        % tolerance: a nearby double root must not add a spurious pi branch.
        closure_tol = 128*eps*scale/(4*a1^2*c_alpha^2);
    end
    if abs(closure) > closure_tol
        continue
    end
    % Eq. (q2): the common positive denominator F1^2+d3^2 cancels in atan2.
    q2 = atan2(d3*X-F1*D, F1*X+d3*D);
    A = a1 + X;
    Y = c_alpha*B - s_alpha*D;
    if rho <= axis_tol
        q1 = 0;  % On axis 1, choose one representative of the free base angle.
    else
        q1 = atan2(A*y-Y*x, A*x+Y*y);
    end
    q = [q1; q2; q3];

    % The first three joints carry the wrist center at home origin 5.
    [~,~,~,F0] = poe_fk(q,poe_model,3);
    F = F0(:,:,end);
    p_check = F(1:3,1:3)*poe_model.r(:,5) + F(1:3,4);
    if norm(p_check-p_w) > position_tol
        continue
    end
    % Compare angles modulo 2*pi to merge repeated roots and the +/-pi boundary.
    difference = atan2(sin(Q-q),cos(Q-q));
    if all(vecnorm(difference,2,1) > angle_tol)
        Q(:,end+1) = q; %#ok<AGROW>
    end
end
end

function r = bounded_real_roots(p)
r = roots(p);
% A small overlap between charts avoids losing a root at t = u = +/-1;
% the final angle comparison removes any duplicates from that overlap.
keep = abs(imag(r)) <= 1e-6*(1+abs(real(r))) & abs(real(r)) <= 1+1e-8;
r = real(r(keep));
end
