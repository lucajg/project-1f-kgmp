function [T, p, w, F0] = poe_fk(q, poe_model)
% Endpoint pose and physical joint axes in base_link; F0 stores PoE prefixes.
n = size(poe_model.xi_hat,3);
validate_joint_vector(q,n);
q = q(:);
p = zeros(3,n);
w = zeros(3,n);
F0 = zeros(4,4,n+1);
if isa(q,'sym') || isa(poe_model.r,'sym')
    p = sym(p);
    w = sym(w);
    F0 = sym(F0);
end
F0(:,:,1) = eye(4);

for i = 1:n
    F = F0(:,:,i);
    p(:,i) = F(1:3,1:3)*poe_model.r(:,i) + F(1:3,4);
    w(:,i) = F(1:3,1:3)*poe_model.omega(:,i);

    % Closed-form exp(xi_hat*q) for a unit revolute axis through r(:,i).
    w_skew = poe_model.xi_hat(1:3,1:3,i);
    R = eye(3) + sin(q(i))*w_skew + (1-cos(q(i)))*w_skew^2;
    E = [R, (eye(3)-R)*poe_model.r(:,i); 0, 0, 0, 1];
    F = F * E;
    if isa(F,'sym')
        F = simplify(F);
    end
    F0(:,:,i+1) = F;
end
T = F0(:,:,end) * poe_model.M;
end
