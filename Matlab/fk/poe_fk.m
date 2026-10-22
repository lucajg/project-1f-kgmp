function [T, p, w, F0] = poe_fk(q, poe_model, n_active)
% Endpoint pose and physical joint axes in base_link; F0 stores PoE prefixes.
% n_active selects the first joint motions; M defines the endpoint independently.
% F0(:,:,end) maps a home point carried by those motions into base_link.
if nargin < 3
    n_active = size(poe_model.xi_hat,3);
end
validate_joint_vector(q,n_active);
q = q(:);
p = zeros(3,n_active);
w = zeros(3,n_active);
F0 = zeros(4,4,n_active+1);
if isa(q,'sym') || isa(poe_model.r,'sym')
    p = sym(p);
    w = sym(w);
    F0 = sym(F0);
end
F0(:,:,1) = eye(4);

for i = 1:n_active
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
