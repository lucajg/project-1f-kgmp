function [T, p, w] = poe_fk(q, poe_model)
% q: joint angles in radians. T: endpoint pose in the base frame.
% p(:,i), w(:,i): current joint origin and axis direction in the base frame.

n = size(poe_model.xi_hat, 3);
validateattributes(q, {'numeric'}, {'real','finite','vector','numel',n});
q = q(:);
p = zeros(3,n);
w = zeros(3,n);
F = eye(4);

for i = 1:n
    % Here F contains only joints 1,...,i-1: they carry joint i's home axis.
    R = F(1:3,1:3);
    t = F(1:3,4);
    p(:,i) = R*poe_model.r(:,i) + t;
    w(:,i) = R*poe_model.omega(:,i);

    % Add joint i's motion to the running product.
    F = F * expm(poe_model.xi_hat(:,:,i)*q(i));
    % Now F contains joints 1,...,i.
end
T = F * poe_model.M;
end
