function [T, p, w, T0] = dh_fk(q, dh_model)
% Endpoint pose and joint axes in base_link; T0(:,:,i+1) is DH frame i.
n = numel(dh_model.theta);
validate_joint_vector(q,n);
q = q(:);
p = zeros(3,n);
w = zeros(3,n);
T0 = zeros(4,4,n+1);
if isa(q,'sym') || isa(dh_model.theta,'sym')
    p = sym(p);
    w = sym(w);
    T0 = sym(T0);
end
T0(:,:,1) = eye(4);

for i = 1:n
    % The preceding DH origin and z-axis locate joint i.
    p(:,i) = T0(1:3,4,i);
    w(:,i) = T0(1:3,3,i);
    A = dh_transform(dh_model.theta(i)+q(i), dh_model.d(i), ...
        dh_model.a(i), dh_model.alpha(i));
    F = T0(:,:,i) * A;
    if isa(F,'sym')
        F = simplify(F);
    end
    T0(:,:,i+1) = F;
end
T = T0(:,:,end) * dh_model.tool_transform;
end
