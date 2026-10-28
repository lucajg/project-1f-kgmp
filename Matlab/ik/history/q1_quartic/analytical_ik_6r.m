function Q = analytical_ik_6r(T_d, poe_model, dh_model)
% Full pose IK for CuRo6R; each column of Q is [q1; ...; q6] in radians.
% Solutions lie in [-pi, pi]; unreachable targets return zeros(6,0).
% Singular families use one representative, with q4 = 0 at wrist singularities.

Q_arm = analytical_ik_3r(T_d,poe_model,dh_model);
Q = zeros(6,0);
R_d = T_d(1:3,1:3);
p_d = T_d(1:3,4);
position_tol = 1e-8*max([1, norm(p_d), norm(dh_model.a), norm(dh_model.d)]);
rotation_tol = 1e-8;
singular_tol = 1e-10;
angle_tol = 1e-7;

for i = 1:size(Q_arm,2)
    q_arm = Q_arm(:,i);
    wrist = analytical_ik_wrist(q_arm,R_d,poe_model,singular_tol);

    for j = 1:size(wrist,2)
        q = [q_arm; wrist(:,j)];
        T = poe_fk(q,poe_model,6);
        % Check position and orientation separately: they have different units.
        if norm(T(1:3,4)-p_d) > position_tol ...
                || norm(T(1:3,1:3)-R_d,'fro') > rotation_tol
            continue
        end
        difference = atan2(sin(Q-q),cos(Q-q));
        if all(vecnorm(difference,2,1) > angle_tol)
            Q(:,end+1) = q; %#ok<AGROW>
        end
    end
end
end
