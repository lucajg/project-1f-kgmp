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
    [~,~,~,F0] = poe_fk(q_arm,poe_model,3);
    % Link 3 and link_6 share home axes. The PoE prefix moves those home axes.
    R_arm = F0(1:3,1:3,end)*poe_model.M(1:3,1:3);
    R = R_arm.'*R_d;

    % R = Rx(q4)*Rz(q5)*Rx(q6), so R11 = cos(q5) and rho = abs(sin(q5)).
    rho = hypot(R(2,1),R(3,1));
    if rho > singular_tol
        wrist = zeros(3,2);
        for branch = 1:2
            s5 = (-1)^(branch-1)*rho;
            q5 = atan2(s5,R(1,1));
            % Dividing by the signed sine preserves both wrist branches.
            q4 = atan2(R(3,1)/s5,R(2,1)/s5);
            % Remove q4: Rz(q5)*Rx(q6) has third row [0, sin(q6), cos(q6)].
            % This avoids dividing by a small sin(q5) when extracting q6.
            R4 = [1,0,0; 0,cos(q4),-sin(q4); 0,sin(q4),cos(q4)];
            remaining = R4.'*R;
            q6 = atan2(remaining(3,2),remaining(3,3));
            wrist(:,branch) = [q4; q5; q6];
        end
    elseif R(1,1) >= 0
        % q5 = 0: only sigma = q4 + q6 is determined.
        sigma = atan2(R(3,2),R(2,2));
        wrist = [0; 0; sigma];
    else
        % q5 = pi: R*Rz(pi)' = Rx(delta), where delta = q4 - q6.
        delta = atan2(-R(3,2),-R(2,2));
        wrist = [0; pi; -delta];
    end

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
