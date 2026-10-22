function [wrist, is_singular] = analytical_ik_wrist(q_arm, R_d, poe_model, singular_tol)
% Wrist orientation IK for a prescribed CuRo6R arm configuration.
% q_arm contains q1:q3 in radians; row and column vectors are accepted.
% R_d is the desired link_6 orientation in base_link (a 3 x 3 rotation).
% Each column of wrist is [q4; q5; q6], in radians in [-pi, pi].
% A regular wrist returns both flips, with positive sin(q5) first.
% singular_tol defaults to 1e-10 and thresholds abs(sin(q5)).
% At or below that tolerance, is_singular is true and one representative
% with q4 = 0 is returned. It does not define a continuous wrist lift.

if nargin < 4
    singular_tol = 1e-10;
end
validateattributes(R_d,{'numeric'},{'real','finite','size',[3,3]});
validateattributes(singular_tol,{'numeric'},{'scalar','real','finite','nonnegative'});

[~,~,~,F0] = poe_fk(q_arm,poe_model,3);
% Link 3 and link_6 share home axes. The PoE prefix moves those home axes.
R_arm = F0(1:3,1:3,end)*poe_model.M(1:3,1:3);
R = R_arm.'*R_d;

% R = Rx(q4)*Rz(q5)*Rx(q6), so R11 = cos(q5) and rho = abs(sin(q5)).
rho = hypot(R(2,1),R(3,1));
is_singular = rho <= singular_tol;
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
end
