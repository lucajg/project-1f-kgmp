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

% Eq. (ik-wrist-solutions) in report/sections/analytical_ik.tex:
% use the same sign in all angles.
% R = Rx(q4)*Rz(q5)*Rx(q6), so R11 = cos(q5) and eta = abs(sin(q5)).
eta = hypot(R(2,1),R(3,1));
is_singular = eta <= singular_tol;
if ~is_singular
    wrist = zeros(3,2);
    for branch = 1:2
        sign_s5 = (-1)^(branch-1);  % Positive branch first for path tracking.
        q5 = atan2(sign_s5*eta,R(1,1));
        q4 = atan2(sign_s5*R(3,1),sign_s5*R(2,1));
        q6 = atan2(sign_s5*R(1,3),-sign_s5*R(1,2));
        if eta < 1e-6
            % Equivalent extraction near singularity: the first-row entries
            % above are tiny, so their independent roundoff can corrupt the
            % coupled q4 +/- q6. Rx(-q4)*R has unit-length row [0,s6,c6].
            s4 = sin(q4); c4 = cos(q4);
            q6 = atan2(-s4*R(2,2)+c4*R(3,2), ...
                       -s4*R(2,3)+c4*R(3,3));
        end
        wrist(:,branch) = [q4; q5; q6];
    end
elseif R(1,1) >= 0
    % q5 = 0: only sigma = q4 + q6 is determined.
    sigma = atan2(R(3,2),R(2,2));
    wrist = [0; 0; sigma];
else
    % q5 = pi: only q6 - q4 is determined.
    difference = atan2(R(3,2),-R(2,2));
    wrist = [0; pi; difference];
end
end
