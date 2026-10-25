function J = point_jacobian(p,w,pk,R0frame)
% Geometric Jacobian [v; omega] at pk, expressed along frame's axes.
% p and w contain revolute joint origins and unit axes in the base frame.
% pk is the reference point in the base frame; R0frame rotates frame to base.
% Reuse FK outputs here when several Jacobians are needed at the same q.
J = [cross(w,pk-p,1); w];
J = blkdiag(R0frame.',R0frame.') * J;
end
