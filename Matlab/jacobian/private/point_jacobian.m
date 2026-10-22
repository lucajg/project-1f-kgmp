function J = point_jacobian(p,w,pk,R0frame)
% Full tool motion referenced to pk, expressed along frame's axes.
J = [cross(w,pk-p,1); w];
J = blkdiag(R0frame.',R0frame.') * J;
end
