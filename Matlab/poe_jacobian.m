function J = poe_jacobian(q, frame, joint, poe_model)
% Full tool Jacobian [v; omega] at physical origin 'joint', in DH frame 'frame'.
n = size(poe_model.xi_hat,3);
validateattributes(frame,{'numeric'},{'real','finite','scalar','integer','>=',0,'<=',n});
validateattributes(joint,{'numeric'},{'real','finite','scalar','integer','>=',1,'<=',n});
[~,p,w,F0] = poe_fk(q,poe_model);
R0frame = F0(1:3,1:3,frame+1) * poe_model.R_home(:,:,frame+1);
J = point_jacobian(p,w,p(:,joint),R0frame);
end
