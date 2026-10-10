function J = dh_jacobian(q, frame, joint, dh_model)
% Full tool Jacobian [v; omega] at DH origin 'joint', in DH frame 'frame'.
n = numel(dh_model.theta);
validateattributes(frame,{'numeric'},{'real','finite','scalar','integer','>=',0,'<=',n});
validateattributes(joint,{'numeric'},{'real','finite','scalar','integer','>=',1,'<=',n});
[~,p,w,T0] = dh_fk(q,dh_model);
J = point_jacobian(p,w,T0(1:3,4,joint+1),T0(1:3,1:3,frame+1));
end
