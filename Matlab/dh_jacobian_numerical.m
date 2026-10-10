function J = dh_jacobian_numerical(q, frame, joint, dh_model)
% Compatibility entry point; use dh_jacobian for either input type.
J = dh_jacobian(q,frame,joint,dh_model);
end
