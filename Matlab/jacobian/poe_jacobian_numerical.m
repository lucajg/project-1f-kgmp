function J = poe_jacobian_numerical(q, frame, joint, poe_model)
% Compatibility entry point; use poe_jacobian for either input type.
J = poe_jacobian(q,frame,joint,poe_model);
end
