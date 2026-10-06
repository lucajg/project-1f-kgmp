function fig = curo6r_compare(poe_model,dh_model)
%CURO6R_COMPARE Open PoE and DH plots with a shared set of joint angles.
% curo6r_compare builds the default models; or pass both existing model structs.

if nargin == 0
    poe_data;
    dh_data;
else
    narginchk(2,2);
end
fig = curo6r_viewer(poe_model,dh_model);
end
