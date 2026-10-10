g_sym = robot_dimensions(true);
q_sym = sym('q',[6,1],'real');
k = 6;  % Velocity reference point.
f = 0;  % DH frame used to express base-relative velocities.
poe_model_sym = poe_data(g_sym);
Jpoe_sym = simplify(poe_jacobian(q_sym,f,k,poe_model_sym));
Jpoe_ad_sym = dh_parameter_form(Jpoe_sym);
