g_sym = robot_dimensions(true);
q_sym = sym('q',[6,1],'real');
k = 6;  % Velocity reference point.
f = 0;  % DH frame used to express base-relative velocities.
dh_model_sym = dh_data(g_sym);
Jdh_sym = simplify(dh_jacobian(q_sym,f,k,dh_model_sym));
Jdh_ad_sym = dh_parameter_form(Jdh_sym);
