dh_data;
q = zeros(5,1);
n=5;
dh_model_wrist_center = dh_model;
dh_model_wrist_center.n = n;
dh_model_wrist_center.theta = dh_model_wrist_center.theta(1:n);
dh_model_wrist_center.d = dh_model_wrist_center.d(1:n);
dh_model_wrist_center.a = dh_model_wrist_center.a(1:n);
dh_model_wrist_center.alpha = dh_model_wrist_center.alpha(1:n);
dh_model_wrist_center
[T, p, w] = dh_fk(q, dh_model_wrist_center);
T = T / dh_model_wrist_center.tool_transform;
T
p
