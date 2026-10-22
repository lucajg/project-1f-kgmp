q_start = pi/2*[1;-1;1;-1;1;-1];
poe_model = poe_data;
[T_d, p, w, F0] = poe_fk(q_start, poe_model);
R_d = T_d(1:3,1:3);
dh_model = dh_data;
Q_ik = analytical_ik_6r(T_d, poe_model, dh_model);

q_goal = Q_ik(:,7);

q_a = @(s) (1-s)*q_start(1:3) + s*q_goal(1:3);
n_samples = 10001;
s_samples = cumsum(1/n_samples*ones(1,n_samples));
s_max=1;
s_min=0;
singular_tol=1e-6;
q_planned=zeros(6,n_samples);
for i=1:n_samples
    s_k = (i-1)*(s_max-s_min)/n_samples;
    current_q_a = q_a(s_k);
    [wrist, is_singular] = analytical_ik_wrist(current_q_a, R_d, poe_model, singular_tol);
    if (not(is_singular))
        q_planned(1:3,i) = current_q_a;
        if (wrist(3,1)>0)
            q_planned(4:6,i) = wrist(:,1);
        else
            q_planned(4:6,i) = wrist(:,2);
        end
    else
        print("Wrist crossed singularity: failure!")
        q_planned(:,i) = [-99;-99;-99];
    end
end

