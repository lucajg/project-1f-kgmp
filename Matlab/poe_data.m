function poe_model = poe_data(g)
% Home screw axes in base_link; endpoint E is link_6.
if nargin == 0
    g = robot_dimensions();
end
pi_value = pi;
if any(structfun(@(v) isa(v,'sym'),g))
    pi_value = sym(pi);
end
c = cos(pi_value/6);
s = sin(pi_value/6);
omega = [
    0, 0,  0, 1,  0, 1;
    0, c, -s, 0, -s, 0;
    1, s,  c, 0,  c, 0
];

% Cumulative physical joint-origin offsets at home.
r = cumsum([0,    g.l2, g.l3,    g.l4,   g.l5, g.l6;
            0,    0,    g.w3*c, -g.h4*s, 0,    0;
            g.h1, g.h2, g.w3*s,  g.h4*c, 0,    0],2);
v = cross(r,omega,1);
xi_hat = zeros(4,4,6);
if isa(r,'sym')
    xi_hat = sym(xi_hat);
end
for i = 1:6
    w = omega(:,i);
    w_skew = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
    xi_hat(:,:,i) = [w_skew, v(:,i); 0, 0, 0, 0];
end
M = [1, 0, 0, r(1,6); 0, c, -s, r(2,6); 0, s, c, r(3,6); 0, 0, 0, 1];
dh_model = dh_data(g);
poe_model = struct('omega',omega, 'r',r, 'S',[omega;v], ...
    'xi_hat',xi_hat, 'M',M, 'R_home',dh_model.R_home);
end
