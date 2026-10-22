function dh_model = dh_data(g)
% Standard DH geometry: joint i rotates about z_(i-1).
if nargin == 0
    g = robot_dimensions();
end
pi_value = pi;
if any(structfun(@(v) isa(v,'sym'),g))
    pi_value = sym(pi);
end

theta = [0, 0, pi_value/2, 0, 0, 0];
d = [g.h1+g.h2, g.w3, g.h4, g.l4+g.l5, 0, g.l6];
a = [g.l2, g.l3, 0, 0, 0, 0];
alpha = [-pi_value/3, pi_value/2, pi_value/2, -pi_value/2, pi_value/2, 0];
n = numel(theta);

% DH frame 6 and link_6 have the same origin but different axes.
tool_transform = [0, 1, 0, 0; 0, 0, 1, 0; 1, 0, 0, 0; 0, 0, 0, 1];
R_home = zeros(3,3,n+1);
if isa(theta,'sym')
    R_home = sym(R_home);
end
R_home(:,:,1) = eye(3);
for i = 1:n
    A = dh_transform(theta(i),0,0,alpha(i));
    R_home(:,:,i+1) = R_home(:,:,i) * A(1:3,1:3);
end
dh_model = struct('n',n, 'theta',theta, 'd',d, 'a',a, ...
    'alpha',alpha, 'tool_transform',tool_transform, 'R_home',R_home);
end
