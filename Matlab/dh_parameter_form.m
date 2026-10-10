function expr = dh_parameter_form(expr)
% Rewrite CuRo6R dimension symbols using the nonzero DH parameters.
g = robot_dimensions(true);
syms a1 a2 d1 d2 d3 d4 d6 real
expr = simplify(subs(expr, ...
    [g.l2,g.l3,g.w3,g.h4,g.l6,g.h2,g.l5], ...
    [a1,a2,d2,d3,d6,d1-g.h1,d4-g.l4]));
end
