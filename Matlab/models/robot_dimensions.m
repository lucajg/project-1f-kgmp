function g = robot_dimensions(use_symbolic)
% Shared CuRo6R lengths in metres, or real symbols when requested.
if nargin == 0
    use_symbolic = false;
end

g = struct('h1',1/2, 'h2',1/2, 'l2',4/5, 'l3',2, ...
           'w3',5/4, 'l4',3/2, 'h4',3/4, 'l5',1, 'l6',3/2);

if use_symbolic
    for field = fieldnames(g).'
        name = field{1};
        g.(name) = sym(name,'real');
    end
end
end
