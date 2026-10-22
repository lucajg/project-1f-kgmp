% CuRo6R joint data from CuRo6R/CuRo6R.urdf, in metres and radians.
% Run urdf_data, then call urdf_fk(q, urdf_model).
% Chain: base_link -> link_1 -> link_2 -> link_3_d3 -> link_4 -> link_5 -> link_6.
% tool0 branches from link_3_d3 and is not part of this endpoint chain.

% Joint-origin positions and orientations relative to their parent links.
% Each column describes one joint at q_i = 0.
%         Joint:  1     2     3     4     5     6
xyz = [          0,  0.8,    2,  1.5,    1,  1.5;
                 0,    0, 1.25,    0,    0,    0;
               0.5,  0.5,    0, 0.75,    0,    0];

% Interpret the URDF's 0.52359877559 as the intended exact 30-degree tilt.
rpy = [          0, pi/6,    0,    0,    0,    0;
                 0,    0,    0,    0,    0,    0;
                 0,    0,    0,    0,    0,    0];

% Unit rotation axes expressed in the local joint frames.
axis = [         0,    0,    0,    1,    0,    1;
                 0,    1,    0,    0,    0,    0;
                 1,    0,    1,    0,    1,    0];

urdf_model = struct('n', size(xyz,2), 'xyz', xyz, 'rpy', rpy, 'axis', axis);
