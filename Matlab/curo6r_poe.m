function [T, S, M, g] = curo6r_poe(q, doPlot)
%CURO6R_POE Space-form forward kinematics from base_link to link_6.
% q: six joint angles in radians, measured from the URDF joint zeros.
% S: 6-by-6 screw matrix, one column per joint, ordered [omega; v].
% M: home pose of link_6 in base_link. T: pose at q.
% g: home geometry, hat matrices, exponentials, and moving joint axes.
%
% Examples (no Robotics Toolbox required):
%   [T0, S, M, g] = curo6r_poe(zeros(6,1));
%   q = [0; pi/6; 0; 0; 0; 0];
%   [T, S, M, g] = curo6r_poe(q, true);
%   g.hat(:,:,2)          % twist matrix for joint 2
%   g.exponential(:,:,2)  % expm(g.hat(:,:,2)*q(2))
%
% Source: supplied CuRo6R.urdf. Distances are metres.
% The supplied tool0 is a child of link_3_d3, so this function deliberately
% targets link_6. For a TCP fixed to link_6, postmultiply T by its fixed pose.

if nargin < 1, q = zeros(6,1); end
if nargin < 2, doPlot = false; end
assert(isnumeric(q) && isreal(q) && numel(q)==6 && all(isfinite(q(:))), ...
    'q must contain six finite real angles in radians.');
q = q(:);

% Match the literal URDF roll. Using pi/6 instead recovers the radical
% expressions in the handwritten derivation (difference about 1e-11).
alpha = 0.52359877559;
c = cos(alpha); s = sin(alpha);
Rx = [1 0 0; 0 c -s; 0 s c];

% Each column is a home joint axis direction, expressed in base_link.
W = [0 0  0 1  0 1;
     0 c -s 0 -s 0;
     1 s  c 0  c 0];

% Each column is a point on that axis: the URDF joint origin at home.
yWrist = 1.25*c - 0.75*s;
zWrist = 1 + 1.25*s + 0.75*c;
P = [0   0.8 2.8        4.3     5.3     6.8;
     0   0   1.25*c     yWrist  yWrist  yWrist;
     0.5 1   1+1.25*s   zWrist  zWrist  zWrist];

% Compute v from the geometry instead of entering it independently.
V = -cross(W, P, 1);
S = [W; V];
M = [Rx, P(:,6); 0 0 0 1];

g.omega = W; g.r = P; g.v = V;
g.hat = zeros(4,4,6);
g.exponential = zeros(4,4,6);
g.movingOrigin = zeros(3,6);
g.movingAxis = zeros(3,6);
F = eye(4);
for i = 1:6
    w = W(:,i);
    skewW = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
    g.hat(:,:,i) = [skewW V(:,i); 0 0 0 0];
    g.exponential(:,:,i) = expm(g.hat(:,:,i)*q(i));
    % Earlier joints carry this joint's home axis into its current pose.
    g.movingOrigin(:,i) = F(1:3,1:3)*P(:,i) + F(1:3,4);
    g.movingAxis(:,i) = F(1:3,1:3)*W(:,i);
    F = F*g.exponential(:,:,i);
end
T = F*M;

if doPlot
    figure('Name','CuRo6R Product of Exponentials');
    home = [zeros(3,1), P];
    moving = [zeros(3,1), g.movingOrigin];
    h0 = plot3(home(1,:),home(2,:),home(3,:),'--o', ...
        'Color',[0.65 0.65 0.65]); hold on;
    h1 = plot3(moving(1,:),moving(2,:),moving(3,:),'-o', ...
        'LineWidth',2);
    for i = 1:6
        p = g.movingOrigin(:,i); w = g.movingAxis(:,i);
        quiver3(p(1),p(2),p(3),w(1),w(2),w(3),0.65, ...
            'Color',[0.5 0.2 0.65],'HandleVisibility','off');
        text(p(1),p(2),p(3),sprintf('  J%d',i));
    end
    % Draw orientation too: rotating joint 6 leaves this endpoint's
    % position fixed, while its orientation changes.
    colors = [0.8 0.2 0.2; 0.2 0.6 0.2; 0.2 0.35 0.85];
    p = T(1:3,4);
    for k = 1:3
        d = 0.7*T(1:3,k);
        quiver3(p(1),p(2),p(3),d(1),d(2),d(3),0, ...
            'Color',colors(k,:),'LineWidth',1.5,'HandleVisibility','off');
    end
    axis equal; grid on; view(35,25); rotate3d on;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    legend([h0 h1],{'Home configuration','Current configuration'}, ...
        'Location','best');
    title('CuRo6R: base_link to link_6 (joint-origin skeleton)');
end
end
