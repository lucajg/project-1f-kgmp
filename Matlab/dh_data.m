% DH parameter table:
%   column 1: theta_i
%   column 2: d_i
%   column 3: a_i
%   column 4: alpha_i
dh_parameters = [
    0, 1, 0.8, -pi/3;
    0, 1.25, 2, pi/2;
    pi/2, 0.75, 0, pi/2;
    0, 2.5, 0, -pi/2;
    0, 0, 0, pi/2;
    0, 1.5, 0, 0;
    ];

n = size(dh_parameters,1);

Rzi = zeros(4,4,n);
Tzi = zeros(4,4,n);
Txi = zeros(4,4,n);
Rxi = zeros(4,4,n);
Ai = zeros(4,4,n);
Rz = @(theta) [
        cos(theta), -sin(theta), 0, 0;
        sin(theta), cos(theta), 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1;
    ];
Rx = @(alpha) [
        1, 0, 0, 0;
        0, cos(alpha), -sin(alpha), 0;
        0, sin(alpha), cos(alpha), 0;
        0, 0, 0, 1;
    ];
Tx = @(a) [
        1, 0, 0, a;
        0, 1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1;
    ];
Tz = @(d) [
        1, 0, 0, 0;
        0, 1, 0, 0;
        0, 0, 1, d;
        0, 0, 0, 1;
    ];


for i=1:n
    Rzi(:, :, i) = Rz(dh_parameters(i,1));
    Tzi(:, :, i) = Tz(dh_parameters(i,2));
    Txi(:, :, i) = Tx(dh_parameters(i,3));
    Rxi(:, :, i) = Rx(dh_parameters(i,4));
    Ai(:, :, i) = Rzi(:, :, i)*Tzi(:, :, i)*Txi(:, :, i)*Rxi(:, :, i);
end

tool_transform = [
    0, 1, 0, 0;
    0, 0, 1, 0;
    1, 0, 0, 0;
    0, 0, 0, 1;
];

T = eye(4);
for i=1:n
    T = T*Ai(:, :, i);
end

T
