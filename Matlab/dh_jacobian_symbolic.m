syms q1 q2 q3 q4 q5 q6 real
syms a1 a2 d1 d2 d3 d4 d6 real

q = [q1 q2 q3 q4 q5 q6];
piSym = sym('pi');

% Standard DH transform: Rz(theta)*Tz(d)*Tx(a)*Rx(alpha)
dhTransform = @(theta, d, a, alpha) [ ...
    cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0,           sin(alpha),             cos(alpha),            d;
    0,           0,                      0,                     1
];

% Transformations between consecutive frames
T01 = dhTransform(q1,           d1, a1, -piSym/3);
T12 = dhTransform(q2,           d2, a2,  piSym/2);
T23 = dhTransform(q3 + piSym/2, d3, 0,   piSym/2);
T34 = dhTransform(q4,           d4, 0,  -piSym/2);
T45 = dhTransform(q5,           0,  0,   piSym/2);
T56 = dhTransform(q6,           d6, 0,         0);

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;
T06 = T05*T56;

% Unit axes of joints 1–6, expressed in frame 0
u01 = sym([0; 0; 1]);
u02 = simplify(T01(1:3, 3));
u03 = simplify(T02(1:3, 3));
u04 = simplify(T03(1:3, 3));
u05 = simplify(T04(1:3, 3));
u06 = simplify(T05(1:3, 3));

% Position: first three entries of the last column
p00 = sym([0; 0; 0]);
p01 = simplify(T01(1:3, 4));
p02 = simplify(T02(1:3, 4));
p03 = simplify(T03(1:3, 4));
p04 = simplify(T04(1:3, 4));
p05 = simplify(T05(1:3, 4));
p06 = simplify(T06(1:3, 4));

dp05_dq1 = simplify(cross(u01, p05 - p00));
dp05_dq2 = simplify(cross(u02, p05 - p01));
dp05_dq3 = simplify(cross(u03, p05 - p02));
dp05_dq4 = simplify(cross(u04, p05 - p03));
dp05_dq5 = simplify(cross(u05, p05 - p04));
dp05_dq6 = simplify(cross(u06, p05 - p05));

J05 = [
    dp05_dq1, dp05_dq2, dp05_dq3, dp05_dq4, dp05_dq5, dp05_dq6;
         u01,      u02,      u03,      u04,      u05,      u06;
]

