phi = pi/6;
c = cos(alpha);
s = sin(alpha);

% Omega for each joint (rotation axis)
omega = zeros(3,6);
omega(:,1) = [ 0 ; 0 ; 1 ];
omega(:,2) = [ 0 ;  c ; s ];
omega(:,3) = [ 0 ; -s ; c ];
omega(:,4) = [ 1 ; 0 ; 0 ];
omega(:,5) = omega(:,3);
omega(:,6) = omega(:,4);
omega

% Compute r for each joint (vector from base origin to joint "origin")
h1 = 0.5;
h2 = 0.5;
l2 = 0.8;
l3 = 2;
w3 = 5/4;
l4 = 3/2;
h4 = 3/4;
l5 = 1;
l6 = 1.5;
r = zeros(3,6);
r(:,1) = [ 0 ; 0 ; h1 ];
r(:,2) = r(:,1) + [ l2 ; 0 ; h2 ];
r(:,3) = r(:,2) + l3 * [ 1 ; 0 ; 0 ] + w3 * [ 0 ;  c ; s ];
r(:,4) = r(:,3) + l4 * [ 1 ; 0 ; 0 ] + h4 * [ 0 ; -s ; c ];
r(:,5) = r(:,4) + [ l5 ; 0 ; 0 ];
r(:,6) = r(:,5) + [ l6 ; 0 ; 0 ];
r

% Compute v for each joint with cross product (v = - omega x r)
v = zeros (3,6);
for i=1:6
    v(:,i) = cross(r(:,i), omega(:,i));
end
v

% Assemble screw matrix S (column i is the screw of joint i)
S = [omega; v];
S

% Convert omegas into skew symmetric matrix
omega_skew = zeros(3,3,6);
for i=1:6
    w = omega(:,i);
    omega_skew(:,:,i) = [
         0,    -w(3),  w(2);
         w(3),  0,    -w(1);
        -w(2),  w(1),  0
    ];
end
omega_skew

% Build screws for product of exponentials
xi_hat = zeros(4,4,6);
for i=1:6
    xi_hat(1:3,1:3,i)=omega_skew(:,:,i);
    xi_hat(1:3,4,i)=v(:,i);
end
xi_hat

% Home pose
M = zeros(4,4);
M(1:3,1:3)=[
    1 , 0 ,  0 ;
    0 , c , -s ; 
    0 , s ,  c ;
    ];
M(1:3,4)=r(:,6);
M(4,4)=1;
M

W_T_E = @(q) expm(xi_hat(:,:,1)*q(1))*expm(xi_hat(:,:,2)*q(2))*expm(xi_hat(:,:,3)*q(3))*expm(xi_hat(:,:,4)*q(4))*expm(xi_hat(:,:,5)*q(5))*expm(xi_hat(:,:,6)*q(6))*M;

q0 = zeros(6,1);

W_T_E_0 = W_T_E(q0);

W_T_E_0