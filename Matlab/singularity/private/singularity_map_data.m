function map = singularity_map_data(poe,n)
% Cache the numerical determinant map and its arm singularity curves.
% det(J(q)) = -det(J_v_arm(q2,q3))*sin(q5); q1, q4 and q6 do not affect it.
persistent cached model samples
if ~isempty(cached) && isequal(poe,model) && isequal(n,samples)
    map = cached;
    return;
end
angles = linspace(-pi,pi,n);
[Q2,Q3] = meshgrid(angles,angles);
arm_det = zeros(size(Q2));
q = [0;0;0;0;pi/2;0];
for i = 1:numel(Q2)
    q(2:3) = [Q2(i);Q3(i)];
    J = poe_jacobian(q,0,5,poe);
    arm_det(i) = det(J(1:3,1:3));
end
% contourc interpolates actual determinants, not binary signs. A finite grid
% can miss zeros with no sign change. NaNs keep separate curves unconnected.
C = contourc(angles,angles,arm_det,[0 0]);
arm = zeros(2,0);
column = 1;
while column < size(C,2)
    count = C(2,column);
    arm = [arm,C(:,column+(1:count)),nan(2,1)]; %#ok<AGROW>
    column = column+count+1;
end
wrist = nan(size(arm));
for i = find(isfinite(arm(1,:)))
    q(2:3) = arm(:,i);
    [~,p] = poe_fk(q,poe);
    wrist(:,i) = [hypot(p(1,5),p(2,5));p(3,5)];
end
map = struct('angles',angles,'arm_det',arm_det,'arm',arm,'wrist',wrist);
cached = map; model = poe; samples = n;
end
