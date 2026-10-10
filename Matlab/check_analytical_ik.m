function check_analytical_ik
% Round-trip both IK solvers against DH and URDF forward kinematics.
stream = RandStream('mt19937ar','Seed',17);
g = robot_dimensions();
g_other = g;
g_other.l2 = 0.35; g_other.l3 = 1.7; g_other.w3 = 0.6;
g_other.l4 = 1.1; g_other.l5 = 0.8; g_other.h4 = 0.4;
g_other.h1 = 0.3; g_other.h2 = 0.9; g_other.l6 = 0.7;
count = 0;
max_error = zeros(1,3);

for geometry = {g, g_other}
    pm = poe_data(geometry{1});
    dm = dh_data(geometry{1});
    samples = [zeros(6,1), -pi+2*pi*rand(stream,6,400)];
    % Exercise the omitted half-angle point and both exact/near wrist singularities.
    for q1 = [0.4, pi, pi-1e-9]
        for q5 = [0, pi, -pi, 1e-6, -1e-6, 1e-8, -1e-8, ...
                1e-9, -1e-9, 1e-11, pi-1e-6, pi-1e-8, pi-1e-9]
            samples(:,end+1) = [q1; -0.4; 0.8; 0.7; q5; -1.1]; %#ok<AGROW>
        end
    end
    for k = 1:size(samples,2)
        source = samples(:,k);
        target = dh_fk(source,dm);
        errors = check_target(target,pm,dm,source);
        max_error = max(max_error,errors);
        count = count+1;
    end

    % The same wrist-center target can have an arbitrary q1 on the base axis.
    a1 = dm.a(1); a2 = dm.a(2);
    d2 = dm.d(2); d3 = dm.d(3); d4 = dm.d(4);
    K = d2^2-a2^2-d3^2-d4^2;
    S = [0.5,-d2]; C = [1,-d2,a1^2+K];
    heights = roots(conv(C,C)+4*a2^2*[0,0,conv(S,S)] ...
        -[0,0,0,0,4*a2^2*d4^2]);
    heights = real(heights(abs(imag(heights))<1e-10));
    assert(~isempty(heights),'Expected reachable points on the base axis.');
    for h = heights.'
        target = eye(4);
        target(1:3,4) = [dm.d(6); 0; dm.d(1)+h];
        arm = analytical_ik_3r(target,pm,dm);
        assert(size(arm,2)==1,'Expected one representative of the arm family.');
        max_error = max(max_error,check_target(target,pm,dm,[]));
        count = count+1;
    end

    target = eye(4); target(1:3,4) = [100;100;100];
    assert(isequal(size(analytical_ik_3r(target,pm,dm)),[3,0]));
    assert(isequal(size(analytical_ik_6r(target,pm,dm)),[6,0]));
end

urdf_data;
pm = poe_data(); dm = dh_data();
for k = 1:30
    source = -pi+2*pi*rand(stream,6,1);
    target = urdf_fk(source,urdf_model);
    max_error = max(max_error,check_target(target,pm,dm,source));
    count = count+1;
end

bad = eye(4); bad(1,1) = 2;
for solver = {@analytical_ik_3r, @analytical_ik_6r}
    try
        solver{1}(bad,pm,dm);
        error('check_analytical_ik:MissingError','Accepted a non-rigid target.');
    catch err
        assert(strcmp(err.identifier,'analytical_ik_3r:InvalidPose'));
    end
end
fprintf('Analytical IK passed %d reachable poses, unreachable and invalid targets.\n',count);
fprintf('Max wrist position: %.3e m; tool position: %.3e m; rotation: %.3e (Frobenius).\n',max_error);
end

function errors = check_target(target,pm,dm,source)
arm = analytical_ik_3r(target,pm,dm);
Q = analytical_ik_6r(target,pm,dm);
assert(~isempty(arm) && ~isempty(Q),'No solution for a reachable target.');
assert(size(arm,1)==3 && size(Q,1)==6);
assert(isreal(Q) && all(isfinite(Q(:))) && all(abs(Q(:))<=pi+1e-12));
errors = zeros(1,3);
p_w = target(1:3,4)-dm.d(6)*target(1:3,1);
for i = 1:size(arm,2)
    [~,p] = dh_fk([arm(:,i);zeros(3,1)],dm);
    errors(1) = max(errors(1),norm(p(:,5)-p_w));
end
for i = 1:size(Q,2)
    T = dh_fk(Q(:,i),dm);
    errors(2) = max(errors(2),norm(T(1:3,4)-target(1:3,4)));
    errors(3) = max(errors(3),norm(T(1:3,1:3)-target(1:3,1:3),'fro'));
    difference = atan2(sin(Q(:,1:i-1)-Q(:,i)),cos(Q(:,1:i-1)-Q(:,i)));
    assert(all(vecnorm(difference,2,1)>1e-7),'Duplicate full-pose solution.');
end
assert(all(errors<1e-7),'An IK candidate failed independent forward kinematics.');

if isempty(source), return; end
% Pose agreement alone does not detect a missing original arm or wrist branch.
difference = atan2(sin(Q(1:3,:)-source(1:3)),cos(Q(1:3,:)-source(1:3)));
same_arm = vecnorm(difference,2,1)<1e-5;
assert(any(same_arm),'The original arm branch is missing (q5 = %.3e).',source(5));
if abs(sin(source(5)))>1e-6
    flipped = source;
    flipped(4:6) = [source(4)+pi; -source(5); source(6)+pi];
    for expected = [source,flipped]
        difference = atan2(sin(Q-expected),cos(Q-expected));
        assert(any(vecnorm(difference,2,1)<1e-5),'A regular wrist branch is missing.');
    end
elseif abs(sin(source(5)))<1e-14
    assert(any(abs(Q(4,same_arm))<1e-10),'A singular wrist should use q4 = 0.');
end
end
