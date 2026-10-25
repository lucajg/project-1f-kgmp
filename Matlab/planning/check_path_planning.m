function check_path_planning
% Check start admissibility, tolerance boundaries, and a regular IK transition.
poe = poe_data;
scale = diag([.5 .5 .5 1 1 1]);
q = [0.1024; -1.1124; 1.8547; -1.4399; -0.3835; -2.6486];

[path,goal,report] = plan(q);
[row_path,row_goal,row_report] = plan(q.');
assert(isequaln(path,row_path) && isequal(goal,row_goal) && isequal(report,row_report));
assert(norm(path.q(:,1)-q,inf)<1e-6 && norm(path.q(:,end)-goal,inf)<1e-6);
assert(all(path.sigma_min>1e-3) && all(path.joint_margin>=0));

outside = q;
outside([1 4]) = [pi+.01;-pi-.01];
err = expect_start_error(outside,'StartJointLimits');
assert(contains(err.message,'q1') && contains(err.message,'q4'));

% Limits are inclusive. A valid start need not have a successful arm segment.
for boundary = [-pi,pi]
    at_limit = q;
    at_limit(1) = boundary;
    try
        plan(at_limit);
    catch err
        assert(strcmp(err.message, ...
            'No candidate passed straight-arm-path validation. A different arm path may still exist.'), ...
            'A joint-limit boundary was incorrectly rejected: %s',err.message);
    end
end

for q5 = [0,1e-7,-1e-7,pi,-pi]
    wrist = q;
    wrist(5) = q5;
    expect_start_error(wrist,'StartWristSingularity');
end
expect_start_error(q,'StartWristSingularity','singular_tol',abs(sin(q(5))));

% Equality violates the same sigma threshold used for endpoints and samples.
sigma_start = min(svd(scale*poe_jacobian(q,0,6,poe)));
expect_start_error(q,'StartJacobianMargin','sigma_tol',sigma_start);
expect_start_error(q,'StartJacobianMargin','sigma_tol',2*sigma_start);

% A nonzero determinant alone does not establish numerical regularity.
q2_singular = fzero(@(q2) arm_determinant([.4;q2;0;.7;.6;-1.1],poe), ...
    [-pi/2,pi/2]);
near_arm = [.4;q2_singular+1e-7;0;.7;.6;-1.1];
assert(arm_determinant(near_arm,poe)~=0 && abs(sin(near_arm(5)))>1e-6);
assert(min(svd(scale*poe_jacobian(near_arm,0,6,poe)))<1e-3);
expect_start_error(near_arm,'StartJacobianMargin');

fprintf('Planner checks passed: start limits, wrist/Jacobian thresholds, near-singular arm, and regular path.\n');
end

function [path,goal,report] = plan(q,varargin)
[path,goal,report] = path_between_to_ik_solutions(q, ...
    'n_samples',101,'plot_results',false,'verbose',false,varargin{:});
end

function err = expect_start_error(q,suffix,varargin)
try
    plan(q,varargin{:});
catch err
    expected = "path_between_to_ik_solutions:" + suffix;
    assert(strcmp(err.identifier,expected),'Expected %s, got %s: %s', ...
        expected,err.identifier,err.message);
    return
end
error('check_path_planning:MissingError','Expected start rejection: %s.',suffix);
end

function value = arm_determinant(q,poe)
J = poe_jacobian(q,0,5,poe);
value = det(J(1:3,1:3));
end
