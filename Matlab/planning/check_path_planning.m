function check_path_planning
% Check start admissibility, regular paths, failure reports, and console output.
poe = poe_data;
scale = diag([.5 .5 .5 1 1 1]);
q = [0.1024; -1.1124; 1.8547; -1.4399; -0.3835; -2.6486];

[path,goal,report] = plan(q);
[row_path,row_goal,row_report] = plan(q.');
assert(isequaln(path,row_path) && isequal(goal,row_goal) && isequaln(report,row_report));
assert(norm(path.q(:,1)-q,inf)<1e-6 && norm(path.q(:,end)-goal,inf)<1e-6);
assert(all(path.sigma_min>1e-3) && all(path.joint_margin>=0));
check_report(path,goal,report);

% Rejected endpoints and failed path attempts must remain inspectable in quiet mode.
q_no_goal = [.4;-.4;.8;.7;.6;-1.1]; %#ok<NASGU> Referenced by evalc below.
quiet = evalc('[none_path,none_goal,none_report] = plan(q_no_goal);');
assert(isempty(quiet) && isempty(none_path) && isempty(none_goal));
assert(all(none_report.Outcome == "endpoint_rejected"));
check_report(none_path,none_goal,none_report);

q_failed = [-.78829;2.8319;1.4577;.61989;-2.1613;-2.1615]; %#ok<NASGU> Referenced by evalc.
quiet = evalc('[failed_path,failed_goal,failed_report] = plan(q_failed);');
assert(isempty(quiet) && isempty(failed_path) && isempty(failed_goal));
failed_row = find(failed_report.Outcome == "path_rejected",1);
assert(~isempty(failed_row),'Expected a rejected path attempt.');
assert(~isempty(failed_report.Attempt{failed_row}.path.s),'Expected a retained valid prefix.');
check_report(failed_path,failed_goal,failed_report);

success_text = evalc(['[printed_path,printed_goal,printed_report] = ' ...
    'path_between_to_ik_solutions(q,n_samples=101,plot_results=false,verbose=true);']);
assert(contains(success_text,'Selected IK ') && contains(success_text,'Margins:'));
assert(~contains(success_text,'position_closure ='));
assert(isequaln(path,printed_path) && isequal(goal,printed_goal) && isequaln(report,printed_report));
failure_text = evalc(['[printed_path,printed_goal,printed_report] = ' ...
    'path_between_to_ik_solutions(q_failed,n_samples=101,plot_results=false,verbose=true);']);
assert(contains(failure_text,'No straight-arm path passed:'));
assert(contains(failure_text,failed_report.Status(failed_row)));
assert(isempty(printed_path) && isempty(printed_goal) && isequaln(failed_report,printed_report));
empty_report = report([],:); %#ok<NASGU> Referenced by evalc below.
empty_text = evalc('print_planning_summary([],empty_report);');
assert(contains(empty_text,'IK solver returned no candidates'));

outside = q;
outside([1 4]) = [pi+.01;-pi-.01];
err = expect_start_error(outside,'StartJointLimits');
assert(contains(err.message,'q1') && contains(err.message,'q4'));

% Limits are inclusive. A valid start need not have a successful arm segment.
for boundary = [-pi,pi]
    at_limit = q;
    at_limit(1) = boundary;
    [boundary_path,boundary_goal,boundary_report] = plan(at_limit);
    check_report(boundary_path,boundary_goal,boundary_report);
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

fprintf('Planner checks passed: start validation, paths, structured failure reports, and verbosity.\n');
end

function check_report(path,goal,report)
assert(isequal(size(report.Goal),[height(report),6]));
assert(all(ismember(report.Outcome,["endpoint_rejected","path_rejected","passed"])));
for i = 1:height(report)
    codes = report.ReasonCodes{i};
    assert(isstring(codes));
    if report.Outcome(i) == "endpoint_rejected"
        assert(~isempty(codes) && isempty(report.Attempt{i}));
        continue
    end
    attempt = report.Attempt{i};
    assert(isequal(codes,attempt.reason_codes));
    assert(size(attempt.path.q,2)==numel(attempt.path.s));
    assert(size(attempt.path.p,2)==numel(attempt.path.s));
    if report.Outcome(i) == "passed"
        assert(isempty(codes) && strlength(attempt.reason)==0 && isnan(attempt.s_failed));
    else
        assert(~isempty(codes) && strlength(attempt.reason)>0 && isfinite(attempt.s_failed));
        assert(all(attempt.path.s<=attempt.s_failed));
        if any(codes == "joint_limits")
            assert(~isempty(attempt.violated_joints));
            assert(all(abs(attempt.rejected_q(attempt.violated_joints))>pi));
        end
        if any(codes == "arm_sign" | codes == "jacobian_margin")
            assert(isfinite(attempt.rejected_sigma) && isfinite(attempt.rejected_arm_det));
        end
    end
end
if isempty(path)
    assert(isempty(goal) && ~any(report.Selected) && ~any(report.Outcome == "passed"));
else
    selected = find(report.Selected);
    assert(isscalar(selected) && report.Outcome(selected) == "passed");
    assert(isequaln(path,report.Attempt{selected}.path));
    assert(isequal(goal,report.Goal(selected,:).'));
end
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
