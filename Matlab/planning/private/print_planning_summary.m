function print_planning_summary(selected_path,report)
% Print a compact summary; full candidate and attempt details stay in report.
attempted = nnz(report.Outcome ~= "endpoint_rejected");
passed = nnz(report.Outcome == "passed");
if isempty(selected_path)
    fprintf('No straight-arm path passed: %d attempted, %d endpoints screened out.\n', ...
        attempted,height(report)-attempted);
    if isempty(report)
        fprintf('  The IK solver returned no candidates.\n');
    else
        rows = find(report.Outcome == "path_rejected");
        if isempty(rows)
            rows = (1:height(report)).';
        end
        for i = rows.'
            fprintf('  IK %d: %s\n',report.IK_index(i),report.Status(i));
        end
    end
    fprintf('A different arm path may still exist.\n');
    return
end

goal_index = report.IK_index(report.Selected);
[sigma_worst,k_worst] = min(selected_path.sigma_min);
position_closure = norm(selected_path.p(:,end)-selected_path.p(:,1));
joint_change = norm(selected_path.q(:,end)-selected_path.q(:,1));
fprintf('Selected IK %d: %d of %d attempted paths passed (%d IK candidates).\n', ...
    goal_index,passed,attempted,height(report));
fprintf('  Margins: sigma_min %.3g at s=%.4f; joint clearance %.3g rad.\n', ...
    sigma_worst,selected_path.s(k_worst),min(selected_path.joint_margin));
fprintf('  Endpoint: position closure %.3g m; goal error %.3g rad; joint change %.3g rad.\n', ...
    position_closure,selected_path.goal_error,joint_change);
fprintf('  Pose residuals: max orientation %.3g rad; max position consistency %.3g m.\n', ...
    max(selected_path.theta),max(selected_path.error_position));
end
