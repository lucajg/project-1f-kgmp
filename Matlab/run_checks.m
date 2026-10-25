function run_checks(include_symbolic,include_ui)
%RUN_CHECKS Run the CuRo6R checks after configuring the MATLAB path.
% run_checks runs numerical FK, Jacobian, IK, planning, and invisible app checks.
% run_checks(true) also runs exact Jacobian checks (Symbolic Math Toolbox).
% run_checks(false,false) runs numerical checks without creating figures.
if nargin < 1
    include_symbolic = false;
end
if nargin < 2
    include_ui = true;
end
validateattributes(include_symbolic,{'logical'},{'scalar'},mfilename,'include_symbolic');
validateattributes(include_ui,{'logical'},{'scalar'},mfilename,'include_ui');

curo6r_setup;
check_dh_poe;
check_jacobians(include_symbolic);
check_analytical_ik;
check_path_planning;
if include_ui
    check_singularity_map;
else
    fprintf('Singularity app checks skipped (include_ui = false).\n');
end
if ~include_symbolic
    fprintf('Symbolic Jacobian checks skipped (include_symbolic = false).\n');
end
fprintf('All requested CuRo6R checks passed.\n');
end
