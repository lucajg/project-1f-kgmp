function [tracked,memory] = match_ik_solutions(Q,memory)
% Assign up to eight unordered IK solutions to persistent, distinct IDs.
% Compare all six joint angles on the torus, so +/-pi is not a jump.
% Absent solutions retain their last position for when they reappear.
% Branch identity is necessarily ambiguous where branches merge.
persistent assignments
if isempty(assignments), assignments = flipud(perms(1:8)); end
tracked = nan(6,8);
if isempty(Q), return; end
Q = sortrows(Q.',[1 2 3 5 4 6]).';  % Deterministic first numbering and ties.
n = size(Q,2);
assert(n <= 8,'Expected at most eight isolated IK solutions.');
if all(isnan(memory(:)))
    ids = 1:n;
else
    cost = zeros(n,8);
    for j = 1:n
        delta = atan2(sin(memory-Q(:,j)),cos(memory-Q(:,j)));
        cost(j,:) = sum(delta.^2,1);
    end
    cost(:,any(isnan(memory),1)) = 100;  % Prefer existing IDs to unused slots.
    % Only 8! assignments: a small exact matching, with no extra toolbox.
    total = zeros(size(assignments,1),1);
    for j = 1:n, total = total + cost(j,assignments(:,j)).'; end
    [~,best] = min(total);
    ids = assignments(best,1:n);
end
tracked(:,ids) = Q;
memory(:,ids) = Q;
end
