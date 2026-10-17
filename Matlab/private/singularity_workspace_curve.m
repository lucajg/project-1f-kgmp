function curve = singularity_workspace_curve(wrist,offset,azimuth)
% Intersect the arm singularity surface with the current tool-position ray.
% An arm-singularity circle of wrist centers has radius rw and height zw. For a fixed tool
% orientation, p_tool = p_wrist + offset. With p_tool,xy = rho*u:
%   |rho*u - offset_xy|^2 = rw^2,   z_tool = zw + offset_z.
% Both nonnegative roots belong to the slice; it is NOT just rw + |offset|.
u = [cos(azimuth);sin(azimuth)];
along = u.'*offset(1:2);
across = [-u(2),u(1)]*offset(1:2);
discriminant = wrist(1,:).^2-across^2;
root = sqrt(max(discriminant,0));
rho = [along+root;along-root];
z = repmat(wrist(2,:)+offset(3),2,1);
invalid = repmat(discriminant < 0,2,1) | rho < 0;
rho(invalid) = nan; z(invalid) = nan;
curve = [rho(1,:),nan,rho(2,:);z(1,:),nan,z(2,:)];
end
