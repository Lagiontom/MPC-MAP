function [estimated_pose] = estimate_pose(public_vars)


estimated_pose = nan(1,3);

if isfield(public_vars, 'mocap_pose') && numel(public_vars.mocap_pose) >= 3
    estimated_pose = public_vars.mocap_pose(1:3);
elseif isfield(public_vars, 'mu') && numel(public_vars.mu) >= 3
    estimated_pose = public_vars.mu(1:3);
end

end

