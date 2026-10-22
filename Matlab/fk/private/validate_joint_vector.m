function validate_joint_vector(q,n)
validateattributes(q,{'numeric','sym'},{'vector','numel',n});
if isnumeric(q)
    validateattributes(q,{'numeric'},{'real','finite'});
end
end
