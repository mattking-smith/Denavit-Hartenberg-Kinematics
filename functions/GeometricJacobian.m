function J = GeometricJacobian(T_B_n)
% Compute Geometric Jacobian for every link from the homogeneous transformation matrix
dof = size(T_B_n,3);
J = zeros(6,dof,dof);
for ii = 1:1:dof
    p_ii = T_B_n(1:3,4,ii);
    for jj = 1:1:ii
        z_jj = T_B_n(1:3,3,jj);
        p_jj = T_B_n(1:3,4,jj);
        J(:,jj,ii) = [cross(z_jj, p_ii-p_jj);z_jj];
    end
end
J = J(:,1:dof-1,:);
end

