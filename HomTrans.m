function T_nm1_n = HomTrans(d,a,alpha,theta,param)
%{ 
Construct homogeneous transformation matrix based of convention
Parameters:
@a: length of the common normal (m)
@d: offset along previous z to common normal (m)
@alpha: angle about common normal, from old z axis to new z axis (radians)
@theta: angle about previous z, from old x to new x (radians)
@param: parameterization of kinematics
@T_nm1_n: Homogeneous transformation from concurrent coordinate system n,
to the previous coordinate system n-1 (nm1)
%}
if strcmp(param, 'DH')
    T_nm1_n = ...
    [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta);
     sin(theta) cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)  a*sin(theta);
     0          sin(alpha)              cos(alpha)              d;
     0          0                       0                       1];
elseif strcmp(param, 'modDH')
    T_nm1_n = ...
    [cos(theta)             -sin(theta)             0           a;
     sin(theta)*cos(alpha)  cos(theta)*cos(alpha)   -sin(alpha) -d*sin(alpha);
     sin(theta)*sin(alpha)  cos(theta)*sin(alpha)   cos(alpha)  d*cos(alpha);
     0                      0                       0           1];
else
    error('Input incorrect parameterization (neither DH or modDH)');
end


end

