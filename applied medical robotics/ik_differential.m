% Compute Differential Inverse Kinematics
%   @author         Mirroyal Ismayilov
%   @organisation   King's College London
%   @module         Applied Medical Robotics
%   @year           2024
function [new_th1,new_th2] = ik_differential(r,th1, th2, x_d)
%forward kinematics for given theta values

 
[x, y] = forward_kinematics(th1, th2, r)

%distance between desired and current position
del_x=[x_d(1)-x;x_d(2)-y];

%compute jacobian
J = ik_jacobian(r,th1,th2)

%compute inverse jacobian
inv_J= inv(J);


%compute change of joint angles
%delta_q= [(inv_j)*del_x(1); inv_j*del_x(2)];

delta_q = inv_J*del_x

%compute the next joint angles
new_th1= th1+ delta_q(1);
new_th2= th2+ delta_q(2);

end