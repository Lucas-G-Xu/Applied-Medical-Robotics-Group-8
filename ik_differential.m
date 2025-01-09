
function [new_th1,new_th2] = ik_differential(r,th1, th2, xd, yd)

[x, y] = forward_kinematics(th1, th2, r);
del_x=[xd-x;yd-y];
J = ik_jacobian(r,th1,th2);
inv_J= inv(J);
delta_q = inv_J*del_x
new_th1= th1+ delta_q(1)
new_th2= th2+ delta_q(2)

end




