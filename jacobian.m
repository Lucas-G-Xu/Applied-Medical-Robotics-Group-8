 
x_d = [40, 147];

%distance between desired and current position
del_x=[x_d(1)-x;x_d(2)-y];

kJ = ikik_jacobian(78,60,30)
%compute inverse jacobian
inv_kJ= inv(kJ);


%compute change of joint angles
%delta_q= [(inv_j)*del_x(1); inv_j*del_x(2)];

delta_q = [(inv_kJ)*del_x(1); inv_kJ*del_x(2)];

%compute the next joint angles
new_theta= theta+ delta_q(1)
new_phi= phi+ delta_q(2)

