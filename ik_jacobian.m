
function J = ik_jacobian(r,th1,th2)
N = 2;  % Number of joints
J = zeros(2, 2);  % Initialize the Jacobian matrix


J = [-r*sind(th1), -r*sind(th2);
    r*cosd(th1), r*cosd(th2)];
end

