% Compute Jacobian
%   @author         Mirroyal Ismayilov
%   @organisation   King's College London
%   @module         Applied Medical Robotics
%   @year           2024
function J = ik_jacobian(r,th1,th2)
N = 2;  % Number of joints
J = zeros(2, 2);  % Initialize the Jacobian matrix




J = [-r*sind(th1), -r*sind(th2);
    r*cosd(th1), r*cosd(th2)];
end

