function kJ = ikik_jacobian(r,theta,phi)
N = 2;
J=zeros(2, N);
% Computes the Jacobian matrix for a 2-joint planar manipulator
% r1, r2: link lengths
% t1, t2: joint angles in degrees
% J: 2x2 Jacobian matrix

% Ensure angles are in degrees
%t1 = deg2rad(t1); % Convert to radians for internal computation
%t2 = deg2rad(t2);

% Compute the Jacobian matrix components
%J11 = -r1 * sin(t1) - r2 * sind(t1 + t2); % Partial derivative wrt t1
%J12 = -r2 * sin(t1 + t2);               % Partial derivative wrt t2
%J21 = r1 * cos(t1) + r2 * cosd(t1 + t2);  % Partial derivative wrt t1
%J22 = r2 * cos(t1 + t2);                % Partial derivative wrt t2

% Construct the Jacobian matrix
%J = [J11, J12; 
     %J21, J22];


  % Number of joints
kJ = zeros(2, 2);  % Initialize the Jacobian matrix


kJ= [-r*sind(theta+phi), -r *sind(theta+phi);
    r*cosd(theta) + r * cosd(theta+phi), r * cos(theta+phi)]

end