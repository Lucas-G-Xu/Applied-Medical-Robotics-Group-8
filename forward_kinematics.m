


function[x,y] = forward_kinematics(th1, th2, r)
gamma = (th1-th2)/2;
N = th1-gamma ;
hyp = 2*r*cosd(gamma);
x= cosd(N)*hyp
y= sind(N)*hyp