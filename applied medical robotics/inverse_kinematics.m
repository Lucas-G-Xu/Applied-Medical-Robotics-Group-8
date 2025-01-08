function [th1, th2] = inverse_kinematics(x, y, r)
N = atand(y/x);
hyp = y/sind(N);
R=2*r;
rob = hyp./R;
gamma = acosd(rob);
th1 = N + gamma;
th2= th1 - (2*gamma);
end