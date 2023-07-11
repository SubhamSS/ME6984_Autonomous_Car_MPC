function [x_set,y_set] = gentraj_sin(vel,rad,tspan)
tvec=0:1:tspan+30;

omega = vel/rad;
theta = omega*tvec;
y_set = sind((theta-pi)*180/pi);
x_set = theta;

