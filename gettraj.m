function [x_set,y_set] = gettraj(vel,rad,tspan)
tvec=0:0.01:tspan;

omega = vel/rad;
theta = omega*tvec;
y_set = -rad*sind((theta-pi)*180/pi);
x_set = zeros(1,length(tvec));
% y_vel_set = -omega*cos(theta-pi);
% x_vel_set = zeros(1,100001);

for i = 1:1:length(tvec)
    if mod(fix(theta(i)/(2*pi)),2)==0
        x_set(i)=-rad-rad*cos(theta(i)-pi);
        %x_vel_set(i) = rad*sin(theta(i)-pi);
    else
        x_set(i)=rad+rad*cos(theta(i)-pi);
        %x_vel_set(i) = -rad*sin(theta(i)-pi);
    end
end
