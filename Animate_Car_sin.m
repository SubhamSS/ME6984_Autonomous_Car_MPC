function Animate_Car_sin(qqData,Time,xref,yref)

%Video Start lines
v = VideoWriter('kinematic_car_sin.mp4','MPEG-4');
v.FrameRate = 10;
open(v);

%Loop over time interval, determine position for all points are plot

for time = 51:50:length(Time)
    clf;
    plot(qqData(1,time-50:time),qqData(2,time-50:time),'LineWidth',3,'Color','b','MarkerSize',50)
    hold on;
    plot(xref,yref,'LineWidth',0.1,'Color','k','LineStyle','--');
    axis equal;
    

    xlabel 'X', ylabel 'Y'; 
    title(sprintf('Trajectory of COM at time = %6.3f seconds',Time(time)));
    frame = getframe(gcf);
    writeVideo(v,frame);
    drawnow;
    
end

%Closes Video 
close(v);   
end