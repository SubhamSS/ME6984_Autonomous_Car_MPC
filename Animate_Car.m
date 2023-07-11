function Animate_Car(qqData,Time,xref,yref)

%Video Start lines
v = VideoWriter('kinematic_car.mp4','MPEG-4');
v.FrameRate = 10;
open(v);

%Loop over time interval, determine position for all points are plot

for time = 501:500:length(Time)
    clf;
    plot(qqData(1,time-500:time),qqData(2,time-500:time),'LineWidth',3,'Color','b','MarkerSize',50)
    hold on;
    plot(xref,yref,'LineWidth',0.1,'Color','k','LineStyle','--');
    axis equal;
    xlim([-220 220]);
    ylim([-110 110]);

    xlabel 'X', ylabel 'Y'; 
    title(sprintf('Trajectory of COM at time = %6.3f seconds',Time(time)/100));
    frame = getframe(gcf);
    writeVideo(v,frame);
    drawnow;
    
end

%Closes Video 
close(v);   
end