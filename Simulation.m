%% Simulation parameters
clear all;
close all;
clc;


% create the video writer with 1 fps
writerObj = VideoWriter('my_video1.avi');
writerObj.FrameRate = 30;
% set the seconds per image

% open the video writer
open(writerObj);

N=10000; % Number of frames

dt = 0.01;
%% Ownship initialisation
initialLocation = [0,0,0];
V_u = 40;
WP_0 = [0,0,500];
WP_1 = [600,800,500];
initialHeading = atan((WP_1(2)-WP_0(2)) / (WP_1(1)-WP_0(1)));
flightCourseAngle = 0;
turnRadius = 40;

ownship = UAV(initialLocation, V_u, initialHeading, WP_0, WP_1, flightCourseAngle, turnRadius);

%% Intruder initilisation
initLocation = [330, 400, 0];
avoidanceRadius = 100;
V_a = 0;
ObsHeading = 0;
ObsFlightCOurseAngle = 0;

intruder = obstacle(initLocation, avoidanceRadius, V_a, ObsHeading, ObsFlightCOurseAngle); 

%% Run Simulation 
while abs(WP_1(1) - ownship.x) > 5 && abs(WP_1(2) - ownship.y) > 5
    
    stateMachine(ownship.TC, ownship.OC, ownship.SS, ownship, intruder, dt);
    
end

ownship.pathTrim();

for i = 1:length(ownship.path)
    ownship.dynamicPlot2D();
    intruder.staticPlot2D();
    M(i)=getframe(gcf); % leaving gcf out crops the frame in the movie.
    writeVideo(writerObj,M(i));
end

close(writerObj);

function stateMachine(TC, OC, SS, own, Obstacle, dt)
    
    pause(0.01);
    
    if TC && OC && ~SS
        own.turn(dt, own.dir, Obstacle);
    elseif TC && ~OC && SS
         own.turn(dt, -own.dir, Obstacle);
    elseif TC && OC && SS
         own.turn(dt, own.dir, Obstacle);
    else
        own.moveForward(dt, Obstacle);
    end
    
end