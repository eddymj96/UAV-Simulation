classdef obstacle < handle
    properties
        avoidanceRadius
        x
        y
        z
        v
        heading
        flightCourseAngle
        step
        path
        
    end
    
    methods 
        
        function obj = obstacle(i, avoidanceRadius, v, heading, flightCourseAngle)
            obj.x = i(1);
            obj.y = i(2);
            obj.z = i(3);
            obj.avoidanceRadius = avoidanceRadius;
            obj.v = v;
            obj.heading = heading;
            obj.flightCourseAngle = flightCourseAngle;
            
            obj.step = 1;
            obj.path = zeros(3, 1000);
        end
        
        function moveForward(obj, dt)
            obj.x = obj.x  + dt*obj.v*cos(obj.heading);
            obj.y = obj.y + dt*obj.v*sin(obj.heading);
            obj.z = obj.z - dt*obj.v*sin(obj.flightCourseAngle);
            
            obj.steps = obj.steps + 1;
            obj.path(:, obj.step) = [obj.x; obj.y; obj.z];
        end
        
        function staticPlot2D(obj)
            t = 0:2*pi/100:2*pi;
            xcord = obj.avoidanceRadius*cos(t) + obj.x;
            ycord = obj.avoidanceRadius*sin(t) + obj.y;
            
            plot(xcord, ycord)
        end
    end
end