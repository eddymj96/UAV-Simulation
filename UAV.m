classdef UAV < handle
    properties
        x
        y
        z
        v
        heading
        waypoint0
        waypoint1
        flightCourseAngle
        turnRadius
        path
        step  
        dir
        states
        TurnCircle
        OppositeCircle
        TC
        OC
        SS
    end
    
    methods
        function obj = UAV(i, v, heading, wp0, wp1, flightCourseAngle, turnRadius)
            obj.x = i(1);
            obj.y = i(2);
            obj.z = i(3);
            obj.v = v;
            obj.heading = heading;
            obj.waypoint0 = wp0;
            obj.waypoint1 = wp1;
            obj.flightCourseAngle = flightCourseAngle;
            obj.turnRadius = turnRadius;
            obj.path = zeros(3, 1000);
            obj.states = zeros(3, 1000);
            obj.step = 1;
            obj.TC = false;
            obj.OC = false;
            obj.SS = true;
        end
        
        function circleStates(obj, obstacle)
            obj.TurnCircle = [obj.x + obj.turnRadius*cos(pi/2 + obj.heading), obj.y + obj.turnRadius*sin(pi/2 + obj.heading)];
            obj.OppositeCircle = [obj.x + obj.turnRadius*cos(-pi/2 + obj.heading), obj.y + obj.turnRadius*sin(-pi/2 + obj.heading)];
            distanceTC = norm(obj.TurnCircle - [obstacle.x,obstacle.y]);
            distanceOC = norm(obj.OppositeCircle - [obstacle.x,obstacle.y]);
            
            headingVector = [cos(obj.heading), sin(obj.heading)];
            posVector = [obstacle.x, obstacle.y] - [obj.x, obj.y];
            beta = acos(dot(headingVector, posVector)/(norm(headingVector)*norm(posVector)));

            obj.TC = distanceTC < obstacle.avoidanceRadius + obj.turnRadius && beta < pi/2;
            obj.OC = distanceOC < obstacle.avoidanceRadius + obj.turnRadius;
            
            % y = -(a/b)*x - c/b
            % c = y - mx
            a = -(obj.waypoint1(2) - obj.waypoint0(2));
            b = (obj.waypoint1(1) - obj.waypoint0(1));
            c = -(obj.waypoint1(2) + (a/b)*obj.waypoint1(1));
            perpDistance = (a*obstacle.x  + b*obstacle.y + c)/norm([a,b]);
            distanceTC =  (a*obj.TurnCircle(1)  + b*obj.TurnCircle(2) + c)/norm([a,b]);
            obj.SS = sign(sign(distanceTC)*(abs(distanceTC) - obj.turnRadius)) == -sign(perpDistance);
            
            obj.dir = sign(perpDistance);
            
            obj.states(:, obj.step) = [obj.TC; obj.OC; obj.SS];
        end
        
        function moveToWayPoint(obj, dt, obstacle)
            wpVector = [obj.waypoint1(1) - obj.x, obj.waypoint1(2) - obj.y];
            headingVector = [cos(obj.heading), sin(obj.heading)];
            
            direction = sign(headingVector(1)*wpVector(2) - headingVector(2)*wpVector(1)); %Artifical 2d Curl
            obj.turn(dt, direction, obstacle);
        end

        
        function moveForward(obj, dt, obstacle)
            obj.x = obj.x  + dt*obj.v*cos(obj.heading);
            obj.y = obj.y + dt*obj.v*sin(obj.heading);
            obj.z = obj.z - dt*obj.v*sin(obj.flightCourseAngle);
            
            obj.step = obj.step + 1;
            
            obj.circleStates(obstacle);
            
            obj.path(:, obj.step) = [obj.x; obj.y; obj.z];
            
        end
        
        function turn(obj, dt, dir, obstacle)
            obj.x = obj.x  + dt*obj.v*cos(obj.heading);
            obj.y = obj.y + dt*obj.v*sin(obj.heading);
            obj.z = obj.z - dt*obj.v*sin(obj.flightCourseAngle);            
            obj.heading = obj.heading + dt*dir*(obj.v/obj.turnRadius);
            
            obj.step = obj.step + 1;
            
            obj.circleStates(obstacle);
            
            obj.path(:, obj.step) = [obj.x; obj.y; obj.z];
        end
        

        function pathTrim(obj)
            for i = length(obj.path):-1:1
                if obj.path(:,i) ~= [0;0;0]
                    obj.path = obj.path(:,i);
                    obj.states = obj.states(:,i);
                    break
                end
            end
        end
        
        function staticPlot2D(obj)
             plot(obj.path(1,:), obj.path(2,:))
             axis equal
        end
        
        function dynamicPlot2D(obj)
            plot(obj.x, obj.y, '*')
            quiver(obj.x, obj.y, 2*cos(obj.heading), 2*sin(obj.heading), 15)
            %             t = 0:2*pi:2*pi/100;
            %             xcord = obj.turnRadius*cos(t);
            %             ycord = obj.turnRadius*sin(t);
            %             
            %             tc = [xcord, ycord] + obj.TurnCircle;
            %             oc = [xcord, ycord] + obj.OppositeCircle;
            
            if  obj.TC && obj.OC 
                rectangle('Position',[obj.TurnCircle(1)- obj.turnRadius, obj.TurnCircle(2) - obj.turnRadius, 2*obj.turnRadius, 2*obj.turnRadius], 'FaceColor',[0 .5 .5],'Curvature',[1,1])
                rectangle('Position',[obj.OppositeCircle(1)- obj.turnRadius, obj.OppositeCircle(2)- obj.turnRadius, 2*obj.turnRadius, 2*obj.turnRadius],'FaceColor',[0 .5 .5], 'Curvature',[1,1])
            elseif  obj.TC 
                rectangle('Position',[obj.TurnCircle(1)- obj.turnRadius, obj.TurnCircle(2)- obj.turnRadius, 2*obj.turnRadius, 2*obj.turnRadius], 'FaceColor',[0 .5 .5],'Curvature',[1,1])
                rectangle('Position',[obj.OppositeCircle(1)- obj.turnRadius, obj.OppositeCircle(2)- obj.turnRadius, 2*obj.turnRadius, 2*obj.turnRadius], 'Curvature',[1,1])
            elseif obj.OC 
                rectangle('Position',[obj.TurnCircle(1)- obj.turnRadius, obj.TurnCircle(2)- obj.turnRadius, 2*obj.turnRadius, 2*obj.turnRadius],'Curvature',[1,1])
                rectangle('Position',[obj.OppositeCircle(1)- obj.turnRadius, obj.OppositeCircle(2)- obj.turnRadius, 2*obj.turnRadius, 2*obj.turnRadius], 'FaceColor',[0 .5 .5], 'Curvature',[1,1])
            else
                rectangle('Position',[obj.TurnCircle(1)- obj.turnRadius, obj.TurnCircle(2)- obj.turnRadius, 2*obj.turnRadius, 2*obj.turnRadius],'Curvature',[1,1])
                rectangle('Position',[obj.OppositeCircle(1)- obj.turnRadius, obj.OppositeCircle(2)- obj.turnRadius, 2*obj.turnRadius, 2*obj.turnRadius], 'Curvature',[1,1])
            end
            
        end
        

    end
end