%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2014
%
% Homework 1
%
% Team number: 10
% Team leader: Brian Slakter (bjs2135)
% Team members: Sinjihn Smith (sss2208), Sheng Qian(sq2168)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function hw4_team_10_new_117(serPort)
path=[[0.58, -3.107];[0.528248, -2.43278];[0.528248, -1.58278];[0.629399, -0.192911];[0.629399, 0.657089];[-0.222342, 4.6695899999999995];[-0.222342, 5.51959];[-0.03, 10.657]];


    %Initialize variables
    len = 6.3543;
    radius = 13.6754;
    calcAngle = -0.1128;
    maxFwdVel= 0.3;         % Max allowable forward velocity 
    maxDuration=1200000;    % Max Duration
    angularVeloRotate = 0.2;% Angular velocity for rotation
    %angularVeloTurn = -0.1; % Angular velocity for movement
    %xpos = 0;               % Initial x position set to 0, only updates after first bump
    %ypos = 0;               % Initial y position set to 0, only updates after first bump
    angle_e = 0;              % Initial angle set to 0, only updates after first bump
    %bumpedCount = 0;        % Keep track of bump count so we know 1st bump
    tStart= tic;            % Time limit marker
    distThresh = 0.05;      % Distance from origin - can be within 0.05 and stop
    %bumpThresh = 4;         % Bump count threshold so we dont stop immediately upon hitting first wall
    angleThreshold=0.02;
    % Uncomment this if you would like to see plot of X-Y in real time 
    % Must also uncomment plot and hold on lines in updatePosition function
    figure(1);
    distance=0;
    angle=0;
    xPos=0.58;
    yPos=-3.107;
    angle=0;
    %Start by moving forward until first bump
    %SetFwdVelAngVelCreate(serPort,maxFwdVel,0);
    SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);
    AngleSensorRoomba(serPort);
    DistanceSensorRoomba(serPort);
    n=length(path);
    Dis=[];
    AngleT=[];
    AngleE=[];
        for i=1:n-1
            xpos=path(i,1);
            ypos=path(i,2);
            xnext=path(i+1,1);
            ynext=path(i+1,2);
            disX=xnext-xpos;
            disY=ynext-ypos;
            angleT=atan(disY/disX);
            
            angle_e=mod(angle_e,2*pi);
            if disY>=0 
                if disX>=0
                    angleT=angleT-pi/2;
                else
                    angleT=angleT+pi/2;
                end
            else
                if disX>=0
                    angleT=angleT-pi/2;
                else
                    angleT=angleT+pi/2;
                end
            end
            AngleE=[AngleE angleT];
            angleT=angleT-angle_e;
            % make the turned angle smallest
            angleT=mod(angleT,2*pi);
            if angleT>pi
                angleT=angleT-2*pi;
            end
            %disp(angleT * 180/pi)
            angle_e=angleT+angle_e;
            
            AngleT=[AngleT angleT];
            distance=sqrt((xnext-xpos)^2+(ynext-ypos)^2);
            Dis=[Dis distance];
        end
        figure(1);
        Dis(3) = len;
    while toc(tStart) < maxDuration
        for i=1:2
            %tTurn=abs(angleT/angularVeloRotate);
            angularVeloRotateNew=angularVeloRotate*sign(AngleT(i));
            angle_temp=0;
            distance_temp=0;
            while abs(angle_temp-AngleT(i))>=angleThreshold
               SetFwdVelAngVelCreate(serPort, 0, angularVeloRotateNew);
               pause(0.01);
               [xPos, yPos, angle,D_distance,D_angle] = updatePosition(serPort, xPos, yPos, angle);
               angle_temp=angle_temp+D_angle;
            end
            %disp('angle')
            %disp(AngleT(i)*180/pi)
            
            while abs(distance_temp-Dis(i))>=distThresh
                SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);
                pause(0.01);
                [xPos, yPos, angle,D_distance,D_angle] = updatePosition(serPort, xPos, yPos, angle);
                distance_temp=distance_temp+D_distance;
            end
            disp('dis')
            disp(i)
            disp(distance_temp)
        end
        %angle=mod(angle,pi*2);
        AngleT(3) = calcAngle - angle;
        angularVeloRotateNew=angularVeloRotate*sign(AngleT(3));
        angle_temp=0;
        distance_temp=0;
        disp(angle*180/pi)
        disp(AngleT(3)*180/pi)
        while abs(angle_temp-AngleT(3))>=angleThreshold
           SetFwdVelAngVelCreate(serPort, 0, angularVeloRotateNew);
           pause(0.01);
           [xPos, yPos, angle,D_distance,D_angle] = updatePosition(serPort, xPos, yPos, angle);
           angle_temp=angle_temp+D_angle;
        end
        disp('done')
        while abs(distance_temp-Dis(3))>=distThresh
            SetFwdVelAngVelCreate(serPort, maxFwdVel, maxFwdVel/radius);
            pause(0.01);
            [xPos, yPos, angle,D_distance,D_angle] = updatePosition(serPort, xPos, yPos, angle);
            distance_temp=distance_temp+D_distance;
        end
        disp('dis')
        disp(i)
        disp(distance_temp)
        AngleT(6)=AngleE(6)-angle;
        for i=6:n-1
            %tTurn=abs(angleT/angularVeloRotate);
            angularVeloRotateNew=angularVeloRotate*sign(AngleT(i));
            angle_temp=0;
            distance_temp=0;
            while abs(angle_temp-AngleT(i))>=angleThreshold
               SetFwdVelAngVelCreate(serPort, 0, angularVeloRotateNew);
               pause(0.01);
               [xPos, yPos, angle,D_distance,D_angle] = updatePosition(serPort, xPos, yPos, angle);
               angle_temp=angle_temp+D_angle;
            end
            disp('angle')
            disp(AngleT(i)*180/pi)
            
            while abs(distance_temp-Dis(i))>=distThresh
                SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);
                pause(0.01);
                [xPos, yPos, angle,D_distance,D_angle] = updatePosition(serPort, xPos, yPos, angle);
                distance_temp=distance_temp+D_distance;
            end
            disp('dis')
            disp(i)
            disp(distance_temp)
        end
        SetFwdVelAngVelCreate(serPort, 0, 0);
        break
        
    end
    
end
% Function to update x and y positions relative to the original bump
function [xPos, yPos, angle,D_distance,D_angle] = updatePosition(serPort, xold, yold, angleold)
    % Angle will be cumulitave, and relative to first bump
    D_angle=AngleSensorRoomba(serPort);
    angle = angleold +D_angle;
    
    % Distance since last call
    D_distance = DistanceSensorRoomba(serPort);
    % Update x and y positons based on updated angle and distance reading
    xPos = xold - D_distance*sin(angle);
    yPos = yold + D_distance*cos(angle);
    % Uncomment these two lines for plotting
    plot(xPos,yPos,'o');
    hold on;
end