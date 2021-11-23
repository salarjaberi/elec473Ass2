rawDATA = load("robotdata");

timeSTAMPS = rawDATA.Log.ts(:,:);     %% timestamp at current time step (can be ignored)
robotPOSITION = rawDATA.Log.rp(:,:); %% [x_robot, y_robot, theta_robot] in ODOMETRY frame at current time step
laserPOSITION = rawDATA.Log.lp(:,:); %% [x_laser, y_laser, theta_laser] in ODOMETRY frame at current time step, NaN if not 'L' data
rangeDATA = rawDATA.Log.r(:,:);  %% [ 180 columns of laser data  ] in cm, NaN if not 'L' data, laser range [-89.5 deg : 1 deg : 89.5 deg] 

%% Create a matrix consisting of estimated odometry commands sent to the robot
for i = 2:length(timeSTAMPS(:))
    u(i,:) = odometryCOMMAND(robotPOSITION(i,:),robotPOSITION(i-1,:));
end

%% show unaltered occupancy grid map
map = dlmread("map.dat");
imshow(map)
hold on
axis on

%% plots state estimate of the occupancy map, no reference to initial position or map data included
for i = 1:length(timeSTAMPS(:))
    stateEST(i,:) = sampleMODEL(u(i,:),robotPOSITION(i,:));
    plot(stateEST(i,1),stateEST(i,2),'.b')  %% indicated by blue dots on map
    hold on
end

%% Plot robot position
figure (2)
subplot(2,1,1)
plot(robotPOSITION(:,1)./100,robotPOSITION(:,2)./100)
xlabel('x position (m)') 
ylabel('y position (m)') 
legend({'Robot Position'},'Location','southwest')

%% Plot the robot orientation wrt time
subplot(2,1,2)
plot(timeSTAMPS(:),robotPOSITION(:,3))
xlabel('time (sec)') 
ylabel('robot orientation (rads)') 
legend({'theta'},'Location','southwest')

%% initial robot position (350:550,350:425)
xPOINT_0 = 350; %% initial leading point of occupancy on x axis
yPOINT_0 = 350; % initial leading point of occupancy on y axis

%% Odometry Model - command generation
function u = odometryCOMMAND(stateCURRENT,statePREVIOUS)
    xCURRENT = stateCURRENT(1);     xPREVIOUS = statePREVIOUS(1);
    yCURRENT = stateCURRENT(2);     yPREVIOUS = statePREVIOUS(2);
    thetaCURRENT = stateCURRENT(3); thetaPREVIOUS = statePREVIOUS(3);
    
    deltaTRANS = sqrt((xCURRENT-xPREVIOUS)^2 + (yCURRENT-yPREVIOUS)^2);
    deltaROT1 = atan2(yCURRENT-yPREVIOUS,xCURRENT-xPREVIOUS) - thetaPREVIOUS;
    deltaROT2 = thetaCURRENT - thetaPREVIOUS - deltaROT1;
    
    u = [deltaROT1,deltaTRANS,deltaROT2];
end

%% Sample Based Motion Model
function stateESTIMATE = sampleMODEL(u,statePREVIOUS)
    %% Motion model parameters
    a1 = 0.0003; %% (rad^2/rad^2)
    a2 = 0.0001; %% (rad^2/cm^2)
    a3 = 0.006; %% (cm^2/cm^2)
    a4 = 1; %% (cm^2/cm^2)
    
    dhatROT1 = u(1) + normrnd(0,a1*(u(1)^2) + a2*(u(2)^2));
    dhatTRANS = u(2) + normrnd(0,a3*(u(2)^2) + a4*(u(1)^2 + u(3)^2));
    dhatROT2 = u(3) + normrnd(0,a1*(u(3)^2) + a2*(u(2)^2));
    
    est1 = statePREVIOUS(1) + dhatTRANS*cos(statePREVIOUS(3) + dhatROT1);
    est2 = statePREVIOUS(2) + dhatTRANS*sin(statePREVIOUS(3) + dhatROT1);
    est3 = statePREVIOUS(3) + dhatROT1 + dhatROT2;
    
    stateESTIMATE = [est1,est2,est3];
end

%% P_hit function
% function P = probability(z,zEXP,zMAX,sigHIT)
%     lambda = 1;
%     aHIT = 1; 
%     aSHORT = 1;
%     aMAX = 1;
%     aRAND = 1;
%     
%     if zEXP == zMAX
%          nHIT = 2/(erf((zMAX - zEXP)/(sqrt(2)*sigHIT)) - erf((-zEXP)/(sqrt(2)*sigHIT));
%     else 
%         nHIT = 1;
%     end
%     pHIT = nHIT*(1/sqrt(2*pi*(sigHIT^2))*exp(-((0.5)/(sigHIT^2))*(z - zEXP)^2));
%     
%     pRAND = 1/zMAX;
%     
%     if z < zMAX
%        n = 1/(1 - exp(-lambda*z));
%        pSHORT = n*lambda*exp(-lambda*z);
%     else
%        pSHORT = 0;
%     end
%     
%     pMAX = 0;
%     
%     P = [aHIT,aSHORT,aMAX,aRAND]*[pHIT;pSHORT;pMAX;pRAND];
% end

%% incomplete function, based on algorithm shown on pg 172 of the textbook
function q = algorithm(z,currentPOSITION,map) 
    xROBOT = currentPOSITION(1); 
    yROBOT = currentPOSITION(2); 
    thetaROBOT = currentPOSITION(3);
    
    %% Position of the laser sensor based on robot position/orientation
    xLASER = xROBOT + 25*cos(thetaROBOT);
    yLASER = yROBOT +25*sin(thetaROBOT);
    
    %% Laser angles range from -89.5 to 89.5 degrees at each measurement timestep
    thetaLASER = [-89.5:1:89.5];
    
    zMAX = 8000;
    q = 1;
   
    for k = 1:length(z(:))
       if z(k) < zMAX
             %% Position of a sensed obstacle
             xZ(k) = xROBOT + cos(thetaROBOT)*xLASER - sin(thetaROBOT)*yLASER + z(k)*cos(thetaROBOT+thetaLASER(k));
             yZ(k) = yROBOT + cos(thetaROBOT)*yLASER + sin(thetaROBOT)*xLASER + z(k)*sin(thetaROBOT+thetaLASER(k));
             
             %% Minimum distance of sensed obstacle from known obstacle positions on the map
             dist = distance(xZ(k),yZ(k),map);
       end
    end

end

%% finds the minimum distance to an occupied grid square center from a sensed obstacle
function dist = distance(x,y,map)
     dist = 8000; 
     for i = 1:800
         for j = 1:800
             if map(i,j) == 1
                xp = 10*j - 5;
                yp = 10*i - 5;   
                A = sqrt((x - xp)^2 + (y - yp)^2);

                if A < dist
                   dist = A;
                end
             end
         end
     end
end









