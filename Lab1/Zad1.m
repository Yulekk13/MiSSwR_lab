clc
close all

myMap = binaryOccupancyMap(100,100,10 ,"grid");
%with properties:

   % mapLayer Properties
   %            LayerName: 'binaryLayer'
   %             DataType: 'logical'
   %         DefaultValue: 0
   %  GridLocationInWorld: [0 0]
   %    GridOriginInLocal: [0 0]
   %   LocalOriginInWorld: [0 0]
   %           Resolution: 5
   %             GridSize: [50 50]
   %         XLocalLimits: [0 10]
   %         YLocalLimits: [0 10]
   %         XWorldLimits: [0 10]
   %         YWorldLimits: [0 10]

walls = zeros(50,50);
walls(1,:) = 1; % Top wall
walls(end,:) = 1; % Bottom wall
walls(:,1) = 1; % Left wall
walls(:,end) = 1; % Right wall
walls(1:30,15) = 1; % Left division
walls(25:end,25) = 1; % Middle division
walls(1:30,35) = 1; % Right division

walls(8,1:4)=1;


setOccupancy(myMap,[1 1],walls,"grid")
show(myMap)