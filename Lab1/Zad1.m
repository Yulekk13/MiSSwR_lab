clear 
clc

%Tworzenie siatki
myMap = binaryOccupancyMap(100,100,10 ,"grid");

%Tworzenie ścian
walls = zeros(100,100);
walls(1,:) = 1; % Górna ściana
walls(end,:) = 1; % Dolna ściana
walls(:,1) = 1; % Lewa ściana
walls(:,end) = 1; % Prawa ściana

%Ściany wewnętrzne
walls(1:40, 25) = 1;
walls(1:60,50) = 1;
walls(70,15:35) = 1; 
walls(60:100,75) = 1; 
walls(35,75:100) = 1; 
walls(90:100,50) = 1; 

setOccupancy(myMap,[1 1],walls,"grid")
show(myMap)

