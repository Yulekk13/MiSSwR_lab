map = occupancyMap(20, 20, 10); % 20x20m, rozdzielczość 10 komórek na metr

% Dodanie przeszkód (np. ścian)
setOccupancy(map, [5 5; 10 10; 15 15], 1); % Ustawienie zajętości na 1 (przeszkody)

figure;
show(map);
title('Wirtualne środowisko');

%%
map = load('exampleMaps.mat', 'simpleMap'); 
map = occupancyMap(map.simpleMap, 10);
show(map);

%%
lidar = robotics.LidarSensor; 
lidar.SensorOffset = [0, 0]; % Położenie czujnika względem robota
lidar.ScanAngles = linspace(-pi/2, pi/2, 100); % Kąt widzenia 180°
lidar.MaxRange = 10; % Maksymalny zasięg czujnika

% Tworzenie robota
robot = exampleHelperDifferentialDriveRobot();
robot.setMap(map);

% Ustawienie pozycji robota
robot.setRobotPose([2, 2, 0]); % (x, y, theta)

% Pobranie skanów LiDAR
ranges = lidar(robot.getRobotPose);

% Wizualizacja danych LiDAR
figure;
polarplot(lidar.ScanAngles, ranges, 'o');
title('Symulowane skanowanie LiDAR');

%%
for i = 1:50
    robot.drive(0.5, 0.1); % Prędkość liniowa 0.5 m/s, kątowa 0.1 rad/s
    ranges = lidar(robot.getRobotPose);
    polarplot(lidar.ScanAngles, ranges, 'o');
    pause(0.1);
end

%%
% Wczytanie potrzebnych toolboxów (upewnij się, że masz Robotics System Toolbox)
clc; clear; close all;

% Tworzenie mapy zajętości (occupancy map)
map = occupancyMap(20, 20, 10); % 20x20m, rozdzielczość 10 komórek na metr
setOccupancy(map, [5 5; 10 10; 15 15], 1); % Dodanie przeszkód

figure;
show(map);
title('Wirtualne środowisko');

% Tworzenie symulowanego robota
robotRadius = 0.3;
robotPose = [2 2 pi/4]; % Startowa pozycja [x, y, theta]

% Tworzenie czujnika LiDAR
lidar = rangeSensor; % Obiekt symulujący skanowanie
lidar.HorizontalAngle = [-pi/2 pi/2]; % Kąt widzenia 180°
lidar.HorizontalAngleResolution = 100; % Liczba wiązek
lidar.Range = [0 10]; % Zakres pomiaru w metrach

% Pobranie skanów LiDAR
[ranges, angles] = lidar(robotPose, map);

% Wizualizacja wyników skanowania
figure;
polarplot(angles, ranges, 'o');
title('Symulowane skanowanie LiDAR');

%%
% Animacja ruchu robota i skanowania LiDAR
figure;
for i = 1:50
    robotPose(1) = robotPose(1) + 0.1 * cos(robotPose(3)); % Ruch w przód
    robotPose(2) = robotPose(2) + 0.1 * sin(robotPose(3)); % Ruch w przód
    robotPose(3) = robotPose(3) + 0.05; % Obrót

    [ranges, angles] = lidar(robotPose, map);
    
    polarplot(angles, ranges, 'o');
    title(sprintf('Skan LiDAR, krok %d', i));
    pause(0.1);
end
