map = binaryOccupancyMap(100, 100, 1);
occ = zeros(100, 100);

% Ustawianie przeszkód na mapie
occ(1,:) = 1;       % Górna ściana
occ(end,:) = 1;     % Dolna ściana
occ(:,1) = 1;       % Lewa ściana
occ(:,end) = 1;     % Prawa ściana

% Dodawanie bloków na mapie
occ(1:40, 25) = 1;
occ(1:60, 50) = 1;
occ(70, 15:35) = 1;
occ(60:100, 75) = 1;
occ(35, 75:100) = 1;
occ(90:100, 50) = 1;

setOccupancy(map, occ);

figure
show(map)
title('Custom Floor Plan')

% Początkowa pozycja robota
robotPose = [50, 50, pi/4];  % Pozycja [x, y, theta] (start w punkcie (50,50) z kątem 45 stopni)

numScans = 180;  % Liczba pomiarów LIDAR
scanAngles = linspace(-pi/2, pi/2, numScans);  % Kąty skanowania (-90° do 90°)

% Wartości odległości w skanie LIDAR
maxRange = 10;  % Maksymalny zasięg LIDAR w metrach
scanData = zeros(1, numScans);  % Przechowanie wyników skanu

for i = 1:numScans
    angle = robotPose(3) + scanAngles(i);  % Kąt, biorąc pod uwagę orientację robota
    [x_end, y_end] = pol2cart(angle, maxRange);  % Przekształcenie do współrzędnych

    % Symulacja skanowania - sprawdzenie kolizji
    for r = 1:maxRange
        x_check = round(robotPose(1) + r * cos(angle));  % Obliczanie współrzędnych punktów skanowania
        y_check = round(robotPose(2) + r * sin(angle));
        
        % Sprawdzenie, czy punkt wchodzi w przeszkodę
        if x_check < 1 || y_check < 1 || x_check > map.XWorldLimits(2) || y_check > map.YWorldLimits(2) || getOccupancy(map, [x_check, y_check])
            scanData(i) = r;  % Zapisz zmierzoną odległość
            break;
        end
    end
end

figure
show(map)
hold on

% Rysowanie promieni skanów LIDAR
for i = 1:numScans
    angle = robotPose(3) + scanAngles(i);
    distance = scanData(i);
    
    if distance > 0  % Jeśli skan wykrył przeszkodę
        [x_end, y_end] = pol2cart(angle, distance);
        plot([robotPose(1), robotPose(1) + x_end], [robotPose(2), robotPose(2) + y_end], 'r');
    end
end

% Rysowanie pozycji robota
plot(robotPose(1), robotPose(2), 'bo', 'MarkerFaceColor','b');
title('Skanowanie LIDAR na mapie')
hold off


% Symulacja ruchu robota (np. przesunięcie o 1 metr do przodu)
robotPose(1) = robotPose(1) + cos(robotPose(3));  % Ruch w osi x
robotPose(2) = robotPose(2) + sin(robotPose(3));  % Ruch w osi y

% Wykonaj skanowanie w nowej pozycji
scanData = zeros(1, numScans);
for i = 1:numScans
    angle = robotPose(3) + scanAngles(i);
    [x_end, y_end] = pol2cart(angle, maxRange);

    for r = 1:maxRange
        x_check = round(robotPose(1) + r * cos(angle));
        y_check = round(robotPose(2) + r * sin(angle));
        
        if x_check < 1 || y_check < 1 || x_check > map.XWorldLimits(2) || y_check > map.YWorldLimits(2) || getOccupancy(map, [x_check, y_check])
            scanData(i) = r;
            break;
        end
    end
end

% Ponowna wizualizacja
figure
show(map)
hold on
for i = 1:numScans
    angle = robotPose(3) + scanAngles(i);
    distance = scanData(i);
    
    if distance > 0
        [x_end, y_end] = pol2cart(angle, distance);
        plot([robotPose(1), robotPose(1) + x_end], [robotPose(2), robotPose(2) + y_end], 'r');
    end
end
plot(robotPose(1), robotPose(2), 'bo', 'MarkerFaceColor','b');
title('Nowe skanowanie LIDAR po ruchu')
hold off


