% Wyświetlanie danych z symulacji czujników pojazdu
close all
load('out.mat')
% out.mat - plik zapisanych danych wyjściowych z simulink

t = linspace(0, length(out.pos), length(out.pos)); % lub: t = out.time jeśli dostępny


pos = squeeze(out.pos.Data);  
t = out.pos.Time;           

% Wykres trajektorii 3D
figure;
plot3(pos(1,:), pos(2,:), pos(3,:), 'LineWidth', 2);
xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]');
title('Trajektoria pojazdu');
grid on;

%% Wydobycie danych prędkości
vel = squeeze(out.vel.Data); 

% Obliczenie modułu wektora prędkości
v_modul = vecnorm(vel, 2, 1); 

% Wykres modułu prędkości w czasie
figure;
plot(t, v_modul, 'LineWidth', 2);
xlabel('Czas [s]');
ylabel('Prędkość [m/s]');
title('Prędkość pojazdu w czasie');
grid on;


%% Dane orientacji
orien = squeeze(out.orien.Data);  
t = out.orien.Time;

% Wykres orientacji w czasie
figure;
plot(t, orien(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(t, orien(2,:), 'g', 'LineWidth', 1.5);
plot(t, orien(3,:), 'b', 'LineWidth', 1.5);
xlabel('Czas [s]');
ylabel('Orientacja [rad]');
legend('Roll', 'Pitch', 'Yaw');
title('Orientacja pojazdu w czasie');
grid on;


%% Przyspieszenie
acc = squeeze(out.acc.Data);
t = out.acc.Time;

% Wykres przyspieszenia w czasie
figure;
plot(t, acc(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(t, acc(2,:), 'g', 'LineWidth', 1.5);
plot(t, acc(3,:), 'b', 'LineWidth', 1.5);
xlabel('Czas [s]');
ylabel('Przyspieszenie [m/s^2]');
legend('X', 'Y', 'Z');
title('Składowe przyspieszenia w czasie');
grid on;


%% Prędkość kątowa
ang = squeeze(out.ang.Data);
t = out.ang.Time;

figure;
plot(t, ang(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(t, ang(2,:), 'g', 'LineWidth', 1.5);
plot(t, ang(3,:), 'b', 'LineWidth', 1.5);
xlabel('Czas [s]');
ylabel('Prędkość kątowa [rad/s]');
legend('Roll', 'Pitch', 'Yaw');
title('Składowe prędkości kątowej w czasie');
grid on;

%% Wizualizacja wykryć z radaru
n = numel(out.radar.Detections);  % liczba detekcji
meas_sample = out.radar.Detections(1).Measurement.Data;
[dim, ~, T] = size(meas_sample);

meas_all = zeros(dim, T, n);

for i = 1:n
    meas_all(:,:,i) = squeeze(out.radar.Detections(i).Measurement.Data);
end

t = out.radar.Detections(1).Measurement.Time;

figure;
for i = 1:min(n, 5)
     plot(t, squeeze(meas_all(1,:,i))); hold on;
end
xlabel('Czas [s]');
ylabel('Zasięg [m]');
title('Zasięg wykrytych obiektów przez radar');
grid on;

%% Wizualizacja detekcji z kamery
n = numel(out.vision.Detections);  % liczba detekcji
meas_sample = out.vision.Detections(1).Measurement.Data;
[dim, ~, T] = size(meas_sample);

meas_all = zeros(dim, T, n);

for i = 1:n
    meas_all(:,:,i) = squeeze(out.vision.Detections(i).Measurement.Data);
end

meas_all = meas_all * -1;

t = out.vision.Detections(1).Measurement.Time;

figure;
for i = 1:min(n, 5)  % pokaż max 5 obiektów na raz
    plot(t, squeeze(meas_all(1,:,i))); hold on;
end
xlabel('Czas [s]');
ylabel('Zasięg [m]');
title('Zasięg wykrytych obiektów przez kamerę');
grid on;