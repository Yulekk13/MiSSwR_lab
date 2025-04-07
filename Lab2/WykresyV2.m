% Przykład wyświetlania danych z symulacji czujników pojazdu
close all
load('out.mat')
% Zakładamy, że dane są w workspace: out.lidar, out.radar, out.vision, out.pos, out.vel, out.orien, out.acc, out.ang

% Czas symulacji
t = linspace(0, length(out.pos), length(out.pos)); % lub: t = out.time jeśli dostępny

% Wydobycie danych pozycji i czasu
pos = squeeze(out.pos.Data);  % [3x101] po squeeze
t = out.pos.Time;             % [101x1]

% Wykres trajektorii 3D
figure;
plot3(pos(1,:), pos(2,:), pos(3,:), 'LineWidth', 2);
xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]');
title('Trajektoria pojazdu');
grid on;

%% Wydobycie danych prędkości
vel = squeeze(out.vel.Data);  % [3x101]

% Obliczenie modułu wektora prędkości
v_modul = vecnorm(vel, 2, 1);  % [1x101] - norma po każdym wektorze

% Wykres modułu prędkości w czasie
figure;
plot(t, v_modul, 'LineWidth', 2);
xlabel('Czas [s]');
ylabel('Prędkość [m/s]');
title('Prędkość pojazdu w czasie');
grid on;


%% Dane orientacji
orien = squeeze(out.orien.Data);  % [3x101]
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


%% Przyspieszenie (acceleration)
acc = squeeze(out.acc.Data);  % [3x101]
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
ang = squeeze(out.ang.Data);  % [3x101]
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

%% Wizualizacja wykryć z radaru (jeśli struktura zawiera np. dystans do obiektów)
% Wyciągnięcie danych z timeseries
n = numel(out.radar.Detections);  % liczba detekcji
meas_sample = out.radar.Detections(1).Measurement.Data;
[dim, ~, T] = size(meas_sample);  % 6 x 1 x 101 → dim=6, T=101

% Inicjalizacja [składowa x czas x detekcja]
meas_all = zeros(dim, T, n);

for i = 1:n
    meas_all(:,:,i) = squeeze(out.radar.Detections(i).Measurement.Data);
end

% Wykres jednej składowej (np. zasięgu) dla kilku detekcji
t = out.radar.Detections(1).Measurement.Time;

figure;
for i = 1:min(n, 5)  % pokaż max 5 obiektów na raz
    plot(t, squeeze(meas_all(1,:,i))); hold on;
end
xlabel('Czas [s]');
ylabel('Zasięg [m]');
title('Zasięg wykrytych obiektów przez radar');
%legend("Obiekt 1", "Obiekt 2", "Obiekt 3", "Obiekt 4", "Obiekt 5");
grid on;

%% Vision
n = numel(out.vision.Detections);  % liczba detekcji
meas_sample = out.vision.Detections(1).Measurement.Data;
[dim, ~, T] = size(meas_sample);  % 6 x 1 x 101 → dim=6, T=101

% Inicjalizacja [składowa x czas x detekcja]
meas_all = zeros(dim, T, n);

for i = 1:n
    meas_all(:,:,i) = squeeze(out.vision.Detections(i).Measurement.Data);
end

% Wykres jednej składowej (np. zasięgu) dla kilku detekcji
t = out.vision.Detections(1).Measurement.Time;

figure;
for i = 1:min(n, 5)  % pokaż max 5 obiektów na raz
    plot(t, squeeze(meas_all(1,:,i))); hold on;
end
xlabel('Czas [s]');
ylabel('Zasięg [m]');
title('Zasięg wykrytych obiektów przez kamerę');
%legend("Obiekt 1", "Obiekt 2", "Obiekt 3", "Obiekt 4", "Obiekt 5");
grid on;