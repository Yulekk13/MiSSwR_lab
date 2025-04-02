% Pozycja
figure;
plot(out.pos.Time, squeeze(out.pos.Data(1,1,:)), 'r', 'DisplayName', 'X');
hold on;
plot(out.pos.Time, squeeze(out.pos.Data(1,2,:)), 'g', 'DisplayName', 'Y');
plot(out.pos.Time, squeeze(out.pos.Data(1,3,:)), 'b', 'DisplayName', 'Z');
hold off;
title('Pozycja');
xlabel('Czas [s]');
ylabel('Pozycja [m]');
legend;
grid on;
    
% Prędkość
figure;
plot(out.vel.Time, squeeze(out.vel.Data(1,1,:)), 'r', 'DisplayName', 'Vx');
hold on;
plot(out.vel.Time, squeeze(out.vel.Data(1,2,:)), 'g', 'DisplayName', 'Vy');
plot(out.vel.Time, squeeze(out.vel.Data(1,3,:)), 'b', 'DisplayName', 'Vz');
hold off;
title('Prędkość');
xlabel('Czas [s]');
ylabel('Prędkość [m/s]');
legend;
grid on;
    
% Orientacja
figure;
plot(out.orient.Time, squeeze(out.orient.Data(1,1,:)), 'r', 'DisplayName', 'Roll');
hold on;
plot(out.orient.Time, squeeze(out.orient.Data(1,2,:)), 'g', 'DisplayName', 'Pitch');
plot(out.orient.Time, squeeze(out.orient.Data(1,3,:)), 'b', 'DisplayName', 'Yaw');
hold off;
title('Orientacja');
xlabel('Czas [s]');
ylabel('Kąty [rad]');
legend;
grid on;
  
% Przyspieszenie
figure;
plot(out.acc.Time, squeeze(out.acc.Data(1,1,:)), 'r', 'DisplayName', 'Ax');
hold on;
plot(out.acc.Time, squeeze(out.acc.Data(1,2,:)), 'g', 'DisplayName', 'Ay');
plot(out.acc.Time, squeeze(out.acc.Data(1,3,:)), 'b', 'DisplayName', 'Az');
hold off;
title('Przyspieszenie');
xlabel('Czas [s]');
ylabel('Przyspieszenie [m/s²]');
legend;
grid on;