clear
clc
close all

resultCandidates = {
    fullfile('..', 'Results', 'TGC', 'result.dat')
    fullfile('Results', 'TGC', 'result.dat')
    'result.dat'
};

resultFile = '';
for i = 1:numel(resultCandidates)
    if isfile(resultCandidates{i})
        resultFile = resultCandidates{i};
        break;
    end
end

if isempty(resultFile)
    error('Cannot find result.dat. Run from repo root, MatlabHelper, or Results/TGC.');
end

data = load(resultFile);
[~, columnCount] = size(data);
if columnCount < 38
    error('result.dat has %d columns, expected at least 38.', columnCount);
end

time = data(:, 1);
xg = data(:, 2);
yg = data(:, 3);
zg = data(:, 4);
v = data(:, 5);
yaw = data(:, 6);
pitch = data(:, 7);
roll = data(:, 8);
theta = data(:, 9);
psi = data(:, 10);
alpha = data(:, 11);
beta = data(:, 12);
accx = data(:, 13);
accy = data(:, 14);
accz = data(:, 15);
mass = data(:, 16);
thrust = data(:, 17);
accCmdBy = data(:, 18);
accCmdBz = data(:, 19);
xTarget = data(:, 20);
yTarget = data(:, 21);
zTarget = data(:, 22);
rudderX = data(:, 23);
rudderY = data(:, 24);
rudderZ = data(:, 25);
vx = data(:, 26);
vy = data(:, 27);
vz = data(:, 28);
wx = data(:, 29);
wy = data(:, 30);
wz = data(:, 31);
lon = data(:, 32);
lat = data(:, 33);
alt = data(:, 34);
sigmaElv = data(:, 35);
sigmaAz = data(:, 36);
sigmaElvDot = data(:, 37);
sigmaAzDot = data(:, 38);

hasTvc = columnCount >= 41;
if hasTvc
    tvcX = data(:, 39);
    tvcY = data(:, 40);
    tvcZ = data(:, 41);
else
    tvcX = zeros(size(time));
    tvcY = zeros(size(time));
    tvcZ = zeros(size(time));
end

fprintf('Loaded %s: %d rows, %d columns.\n', resultFile, size(data, 1), columnCount);

figure('Name', 'Trajectory')
subplot(2, 2, 1)
plot(xg, yg, 'b-', xTarget, yTarget, 'r--', xTarget(end), yTarget(end), 'r*')
grid on
xlabel('North X (m)')
ylabel('Up Y (m)')
legend('Missile', 'Target', 'Target final', 'Location', 'best')
title('X-Y trajectory')

subplot(2, 2, 2)
plot(zg, yg, 'b-', zTarget, yTarget, 'r--', zTarget(end), yTarget(end), 'r*')
grid on
xlabel('East Z (m)')
ylabel('Up Y (m)')
title('Z-Y trajectory')

subplot(2, 2, 3)
plot(zg, xg, 'b-', zTarget, xTarget, 'r--', zTarget(end), xTarget(end), 'r*')
grid on
xlabel('East Z (m)')
ylabel('North X (m)')
title('Z-X ground track')

subplot(2, 2, 4)
plot3(xg, zg, yg, 'b-', xTarget, zTarget, yTarget, 'r--', xTarget(end), zTarget(end), yTarget(end), 'r*')
grid on
xlabel('North X (m)')
ylabel('East Z (m)')
zlabel('Up Y (m)')
title('3D trajectory')

figure('Name', 'Speed And Altitude')
subplot(2, 1, 1)
plot(time, v, 'b-')
grid on
xlabel('Time (s)')
ylabel('Speed (m/s)')
title('Speed')

subplot(2, 1, 2)
plot(time, alt, 'r-')
grid on
xlabel('Time (s)')
ylabel('Altitude (m)')
title('Altitude')

figure('Name', 'Angles')
subplot(2, 2, 1)
plot(time, alpha, 'b-', time, pitch, 'r-', time, theta, 'c-')
grid on
xlabel('Time (s)')
ylabel('Angle (deg)')
legend('Alpha', 'Pitch', 'Flight path angle', 'Location', 'best')
title('Longitudinal angles')

subplot(2, 2, 2)
plot(time, beta, 'r-', time, yaw, 'b-', time, psi, 'm-')
grid on
xlabel('Time (s)')
ylabel('Angle (deg)')
legend('Beta', 'Yaw', 'Heading', 'Location', 'best')
title('Lateral angles')

subplot(2, 2, 3)
plot(time, roll, 'k-')
grid on
xlabel('Time (s)')
ylabel('Roll (deg)')
title('Roll')

subplot(2, 2, 4)
plot(time, rudderX, 'r-', time, rudderY, 'k-', time, rudderZ, 'b-')
grid on
xlabel('Time (s)')
ylabel('Rudder (deg)')
legend('dx', 'dy', 'dz', 'Location', 'best')
title('Rudder command')

figure('Name', 'Velocity And Angular Rate')
subplot(2, 1, 1)
plot(time, vx, 'r-', time, vy, 'k-', time, vz, 'b-')
grid on
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('Vx', 'Vy', 'Vz', 'Location', 'best')
title('Velocity in launch NUE')

subplot(2, 1, 2)
plot(time, wx, 'r-', time, wy, 'k-', time, wz, 'b-')
grid on
xlabel('Time (s)')
ylabel('Angular rate (deg/s)')
legend('Wx', 'Wy', 'Wz', 'Location', 'best')
title('Body angular rate')

figure('Name', 'Acceleration Command')
subplot(2, 1, 1)
plot(time, accx, 'r-')
grid on
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Body-axis X acceleration')

subplot(2, 1, 2)
plot(time, accy, 'g-', time, accz, 'k-', time, accCmdBy, 'b-.', time, accCmdBz, 'm--')
grid on
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
legend('Body Ay', 'Body Az', 'Command Ay', 'Command Az', 'Location', 'best')
title('Body-axis lateral acceleration')

figure('Name', 'Mass And Thrust')
subplot(2, 1, 1)
plot(time, mass, 'b-')
grid on
xlabel('Time (s)')
ylabel('Mass (kg)')
title('Mass')

subplot(2, 1, 2)
plot(time, thrust, 'r-')
grid on
xlabel('Time (s)')
ylabel('Thrust (N)')
title('Thrust')

figure('Name', 'Line Of Sight')
subplot(2, 2, 1)
plot(time, sigmaElv, 'k-')
grid on
xlabel('Time (s)')
ylabel('Elevation (deg)')
title('LOS elevation')

subplot(2, 2, 2)
plot(time, sigmaAz, 'k-')
grid on
xlabel('Time (s)')
ylabel('Azimuth (deg)')
title('LOS azimuth')

subplot(2, 2, 3)
plot(time, sigmaElvDot, 'k-')
grid on
xlabel('Time (s)')
ylabel('Elevation rate (deg/s)')
title('LOS elevation rate')

subplot(2, 2, 4)
plot(time, sigmaAzDot, 'k-')
grid on
xlabel('Time (s)')
ylabel('Azimuth rate (deg/s)')
title('LOS azimuth rate')

figure('Name', 'TVC Command')
plot(time, tvcX, 'r-', time, tvcY, 'k-', time, tvcZ, 'b-')
grid on
xlabel('Time (s)')
ylabel('Nozzle angle (deg)')
legend('tvc dx', 'tvc dy', 'tvc dz', 'Location', 'best')
if hasTvc
    title('Thrust vector control command')
else
    title('Thrust vector control command unavailable in this result.dat')
end

figure('Name', 'Geographic Track')
subplot(2, 1, 1)
geoplot(lat, lon, 'b-', 'LineWidth', 2)
hold on
geoscatter(lat([1 end]), lon([1 end]), 50, 'r', 'filled')
geobasemap topographic
title('Geographic track')

subplot(2, 1, 2)
plot(lon, alt, 'r-', 'LineWidth', 2)
grid on
xlabel('Longitude (deg)')
ylabel('Altitude (m)')
title('Altitude profile')
