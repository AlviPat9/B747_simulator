%% Simulink plots

%% Definition of parameters
tout = out.tout; % Simulation time
position = out.simout; % Aircraft position
euler_angles = out.simout1;

%% Trayectory plot
figure()
plot3(position(:,1), position(:,2), position(:,3))

title('Aircraft 3D position')
xlabel('X position')
ylabel('Y position')
zlabel('Z position')
grid on
% hold on

%% Euler Angles
figure()
plot(tout,euler_angles(:,1))
hold on
plot(tout,euler_angles(:,2))
plot(tout,euler_angles(:,3))
title('Euler angles')
grid on
hold off
legend({'phi','theta','psi'})
