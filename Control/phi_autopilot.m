%% Autopilot de phi
lat_transferFunctions
%% Transfer Functions
% Actuator
G_a = tf(1,[0.1, 1]);
% Sensor
G_s = tf([3/10000 -3/100 1], [3/10000 3/100 1]);
% Used transfer functions
G_phi_da = Nphi_da/D_lat;
G_p = s * G_phi_da;

%% Actuator
k_p = [0.05, 0.15, 0.3];

for i=1:length(k_p)
    F = G_a * G_p * k_p(i);
    H = G_s;
    G_ol = F*H;
    G_cl = F/(1+G_ol);
    step(G_ol)
    k_p(i)
    hold on
    1;
end
hold off