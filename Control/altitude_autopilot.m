%% Altitude Autopilot
%% Transfer functions definition
%% Actuator
G_a = 1/(0.1*s + 1);

%% Sensors
% G_s = (3/10000*s^2-3/100*s+1)/(3/10000*s^2+3/100*s+1);
G_s = 1;
%% Longitudinal transfer functions
G_theta = G(3);
G_alpha = G(2);
G_h = us/s * (G_theta - G_alpha);

%% Proportional Gain

k_h_prop = [-0.0005,-0.00001,-0.0001]*10^-4;

for i=1:length(k_h_prop)
    F = k_h_prop(i) * G_h * G_a;
    H = G_s;
    G_ol = F*H;
    G_cl = F/(1+G_ol);
    step(G_cl,10^8)
    hold on
    1;
end
hold off

