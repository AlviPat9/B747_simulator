%% STATIC SAS Lateral Directional mode

%% Sensitivity analysis
F_b = [0, 0.5, 1, 2, 3];
k_dr = (F_b-1)*Cn.b/Cn.dr; % Se le pone el signo menos porque se realimenta de manera negativa

%% Transfer Functions definition
%% Actuator
G_a = 1/(0.1*s+1);

%% Sensor
G_s = (3/10000*s^2-3/100*s+1)/(3/10000*s^2+3/100*s+1);

%% Aircraft
G_beta = G_dr(1);

%% Loop definition

F = G_a * G_beta;

for i=1:length(k_dr)
    H = G_s * k_dr(i);
    G_ol = F * H;
    G_cl = F/(1+ G_ol);
    margin(G_cl)
    1;
end

