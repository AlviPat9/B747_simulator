%% STATIC SAS
%% Transfer Functions definition
%% Actuator
G_a = 1/(0.1*s+1);

%% Sensor
G_s = (3/10000*s^2-3/100*s+1)/(3/10000*s^2+3/100*s+1);

%% Aircraft
G_alpha = G(2);

%% Loop definition

F = G_a * G_alpha;

K_de_sas = -3:0.5:0;

for i=1:length(K_de_sas)
    H = K_de_sas(i) * G_s;
    G_ol = F * H;
    G_cl = F/(1 + G_ol);
%     step(G_cl)
    margin(G_cl)
    hold on
    K_de_sas(i)
    1;

end
hold off
H = -2 * G_s;
G_ol = F * H;
G_cl = F / (1 + G_ol);
step(G_cl)
hold on
step(F)
legend({'k_de_sas=-2','No static sas'})
hold off
% hold off