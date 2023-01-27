%% Pitch Autopilot
%% Transfer Functions definition
%% Actuator
G_a = 1/(0.1*s+1);

%% Sensor
G_s = (3/10000*s^2-3/100*s+1)/(3/10000*s^2+3/100*s+1);

%% Ley directa de control de angulo de asiento
G_theta = G(3);
F = G_a * G_theta;
[x,y] = step(F);

k_de_direct = -1/x(end);
% step(F*k_de_direct)

%% Proportional Gain
k_prop = -0.05:-0.1:-0.55;

% for i=1:length(k_prop)
%     F = G_a*G_theta * k_prop(i);
%     H = G_s;
%     G_ol = F*H;
%     G_cl = F/(1+G_ol);
%     step(G_cl)
%     k_prop(i)
%     hold on
% end
% hold off

% Proportional gain value
k_prop = -0.15;

%% Integral gain value
k_i = 0:-0.01:-0.03;

% for i=1:length(k_i)
%     F = G_a * G_theta * (k_prop + k_i(i)/s);
%     H = G_s;
%     G_ol = F*H;
%     G_cl = F/(1+G_ol);
%     step(G_cl,10^5)
%     k_i(i)
%     hold on
% end
% hold off

% Integral Gain
k_i = -0.01;
%% Derivative gain
k_der = 0:-0.02:-0.1;
for i=1:length(k_der)
    F = G_a * G_theta * k_de_direct * (k_prop + k_i/s + k_der(i)*s);
    H = G_s/k_de_direct;
    G_ol = F*H;
    G_cl = F/(1+G_ol);
    step(G_cl)
    hold on
end
hold off

% Derivative Gain
k_der = -0.1;
%% Step analysis
% step(G_theta)
F = G_a * G_theta * k_de_direct * (k_prop);
H = G_s/k_de_direct;
G_ol = F*H;
G_cl = F/(1+G_ol);
hold on
step(G_cl,10^5)
F = G_a * G_theta * k_de_direct * (k_prop + k_i/s);
H = G_s/k_de_direct;
G_ol = F*H;
G_cl = F/(1+G_ol);
step(G_cl,10^5)
F = G_a * G_theta * k_de_direct * (k_prop + k_i/s + k_der*s);
H = G_s / k_de_direct;
G_ol = F*H;
G_cl = F/(1+G_ol);
step(G_cl,10^5)
%%
hold off
legend({'P','PI','PID'})

