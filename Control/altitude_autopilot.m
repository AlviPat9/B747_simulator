%% Altitude Autopilot
%% Transfer functions definition
%% Actuator
G_a = 1/(0.1*s + 1);

%% Sensors
G_s = (3/10000*s^2-3/100*s+1)/(3/10000*s^2+3/100*s+1);
H = G_s;
%% Longitudinal transfer functions
G_theta = G(3);
G_alpha = G(2);
G_gamma = G_theta - G_alpha;
G_q = G_theta*s;
G_h = us/s*G_gamma;
%% Controladores en cascada
%% Proportional gain pitch acceleration

% k = -[-0.05:-0.05:-0.3];
% 
% for i=1:length(k)
%     F = G_a * G_s * k(i);
%     H = G_s;
%     G_ol = F*H;
%     G_cl = F/(1+G_ol);
%     step(G_cl)
%     k(i)
%     hold on
% end
% hold off
k_q = -0.15;

%% Proportional gain angulo de asiento de velocidad

% k_gamma = 0.5:0.5:3;
% for i=1:length(k_gamma)
%     F = k_gamma(i) * k_q * G_a * G_gamma/(1+k_q * G_a * G_q);
%     G_ol = F*H;
%     G_cl = F / (1+G_ol);
%     step(G_cl)
%     k_gamma(i)
%     hold on
% end
% hold off

k_gamma = 0.5;

%% Filtro de avance de fase
alpha = 1:1:5;
w_lead = 10^-3;
% for i=1:length(alpha)
%     G_f = (s/w_lead +1) / (s/(w_lead*alpha(i)) +1);
%     margin(G_f)
%     1;
% end
G_f = (s/w_lead +1) / (s/(w_lead*5) +1);
G_f=1;
%% Proportional gain altitude
k_h = [1,2,3,4]*10-2;
for i=1:length(k_h)

F = G_f* k_h(i) * k_q * k_gamma*G_a*G_h/(1 + k_q*G_a*G_q + k_q*k_gamma*G_a*G_gamma);
G_ol = F*H;
G_cl = F/(1+G_ol);
margin(G_cl)
k_h(i)
hold on
end
hold off




