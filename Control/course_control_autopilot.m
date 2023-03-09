%% Course control autopilot
%% TF da a beta
G_bda = G_da(1);
%% TF dr a beta
G_bdr = G_dr(1);
%% TF da a r
G_rda = G_da(3);
%% TF da a psi
G_psida = G_rda/s;
%% TF da a r
G_rdr = G_dr(3);
%% TF dr a psi
G_psidr = G_rda/s;
%% Course transfer function
G_zi = G_bdr + G_psidr;
%% Actuator
G_a = 1/(0.1*s+1);
%% Sensor
G_s = (3/10000*s^2-3/100*s+1)/(3/10000*s^2+3/100*s+1);
%% Loop analysis definition 

k_p = [0.1, 0.2];
% for i=1:length(k_p)
%     F = k_p(i) * G_a * G_zi;
%     H = G_s;
%     G_ol = F*H;
%     G_cl = F/(1+G_ol);
%     margin(G_cl)
%     hold on
% 
% end
% hold off
% legend(string(k_p))

F = k_p(1) * G_a * G_zi;
H = G_s;
G_ol = F*H;
G_cl_1 = F/(1+G_ol);
F = k_p(2) * G_a * G_zi;
G_ol = F*H;
G_cl_2 = F/(1+G_ol);

k_p = 0.1;
%% Integral Gain

k_der = [-0.01, -0.001, -0.1];
for i=1:length(k_der)
    F = G_a * G_zi * (k_p + k_der(i)*s);
    H = G_s;
    G_ol = F*H;
    G_cl = F/(1+G_ol);
    nichols(G_ol)
    hold on

end
hold off
legend(string(k_der))

F = G_a * G_zi * (k_p + k_der(1)*s);
H = G_s;
G_ol = F*H;
G_cl_1 = F/(1+G_ol);
F = G_a * G_zi * (k_p + k_der(2)*s);
H = G_s;
G_ol = F*H;
G_cl_2 = F/(1+G_ol);


