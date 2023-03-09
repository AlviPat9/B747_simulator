%% Bank angle autopilot
%% Transfer Functions

%% TF da a beta
G_bda = G_da(1);
%% TF da a phi
G_phida = G_da(2);
%% TF da a p
G_pda = s*G_phida;
%% TF da a r
G_rda = G_da(3);
%% TF da a psi
G_psida = G_rda/s;

%% Actuator
G_a = 1/(0.1*s+1);
%% Sensor
% G_s = (3/10000*s^2-3/100*s+1)/(3/10000*s^2+3/100*s+1);
G_s=1;

%% Ganancia proporcional de velocidad angular de balance
% k_p = [0.05, 0.1, 0.2, 0.32, 0.4];
% 
% for i=1:length(k_p)
%     F = G_a * G_pda * k_p(i);
%     H = G_s;
%     G_ol = F*H;
%     G_cl = F/(1+G_ol);
%     nichols(G_ol)
%     G_cl_phi = G_cl/s;
%     G_cl_b = k_p(i)*G_a*G_bda/(1+G_ol);
%     G_cl_r = k_p(i)*G_a*G_rda/(1+G_ol);
%     hold on
% end
% 
% hold off
% legend(string(k_p))

k_p_bank_angle = 0.3;

%% Ganancia proporcional de angulo de balance
k_phi = [0.05, 0.1, 0.4, 0.8, 4];
i=3;
% for i=1:length(k_phi)
    dphi = k_p_bank_angle * G_a * G_phida/(1+ k_p_bank_angle*G_pda*G_a);
    G_ol = k_phi(i)*dphi;
    G_cl = dphi/(1+G_ol);
    TF_da = k_p_bank_angle*(k_phi(i)*(1-G_cl)-s*G_cl);
    G_cl_p = s * G_cl;
    G_cl_b = G_bda/G_phida * G_cl;
    G_cl_r = G_rda/G_phida * G_cl;

%     pzmap(G_cl)
%     hold on
% end
% hold off
% legend(string(k_phi))

%% Heading Mode autopilot
k_psi = [1, 2, 5, 10];

for i=1:length(k_psi)
G_ol_psi = k_psi(i) * G_cl_r / s;
G_cl_psi = G_ol_psi/(1+G_ol_psi);
% Transfer function definition
G_cl_rpsi = s*G_cl_psi;
G_cl_bpsi = G_bda/G_psida * G_cl_psi;
G_cl_ppsi = G_pda/G_psida * G_cl_psi;
G_cl_phipsi = G_phida/G_psida * G_cl_psi;

% Aileron deflection
TF_da = (k_p_bank_angle*k_phi(3)*k_psi(i)*(1-G_cl_psi)...
    - k_p_bank_angle*k_phi(3)*G_cl_phipsi) - k_p_bank_angle*G_cl_ppsi;

    pzmap(G_cl_psi);
    hold on
end
hold off
legend(string(k_psi))




