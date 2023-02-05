%% Dynamic SAS (YAW DAMPER)
%% Transfer Functions
%% TF dr a beta
G_bdr = G_dr(1);
%% TF da a beta
G_bda = G_da(1);
%% TF dr a phi
G_phidr = G_dr(2);
%% TF da a phi
G_phida = G_da(2);
%% TF dr a p
G_pdr = s*G_phidr;
%% TF da a p
G_pda = s*G_phida;
%% TF dr a r
G_rdr = G_dr(3);
%% TF da a r
G_rda = G_da(3);
%% Actuator
G_a = 1/(0.1*s+1);
%% Sensor
G_s = (3/10000*s^2-3/100*s+1)/(3/10000*s^2+3/100*s+1);
%% Filtro wash out
G_f = (s/10)/(s/10+1);
%% Analysis
F_r = [0,1.0,1.5,2.0,2.5,3.0];

% for i=1:length(F_r)
%     A_lat_temp = [2*mub*X-Cy.b, -(Cy.p*X-Cz.s), 2*mub-Cy.r;...
%     -Cl.b, Ix*X^2-Cl.p*X, -(Ixz*X+Cl.r);...
%     -Cn.b, -(Ixz*X^2+Cn.p*X), Iz*X-Cn.r*F_r(i)];
%     temp = double(solve(det(A_lat_temp)))/adim_lat
%     sqrt(real(temp(3))^2 + imag(temp(3))^2)
%     -real(temp(3))/ans
% end
% 
% %% Open and closed loop analysis
k_dr = -(F_r-1)*Cn.r*C.b/(Cn.dr*us*2);

% for i=1:length(k_dr)
%     F = G_a * G_rdr * k_dr(i);
%     H = G_s*G_f;
%     G_ol = F*H;
%     G_cl = F/(1+G_ol);
%     pzmap(G_cl)
%     hold on
% end
% 
% hold off
% legend(string(k_dr))

%% Respuesta temporal en viraje
k_dr = 1.556;
% G_ol = [G_bdr , G_bda;...
%     G_phidr, G_phida;...
%     G_rdr, G_rda];
% Funcion de transferencia en lazo cerrado para velocidad angular de
% guiñada (señal de realimentacion)
G_rdr_hat = G_a*G_rdr/(1+k_dr*G_s*G_f*G_a*G_rdr);
G_rda_hat = G_a*G_rda/(1+k_dr*G_s*G_f*G_a*G_rdr);

% Funcion de transferencia en lazo cerrado para angulo de resbalamiento
G_b_drda = [G_a*G_bdr*(1-k_dr*G_s*G_f*G_rdr_hat), G_a*G_bda-k_dr*G_f*G_s*G_a*G_bdr*G_rda_hat];

% Funcion de transferencia en lazo cerrado para angulo de balance
G_phi_drda = [G_a*G_phidr*(1-k_dr*G_s*G_f*G_rdr_hat), G_a*G_phida-k_dr*G_s*G_f*G_a*G_phidr*G_rda_hat];

% Funcionde transferencia para velocidad angular de balance
G_p_drda = [G_a*G_pdr*(1-k_dr*G_s*G_f*G_rdr_hat), G_a*G_pda - k_dr*G_a*G_f*G_s*G_pdr*G_rda_hat];

%% No filter
G_f = 1;
G_rdr_hatno = G_a*G_rdr/(1+k_dr*G_s*G_f*G_a*G_rdr);
G_rda_hatno = G_a*G_rda/(1+k_dr*G_s*G_f*G_a*G_rdr);

% Funcion de transferencia en lazo cerrado para angulo de resbalamiento
G_b_drdano = [G_a*G_bdr*(1-k_dr*G_s*G_f*G_rdr_hatno), G_a*G_bda-k_dr*G_f*G_s*G_a*G_bdr*G_rda_hatno];

% Funcion de transferencia en lazo cerrado para angulo de balance
G_phi_drdano = [G_a*G_phidr*(1-k_dr*G_s*G_f*G_rdr_hatno), G_a*G_phida-k_dr*G_s*G_f*G_a*G_phidr*G_rda_hatno];

% Funcionde transferencia para velocidad angular de balance
G_p_drdano = [G_a*G_pdr*(1-k_dr*G_s*G_f*G_rdr_hatno), G_a*G_pda - k_dr*G_a*G_f*G_s*G_pdr*G_rda_hatno];