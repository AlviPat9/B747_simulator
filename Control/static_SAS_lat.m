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
%% Effect analysis
F = [-0.5, -0.2, 0, 0.5, 1, 2, 3, 4];
for i=1:length(F)
A_lat = [2*mub*X-Cy.b, -(Cy.p*X-Cz.s), 2*mub-Cy.r;...
    -Cl.b, Ix*X^2-Cl.p*X, -(Ixz*X+Cl.r);...
    -Cn.b*F(i), -(Ixz*X^2+Cn.p*X), Iz*X-Cn.r];

lambda_lat_aug = double(solve(det(A_lat)))/adim_lat;
wn_dr_aug = sqrt(real(lambda_lat_aug(3))^2+imag(lambda_lat_aug(3))^2)
chi_dr_aug = - real(lambda_lat_aug(3))/wn_dr_aug
chi_dr_aug*wn_dr_aug
end

%% Open and closed loop analysis
%% Root locus
F = [1.5, 1.25,1.10,1.05,1];
kb_dr = -(F-1)*Cn.b/Cn.dr;

for i=1:length(kb_dr)
    F = G_a * G_beta * kb_dr(i);
    H = G_s;
    G_ol = F*H;
    G_cl = F/(1+ G_ol);
    pzmap(G_cl)
    hold on

end

hold off
legend(string(kb_dr))

%% Nichols plot
for i=1:length(kb_dr)
    F = G_a * G_beta * kb_dr(i);
    H = G_s;
    G_ol = F*H;
    G_cl = F/(1+ G_ol);
    nichols(G_ol)
    hold on

end

hold off
legend(string(kb_dr))
