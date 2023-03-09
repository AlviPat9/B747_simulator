%% SAS Longitudinal -> lazo cerrado
% Actuator
G_a = tf(1,[0.1, 1]);
% Sensor
G_s = tf([3/10000 -3/100 1], [3/10000 3/100 1]);
% Variable to control
G_alpha = G(2);
F = [-0.5, -0.2, 0, 0.5, 1, 2, 3, 4];



%% Open loop analysis
for i=1:length(F)
    P = -((2*muc - Cz.ap)*X - Cz.aoa) * (Iy*X - Cm.q) + (2*muc + Cz.q)*(Cm.aoa*X + F(i)*Cm.aoa);

lambda_lon_aug = double(solve(P))/adim_lon
wn_aug = sqrt(real(lambda_lon_aug(1))^2 + imag(lambda_lon_aug(1))^2)
chi_aug = -real(lambda_lon_aug(1))/wn_aug
CAP_aug = wn_aug^2/(Cz.aoa/Cz.s)
end
k = -(F - 1)*Cm.aoa/Cm.de;
%% Root locus
for i=1:length(k)
    F = G_a * G_alphat;
    H = G_s* k(i);
    G_ol = F*H;
    G_cl = F/(1 + G_ol);
    pzmap(G_cl)
    hold on
end
hold off
legend(string(k))

%% Nichols
for i=1:length(k)
    F = G_a * G_alpha ;
    H = G_s* k(i);
    G_ol = F*H;
    G_cl = F/(1 + G_ol);
    nichols(G_ol)
    hold on
end
hold off
legend(string(k))


%% Step analysis

for i=4:length(k)
    F = G_a * G_alpha ;
    H = G_s* k(i);
    G_ol = F*H;
    G_cl = F/(1 + G_ol);
    step(G_cl)
    k(i)
    hold on
end
hold off
legend(string(k(4:end)))

k_static_SAS_lon = k(7);