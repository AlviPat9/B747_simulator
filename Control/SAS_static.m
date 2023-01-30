%% SAS Longitudinal -> lazo cerrado
% Actuator
G_a = tf(1,[0.1, 1]);
% Sensor
G_s = tf([3/10000 -3/100 1], [3/10000 3/100 1]);
% Variable to control
G_alpha = G(2);
F = [-0.5, -0.2, 0, 0.5, 1, 2, 3, 4];

k = -(F - 1)*Cm.aoa/Cm.de;

%% Root locus
for i=1:length(k)
    F = G_a * G_alpha ;
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
