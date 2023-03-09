%% Longitudinal transfer functions
%% Cuartica de estabilidad longitudinal
A = 2*muc*Iy*(2*muc-Cz.ap);
B = -2*muc*Iy*(Cz.aoa+Cx.u)+Iy*Cx.u*Cz.ap-2*muc*(Cz.q*Cm.ap-Cm.q*Cz.ap) - ...
    4*muc^2*(Cm.ap+Cm.q);
C = Iy*(Cx.u*Cz.aoa-Cx.aoa*Cz.u)+2*muc*(Cz.aoa*Cm.q-Cm.aoa*Cz.q+Cx.u*Cm.q+Cx.u*Cm.ap)-...
    -4*muc^2*Cm.aoa-Cx.u*(Cm.q*Cz.ap-Cz.q*Cm.ap)-2*Iy*Cz.s*Cx.aoa;
D = -2*Cz.s^2*Cm.ap+2*muc*(Cx.u*Cm.aoa-Cx.aoa*Cm.u-Cz.s*Cm.u)+...
    Cx.u*(Cm.aoa*Cz.q-Cm.q*Cz.aoa)-Cx.aoa*(Cm.u*Cz.q-Cm.q*Cz.u)+...
    Cz.s*(Cm.u*Cz.ap-Cz.u*Cm.ap)+2*Cz.s*Cm.q*Cx.aoa;
E = Cz.s*(-Cm.aoa*(2*Cz.s+Cz.u)+Cm.u*Cz.aoa);
D_lon = (A*s^4 + B*s^3 + C*s^2 + D*s + E)/A;

%% TF de Elevator a angulo de ataque
B_aoa_de = 2*muc*(Cz.de*Iy);
C_aoa_de = -Cz.de*(2*muc*Cm.q+Iy*Cx.u)+Cm.de*(2*muc+Cz.q)*2*muc;
D_aoa_de = Cz.de*Cx.u*Cm.q-Cm.de*Cx.u*(2*muc+Cz.q);
E_aoa_de = Cz.s*(-Cz.de*Cm.u+Cm.de*(Cz.u+2*Cz.s));

N_aoa_de = B_aoa_de*s^3 + C_aoa_de*s^2 + D_aoa_de*s + E_aoa_de;