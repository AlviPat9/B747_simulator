%% Lateral directional coefficients definition
%% Geometry properties
UnitConversion;
S = 5500 * (Units.ft2m)^2;
c = 27.3 * 0.3048;
b = 196 * 0.3048;

%% 
CY_b = -0.9; CY_p = 0.0; CY_r = 0.0; CY_da = 0.0; CY_dr = 0.12;
Cl_b = -0.16; Cl_p = -0.34; Cl_r = 0.13; Cl_da = 0.013; Cl_dr = 0.008;
Cn_b = 0.16; Cn_p = -0.026; Cn_r = -0.28; Cn_da = 0.0018; Cn_dr = 0.013;

Cz_s = -0.4;

%% Reference data
atm = atmospheric_model(20000*Units.ft2m);
us = atm(4) * 0.65;
%% Mass properties
mass = 636636 * Units.lb2kg;
mu = mass/(atm(3)*S*0.5*b);
Ix = 18200000/(atm(3)*S*(0.5*b)^3);
Iz = 49700000/(atm(3)*S*(0.5*b)^3);
Ixz = 970000/(atm(3)*S*(0.5*b)^3);



%% Lateral directional transfer functions
s = tf('s');

%% Cuartica de estabilidad
A = 2*mu*(Ix*Iz-Ixz^2);
B = CY_b*(Ixz^2-Ix*Iz) - 2*mu*(Iz*Cl_p + Ix*Cn_r + Ixz*(Cl_r + Cn_p));
C = 2*mu*(Cn_r*Cl_p - Cn_p*Cl_r + Ix*Cn_b + Ixz*Cl_b) + Ix*(CY_b*Cn_r - Cn_b*CY_r)+...
    Iz*(CY_b*Cl_p - Cl_b*CY_p) + Ixz*(CY_b*Cn_p-Cn_b*CY_p + CY_b*Cl_r -Cl_b*CY_r);
D = CY_b*(Cn_p*Cl_r - Cn_r*Cl_p) + CY_p*(Cl_b*Cn_r - Cn_b*Cl_r) + ...
    (2*mu - CY_r)*(Cl_b*Cn_p -Cn_b*Cl_p) + Cz_s*(Iz*Cl_b + Ixz*Cn_b);
E = -Cz_s*(Cl_b*Cn_r - Cn_b*Cl_r);
D_lat = A*s^4 + B*s^3 + C*s^2 + D*s + E;

%% Da a beta
B = 0;
C = -Cl_da*(-Iz*CY_p + (2*mu - CY_r)*Ixz) + Cn_da*(Ixz*CY_p - (2*mu - CY_r)*Ix);
D = -Cl_da*(CY_p*Cn_r + Iz*Cz_s + (2*mu - CY_r)*Cn_p) + ...
    Cn_da*(CY_p*Cl_r - Ixz*Cz_s + (2*mu - CY_r)*Cl_p);
E = Cz_s * (Cl_da*Cn_r - Cn_da*Cl_r);
Nb_da = B*s^3 + C*s^2 + D*s + E;

%% Da a phi
B = 0;
C = 2*mu*Iz*Cl_da + 2*mu*Ixz*Cn_da;
D = -Cl_da*(2*mu*Cn_r + Iz*CY_b) - Cn_da*(-2*mu*Cl_r + Ixz*CY_b);
E = Cl_da*(CY_b*Cn_r + (2*mu - CY_r)*Cn_b) - Cn_da*(CY_b*Cl_r + (2*mu - CY_r)*Cl_b);
Nphi_da = B*s^3 + C*s^2 + D*s + E;

%% Da a velocidad angular de guiñada
A = 0;
B = 2*mu*Ixz*Cl_da + 2*mu*Ix*Cn_da;
C = -Cl_da*(-2*mu*Cn_p + Ixz*CY_b) - Cn_da*(2*mu*Cl_p + Ix*CY_b);
D = -Cl_da*(CY_b*Cn_p - Cn_b*CY_p) + Cn_da*(CY_b*Cl_p - Cl_b*CY_p);
E = Cz_s*(-Cl_da*Cn_b + Cn_da*Cl_b);

Nr_da = 2* us/b * (A*s^4 + B*s^3 + C*s^2 + D*s + E);

%% Dr a beta
B = CY_dr*(Ix*Iz - Ixz^2);
C = -CY_dr*(Ix*Cn_r + Iz*Cl_p + Ixz*(Cn_p + Cl_r)) + Cl_dr*(Iz*CY_p - (2*mu - CY_r)*Ixz)...
    + Cn_dr*(Iz*CY_p - (2*mu - CY_r)*Ix);
D = CY_dr*(Cn_r*Cl_p - Cn_p*Cl_r) - Cl_dr*(CY_p*Cn_r + Iz*Cz_s + (2*mu - CY_r)*Cn_p)...
    + Cn_dr*(CY_p*Cl_r - Ixz*Cz_s + (2*mu - CY_r)*Cl_p);
E = Cz_s*(Cl_dr*Cn_dr - Cn_dr*Cl_r);
Nb_dr = B*s^3 + C*s^2 + D*s + E;

%% Dr a phi
B = 0;
C = 2*mu*Iz*Cl_dr + 2*mu*Ixz*Cn_dr;
D = CY_dr*(Iz*Cl_b + Ixz*Cn_b) - Cl_dr*(2*mu*Cn_r + Iz*CY_b) - Cn_dr*(-2*mu*Cl_r + Ixz*CY_b);
E = -CY_dr*(Cl_b*Cn_r - Cn_b*Cl_r) + Cl_dr*(CY_b*Cn_r + (2*mu - CY_r)*Cn_b)...
    - Cn_dr*(CY_b*Cl_r + (2*mu - CY_r)*Cl_b);

Nphi_dr = B*s^3 + C*s^2 + D*s + E;

%% Dr a velocidad angular de guiñada
A = 0;
B = 2*mu*Cl_dr*Ixz + 2*mu*Ix*Cn_dr;
C = CY_r*(Ixz*Cl_b + Ix*Cn_b) - Cl_dr*(-2*mu*Cn_p + Ixz*CY_b) - ...
    Cn_dr*(2*mu*Cl_p + Ix*CY_b);
D = CY_dr*(Cl_b*Cn_p - Cn_b*Cl_p) - Cl_dr*(CY_b*Cn_p - Cn_b*CY_p) + ...
    Cn_dr*(CY_b*Cl_p - Cl_b*CY_p);
E = Cz_s*(-Cl_dr*Cn_b + Cn_dr*Cl_b);

Nr_dr = 2*us/b * (A*s^4 + B*s^3 + C*s^2 + D*s + E);
