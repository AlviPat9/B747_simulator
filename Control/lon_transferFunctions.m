%% Longitudinal coefficients definition
%% Geometry properties
UnitConversion;
S = 5500 * (Units.ft2m)^2;
c = 27.3 * 0.3048;
b = 196 * 0.3048;

%% Reference data
atm = atmospheric_model(20000*Units.ft2m);
us = atm(4) * 0.65;

%% Mass properties
mass = 636636 * Units.lb2kg;
mu = mass/(atm(3)*S*0.5*c);
Iy = 33100000/(atm(3)*S*(0.5*c)^3);

%% Longitudinal coefficients
CD_s = 0.025; CD_0 = 0.0164; CD_a = 0.2; CD_de = 0.0;CD_u = 0.0;
CTs = 0.025; CT_u = -0.055;
CL_s = 0.4; CL_0 = 0.21; CL_u = 0.13; CL_a = 4.4;CL_ap = 7.0; CL_q = 6.6; CL_de = 0.32;
Cm_s = 0.0; Cm_u = 0.013; Cm_a = -1.0;Cm_ap = -4.0; Cm_q = -20.5; Cm_de = -1.3;

Cx.u=CT_u-CD_u;Cx.aoa=(CL_s-CD_a); Cx.de = CD_de;
Cz.s=-CL_s;Cz.u=-CL_u;Cz.aoa=-(CL_a+CD_s);Cz.q=-CL_q;Cz.ap=-CL_ap;Cz.de=-CL_de;
Cm.u=Cm_u-CT_u*0;Cm.aoa=Cm_a;Cm.q=Cm_q;Cm.ap=Cm_ap;Cm.de=Cm_de;
%% Definition of variable s
s = tf('s');
%% Cuartica de estabilidad
A = 2*mu*Iy*(2*mu - Cz.ap);
B = -2*mu*Iy*(Cz.aoa + Cx.u) + Iy*Cx.u*Cz.ap - 2*mu*(Cz.q*Cm.ap - Cm.q*Cz.ap)-...
    4*mu^2*(Cm.ap + Cm.q);
C = Iy*(Cx.u*Cz.aoa - Cx.aoa*Cz.u) + 2*mu*(Cz.aoa*Cm.q - Cm.aoa*Cz.q + Cx.u*Cm.q + Cx.u*Cm.ap)-...
    -4*mu^2*Cm.aoa - Cz.u*(Cm.q*Cz.ap - Cz.q*Cm.ap) - 2*Iy^Cz.s*Cx.aoa;
D = -2*Cz.s^2*Cm.ap + 2*mu*(Cx.u*Cm.aoa - Cz.aoa*Cm.u - Cz.s*Cm.u) + ...
    Cx.u*(Cm.aoa*Cz.q - Cm.q*Cz.aoa) - Cx.aoa*(Cm.u*Cz.q - Cm.q*Cz.u) + ...
    Cz.s*(Cm.u*Cz.ap - Cz.u*Cm.ap) + 2*Cz.s*Cm.q*Cx.aoa;
E = Cz.s*(-Cm.aoa*(2*Cz.s + Cz.u) + Cm.u*Cz.aoa);
Dlon = A*s^4 + B*s^3 + C*s^2 + D*s + E;

%% de a u
B = Cx.de*Iy*(2*mu-Cz.ap);
C = Cx.de*((-2*mu + Cz.ap)*Cm.q - Cz.aoa*Iy - (2*mu + Cz.q)*Cm.ap) + Cz.de*Cx.aoa*Iy;
D = Cx.de*(Cz.aoa*Cm.q - (2*mu+Cz.q)*Cm.aoa) + Cz.de*(-Cx.aoa*Cm.q + Cz.s*Cm.ap) + ...
    Cm.de*((2*mu + Cz.q)*Cx.aoa + (2*mu - Cz.ap)*Cz.s);
E = Cz.s*(Cz.de*Cm.aoa - Cm.de*Cz.aoa);

Nu_de = B*s^3 + C*s^2 + D*s + E;