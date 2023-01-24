%% Coef_definition
UnitConversion;
% Geometry Definition
C.S = 5500 * (Units.ft2m)^2;
C.c = 27.3 * 0.3048;
C.b = 196 * 0.3048;
C.mass = 636636 * Units.lb2kg;
C.cg = 26.1; % [%CMA]
C.I = [18200000, 0, -970000; 0, 33100000, 0; -970000, 0, 49700000] * Units.slugft22kgm2;
% flight Conditions
C.M = 0.65;
C.H = 20000 * Units.ft2m;
% Coefficients -> longitudinal mode
C.CD_s = 0.025; C.CD_0 = 0.0164; C.CD_a = 0.2; C.CD_de = 0.0;
CTs = 0.025; CT_u = -0.055;
C.CL_s = 0.4; C.CL_0 = 0.21; C.CL_u = 0.13; C.CL_a = 4.4;C.CL_ap = 7.0; C.CL_q = 6.6; C.CL_de = 0.32;
C.Cm_s = 0.0; C.Cm_u = 0.013; C.Cm_a = -1.0;C.Cm_ap = -4.0; C.Cm_q = -20.5; C.Cm_de = -1.3;

% Coefficients -> Laterational-Directional mode
C.CY_b = -0.9; C.CY_p = 0.0; C.CY_r = 0.0; C.CY_da = 0.0; C.CY_dr = 0.12;
C.Cl_b = -0.16; C.Cl_p = -0.34; C.Cl_r = 0.13; C.Cl_da = 0.013; C.Cl_dr = 0.008;
C.Cn_b = 0.16; C.Cn_p = -0.026; C.Cn_r = -0.28; C.Cn_da = 0.0018; C.Cn_dr = 0.013;

%% Mass properties
atm = atmospheric_model(C.M);
% Longitudinal mass properties
muc = C.mass/(atm(3)*C.S*0.5*C.c);
Iy = C.I(2,2)/(atm(3)*C.S*(0.5*C.c)^3);
% Lateral mass properties
mub = C.mass/(atm(3)*C.S*0.5*C.b);
Ix = C.I(1,1)/(atm(3)*C.S*(0.5*C.b)^3);
Iz = C.I(3,3)/(atm(3)*C.S*(0.5*C.b)^3);
Ixz = -C.I(1,3)/(atm(3)*C.S*(0.5*C.b)^3);

%% Stability derivatives calculation
% Longitudinal modes
Cx.u=-2*C.CD_s;Cx.aoa=-(C.CL_s-C.CD_a);
Cz.s=-C.CL_s;Cz.u=0;Cz.aoa=-(C.CL_a+C.CD_s);Cz.q=-C.CL_q;Cz.ap=-C.CL_ap;Cz.de=-C.CL_de;
Cm.u=C.Cm_u;Cm.aoa=C.Cm_a;Cm.q=C.Cm_q;Cm.ap=C.Cm_ap;Cm.de=C.Cm_de;
% Lateral Directional modes
Cy.b = -0.9; Cy.p = 0.0; Cy.r = 0.0; Cy.da = 0.0; Cy.dr = 0.12;
Cl.b = -0.16; Cl.p = -0.34; Cl.r = 0.13; Cl.da = 0.013; Cl.dr = 0.008;
Cn.b = 0.16; Cn.p = -0.026; Cn.r = -0.28; Cn.da = 0.0018; Cn.dr = 0.013;

%% Longitudinal modes
s=tf('s');
syms X
adim_lon = C.c/(2*C.M*atm(4));
Alon = [2*muc*s-Cx.u, -Cx.aoa, -Cz.s;...
    -(Cz.u+2*Cz.s), (2*muc-Cz.ap)*s-Cz.aoa, -(2*muc+Cz.q)*s;...
    -Cm.u, -(Cm.ap*s+Cm.aoa), Iy*s^2-Cm.q*s];

A_lon = [2*muc*X-Cx.u, -Cx.aoa, -Cz.s;...
    -(Cz.u+2*Cz.s), (2*muc-Cz.ap)*X-Cz.aoa, -(2*muc+Cz.q)*X;...
    -Cm.u, -(Cm.ap*X+Cm.aoa), Iy*X^2-Cm.q*X];

lambda_lon = double(solve(det(A_lon)))/adim_lon;
wn = [sqrt(real(lambda_lon(1))^2 + imag(lambda_lon(1))^2), sqrt(real(lambda_lon(3))^2 + imag(lambda_lon(3))^2)];
chi = [-real(lambda_lon(1))/wn(1), -real(lambda_lon(3))/wn(2)];

% Transfer functions of the aircraft
% Definition of elevator derivatives
de = [0, Cz.de, Cm.de]';
G = Alon\de;

%% Lateral directional modes
%% Longitudinal modes
adim_lat = C.b/(2*C.M*atm(4));
Alat = [2*mub*s-Cy.b, -(Cy.p*s-Cz.s), 2*mub-Cy.r;...
    -Cl.b, Ix*s^2-Cl.p*s, -(Ixz*s+Cl.r);...
    -Cn.b, -(Ixz*s^2+Cn.p*s), Iz*s-Cn.r];

A_lat = [2*mub*X-Cy.b, -(Cy.p*X-Cz.s), 2*mub-Cy.r;...
    -Cl.b, Ix*X^2-Cl.p*X, -(Ixz*X+Cl.r);...
    -Cn.b, -(Ixz*X^2+Cn.p*X), Iz*X-Cn.r];

lambda_lat = double(solve(det(A_lat)))/adim_lat;
dr = [Cy.dr, Cl.dr, Cn.dr]';
da = [Cy.da, Cl.da, Cn.da]';
G_dr = Alat\dr;
G_da = Alat\da;
% Autovalor asociado a convergencia en balance -> mayor parte real (no es complejo)
% Cumple condiciones de convergencia en balance

% Autovalor asociado a modo espiral -> menor parte real (no es complejo)
% Cumple con T2min (es convergente) -> la amplitud no se duplica

% Dutch roll
wn_dr = sqrt(real(lambda_lat(2))^2+imag(lambda_lat(2))^2);
chi_dr = - real(lambda_lat(2))/wn_dr;
% Requerimientos (caso de mayor restriccion) ->
% chi_min =0.19; (chi*wn)_min=0.35; wn_min=1.0;
% Necesidad de aumentar el amortiguamiento, a costa de una disminucion de
% la frecuencia

