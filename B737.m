%% BOEING 737 Simulator

UnitConversion;
% Geometrical properties
c = 27.3 * Units.ft2m;
b = 196 * Units.ft2m;
S = 5500 * (Units.ft2m)^2;

z_t = 0/(c/2);
x_t = 0;
% mass properties
W = 636636 * Units.lb2kg;
I = [18200000, 0, -970000; 0, 33100000, 0; -970000, 0, 49700000] * Units.slugft22kgm2;
M = 0.65;
x0 = [0, 0, 20000*0.3048];
atm = atmospheric_model(x0(3));
v = [atm(4)*0.65, 0, atm(4)*0.65*sind(2.5)];
% Steady state coeff
CLs = 0.4; CDs = 0.025; CTs = 0.025;

%% Longitudinal coefficients
% Drag Coefficients
CD0 = 0.0164; CDu = 0; CD_a = 0.2;
CT_u = -0.055;
% Lift Coefficients
CL0 = 0.21; CL_u = 0.13; CL_a = 4.4;
CL_ap = 7.0; CL_q = 6.6; CL_de = 0.32;
% Pitch Coefficients
Cm_0 = 0; Cm_u = 0.013; Cm_a = -1.0;
Cm_ap = -4.0; Cm_q = -20.5; Cm_de = -1.3;

%% Lateral-Directional coefficients
% Side force Coefficients
CY_b = -0.9; CY_p = 0; CY_r = 0; CY_da = 0;
CY_dr = 0.120;
% Roll Coefficients;
Cl_b = -0.16; Cl_p = -0.34; Cl_r = -0.13;
Cl_da = 0.013; Cl_dr = 0.008;
% Yaw Coefficients
Cn_b = 0.16; Cn_p = -0.026; Cn_r = -0.28;
Cn_dr = 0.0018; Cn_da = 0.013;

%% Longitudinal mode 
% Cxu = CT_u(1) - CD_u(1); Czu = -CL_u(1); Cmu = Cm_u(1) - CT_u(1) * z_t;
% Cxa = CLs(1) - CD_a(1); Cza = -CL_a(1) - CDs(1); Cma = Cm_a(1);
% Czap = -CL_ap(1); Cmap = 