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
x0 = [0; 0; 20000*0.3048];
atm = atmospheric_model(x0(3));

% Steady state coeff
CLs = 0.4; CDs = 0.025; CTs = 0.025;
fun = @(x)StationaryState(x,x0(3),M);
x_0 = [0, 0.5, 0];
[out_tr,F, ~] = fsolve(fun,x_0);
v = [atm(4)*0.65, 0,atm(4)*0.65*sin(out_tr(1))];
euler_angles=[0,out_tr(1),0];

Initial_cond = [0;0;0;0;out_tr(1);0;v(1);0;v(3);x0];
%% Function
function out=StationaryState(x,h,M)
%% Trimmer
%% Vars
% x(1)=alpha;x(2)=T_lever;x(3)=de;
alpha=x(1);
T_lever=x(2);
de=x(3);

atm = atmospheric_model(h);
%% Longitudinal coefficients
% Drag Coefficients
CD0 = 0.0164;CD_a = -0.2;
% Lift Coefficients
CL0 = 0.21;CL_a = 4.4;CL_de = 0.32;
% Pitch Coefficients
Cm_0 = 0;Cm_a = -1.0;Cm_de = -1.3;
%% geometry
c = 27.3 * 0.3048;
b = 196 * 0.3048;
S = 5500 * (0.3048)^2;
W = 636636 * 0.453592;

us = M*atm(4);
qdyn = 0.5*atm(3)*us^2;

T_max=275.8*1000;

T =4*T_max*M*cos(alpha)*T_lever;
D = qdyn * S * (CD0+CD_a*alpha);
L = qdyn * S * (CL0+CL_a*alpha+CL_de*de);
out(1) = T+L*sin(alpha)-D*cos(alpha)-W*9.81*sin(alpha);
out(2) = -L*cos(alpha)-D*sin(alpha)+W*9.81*cos(alpha);
out(3) = qdyn * S * c * (Cm_a*alpha+Cm_de*de);
end
