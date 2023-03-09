function ydot = solver(y,actuator)
%% Definition of constants
g = 9.81;  % acceleration of gravity [m/s^2]
%% Geometry data
c = 27.3 * 0.3048;
b = 196 * 0.3048;
S = 5500 * (0.3048)^2;
%% Mass properties
% Inertia definition
Ix = 18200000 * 1.3558179619;
Iy = 33100000 * 1.3558179619; 
Iz = 11158.75 * 1.3558179619;
Ixz = 49700000 * 1.3558179619;
% Aircraft mass
m = 636636 * 0.453592; % [kg] -> assumed to be constant

%% Input definition
%% Pilot Inputs
de = actuator(1); % Elevator deflection [rad]
da = actuator(2); % Aileron deflection [rad]
dr = actuator(3); % Rudder deflection [rad]

%% Definition of variables
%% Angular speeds
p = y(1); % angular speed x-axis [rad/s]
q = y(2); % angular speed y-axis [rad/s]
r = y(3); % angular speed z-axis [rad/s]

%% Euler Angles
phi = y(4); % roll angle [rad]
theta = y(5); % pitch angle [rad]
psi = y(6); % yaw angle [rad]


%% Linear speed
u = y(7); % linear speed x-axis [m/s]
v = y(8); % linear speed y-axis [m/s]
w = y(9); % linear speed z-axis [m/s]

%% Position
xe = y(10);
ye = y(11);
ze = y(12);
%% 
V = norm([u,v,w]); % Airspeed [m/s]
alpha = atan2(w,u);
beta = asin(v,V);
atm = atmospheric_model(ze);
qdyn = 0.5 * atm(3) * V^2;

%% Aerodynamic Forces
%% Aerodynamic Coefficients
%% Longitudinal coefficients
% Drag Coefficients
CD0 = 0.0164; CD_a = -0.2;CD_q=-0.0;
% Lift Coefficients
CL0 = 0.21; CL_a = 4.4; CL_q = 6.6; CL_de = 0.32;
% Pitch Coefficients
Cm_0 = 0; Cm_a = -1.0; Cm_q = -20.5; Cm_de = -1.3;

%% Lateral-Directional coefficients
% Side force Coefficients
CY_b = -0.9; CY_p = 0; CY_r = 0.0; CY_da = 0;CY_dr = 0.120;
% Roll Coefficients;
Cl_b = -0.16; Cl_p = -0.34; Cl_r = 0.13; Cl_da = 0.013; Cl_dr = 0.008;
% Yaw Coefficients
Cn_b = 0.16; Cn_p = -0.026; Cn_r = -0.28; Cn_dr = 0.0018; Cn_da = 0.013;

%% Forces calculation
D = qdyn * S * (CD0+CD_a*alpha+CD_q*q*c/(2*V));
Fy = qdyn * S * (CY_b*beta+CY_p*p*b/(2*V)+CY_r*r*b/(2*V)+CY_da*da+CY_dr*dr);
L = qdyn * S * (CL0+CL_a*alpha+CL_q*q*c/(2*V)+CL_de*de);
Fx = (L*sin(alpha)-D*cos(alpha));
Fz = (-L*cos(alpha)-D*sin(alpha));

%% Torques Calculation
L = qdyn * S * b * (Cl_b*beta+Cl_p*p*b/(2*V)+Cl_r*r*b/(2*V)+Cl_da*da+Cl_dr*dr);
M = qdyn * S * c * (Cm_0+Cm_a*alpha+Cm_q*q*c/(2*V)+Cm_de*de);
N = qdyn * S * b * (Cn_b*beta+Cn_p*p*b/(2*V)+Cn_r*r*b/(2*V)+Cn_dr*dr+Cn_da*da);

    
%% Equations integration definition
%% Initialization of output vector
ydot = nan(12,1);
    
%% Angular accelerations computation
% x-axix and z-axix obtainded with matlab symbolic math toolbox
ydot(1) = (Iz*L + Ixz*N - Ixz^2*q*r - Iz^2*q*r - Ixz*Iy*p*q +Ixz*Iz*p*q+Ixz*Ix*p*q+ Iy*Iz*q*r)/(Ix*Iz-Ixz^2);

ydot(2) = (M - Ixz*p^2 + Ixz*r^2 - Ix*p*r + Iz*p*r)/Iy;

ydot(3) = (Ixz*L + Ix*N + Ix^2*p*q - Ix*Iy*p*q - Ixz^2*p*q - Ix*Ixz*q*r + Ixz*Iy*q*r - Ixz*Iz*q*r)/(Ix*Iz-Ixz^2);
    
%%  Computation of euler angle rates
euler = [[1, sin(phi)*tan(theta), cos(phi)*tan(theta)];...
         [0, cos(phi), -sin(phi)];...
         [0, sin(phi)*(1/cos(theta)), cos(phi)*(1/cos(theta))]] * ...
         [p; q; r];

ydot(4) = euler(1);
ydot(5) = euler(2);
ydot(6) = euler(3);

    
%% Computation of linear accelerations
ydot(7) = (Fx- m*q*w + m*r*v - g*m*sin(theta))/m;
ydot(8) = (Fy + m*p*w - m*r*u + g*m*cos(theta)*sin(phi))/m;
ydot(9) = (Fz - m*p*v + m*q*u + g*m*cos(theta)*cos(phi))/m;
    
%% Computation of body coordinates
Rot_matrix = [[cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-(cos(phi)*sin(psi)), cos(phi)*sin(theta)*cos(psi)+(sin(phi)*sin(psi))];
     [cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+(cos(phi)*cos(psi)), cos(phi)*sin(theta)*sin(psi)-(sin(phi)*cos(psi))];
     [-sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)]];

x = Rot_matrix * [u; v; w];

ydot(10) = x(1);
ydot(11) = x(2);
ydot(12) = x(3);

%% Output resize matrix
ydot = [ydot(1); ydot(2); ydot(3); ydot(4); ydot(5); ydot(6);...
        ydot(7); ydot(8); ydot(9); ydot(10); ydot(11); ydot(12)];
end