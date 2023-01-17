function atm = atmospheric_model(altitude) 
    % Atmospheric model function. 
    % This class serves as a base class for all calculations done in the simulation.
    % It generates the ISA model variables (T, P, rho, speed of sounD) related to the position of the aircraft
            % Function to calculate the 
            % sea level conditions
    T_sl = 288.15; % [K]
    P_sl = 101325; % [Pa]
%     rho_sl = 1.225; % [kg/m^3]
    R = 287.053; % [J/(kgK)]
    gamma = 1.4;
    atm = zeros(4,1);
    h = altitude;
    if h<11000
        % Troposphere atmospheric parameters calculation
        atm(1) = T_sl - 6.5*h/1000;
        atm(2) = P_sl *(T_sl/(T_sl - 6.5*h/1000))^(34.1632/-6.5);
        atm(3) = atm(2)/(R*atm(1));
        atm(4) = sqrt(gamma*R*atm(1));
    elseif (11000<=h) && (h<20000)
        % Low stratosphere atmospheric parameters calculation
        atm(1) = 216.65;
        atm(2) = 22632.06*exp(-34.1632*(h-11000)/(1000*216.65));
        atm(3) = atm(2)/(R*atm(1));
        atm(4) = sqrt(gamma*R*atm(1));
    elseif (h>=20000) && (h<32000)
        % Mid stratosphere atmospheric parameters calculation
        atm(1) = 196.65 + h/1000;
        atm(2) = 5474.889*(216.65/(216.65+(h-20000)/1000))^34.1632;
        atm(3) = atm(2)/(R*atm(1));
        atm(4) = sqrt(gamma*R*atm(1));
    elseif (h>=32000) && (h<47000)
        % High stratosphere atmospheric parameters calculation
        atm(1) = 139.05 + 2.8*h/1000;
        atm(2) = 868.0187*(228.65/(228.65 + 2.8*(h-32000)/1000))^(34.1632/2.8);
        atm(3) = atm(2)/(R*atm(1));
        atm(4) = sqrt(gamma*R*atm(1));
    elseif (h>=47000) && (h<51000)
        % Low mesosphere atmospheric parametes calculation
        atm(1) = 270.65;
        atm(2) = 110.9063*exp(-34.1632*(h-47000)/(1000*270.65));
        atm(3) = atm(2)/(R*atm(1));
        atm(4) = sqrt(gamma*R*atm(1));
    elseif (h>=51000) && (h<71000)
        % Mid mesosphere atmospheric parameters calculation
        atm(1) = 413.45 - 2.8*h/1000;
        atm(2) = 66.93887*(270.65/(270.65-2.8*(h-51000)/1000))^(34.1632/-2.8);
        atm(3) = atm(2)/(R*atm(1));
        atm(4) = sqrt(gamma*R*atm(1));
    elseif (h>=71000) && (h<84852)
        % High mesosphere atmospheric parameters calculation
        atm(1) = 365.65 - 2.0*h/1000;
        atm(2) = 3.95642*(214.65/(214.65-2*(h-71000)/1000))^(34.1632/-2);
        atm(3) = atm(2)/(R*atm(1));
        atm(4) = sqrt(gamma*R*atm(1));
    end
end
