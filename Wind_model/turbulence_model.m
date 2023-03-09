classdef turbulence_model < matlab.System & Aircraft_data
    %% Turbulence model class
    properties (Access = private)
        W20  
    end
    methods (Access = protected)
        function setupImpl(obj)
            %% Constructor method for turbulence model class
            obj.W20 = rand*45;
        end
        function [num,den] = stepImpl(obj,V,aircraft_position)
            %% Step method for turbulence class
            % Compute turbulence intensity & scale length
            h=aircraft_position(3)/0.3048;
            if h < 2000
                % turbulence scale
                Lw = h/2;
                Lu = (h)/(0.177 + 0.000823*h)^1.2;
                Lv = Lu/2;
                % turbulence intensity
                sigma_w = 0.1 * obj.W20;
                sigma_v = sigma_w * 1/((0.177 + 0.000823*h)^0.4);
                sigma_u=sigma_v;
            else
                % turbulence scale
                Lw = 1750/2;
                Lu = Lw*2;
                Lv = Lw;
                % turbulence intensity
                sigma_w = (0.1 * obj.W20)*(h/2000)*(5-80)/(23-5);
                sigma_v = sigma_w ;
                sigma_u=sigma_v;
            end
            num = [0, 0, sigma_u*sqrt(2*Lu/(pi^V));...
                0, sigma_v*sqrt(2*Lv/(pi^V))*sqrt(3)*2*Lv/V, sigma_v*sqrt(2*Lv/(pi^V));...
                0, sigma_w*sqrt(2*Lw/(pi^V))*sqrt(3)*2*Lw/V, sigma_w*sqrt(2*Lw/(pi^V))];
%                 0, 0, sigma_w*sqrt(0.8/V)*(pi/(4*b))^(1/6);...
%                 (sigma_w*sqrt(2*Lw/(pi^V))*sqrt(3)*2*Lw/V)/V, sigma_w*sqrt(2*Lw/(pi^V))/V, 0;...
%                 (sigma_v*sqrt(2*Lv/(pi^V))*sqrt(3)*2*Lv/V)/V, sigma_v*sqrt(2*Lv/(pi^V))/V, 0];
            
            den = [0, Lu/V, 1;...
                (2*Lv/V)^2, 4*Lv/V, 1;...
                (2*Lw/V)^2, 4*Lw/V, 1];
%                 0,((2*Lw)^(1/3))*4*b/(pi*V), (2*Lw)^(1/3);...
%                 0, 4*b/(pi*V), 1;...
%                 0, 3*b/(pi*V), 1];
        end
    end
end