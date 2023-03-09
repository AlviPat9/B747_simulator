classdef wind_gust_model < matlab.System 
    % Wind gust model -> half-wavelength model -> only affects vertical speed
    properties(Access = private)
        Vg
        d_m
    end
    
    methods(Access = protected)
        function  setupImpl(obj)
            %% Constructor method for gust model
            % Generate random variable
            min_Vg = 0; % [m/s]
            max_Vg = 50; % [m/s]
            obj.Vg = (max_Vg - min_Vg)*rand + min_Vg;            
            min_thick = 2;
            max_thick = 3;
            obj.d_m = obj.Vg*((max_thick - min_thick)*rand + min_thick);
            
        end
        function wind = stepImpl(obj,aircraft_position)
            
            wind = obj.gust_speed(aircraft_position(3));
        end
    end
    methods
        function wind = gust_speed(obj,h)
            %% Computation of induced wind by the gust 
            if (h<0) || (h>obj.d_m)
                wind = [0 0 0];
            else
                wind = [0 0 (obj.Vg/2)*(1-cos(pi*h/obj.d_m))];
            end
        end
    end
end
