classdef average_wind_model < matlab.System
    %% Average wind model class
    properties
        k = 0.4 % karman constant
    end
    properties (Access = private)
        H0 % surface_roughnes [m]
        V0 % average wind at reference surface [m/s]
    end
%     properties (DiscreteState)
%         H        
%     end
    methods (Access = protected)
        function setupImpl(obj)
            %% Setup method for average wind model
            % Initialize properties of the method
            obj.V0 = rand*10;
            obj.H0 = rand/1000;
        end
        function wind = stepImpl(obj,altitude)
            %% Step method for average wind model
            % Compute average wind at indicated height
            %% Outputs
            %{
            wind -> average wind speed
            %}   
            wind = [obj.V0/obj.k * log(altitude/obj.H0) 0 0];
%             obj.H= obj.H+20;
        end
%         function resetImpl(obj)
%             obj.H=20;
%         end
    end
end
