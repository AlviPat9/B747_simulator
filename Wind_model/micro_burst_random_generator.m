classdef micro_burst_random_generator < matlab.System 
    properties(Access = private)
        ref_vortex
        Gamma
        r
        core_radius
    end
    methods (Access = protected)
         function setupImpl(obj) 
            %% Constructor method
            %% Microburst random seed generator
            %{
            This code creates random seed for microburst model. This code in run at the
            beginning of the simulation in order to stablish the location of the burst.
            It is modeled like a cilinder
            %}
            min_distance_x = 5; % [km] -> distance from the airport of departure
            max_distance_x = 200; % [km] -> distance from the airport of departure
            %Y
            min_distance_y = 5; % [km] -> distance from the airport of departure
            max_distance_y = 200; % [km] -> distance from the airport of departure
            %Z
            min_distance_z = 1; % [km] -> height from airport altitude
            max_distance_z = 10; % [km] -> height from airport altitude
            %R
            min_radius = 1; % [km] -> minimun radius for the vortex ring
            max_radius = 10; % [km] -> maximun radius for the vortex ring
            
            X = (max_distance_x - min_distance_x)*rand + min_distance_x;
            Y = (max_distance_y - min_distance_y)*rand + min_distance_y;
            Z = (max_distance_z - min_distance_z)*rand + min_distance_z;
            R = (max_radius - min_radius)*rand + min_radius;
            obj.ref_vortex = [X Y Z R];
            
            % Circulation
            min_circulation = 0;
            max_circulation = 500;
            obj.Gamma = (max_circulation - min_circulation)*rand + min_circulation;
            
            % Core radius
            min_core = 100/1000; % [km]
            max_core = 5000/1000; % [km]
            obj.core_radius = (max_core-min_core)*rand + min_core;
         end
        
         function wind = stepImpl(obj, aircraft_position)
             %% Step function calculation
             % Computes wind disturbance on each step of the simulation
             wind = obj.burst_effect(aircraft_position);
         end
    end
    methods      
        function wind = burst_effect(obj, aircraft_position)
            %% Burst Effect
            % first calculates the min distance to the primary ring -> then
            % computes the max_distance to the primary ring(delayed pi
            % radians) -> Does the same to the secondary ring
            % distance from vortex center ring to aircraft
            obj.r = obj.distance2center([aircraft_position(1),aircraft_position(2)],obj);
            if obj.r<1           
                wind(1) = 0;
                wind(2) = 0;
                wind(3) = obj.Gamma*obj.ref_vortex(4)^2/2 *((obj.ref_vortex(4)^2 + (aircraft_position(3)+ obj.ref_vortex(3))^2)^(-3/2)-...
                    (obj.ref_vortex(4)^2 + (aircraft_position(3)- obj.ref_vortex(3))^2)^(-3/2));
            else
                %% Calculation of phi for base case
                x0=0;
                up=2*pi;
                lb=0;
                [theta1,r1_p] = fmincon(@(theta)obj.min_distance(theta,aircraft_position, obj.ref_vortex)...
                    , x0,[],[],[],[],lb,up);
                r2_p = obj.min_distance((theta1+pi),aircraft_position,obj.ref_vortex);
                r1_s = obj.min_distance((theta1),aircraft_position,...
                    [obj.ref_vortex(1:2),-obj.ref_vortex(3),obj.ref_vortex(4)]);
                r2_s = obj.min_distance((theta1+pi),aircraft_position,...
                    [obj.ref_vortex(1:2),-obj.ref_vortex(3),obj.ref_vortex(4)]);
                phi_t = (obj.Gamma/(2*pi))*((r1_p + r2_p)*obj.eliptic_integral(r1_p,r2_p) -...
                    (r1_s + r2_s)*obj.eliptic_integral(r1_s,r2_s));
                %% Calculation of derivative
                phi_z = obj.derivative_calculation_z(obj,aircraft_position);
                phi_r = obj.derivative_calculation_r(obj,aircraft_position);
                wind(1) = ((aircraft_position(1)-obj.ref_vortex(1))/(obj.ref_vortex(4)^2))*...
                    (phi_z-phi_t)/0.05;
                wind(2) = ((aircraft_position(2)-obj.ref_vortex(2))/(obj.ref_vortex(4)^2))*...
                    (phi_z-phi_t)/0.05;
                wind(3) = (-1/obj.ref_vortex(4)) * (phi_r(2)-phi_t)/0.05;
                %% Calculate damping factor
                damping = obj.damping_factor(r1_p,obj.core_radius);
                
                % Apply damping factor
                wind = wind * damping;
            end
        end  
    end
    methods (Static)
        function distance = min_distance(theta,aircraft, ref)
            %% Computation of distance between aircraft & burst
            distance = sqrt((ref(1)+ref(4)*cos(theta)- aircraft(1))^2 ...
            + (ref(2)+ref(4)*sin(theta) - aircraft(2))^2 + (ref(3) - aircraft(3))^2);
        end
        function phi = derivative_calculation_z(obj,aircraft_position)
            %% Calculation of derivative 
            % required to compute Phi function
            aircraft_position = [aircraft_position(1:2) aircraft_position(3) + 0.05];
            x0=pi;
            up=2*pi;
            lb=0;
            % Z derivative
            [theta1,r1_p] = fmincon(@(theta)obj.min_distance(theta,aircraft_position, obj.ref_vortex)...
                , x0,[],[],[],[],lb,up);
            r2_p = obj.min_distance((theta1+pi),aircraft_position,obj.ref_vortex );
            r1_s = obj.min_distance((theta1),aircraft_position,...
                [obj.ref_vortex(1:2),-obj.ref_vortex(3),obj.ref_vortex(4)]);
            r2_s = obj.min_distance((theta1+pi),aircraft_position,...
                [obj.ref_vortex(1:2),-obj.ref_vortex(3),obj.ref_vortex(4)]);
            phi(1) = (obj.Gamma/(2*pi))*((r1_p + r2_p)*obj.eliptic_integral(r1_p,r2_p) -...
                (r1_s + r2_s)*obj.eliptic_integral(r1_s,r2_s)) ;% Primary ring + Secondary ring 
        end
        function phi = derivative_calculation_r(obj,aircraft_position)
            % r derivative
            lb = [aircraft_position(1)-0.2, aircraft_position(2)-0.2];
            ub = [aircraft_position(1)+0.2, aircraft_position(2)+0.2];
            x0 = [aircraft_position(1), aircraft_position(2)];
            optim = fmincon(@(x) obj.distance2center(x, obj),x0,[],[],[],[]...
                ,lb, ub, @(x) obj.restriction(x,obj));
            aircraft_position = [optim(1) optim(2) aircraft_position(3)];
            x0=0;
            up=2*pi;
            lb=0;
            [theta1,r1_p] = fmincon(@(theta)obj.min_distance(theta,aircraft_position, obj.ref_vortex)...
                , x0,[],[],[],[],lb,up);
            r2_p = obj.min_distance((theta1+pi),aircraft_position,obj.ref_vortex);
            r1_s = obj.min_distance((theta1),aircraft_position,...
                [obj.ref_vortex(1:2),-obj.ref_vortex(3), obj.ref_vortex(4)]);
            r2_s = obj.min_distance((theta1+pi),aircraft_position,...
                [obj.ref_vortex(1:2),-obj.ref_vortex(3), obj.ref_vortex(4)]);
            phi(2) = (obj.Gamma/(2*pi))*((r1_p + r2_p)*obj.eliptic_integral(r1_p,r2_p) -...
                (r1_s + r2_s)*obj.eliptic_integral(r1_s,r2_s)) ;% Primary ring + Secondary ring 
        end
        
        function [c, ceq] = restriction(x,obj)
            distance = sqrt((x(1) - obj.ref_vortex(1))^2 + (x(2) - obj.ref_vortex(2))^2 +...
                (aircraft_position(3) - obj.ref_vortex(3))^2);
            distance_2 = sqrt((aircraft_position(1) - obj.ref_vortex(1))^2 + (aircraft_position(2) - obj.ref_vortex(2))^2 +...
                (aircraft_position(3) - obj.ref_vortex(3))^2);
            ceq = (abs(distance - distance_2) + 0.005);
            c = [];
        end
        
        function distance = distance2center(x,obj)
            %% Distance to center of the ring
            distance = sqrt((x(1) - obj.ref_vortex(1))^2 + (x(2) - obj.ref_vortex(2))^2 +...
                (aircraft_position(3) - obj.ref_vortex(3))^2);
        end
        function result = eliptic_integral(r1, r2)
            %% Eliptic integral
            lambda = (r2-r1)/(r2+r1);
            result = 0.788*lambda^2 /(0.25+0.75*sqrt(1-lambda^2));
        end
        function damp = damping_factor(r1,core_radius)
            %% Computation of damping factor 
            % see reference -> https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7284558&tag=1
            damp = 1-exp((-4*r1/(core_radius/2))^2);
        end
       
    end
end