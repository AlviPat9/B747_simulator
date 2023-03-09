function func = Interpolation(u)
%% 
func = zeros(2,1);

load('engine/Data.mat')
T_interp = scatteredInterpolant(engine(:,1), engine(:,2), engine(:,3));
% FF_interp = scatteredInterpolant(engine(:,1), engine(:,3), engine(:,4));
func(1) = T_interp(u(1), u(2));
% func(2) = FF_interp(u(1), func(1));

end