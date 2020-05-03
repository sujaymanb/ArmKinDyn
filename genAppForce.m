function Fapplied = genAppForce(mode,T,maxF,maxTau)
% mode: scalar [1,6]
% 1: No external force on tool, gravity compensation turned off
% 2: No external force on tool, gravity compensation turned on
% 3: External force on tool as pure translational force, gravity
%    compensation turned off
% 4: External force on tool as pure translational force, gravity
%    compensation turned on
% 5: External force on tool as pure rotational torque
% 6: General external force on tool
% Fapplied: 6xT array of applied forces, T=10000, to be tuned


if mode==1 || mode==2
    Fapplied = zeros(6,T);
elseif mode==3 || mode==4
    tau = zeros(3,T);
    f = [ ones(1,T/4),  ones(1,T/4),    zeros(1,T/4),  zeros(1,T/4);
          ones(1,T/4),  ones(1,T/4),   -ones(1,T/4),  -ones(1,T/4);
          zeros(1,T/4), zeros(1,T/4),   -ones(1,T/4),  -ones(1,T/4)];
    f = maxF.*f;
    Fapplied = [f; tau];
elseif mode==5
    f = zeros(3,T);
    tau = [ ones(1,T/4),  ones(1,T/4),  -ones(1,T/4),  -ones(1,T/4);
            ones(1,T/4), -ones(1,T/4),   ones(1,T/4),  -ones(1,T/4);
           -ones(1,T/4), -ones(1,T/4),   ones(1,T/4),   ones(1,T/4)];
    tau = maxTau.*tau;
    Fapplied = [f; tau];
elseif mode==6
    theta = linspace(0,720,T).*pi/180;
    f = [ ones(1,T).*cos(theta);
          ones(1,T).*sin(theta);
          -ones(1,T).*0.1];
    tau = zeros(3,T);
    f = maxF.*f;
    Fapplied = [f; tau];
elseif mode==7 || mode==8
    tau = zeros(3,T);
    f = -1 + (1+1).*rand(3,T);
    f = maxF.*f;
    Fapplied = [f; tau];
else
    print('Please enter a valid value for mode')
    Fapplied = Nan;
end        

end