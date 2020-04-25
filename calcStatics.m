function [Ftool] = calcStatics(Fsensor, gSensor, gToolCG)
% Fsensor: 6x1 Reaction force vector measured by sensor (reaction force,
%          NOT force ON sensor
% gSensor: 4x4 current position and orientation of sensor face
% gToolCG: 4x4 current position and orientation of tool CG
% Ftool: 6x1 Total force vector at tool CG, Ftool = Fapp + mg

R = gToolCG(1:3,end) - gSensor(1:3,end);

Rhat = [    0  -R(3)  R(2);
         R(3)     0  -R(1);
        -R(2)   R(1)     0]; %[mm]
    
A = [eye(3) zeros(3,3);
     Rhat   eye(3)];
 
Ftool = linsolve(A,Fsensor);

end