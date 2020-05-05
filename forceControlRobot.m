function [gSensor,gToolSurface,gToolCG,gToolTip,jointPos,vel] = forceControlRobot(masterBool, Fsensor,theta,q,w,gSensor0,gToolCG0,gToolSurface0,gToolTip0,mTool,g,kp,kq,axisGuidanceBool)

% Obtain current FK
[gSensor, gToolSurface, gToolCG, gToolTip, jointPos] = calcFK(theta,q,w,gSensor0,gToolSurface0,gToolCG0,gToolTip0);

% Compute Ftool from Fsensor and FK
if masterBool
    Ftool = calcStatics(Fsensor, gSensor, gToolCG);
    vel = gToolCG(1:3,end);
else
    Ftool = calcStatics(Fsensor, gSensor, gToolTip);
    vel = gToolTip(1:3,end);
end

% Compute FEstApp from gravity compensator based on mode
gravCompBool = true;
FEstApp = gravityComp(gravCompBool, mTool, g, Ftool);
if axisGuidanceBool
    % cancel non-tool axis components of force with new function
    FEstApp = axisGuidance(FEstApp, gToolSurface);
end

% Obtain desired pose from FEstApp, perform IK, and obtain newTheta, and
% update FK with newTheta
theta2 = calcIKSingleStep(FEstApp,theta,q,w,gToolCG,kp,kq);
[gSensor, gToolSurface, gToolCG, gToolTip, jointPos] = calcFK(theta2,q,w,gSensor0,gToolSurface0,gToolCG0,gToolTip0);

if masterBool
    vel = gToolCG(1:3,end)-vel;
else
    vel = gToolTip(1:3,end)-vel;
end

end