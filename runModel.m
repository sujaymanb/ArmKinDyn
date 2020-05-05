% Spring 2020 16-711 KDC
% Project
clear
clc
close all

%% README
% Run this script to run a simulation of the model
% The model can be run in different modes, by editing "mode" variable in
% the line below.

%% Run mode
mode = 4;
% 1: No external force on tool, gravity compensation turned off
% 2: No external force on tool, gravity compensation turned on
% 3: External force on tool as pure translational force, gravity
%    compensation turned off
% 4: External force on tool as pure translational force, gravity
%    compensation turned on
% 5: External force on tool as pure rotational torque
% 6: General external force on tool
% 7: Random force without axis guidance
% 8: Random force with axis guidance

%% !!!DO NOT EDIT BELOW!!!
run('loadSysParams.m')
kp = 0.2;
kq = 0.5;
T = 40;
maxF = 8; %[N]
maxTau = 10; %[Nm]

%% Generate Fapplied vector (inertial frame), 6xN
Fapplied = genAppForce(mode,T,maxF,maxTau);
FtoolSim = Fapplied;
FtoolSim(1:3,:) = FtoolSim(1:3,:) + mTool.*g; % add force due to gravity

gravCompBool = false;
if ismember(mode, [2,4,5,6,8])
    gravCompBool = true;
end

axisGuidanceBool = false;
if mode==8
    axisGuidanceBool = true;
end

%% Initialize system
theta0 = [30; -60; 0; 45; 0; -10; 0].*pi./180; %[radians], starting pose of robot
%theta0 = [0; 0; 0; 0; 0; 0; 0].*pi./180;
theta = theta0; % track joint angles independently for plotting

%% Iterate through time steps
[gSensor, gToolSurface, gToolCG, ~, jointPos] = calcFK(theta,q,w,gSensor0,gToolSurface0,gToolCG0,gToolTip0);

% % ----debug force estimation
% Fsensor = calcInvStatics(FtoolSim(:,1), gSensor0, gToolCG0);
% Ftool = calcStatics(Fsensor, gSensor0, gToolCG0);
% FEstApp = gravityComp(gravCompBool, mTool, g, Ftool, gSensor0);
% disp(' Fapplied    FtoolSim    Fsensor    Ftool    FEstApp')
% disp([Fapplied(:,1), FtoolSim(:,1), Fsensor, Ftool, FEstApp])
% % --------------------

% Initialize animation figure
figure('Renderer', 'painters', 'Position', [10 10 900 600])
hold off
toolPos = [];
axis = [0.1, 0, 0;
        0, 0.1, 0;
        0, 0, 0.1;
        1, 1, 1];
for t = 1:size(FtoolSim,2)
    % Simulator:
    % Compute FK and simulated Fsensor
    toolPos = [toolPos gToolSurface(1:3,end)];
    Fsensor = calcInvStatics(FtoolSim(:,t), gSensor, gToolCG);
    
    % Controller:
    % Compute Ftool from Fsensor and FK
    Ftool = calcStatics(Fsensor, gSensor, gToolCG);
    
    rotAxis = gToolSurface * axis;
    % Plot arm links and tool trajectory
    animateArm(jointPos,toolPos,gToolSurface,rotAxis)
    title('Animated Robot Arm')
    pause(0.001)
    
    % Compute FEstApp from gravity compensator based on mode
    FEstApp = gravityComp(gravCompBool, mTool, g, Ftool);
    if axisGuidanceBool
        % cancel non-tool axis components of force with new function
        FEstApp = axisGuidance(FEstApp, gToolSurface);
    end
    %disp([gToolSurface(1:3,3) FEstApp(1:3)/norm(FEstApp(1:3))])
    %disp(' Fapplied    FtoolSim    Fsensor    Ftool    FEstApp')
    %disp([Fapplied(:,t), FtoolSim(:,t), Fsensor, Ftool, FEstApp])
    % Obtain desired pose from FEstApp, perform IK, and obtain newTheta
    theta2 = calcIKSingleStep(FEstApp,theta,q,w,gToolCG,kp,kq);
    [gSensor, gToolSurface, gToolCG, ~, jointPos] = calcFK(theta2,q,w,gSensor0,gToolSurface0,gToolCG0,gToolTip0);
    theta = theta2;
end