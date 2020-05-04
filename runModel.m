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
mode = 8;
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
if ismember(mode, [2,4,5,6])
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
[gSensor, gToolSurface, gToolCG, jointPos] = calcFK(theta,q,w,gSensor0,gToolSurface0,gToolCG0);

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
    % Plot arm links and tool trajectory
    plot3(toolPos(1,:), toolPos(2,:), toolPos(3,:), 'k-', 'LineWidth', 1.5)
    hold on
    plot3(jointPos([1, 2, 4, 6, 7],1), jointPos([1, 2, 4, 6, 7],2), jointPos([1, 2, 4, 6, 7],3), 'bo-', 'LineWidth', 1.5)
    plot3([jointPos(7,1) toolPos(1,end)], [jointPos(7,2) toolPos(2,end)], [jointPos(7,3) toolPos(3,end)], 'g-', 'LineWidth', 5)
    % plot axis
    rotAxis = gToolSurface * axis;
    plot3([gToolSurface(1,end),rotAxis(1,1)],[gToolSurface(2,end),rotAxis(2,1)],[gToolSurface(3,end),rotAxis(3,1)], 'r-', 'LineWidth', 1.5)
    plot3([gToolSurface(1,end),rotAxis(1,2)],[gToolSurface(2,end),rotAxis(2,2)],[gToolSurface(3,end),rotAxis(3,2)], 'g-', 'LineWidth', 1.5)
    plot3([gToolSurface(1,end),rotAxis(1,3)],[gToolSurface(2,end),rotAxis(2,3)],[gToolSurface(3,end),rotAxis(3,3)], 'b-', 'LineWidth', 1.5)
    hold off
    legend('Tool Path', 'Arm Links', 'Tool')
    grid on
    xlim([-1, 1])
    ylim([-1, 1])
    zlim([0, 2])
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('Animated Robot Arm')
    pause(0.001)
    % Compute FEstApp from gravity compensator based on mode
    FEstApp = gravityComp(gravCompBool, mTool, g, Ftool);
    if axisGuidanceBool
        % cancel non-tool axis components of force with new function
        FEstApp = axisGuidance(FEstApp, gToolSurface);
    end
    disp([gToolSurface(1:3,3) FEstApp(1:3)/norm(FEstApp(1:3))])
    %disp(' Fapplied    FtoolSim    Fsensor    Ftool    FEstApp')
    %disp([Fapplied(:,t), FtoolSim(:,t), Fsensor, Ftool, FEstApp])
    % Obtain desired pose from FEstApp, perform IK, and obtain newTheta
    theta2 = calcIKSingleStep(FEstApp,theta,q,w,gToolCG,kp,kq);
    [gSensor, gToolSurface, gToolCG, jointPos] = calcFK(theta2,q,w,gSensor0,gToolSurface0,gToolCG0);
    theta = theta2;
end