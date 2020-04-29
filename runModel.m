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
mode = 1;
% 1: No external force on tool, gravity compensation turned off
% 2: No external force on tool, gravity compensation turned on
% 3: External force on tool as pure translational force, gravity
%    compensation turned off
% 4: External force on tool as pure translational force, gravity
%    compensation turned on
% 5: External force on tool as pure rotational torque
% 6: General external force on tool

%% !!!DO NOT EDIT BELOW!!!
run('loadSysParams.m')

%% Generate Fapplied vector (inertial frame), 6xN
Fapplied = genAppForce(mode);
FtoolSim = Fapplied;
FtoolSim(1:3,:) = FtoolSim(1:3,:) - mTool.*g; % add force due to gravity

gravCompBool = false;
if ismember(mode, [2,4,5,6])
    gravCompBool = true;
end

%% Initialize system
theta0 = [30; 30; 0; 60; 0; 45; 0].*pi./180; %[radians], starting pose of robot
theta = theta0; % track joint angles independently for plotting

%% Iterate through time steps
% Initialize animation figure
figure()
hold off

[gSensor, gToolSurface, gToolCG, jointPos] = calcFK(theta,q,w,gSensor0,gToolSurface0,gToolCG0);

toolPos = [];
for t = 1:size(FtoolSim,2)
    % Simulator:
    % Compute FK and simulated Fsensor
    toolPos = [toolPos gToolCG(1:3,end)];
    Fsensor = calcInvStatics(FtoolSim(:,t), gSensor, gToolCG);
    
    % Controller:
    % Compute Ftool from Fsensor and FK
    Ftool = calcStatics(Fsensor, gSensor, gToolCG);
    % Plot arm links and tool trajectory
%     animateArm(jointPos)
    plot3(jointPos([1, 2, 4, 6, 7],1), jointPos([1, 2, 4, 6, 7],2), jointPos([1, 2, 4, 6, 7],3), 'o-', 'LineWidth', 1.5)
    grid on
    xlim([-1, 2])
    ylim([-1, 2])
    zlim([-1, 4.5])
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('Animated Robot Arm')
    pause(0.2)
    % Compute FEstApp from gravity compensator based on mode
    FEstApp = gravityComp(gravCompBool, mTool, g, Ftool, gSensor);
    % Obtain desired pose from FEstApp
%     desiredPose = forceToPose(gToolSurface, FEstApp);
    % Perform IK and obtain newTheta
%     theta2 = calcIK(desiredPose,theta,q,w,gSensor0,gToolSurface0,gToolCG0);
    theta2 = calcIKSingleStep(FEstApp,theta,q,w,gToolCG);
    disp((theta2-theta)*180/pi)
    [gSensor, gToolSurface, gToolCG, jointPos] = calcFK(theta2,q,w,gSensor,gToolSurface,gToolCG);
    theta = theta2;
end

hold on
plot3(toolPos(1,:), toolPos(2,:), toolPos(3,:), '--', 'LineWidth', 1.5)
legend('Arm Links', 'Tool Path')