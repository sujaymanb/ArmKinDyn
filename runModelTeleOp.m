% Spring 2020 16-711 KDC
% Project
clear
clc
close all

%% READ ME

%% Run mode
mode = 4;
% 4: External force on tool as pure translational force, gravity
%    compensation turned on
% 7: Random force without axis guidance, free environment
% 8: Random force with axis guidance, manipulate environment

%----------------------
% !!DO NOT EDIT BELOW!!
%----------------------

%% Load system parameters
run('loadSysParams.m')
kp = 0.2;
kq = 0.5;
T = 40;
maxF = 8; %[N]
maxTau = 10; %[Nm]

%% Load environment parameters
bfUser = 4;
bfEnv = 4;

%% Generate Fapplied vector (inertial frame), 6xN
Fapplied = genAppForce(mode,T,maxF,maxTau);
FtoolSim = Fapplied;
FtoolSim(1:3,:) = FtoolSim(1:3,:) + mTool.*g; % add force due to gravity

gravCompBool = true;

axisGuidanceBool = false;
if mode==8
    axisGuidanceBool = true;
end

%% Initialize system
theta0 = [30; -60; 0; 45; 0; -10; 0].*pi./180; %[radians], starting pose of robot
%theta0 = [0; 0; 0; 0; 0; 0; 0].*pi./180;
thetaMas = theta0; % track joint angles independently for plotting
thetaSlv = theta0;

masBool = true;
slvBool = false;
userBool = true;
envBool = false;

FEnv = zeros(6,1);
FUse = zeros(6,1); % feedback interaction forces from environment and user

toolPosMs = [];
toolPosSl = [];

figure('Renderer', 'painters', 'Position', [10 10 900 600])
hold off
axis = [0.1, 0, 0;
        0, 0.1, 0;
        0, 0, 0.1;
        1, 1, 1];

%% Iterate through time steps
for t = 1:size(FtoolSim,2)
    % Compute master sensor forces
    if t==1
        FsensorMas = calcInvStatics(FtoolSim(:,t)-FUse, gSensor0, gToolCG0);
    else
        FsensorMas = calcInvStatics(FtoolSim(:,t)-FUse, gSensorMs, gToolCGMs);
    end
    
    % Move slave arm
    [gSensorSl,gToolSurfaceSl,gToolCGSl,gToolTipSl,jointPosSl,velSl] = forceControlRobot(slvBool,...
        FsensorMas,thetaSlv,q,w,gSensor0,gToolCG0,gToolSurface0,gToolTip0,...
        mTool,g,kp,kq,axisGuidanceBool);
    
    % Simulate env interaction forces
    Fenv = calcInteractForce(velSl,bfEnv,envBool);
    FsensorSl = calcInvStatics(Fenv, gSensorSl, gToolTipSl);
    
    % Move master arm
    [gSensorMs,gToolSurfaceMs,gToolCGMs,gToolTipMs,jointPosMs,velMs] = forceControlRobot(masBool,...
        FsensorSl-FsensorMas, thetaMas,q,w,gSensor0,gToolCG0,gToolSurface0,gToolTip0,...
        mTool,g,kp,kq,axisGuidanceBool);
    
    toolPosMs = [toolPosMs gToolSurfaceMs(1:3,end)];
    toolPosSl = [toolPosSl gToolSurfaceSl(1:3,end)];
    rotAxisMs = gToolSurfaceMs * axis;
    rotAxisSl = gToolSurfaceSl * axis;
    
    subplot(1,2,1)
    animateArm(jointPosMs,toolPosMs,gToolSurfaceMs,rotAxisMs)
    title('Master Arm')
    
    subplot(1,2,2)
    animateArm(jointPosSl,toolPosSl,gToolSurfaceSl,rotAxisSl)
    title('Slave Arm')
    
    pause(0.025)
    
    % Simulate user interaction forces
    Fuse = calcInteractForce(velMs,bfUser,userBool);
    
end