function RMSErr = testModelTeleOp(slGain)

%% Run mode
mode = 9;
% gravity compensation turned on for all modes
% 4: External force on tool as pure translational force
% 7: Random force without axis guidance, free environment
% 8: Random force with axis guidance, manipulate environment
% 9: External force on tool as pure translational force in single straight
%    line, axis guidance is turned on midway through the simulation

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
bfUser = 5;
bfEnv = 20;

%% Generate Fapplied vector (inertial frame), 6xN
Fapplied = genAppForce(mode,T,maxF,maxTau);
FtoolSim = Fapplied;
FtoolSim(1:3,:) = FtoolSim(1:3,:) + mTool.*g; % add force due to gravity

gravCompBool = true;

axisGuidanceBool = false;
if mode==4
    axisGuidanceBool = true;
end

%% Initialize system
theta0 = [30; -60; 0; 60; 0; -10; 60].*pi./180; %[radians], starting pose of robot
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

figure('Renderer', 'painters', 'Position', [80 80 1800 600])
sgtitle(['Simulating Tele-Operation; EnvFeedbackGain = ' num2str(slGain)])
hold off
axis = [0.1, 0, 0;
        0, 0.1, 0;
        0, 0, 0.1;
        1, 1, 1];
    
RMSErr = [];

%% Iterate through time steps
for t = 1:size(FtoolSim,2)
    if mode==9 && t>=T/2
        axisGuidanceBool=true;
        if t>=0.75*T
            envBool = true;
        end
    else
        envBool = false;
    end
    
    % Compute master sensor forces
    if t==1
        FsensorMas = calcInvStatics(FtoolSim(:,t)+FUse, gSensor0, gToolCG0);
    else
        FsensorMas = calcInvStatics(FtoolSim(:,t)+FUse, gSensorMs, gToolCGMs);
    end
    
%     if t==1
%         FsensorMas = calcInvStatics(FtoolSim(:,t), gSensor0, gToolCG0);
%     else
%         FsensorMas = calcInvStatics(FtoolSim(:,t), gSensorMs, gToolCGMs);
%     end
    
    % Move slave arm
    [gSensorSl,gToolSurfaceSl,gToolCGSl,gToolTipSl,jointPosSl,velSl,thetaSlv2] = forceControlRobot(slvBool,...
        FsensorMas,thetaSlv,q,w,gSensor0,gToolCG0,gToolSurface0,gToolTip0,...
        mTool,g,kp,kq,axisGuidanceBool);
    
    % Simulate env interaction forces
    Fenv = calcInteractForce(velSl,bfEnv,envBool);
    FsensorSl = slGain.*calcInvStatics(Fenv, gSensorSl, gToolTipSl);
    
    % Move master arm
    [gSensorMs,gToolSurfaceMs,gToolCGMs,gToolTipMs,jointPosMs,velMs,thetaMas2] = forceControlRobot(masBool,...
        FsensorMas+FsensorSl, thetaMas,q,w,gSensor0,gToolCG0,gToolSurface0,gToolTip0,...
        mTool,g,kp,kq,axisGuidanceBool);
    
    % Simulate user interaction forces
    Fuse = calcInteractForce(velMs,bfUser,userBool);
    
    thetaMas = thetaMas2;
    thetaSlv = thetaSlv2;
    
    disp('FsensorMas      Fenv      FsensorSl      Fuse')
    disp([FsensorMas,Fenv,FsensorSl,Fuse])
    
    toolPosMs = [toolPosMs gToolSurfaceMs(1:3,end)];
    toolPosSl = [toolPosSl gToolSurfaceSl(1:3,end)];
    rotAxisMs = gToolSurfaceMs * axis;
    rotAxisSl = gToolSurfaceSl * axis;
    
    if mod(t,4) == 0
        subplot(1,2,1)
        animateArm(jointPosMs,toolPosMs,gToolSurfaceMs,rotAxisMs)
        title('Master Arm')

        subplot(1,2,2)
        animateArm(jointPosSl,toolPosSl,gToolSurfaceSl,rotAxisSl)
        title('Slave Arm')

        pause(0.025)
    end
    
    RMSErr = [RMSErr norm(gToolCGMs(1:3,end) - gToolCGSl(1:3,end))];
    
end

end