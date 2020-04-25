% Created: 20200423
% Run this script to load system parameters into workspace

%% 7-DoF Barrett Arm (with second link mod)
% Position of Joints in 0 config, following drawing of Barret_WAM.pdf
% Second long link of arm has been made longer by 250mm
q = [220    140   346-186;
     220    140   346;
     220    140   346;
     220+45 140   346+550;
     220    140   346+550;
     220    140   346+1100;
     220    140   346+1160]; %[mm]

% Unit Vector along each axis of rotation
w = [0 0 1
     0 1 0
     0 0 1
     0 1 0
     0 0 1
     0 1 0
     0 0 1];
 
%% Position and orientation of FT-Sensor tool-mounting surface with robot in 0 config
%  (the other surface being attached to the end of the robot arm)
gSensor0 = [1 0 0 220
            0 1 0 140
            0 0 1 q(end,end)+30
            0 0 0 1]; %[mm]

%% Position and orientation of Tool Surface
toolSurfaceAngle0 = -60; %[deg]
gToolSurface0 = [cosd(toolSurfaceAngle0)  0  sind(toolSurfaceAngle0)  6.14+220
                 0                        1        0                  140
                 -sind(toolSurfaceAngle0) 0  cosd(toolSurfaceAngle0)  gSensor0(3,end)+157.64
                 0 0 0 1]; %[mm]

%% Tool mass and position of tool CG
gToolCG0 = [1 0 0 2.44+220
            0 1 0 140
            0 0 1 gSensor0(3,end)+54.75
            0 0 0 1]; %[mm]
        
%% Change log
% YYYYMMDD: <description_of_changes>