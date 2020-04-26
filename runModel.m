% Spring 2020 16-711 KDC
% Project

%% README
% Run this script to run a simulation of the model
% The model can be run in different modes, by editing "mode" variable in
% the line below.

%% Run mode
mode = 1;
% 1: No external force on tool, gravity compensation turned off
% 2: No external force on tool, gravity compensation turned on
% 3: External force on tool as pure translational force
% 4: External force on tool as pure rotational torque
% 5: General external force on tool

%% DO NOT EDIT BELOW

%% Generate Fapplied vector (inertial frame), 6xN
Fapplied = [];

%% Initialize system
% Set starting joint angles
% Initialize joint angles variable to plot

%% Iterate through time steps

for t = 1:size(Fapplied,2)
    % Simulator:
    % Compute FK and simulated Fsensor
    
    % Controller:
    % Compute Ftool from Fsensor and FK
    % Plot arm links and tool trajectory
    % Compute FEstApp
    % Perform IK and obtain newTheta
end