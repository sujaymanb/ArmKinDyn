%% only move according to force in tool axis direction
function FEstApp = axisGuidance(F, gToolSurface)
% F: input force
% gToolSurface: tool surface transform

    % transform F to ToolSurface frame
    forceToolFrame = gToolSurface * [F(1:3);1];
    torqueToolFrame = gToolSurface * [F(4:6);1];
    
    % consider only components in tool direction
    forceToolFrame(2:3) = 0;
    torqueToolFrame(2:3) = 0;
    
    % transform back to world frame
    force = inv(gToolSurface) * forceToolFrame;
    torque = inv(gToolSurface) * torqueToolFrame;
    FEstApp = [force(1:3);torque(1:3)];
end