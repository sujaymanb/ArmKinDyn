%% only move according to force in tool axis direction
function FEstApp = axisGuidance(F, gToolSurface)
% F: input force torque
% gToolSurface: tool surface transform
    force = dot(F(1:3),gToolSurface(1:3,3)) .* gToolSurface(1:3,3);
    torque = dot(F(4:6),gToolSurface(1:3,3)) .* gToolSurface(1:3,3);
    FEstApp = [force;torque];
end