function FEstApp = gravityComp(gCompBool, mTool, g, Ftool)
% gCompBool: gravity compensation turned on or off
% Ftool: 6x1 force vector
% gSensor: 4x4 position and orientation of sensor
% FEstApp: 6x1 estimate of applied force on tool after subtracting gravity

FEstApp = Ftool;
if gCompBool
    FEstApp(1:3) = FEstApp(1:3) - mTool.*g;
end
end