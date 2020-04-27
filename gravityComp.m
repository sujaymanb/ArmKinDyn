function FEstApp = gravityComp(gCompBool, mTool, g, Ftool, gSensor)
% gCompBool: gravity compensation turned on or off
% Ftool: 6x1 force vector
% gSensor: 4x4 position and orientation of sensor
% FEstApp: 6x1 estimate of applied force on tool after subtracting gravity

FEstApp = Ftool;
if gCompBool
    RotMat = inv(gSensor(1:3,1:3));
    gRot = RotMat*g;
    FEstA(1:3) = FEstA(1:3) - mTool.*gRot;
end
end