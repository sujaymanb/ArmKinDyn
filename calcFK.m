function [gSensor, gToolSurface, gToolCG, jointPos] = calcFK(theta,q,w,gSensor0,gToolSurface0,gToolCG0)
% theta: 7x1 joint angles
% gSensor: transform to obtain position and orientation of sensor face
% gToolSurface: transform to obtain position and orientation of tool face
% gToolCG: transform to obtain position and orientation of tool CG
% jointPos: 7x3 of x,y,z of joint positions (for animation)
    
    % temp
    axisR = [0,-90,0,-90,0,-90,0]';

    % fk
    g = eye(4);
    jointPos = nan(7,3);
    for i = 1:size(theta,1)
        twist = [cross(-w(i,:)',q(i,:)');w(i,:)'];
        g = g * expTwist(twist,theta(i));
        % joint positions
        %rotx(axisR(i))
        g0 = [eye(3), q(i,:)';
            [0,0,0], 1];
        gJoint = g0 * g;
        jointPos(i,:) = gJoint(1:3,4)';
    end
    gSensor = g * gSensor0;
    gToolSurface = g * gToolSurface0;
    gToolCG = g * gToolCG0;

end

