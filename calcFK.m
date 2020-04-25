function [gSensor, gToolSurface, gToolCG, jointPos] = calcFK(theta)
% theta: 7x1 joint angles
% gSensor: transform to obtain position and orientation of sensor face
% gToolSurface: transform to obtain position and orientation of tool face
% gToolCG: transform to obtain position and orientation of tool CG
% jointPos: 7x3 of x,y,z of joint positions (for animation)

    % Load system parameters fil
    run('loadSysParams.m')
    
    % temp
    axisR = [0,-90,0,-90,0,-90,0]';

    % fk
    g = eye(4);
    jointPos = nan(7,3);
    for i = 1:size(theta,2)
        twist = [cross(-w(i,:)',q(i,:)');w(i,:)'];
        g = g * expTwist(twist,theta(i));
        % joint positions
        g0 = [rotx(axisR(i)), q(i,:)';
            [0,0,0], 1];
        gJoint = g0 * g
        jointPos(i,:) = gJoint(1:3,4)';
    end
    gSensor = g * gSensor0;
    gToolSurface = g * gToolSurface0;
    gToolCG = g * gToolCG0;

end

%% FK for specific gst0

function g = FK(g0, twists, angles)
    g = expTwist(twists(:,1),angles(1));
    for i = 2:size(angles,2)
        g = g * expTwist(twists(:,i),angles(i));
    end
    g = g * g0;
end

%% calculate exponent of twists
function ex = expTwist(twist,theta)
    v = twist(1:3);
    omega = twist(4:6);
    
    if all(omega == 0)
        ex = [eye(3), v*theta;
              [0,0,0], 1];
    else
        ex = [rodrigues(omega,theta), (eye(3) - rodrigues(omega,theta))*(cross(omega,v))+(omega*omega'*v*theta);
              [0,0,0], 1];
    end   
end


%% rodrigues formula
function rod = rodrigues(omega,theta)
    omegaHat = hatMatrix(omega);
    rod = eye(3) + omegaHat*sin(theta) + omegaHat^2*(1-cos(theta));
end

%% hat operator (skew symmetric cross product matrix)
function hat = hatMatrix(w)
    hat = [ 0,   -w(3), w(2);
            w(3), 0,   -w(1);
           -w(2), w(1), 0  ];
end

