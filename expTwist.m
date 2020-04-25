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

