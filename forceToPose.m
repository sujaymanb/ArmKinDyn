%% outputs desired pose for step
function newPose = forceToPose(currPose,Fapp)
    pc = currPose(1:3,4);
    qc = currPose(1:3,1:3);
    
    force = Fapp(1:3);
    if norm(force) ~= 0
        force = force/norm(force);
    end
    
    torque = Fapp(4:6);
    if norm(torque) ~= 0
        torque = torque/norm(torque);
    end
    
    kp = 0.01;
    kq = 0.1;
    
    pd = pc + kp * force;
    qrot = eul2rotm(kq * torque');
    qd = qrot * qc
    
    newPose = [qd,pd;
               0,0,0,1];
end