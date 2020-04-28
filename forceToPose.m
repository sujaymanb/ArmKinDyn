%% outputs desired pose for step
function newPose = forceToPose(currPose,Fapp)
    pc = currPose(1:3);
    qc = [currPose(7),currPose(4:6)'];
    
    force = Fapp(1:3);
    if norm(force) ~= 0
        force = force/norm(force);
    end
    
    torque = Fapp(4:6);
    if norm(torque) ~= 0
        torque = torque/norm(torque);
    end
    
    kp = 0.01;
    kq = 0.01;
    
    pd = pc + kp * force;
    qrot = eul2quat(kq * torque');
    qd = quatmultiply(qc,qrot);
    
    newPose = [pd;qd'];
end