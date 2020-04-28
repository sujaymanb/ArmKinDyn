function theta = calcIK(pose,thetaInit,q,w,gSensor0,gToolSurface0,gToolCG0)
    % convert pose from homog transform 4x4 to translation and quat
    %pd = pose(1:3);
    %qd = quaternion([pose(7),pose(4:6)']);
    
    pd = pose(1:3,4);
    qd = quaternion(pose(1:3,1:3),'rotmat','point');
    
    error = 1;
    iter = 0;
    
    kp = 1;
    ko = 3;
    step = 0.002;
    
    thetaCur = thetaInit;
    
    thetas = nan(5000, 7);
    thetas(1,:) = thetaCur';
    
    twists = calcTwists(q,w);
    
    while((error > 1e-2) && (iter < 100))
       iter = iter + 1;
       [gSensor, gToolSurface, gToolCG, jointPos] = calcFK(thetaCur,q,w,gSensor0,gToolSurface0,gToolCG0);
      
       g = gToolSurface;
       pc = g(1:3,4);
       qc = quaternion(g(1:3,1:3),'rotmat','point');
       
       %fprintf('position: ');
       %fprintf('%g ',pc');
       %fprintf('%g ',qc.compact);
       %fprintf('\n');
       
       [v,error] = calcV(pd, qd, pc, qc, kp, ko);
       J = jacobian(twists, thetaCur);
       thetaDot = pinv(J)*v;
       thetaCur = thetaCur + thetaDot * step;
       
       thetas(iter+1,:) = thetaCur';
    end
    %fprintf('iterations: %g',iter);
    theta = thetaCur;
end