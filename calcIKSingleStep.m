function theta = calcIKSingleStep(FEstApp,thetaInit,q,w,currPose,kp,kq)
% currPose: 4x4 gToolCG

% Direction and magnitude of force
force = FEstApp(1:3);
kp = norm(force).*kp;
if norm(force) ~= 0
    force = force/norm(force);
end

% Direction and magnitude of torque
torque = FEstApp(4:6);
kq = norm(torque)*kq;
if norm(torque) ~= 0
    torque = torque/norm(torque);
end

% Translational velocity using force
velT = kp*force;

% Rotational velocity using torque
% xsquat = quaternion(eul2quat([0 0 0]));
% xd1quat = quaternion(eul2quat(torque'));
% xomega1 = xd1quat./xsquat;
% [wd1, id1, jd1, kd1] = parts(xomega1);
% id1 = sign(wd1)*id1;
% jd1 = sign(wd1)*jd1;
% kd1 = sign(wd1)*kd1;
% omega = kq.*[id1; jd1; kd1];
omega = kq.*torque;

V = [velT; omega];
disp('Wrench       Velocity')
disp([[force;torque] V])

% Jacobian of arm
twists = calcTwists(q,w);
J = jacobian(twists, thetaInit);

% Step in direction of end effector velocity
step = 0.01;
thetaDot = pinv(J)*V;
theta = thetaInit + thetaDot*step;

end