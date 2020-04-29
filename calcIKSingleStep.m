function theta = calcIKSingleStep(FEstApp,thetaInit,q,w,currPose)
% currPose: 4x4 gToolCG

pc = currPose(1:3,4);
qc = currPose(1:3,1:3);

force = FEstApp(1:3);
kp = norm(force).*1;
if norm(force) ~= 0
    force = force/norm(force);
end

torque = FEstApp(4:6);
kq = norm(torque)*1;
if norm(torque) ~= 0
    torque = torque/norm(torque);
end

velT = kp*force;

xsquat = quaternion(eul2quat([0 0 0]));
xd1quat = quaternion(eul2quat([0 0 0]));

xomega1 = xd1quat./xsquat;
[wd1, id1, jd1, kd1] = parts(xomega1);
id1 = sign(wd1)*id1;
jd1 = sign(wd1)*jd1;
kd1 = sign(wd1)*kd1;
V = [velT; id1; jd1; kd1];

twists = calcTwists(q,w);
step = 0.01;

J = jacobian(twists, thetaInit);
thetaDot = pinv(J)*V;
theta = thetaInit + thetaDot*step;

end