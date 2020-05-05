function Ffeedback = calcInteractForce(vel,bf,userBool)
% vel: 3x1 vector, translational position and velocity
% bf: damping constant
% userBool: model user hand if true, model env forces otherwise
% Ffeedback: 6x1 feedback wrench from environment/user (f only, tau tbd)

Ffeedback = zeros(6,1);

if userBool
    Ffeedback(1:3) = vel(1:3).*bf;
end

end