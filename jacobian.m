%% calc Jacobian
function J = jacobian(twists, angles)
    n = size(twists,2);
    J = nan(6,n);
    J(:,1) = twists(:,1);
    g = exp_twist(twists(:,1),angles(1));
    for i = 2:n
        J(:,i)=adjoint(g) * twists(:,i);
        g = g * expTwist(twists(:,i),angles(i));
    end
end