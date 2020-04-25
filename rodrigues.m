%% rodrigues formula
function rod = rodrigues(omega,theta)
    omegaHat = hatMatrix(omega);
    rod = eye(3) + omegaHat*sin(theta) + omegaHat^2*(1-cos(theta));
end
