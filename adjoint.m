%% calc adjoint
function Adg = adjoint(g)
    R = g(1:3,1:3);
    p = g(1:3,4);
    Adg = [R, hatMatrix(p)*R;
           zeros(3,3), R];
end