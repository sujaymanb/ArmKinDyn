%% hat operator (skew symmetric cross product matrix)
function hat = hatMatrix(w)
% w: vector to convert into skew symmetric matrix form
    hat = [ 0,   -w(3), w(2);
            w(3), 0,   -w(1);
           -w(2), w(1), 0  ];
end
