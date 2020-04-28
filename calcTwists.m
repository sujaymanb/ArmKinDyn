%% calc twists as columns of matrix
function twists = calcTwists(q,w)
    n = size(w,1);
    twists = nan(6,n);
    for i=1:n
        twists(:,i) = [cross(-w(i)',q(i)');w(i)'];
    end
end