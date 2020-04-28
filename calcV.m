%% calc V
function [v,error] = calcV(pd, qd, pc, qc, kp, ko)

    p_err = pd - pc;
    q_err = qd * (qc.conj);
    q_err_comp = q_err.compact';
    
    error = norm([p_err;q_err_comp]);
    
    v = [kp * p_err;
        ko * q_err_comp(1) * q_err_comp(2:end)];
end