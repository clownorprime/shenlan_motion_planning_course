function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = zeros(3, n_all_poly);
    %gsh: p constraint. C1,0 = start_cond(0);
    Aeq_start(1, 1) = 1;   
    %gsh: v constraint. C1,1 - C1,0 = 0;
    Aeq_start(2, 2) = 1; Aeq_start(2, 1) = -1; 
    %gsh: a constraint. C1,2 + C1,0 - 2 * C1,1 = 0
    Aeq_start(3, 1) = 1; Aeq_start(3, 2) = -2; Aeq_start(3, 3) = 1; 
    beq_start = start_cond';
    
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = zeros(3, n_all_poly);
    %gsh: p constraint, Cn,s = end_cond(0)
    Aeq_end(1, end) = 1;
    %gsh: v constraint, n(Cn,s - Cn,s-1) = 0
    Aeq_end(2, end) = 1; Aeq_end(2, end - 1) = -1;
    %gsh: a constaint, Cn,1 - 2 * Cn,0 + Cn,s = 0;
    Aeq_end(3, end) = 1; Aeq_end(3, end - 1) = -2; Aeq_end(3, end - 2) = 1;
    beq_end = end_cond';
    
    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments
    %gsh: n_seg - 1 constraint.
    Aeq_con_p = zeros(n_seg - 1, n_all_poly);
    for i = 1 : n_seg - 1
        Aeq_con_p(i, i * (n_order + 1)) = 1;
        Aeq_con_p(i, i * (n_order + 1) + 1) = -1;
    end
    beq_con_p = zeros(n_seg - 1, 1);

    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments
    %gsh: n_seg - 1 constraint. Ci,n - Ci,n-1 - (Ci+1,1 - Ci+1,0) = 0;
    Aeq_con_v = zeros(n_seg - 1, n_all_poly);
    for i = 1 : n_seg - 1
        Aeq_con_v(i, i * (n_order + 1)) = 1;
        Aeq_con_v(i, i * (n_order + 1) - 1) = -1;
        Aeq_con_v(i, i * (n_order + 1) + 1) = 1;
        Aeq_con_v(i, i * (n_order + 1) + 2) = -1;
    end
    beq_con_v = zeros(n_seg - 1, 1);

    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments
    %gsh: Ci,n+1 + Ci,0 - 2*Ci,n - (Ci+1,2 - 2*Ci+1,1 + Ci+1,0) = 0;
    Aeq_con_a = zeros(n_seg - 1, n_all_poly);
    for i = 1 : n_seg - 1
        Aeq_con_a(i, i * (n_order + 1) - 2) = 1;
        Aeq_con_a(i, i * (n_order + 1) - 1) = -2;
        Aeq_con_a(i, i * (n_order + 1)) = 1;
        Aeq_con_a(i, i * (n_order + 1) + 3) = -1;
        Aeq_con_a(i, i * (n_order + 1) + 2) = 2;
        Aeq_con_a(i, i * (n_order + 1) + 1) = -1;
    end
    beq_con_a = zeros(n_seg - 1, 1);

    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end
