function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 3.2.1 p constraint
    %gsh: 2 * n_seg * (n_order + 1) constraint.
    Aieq_p = zeros(2 * n_seg * (n_order + 1), n_all_poly);
    for i = 1 : n_seg * (n_order + 1)
        Aieq_p(i, i) = -1;
    end
    for i = 1 : n_seg * (n_order + 1)
        Aieq_p(i + n_seg * (n_order + 1), i) = 1;
    end

    bieq_p = zeros(2 * n_seg * (n_order + 1), 1);
    for i = 1 : n_seg * (n_order + 1) 
        row = floor((i - 1) / (n_order + 1)) + 1;
        bieq_p(i, 1) = -corridor_range(row, 1);
    end
    for i = 1 : n_seg * (n_order + 1) 
        row = floor((i - 1) / (n_order + 1)) + 1;
        bieq_p(i + n_seg * (n_order + 1), 1) = corridor_range(row, 2);
    end

    %#####################################################
    % STEP 3.2.2 v constraint   
    %gsh: 2 * n_seg * (n_order + 1) constraint. -v <= v_max; v <= v_max;
    Aieq_v = zeros(2 * n_seg * (n_order + 1), n_all_poly); 
    for i = 1 : n_seg * (n_order + 1)
        if (mod(i, n_order + 1) ~= 0)
            Aieq_v(i, i + 1) = n_order;
            Aieq_v(i, i) = -n_order;
        end
    end

    %-v = Ci+1 - Ci
    for i = 1 : n_seg * (n_order + 1)
        if (mod(i, n_order + 1) ~= 0)
            Aieq_v(i + n_seg * (n_order + 1), i + 1) = -n_order;
            Aieq_v(i + n_seg * (n_order + 1), i) = n_order;
        end
    end

    bieq_v = zeros(2 * n_seg * (n_order + 1), 1);
    for i = 1 : n_seg * (n_order + 1)
        if (mod(i, n_order + 1) ~= 0)
            bieq_v(i, 1) = v_max;
        end
    end

    for i = 1 : n_seg * (n_order + 1)
        if (mod(i, n_order + 1) ~= 0)
            bieq_v(i + n_seg * (n_order + 1), 1) = v_max;
        end
    end

    %#####################################################
    % STEP 3.2.3 a constraint   
    %gsh 2 * n_seg * (n_order + 1) constraint. -a <= a_max; a <= a_max;
    Aieq_a = zeros(2 * n_seg * (n_order + 1), n_all_poly);
    for i = 1 : n_seg * (n_order + 1)
        if (mod(i, n_order + 1) ~= 0) && (mod(i + 1, n_order + 1) ~= 0)
            Aieq_a(i, i) = n_order * (n_order - 1);
            Aieq_a(i, i + 1) = -2 * n_order * (n_order - 1);
            Aieq_a(i, i + 2) =  n_order * (n_order - 1);
        end
    end

    for i = 1 : n_seg * (n_order + 1)
        if (mod(i, n_order + 1) ~= 0) && (mod(i + 1, n_order + 1) ~= 0)
            Aieq_a(i + n_seg * (n_order + 1), i) = -n_order * (n_order - 1);
            Aieq_a(i + n_seg * (n_order + 1), i + 1) = 2 * n_order * (n_order - 1);
            Aieq_a(i + n_seg * (n_order + 1), i + 2) =  -n_order * (n_order - 1);
        end
    end

    bieq_a = zeros(2 * n_seg * (n_order + 1), 1);
    for i = 1 : n_seg * (n_order + 1)
        if (mod(i, n_order + 1) ~= 0) && (mod(i + 1, n_order + 1) ~= 0)
            bieq_a(i, 1) = a_max;
        end
    end

    for i = 1 : n_seg * (n_order + 1)
        if (mod(i, n_order + 1) ~= 0) && (mod(i + 1, n_order + 1) ~= 0)
            bieq_a(i + n_seg * (n_order + 1), 1) = a_max;
        end
    end

    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
end
