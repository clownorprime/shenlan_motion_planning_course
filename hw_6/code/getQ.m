function Q = getQ(n_order, t)
    Q = zeros(n_order + 1, n_order + 1);
    for i = 1 : n_order + 1
        for j = 1 : n_order + 1
            if ((i >= 5) && (j >= 5))
                numerator = (i-1) * (i - 2) * (i - 3) * (i - 4) * (j - 1) * (j - 2) * (j - 3) * (j - 4);
                denomerator = i + j - 9;
                Q(i, j) = numerator / denomerator * power(t, denomerator);
            end
        end
    end
end
