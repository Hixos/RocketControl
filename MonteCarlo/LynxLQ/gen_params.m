function [param_values] = gen_params(n_params,run_setup)
    param_values = zeros(n_params, 1);
    for j = 1:n_params
        if strcmp(run_setup.params{j}.distrib, "pick")
            val = randsample(run_setup.params{j}.range, 1);
        else
            val = rand() * (run_setup.params{j}.range(2) - run_setup.params{j}.range(1)) + run_setup.params{j}.range(1);
            if strcmp(run_setup.params{j}.distrib, "integer")
                val = round(val);
            end
        end
        param_values(j) = val;
    end
end

