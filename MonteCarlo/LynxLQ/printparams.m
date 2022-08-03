function printparams(param_vals)
fprintf("dir = {%.8f, %.8f, %.8f}\n", param_vals(1), param_vals(2), param_vals(3));
fprintf("mag = {%.8f, %.8f, %.8f}\n", param_vals(4), param_vals(5), param_vals(6));
fprintf("h = {%.8f, %.8f, %.8f}\n", param_vals(7), param_vals(8), param_vals(9));
fprintf("seed = %.0f\n", param_vals(10));
fprintf("sev = %.0f\n", param_vals(11));
end