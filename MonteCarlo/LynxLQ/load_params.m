function [h, dir, mag] = loadparams(folder, i)
params = load(sprintf("%s/setup.mat", folder));

if nargin > 1
    dir = params.param_values(1:3, i);
    mag = params.param_values(4:6, i);
    h = params.param_values(7:9, i);
else
    error("Load all params not implemented");
end
end