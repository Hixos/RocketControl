clearvars -except p
setup

xml = setup_xml(xml_name, run_setup);

%%
n_params = length(run_setup.params);
for i = 1:n_params
    xml = setscalarparam(xml, run_setup.params{i}.name, sprintf("$$$par%04d$$$",i));
end

xml_template = struct2xml(xml);

param_values = zeros(n_params, run_setup.N_sim);
param_values_2 = zeros(n_params, run_setup.N_sim);
for i = 1:run_setup.N_sim
    param_values(:, i) = gen_params(n_params, run_setup);
    param_values_2(:, i) = gen_params(n_params, run_setup);
end


%%
if isfolder(output_dir)
   rmdir(output_dir, 's')
end
mkdir(output_dir);

save(fullfile(output_dir, "setup.mat"), "param_values", "run_setup");

addpath("../");
runmontecarlo