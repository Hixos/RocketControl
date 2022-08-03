clearvars -except p
clc

run_folder = "data/30_drogue_ol";
run_id = 115;


sim_name = "LynxLQ";
exe_name = sprintf("%s.exe", sim_name);
xml_name = sprintf("%s_init.xml", sim_name);
output_name = sprintf("%s_res.mat", sim_name);
output_dir = "single_run";

load(fullfile(run_folder, "setup.mat"));

run_setup.N_sim = 1;
run_setup.opt.variableFilter = ".*";
param_values = param_values(:, run_id);
param_values(end-1) = param_values(end-1) + 1;
printparams(param_values);

xml = setup_xml(xml_name, run_setup);

n_params = length(run_setup.params);
for i = 1:n_params
    xml = setscalarparam(xml, run_setup.params{i}.name, sprintf("$$$par%04d$$$",i));
end

xml_template = struct2xml(xml);

if isfolder(output_dir)
   rmdir(output_dir, 's')
end
mkdir(output_dir);

save(fullfile(output_dir, "setup.mat"), "param_values", "run_setup");

addpath("../");
runmontecarlo