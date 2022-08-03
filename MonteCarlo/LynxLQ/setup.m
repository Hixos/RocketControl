clearvars -except p
clc

run_setup.N_sim = 12*30;
run_setup.enable_guidance = true;
run_setup.enable_drogue = true;
run_setup.enable_main = true;

run_setup.control_start_met = 1;
run_setup.control_stop_met = 10;

% Not working!
% azimuth = 0;
% elevation = 84;


sim_name = "LynxLQ";
exe_name = sprintf("%s.exe", sim_name);
xml_name = sprintf("%s_init.xml", sim_name);
output_name = sprintf("%s_res.mat", sim_name);
output_dir = "outputs";

params = {};

params{end+1}.name = "atmosphere.wind_direction[1]";
params{end}.range = deg2rad([0, 360]);
params{end}.distrib = "uniform";

params{end+1}.name = "atmosphere.wind_direction[2]";
params{end}.range = deg2rad([0, 360]);
params{end}.distrib = "uniform";

params{end+1}.name = "atmosphere.wind_direction[3]";
params{end}.range = deg2rad([0, 360]);
params{end}.distrib = "uniform";



params{end+1}.name = "atmosphere.wind_magnitude[1]";
params{end}.range = [0, 15];
params{end}.distrib = "uniform";

params{end+1}.name = "atmosphere.wind_magnitude[2]";
params{end}.range = [0, 30];
params{end}.distrib = "uniform";

params{end+1}.name = "atmosphere.wind_magnitude[3]";
params{end}.range = [0, 20];
params{end}.distrib = "uniform";



params{end+1}.name = "atmosphere.wind_layer_height[1]";
params{end}.range = [50, 500];
params{end}.distrib = "uniform";

params{end+1}.name = "atmosphere.wind_layer_height[2]";
params{end}.range = [50, 300];
params{end}.distrib = "uniform";

params{end+1}.name = "atmosphere.wind_layer_height[3]";
params{end}.range = [10000, 10000];
params{end}.distrib = "uniform";

params{end+1}.name = "globalSeed.fixedSeed";
params{end}.range = [-10000000, 10000000];
params{end}.distrib = "integer";

params{end+1}.name = "atmosphere.wind_severity_high";
params{end}.range = [0,0,0,1,1,1,2,2,3];
params{end}.distrib = "pick";

run_setup.params = params;
clearvars params

run_setup.filters = ["true_navigation.bus.x_est[\d+]"
            "true_navigation.bus.v_est[\d+]"
            "true_navigation.bus.w_est[\d+]"
            "true_navigation.bus.a_meas[\d+]"
            "atmosphere.turb_wind_speed.v_wind_world[\d+]"
            "atmosphere.turb_wind_speed.w_wind_world[\d+]"
            "atmosphere.turb_wind_speed.baseWindSpeed.v_wind[\d+]"
            "gnc.guidance_control.parabolic_traj.flightpathangle"
            "true_navigation.trueNavigation.flightpathangle.angle"
            "true_navigation.trueNavigation.track_unwrapped.angle[1]"
            "gnc.bus.control_position_meas[\d+]"
            "opt.guidance_disable_met"
            "opt.guidance_enable_met"
            "opt.guidance_disable"
            "atmosphere.wind_direction[\d]]"
            "atmosphere.wind_layer_height[\d]"
            "atmosphere.wind_magnitude[\d]"
            "atmosphere.wind_severity_high"
            "globalSeed.fixedSeed"
            "atmosphere.turb_wind_speed.v_turb_world[\d]"];

% Update sim options
opt.startTime = 0;
opt.stopTime = 300;
opt.stepSize = 0.01;
opt.tolerance = 1e-4;
opt.variableFilter = makefilter(run_setup.filters);

run_setup.opt = opt;
clearvars opt


function filter_str = makefilter(filters)
   filter_str = "";
   for i = 1:length(filters)
       str = strrep(filters(i), ".", "\.");
       str = strrep(str, "[", "\[");
       str = strrep(str, "]", "\]");
       
       filter_str = strcat(filter_str, "(", str, ")");
       if i < length(filters)
           filter_str = strcat(filter_str, "|");
       end
   end
end
