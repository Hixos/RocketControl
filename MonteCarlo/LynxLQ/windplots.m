close all
folder = "outputs";

setup = load(fullfile(folder, "setup.mat"));
[avgwind, avgbase, avgturb] = loadData(folder, setup.run_setup.N_sim);

figure
avgwindplot(avgbase, "Base wind");
figure
avgwindplot(avgturb, "Turb wind");
figure
avgwindplot(avgwind, "Total wind");

function avgwindplot(winds, tit_str)
totavg = mean(winds, 2);

compass(winds(1,:),winds(2,:));
hold on
compass(totavg(1),totavg(2), 'r');
camorbit(0,180)
camroll(90)
grid on
title(tit_str);
end

function [avgwinds, avgbase, avgturb] = loadData(output_dir, N_sim)

avgwinds = zeros(3,N_sim);
avgbase = zeros(3,N_sim);
avgturb = zeros(3,N_sim);

good_runs = [];
for i = 1:N_sim
    filename = fullfile(output_dir, sprintf("out_%d.mat", i));
    res = loadsim(filename);
    
%     time = getvar("time", res);
%     [t, u_t] = unique(time);
    
    x_est = getvector(res, "true_navigation.bus.x_est", 3);
    w_w = getvector(res, "atmosphere.turb_wind_speed.v_wind_world", 3);
    w_base = getvector(res, "atmosphere.turb_wind_speed.baseWindSpeed.v_wind", 3);
    w_turb = w_w - w_base;
    
    avgwinds(:,i) = mean(w_w,2);
    avgbase(:,i) = mean(w_base,2);
    avgturb(:,i) = mean(w_turb,2);
    
    if abs(x_est(3, end)) > 1
        warning("Final altitude of run '%s': %.2f", filename, -x_est(3, end));
    else
        good_runs = [good_runs; i];
    end
end
end