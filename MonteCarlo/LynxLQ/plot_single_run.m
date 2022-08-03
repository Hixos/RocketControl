clearvars


close all
% folders = ["data/30_main_ol", "data/30_main_cl_1"];
folders = ["data/30_main_cl_1"];
% ids = [53,62,95];
ids = 1:20;%[14,25,44,55,66,77,214,123,319];

for j=1:length(folders)
    for i=1:length(ids)
        doPlots(folders(j), ids(i), i>1);
    end
end


function doPlots(folder, id, dohold)
if nargin < 3
    dohold = false;
end
res = loadsim(sprintf("%s/out_%d.mat", folder, id));
time = getvar("time", res);

t = linspace(min(time), max(time), 1000);
x = getvector(res, "true_navigation.bus.x_est", 3, time, t);
v = getvector(res, "true_navigation.bus.v_est", 3, time, t);
w = getvector(res, "true_navigation.bus.w_est", 3, time, t);
v_w = getvector(res, "atmosphere.turb_wind_speed.v_wind_world", 3, time, t);

cpos = getvector(res, "gnc.bus.control_position_meas", 4, time, t);
figure(1);
if dohold
    hold on
end
plot3(x(2,:), x(1,:), -x(3,:));
grid on
axis equal
title("Trajectory");

figure(2);
hold on
vn = vecnorm(v, 2);
plot(t, -rad2deg(asin(v(3,:)./vn)));
grid on
title("Flight Path Angle");

figure(3);
plot(t, rad2deg(w(1,:)));
plot(t, rad2deg(w(2,:)));
hold on
plot(t, rad2deg(w(3,:)));
grid on
title("Angular rates");

figure(4);
hold on
% plot(t, rad2deg(w(1,:)));
plot(x(2,:), x(1,:));
grid on
axis equal
title("Ground track");

figure(5);
hold on
plot(t, rad2deg(cpos));
grid on
title("Control position");

figure(6);
hold on
plot(t, -v(3,:));
grid on
title("Vertical speed");

figure(7);
[w_theta, w_rho] = cart2pol(v_w(1,:), v_w(2,:));
polarplot(w_theta, w_rho);
grid on
set(gca,'ThetaZeroLocation','top')
set(gca,'ThetaDir','clockwise');
title("Wind polar plot");
hold on

figure(8);
hold on
plot(t, rad2deg(atan2(v_w(2,:), v_w(1,:))));
grid on
title("Wind direction");

figure(9);
hold on
plot(t, vecnorm(v_w(1:2,:)));
grid on
title("Wind magnitude");

figure(10);
hold on
plot(t, -v_w(3,:));
grid on
title("Vertical wind");


% v_north = v_w(1,:)';
% v_east = v_w(2,:)';
% v_down = v_w(3,:)';
% t = t';
% 
% writetable(table(t, v_north, v_east, v_down), "wind.csv");

end