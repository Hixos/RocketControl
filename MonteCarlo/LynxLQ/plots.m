%%
global savefig figset nfig
savefig = false;
nfig = 3;
close all
clc
format short g
conf = 0.95;

colors = ["plot_blue", "plot_red", "plot_green", "plot_green", "plot_lightblue"]';
%%
figset = 1;
e1 = doshit("data/30_ballistic_ol", 'b', conf, "Open Loop", 1);
e2 = doshit("data/30_ballistic_cl_1", 'r', conf, "t_0 = 1 s", 2);
e3 = doshit("data/30_ballistic_cl_35", 'g', conf, "t_0 = 3.5 s", 3);


e_str = "";
e_str = e_str + ellipse_latex(e1, colors(1), "Open loop") + newline;
e_str = e_str + ellipse_latex(e2, colors(2), "$t_0$ = 1 s") + newline;
e_str = e_str + ellipse_latex(e3, colors(3), "$t_0$ = 3.5 s") + newline;
clipboard("copy", e_str);
ball_str = e_str

figure((figset-1)*nfig + 1);
scatter(0,0,80, "ko");
legend("", "open_loop", "", "t_0 = 1 s", "", "t_0 = 4 s", "Launch pad");

return
%%

figset = 2;
e1 = doshit("data/30_drogue_ol", 'b', conf, "Open loop", 1);
e2 = doshit("data/30_drogue_cl_1", 'r', conf, "t_0 = 1 s", 2);

figure((figset-1)*nfig + 1);
scatter(0,0,80, "ko");
legend("", "open_loop", "", "t_0 = 1 s", "Launch pad");

e_str = "";
e_str = e_str + ellipse_latex(e1, colors(1), "Open loop") + newline;
e_str = e_str + ellipse_latex(e2, colors(2), "$t_0$ = 1 s") + newline;
clipboard("copy", e_str);
drogue_str = e_str


%%
figset = 3;
e1 = doshit("data/30_main_ol", 'b', conf, "Open loop", 1);
e2 = doshit("data/30_main_cl_1", 'r', conf, "t_0 = 1 s", 2);

figure((figset-1)*nfig + 1);
scatter(0,0,80, "ko");
legend("", "open_loop", "", "t_0 = 1 s", "Launch pad");

e_str = "";
e_str = e_str + ellipse_latex(e1, colors(1), "Open loop") + newline;
e_str = e_str + ellipse_latex(e2, colors(2), "$t_0$ = 1 s") + newline;
main_str = e_str

%%
function ellipse = doshit(folder, color, conf, legend_str, i)
global figset nfig
setup = load(fullfile(folder, "setup.mat"));
[impacts, apogees, downrange_apo, gr] = loadData(folder, setup.run_setup.N_sim);

figure((figset-1)*nfig + 1)
hold on
plotImpact(impacts(gr,:), color, conf, i);
axis equal
grid on
xlabel("East (m)");
ylabel("North (m)");

figure((figset-1)*nfig + 2)
hold on
scatter(downrange_apo(gr), apogees(gr), 'x', 'Displayname', legend_str);
grid on
legend;
xlabel("Downrange distance (m)");
ylabel("Altitude (m)");
[c, a, b, angle, R] = ellipse_data(impacts(gr,:), conf);
ellipse.cx = c(1);
ellipse.cy = c(2);
ellipse.a = a;
ellipse.b = b;
ellipse.angle = angle;
ellipse.R = R;
impacts = impacts(gr,:);
apogees =  apogees(gr);
downrange_apo = downrange_apo(gr);
tab = table(impacts, apogees, downrange_apo);
writetable(tab, sprintf("csv/%s.csv", folder));

disp(legend_str);
c = c
area = pi*a*b/1000/1000

figure((figset-1)*nfig + 3)
tiledlayout(1,3);
for i=1:3
    nexttile
    [U,V] = pol2cart(setup.param_values(0+i, gr), setup.param_values(3+i, gr));  
    compass(U,V);
    
camorbit(0,180)
camroll(90)
    grid on
end
end

function h = plotImpact(impacts, col, conf, i)
mean_impact = [mean(impacts(:,1)), mean(impacts(:,2))];
impact_C = cov(impacts(:,1) - mean_impact(1), impacts(:,2) - mean_impact(2));

scatter(impacts(:,2), impacts(:,1), strcat('x',col));
scatter(mean_impact(2), mean_impact(1), 60, '.', 'SeriesIndex', i);

% error_ellipse(adtranspose(impact_nc_C), flip(mean_impact_nc), 'conf', 0.997);
h = error_ellipse(adtranspose(impact_C), flip(mean_impact), 'conf', conf);
h.SeriesIndex = i;
end

function [impacts, apogees, downrange_apo, good_runs] = loadData(output_dir, N_sim)

heading_0 = deg2rad(130);
dir = -[cos(heading_0); sin(heading_0); 0];

impacts = zeros(N_sim, 2);
apogees = zeros(N_sim, 1);
downrange_apo = zeros(N_sim, 1);
N_step = 1000;

ground_tracks_x = zeros(N_sim, N_step);
ground_tracks_y = zeros(N_sim, N_step);

worst_impact = 0;
worst_dist = 0;

strange = false(N_sim,1);
good_runs = [];
for i = 1:N_sim
    filename = fullfile(output_dir, sprintf("out_%d.mat", i));
    res = loadsim(filename);
    
    time = getvar("time", res);
    [t, u_t] = unique(time);
    
    x_est = getvector(res, "true_navigation.bus.x_est", 3);
    
    impacts(i,:) = x_est(1:2, end);
    [apogees(i), i_apo] = max(-x_est(3,:));
    downrange_apo(i) = norm(x_est(1:2,i_apo));
    
    if abs(x_est(3, end)) > 1 || (downrange_apo(i) > 500 && apogees(i) > 1654)
        warning("Final altitude of run '%s': %.2f", filename, -x_est(3, end));
    else
%         if apogees(i) > 1612
        good_runs = [good_runs; i];
%          end
    end
    
    dist = dot(dir, [impacts(i,:)'; 0]);
    if worst_dist == 0 || dist > worst_dist
        worst_dist = dist;
        worst_impact = i;
    end
    ground_tracks_x(i, :) = interp1(t, x_est(1,u_t), linspace(t(1), t(end), N_step));
    ground_tracks_y(i, :) = interp1(t, x_est(2,u_t), linspace(t(1), t(end), N_step));
    
    if abs(x_est(2,end) - 492) < 5
        strange(i) = true;
    end
end
worst_impact = worst_impact
worst_dist = worst_dist
impacts(worst_impact,:)
end

function [c, a, b, angle, R] = ellipse_data(data, conf)
c = mean(data);
data = data - c;
C = cov(data);
[V, D] = eig(C);
sigma = sqrt(diag(D));
s = sqrt(chi2inv(conf, 2));
a = sigma(1)*s;
b = sigma(2)*s;
[~, imax] = max(diag(D));
angle = atan2(V(1,imax), V(2,imax));

R = V;
end

function str = ellipse_latex(ellipse, col, name)
str = "\filldraw[$color,fill opacity=0.2]"+newline+"(axis cs:$cy,$cx) ellipse [x radius=$b,y radius=$a, rotate around={$ang:(\Xmin,\Ymin)}];";
str_leg = "\addlegendentry{$name}";
str_cent = "\addplot[mark=+, mark size=4pt, $color] coordinates {($cy,$cx)} ;";
str = strrep(str, "$color", col);
str = strrep(str, "$ang", num2str(rad2deg(ellipse.angle)));
str = strrep(str, "$cx", num2str(ellipse.cx));
str = strrep(str, "$cy", num2str(ellipse.cy));
str = strrep(str, "$a", num2str(ellipse.a));
str = strrep(str, "$b", num2str(ellipse.b));

str_leg = strrep(str_leg, "$name", name);
str_cent = strrep(str_cent, "$cx", num2str(ellipse.cx));
str_cent = strrep(str_cent, "$cy", num2str(ellipse.cy));
str_cent = strrep(str_cent, "$color", col);

str = str + newline + str_cent + newline + str_leg + newline;
end

function drawEllipse(el)
el.a = el.a*0.99;
vx = linspace(-el.a, el.a, 1000);
vy = el.b*sqrt(1-(vx/el.a).^2);

P = [vx, flip(vx);
     vy, -vy];
 
for i=1:size(P, 2)
    P(:,i) = el.R*P(:,i) + [el.cx; el.cy];
end
 
plot(P(2,:), P(1,:));
end
