%%%%% Run model in parallel
% Setup
p = gcp;
fprintf("Number of parallel workers: %d\n", p.NumWorkers);

n_params = length(run_setup.params);

global progres_counter progress_step;
progres_counter = 0;
progress_step = 0;

global tasks;

tasks = zeros(p.NumWorkers, 1);

showProg = @(data) showProgress(run_setup.N_sim, data);

progress = parallel.pool.DataQueue;
taskinfo = parallel.pool.DataQueue;

afterEach(progress, showProg);
afterEach(taskinfo, @updateTaskInfo);
tic
% Init folders
parfor i = 1:p.NumWorkers
    dir = sprintf("worker_%02d", i);
    if not(isfolder(dir))
       mkdir(dir);
    end
    
    copyfile(exe_name, fullfile(dir, exe_name));
end




parfor i = 1:run_setup.N_sim
    task = getCurrentTask(); 
    
    if isempty(task)
        dir = sprintf("worker_%02d", 1);
    else
        dir = sprintf("worker_%02d", task.ID);
    end
    xml_str = xml_template;
    % Assign parameters
    for param_i=1:2
        for j = 1:n_params
            if param_i == 1
                val = param_values(j, i);
            else
                val = param_values_2(j, i);
                fprintf("Using second param set\n");
            end
            xml_str = strrep(xml_str, sprintf("$$$par%04d$$$",j), num2str(val));
        end

        fid = fopen(fullfile(dir, xml_name), 'wt');
        fprintf(fid, '%s\n', xml_str);
        fclose(fid);

        send(taskinfo, task.ID);
        cmd = sprintf("cd %s && %s > NUL", dir, exe_name);
    %     cmd = sprintf("cd %s && %s", dir, exe_name);

        system(cmd);
        try
            res = loadsim(fullfile(dir, output_name));
            z = getvar("true_navigation.bus.x_est[3]", res);
            if abs(z(end)) > 2
                pstr = "";
                for s=1:n_params
                    pstr = strcat(pstr, sprintf("%f, ", param_values(s, i)));
                end
                warning("Simulation failed: %d, iteration: %d, params: %s", i, param_i, pstr);
            else
                break;
            end  
        catch
            warning("Simulation failed: %d (runner: %d)", i, task.ID);
            delete(fullfile(dir, output_name));
        end
    end
    
    movefile(fullfile(dir, output_name), fullfile(output_dir, sprintf("out_%d.mat", i)));
    send(progress, i);
end
toc

function showProgress(n, ~)
global progres_counter progress_step;
progres_counter = progres_counter + 1;

step = 0.01;

if progres_counter/n > progress_step
    progress_step = progress_step + step;
    tnow = toc;
   fprintf("Running... %.0f %% (%.0f/%.0f), Elapsed: %.0f s, s/run = %.1f, remaining: %.0f s\n", ...
       progres_counter/n*100, progres_counter, n, tnow, tnow/progres_counter, tnow/progres_counter * (n - progres_counter));
end
printstuck();
end

function updateTaskInfo(id)
    global tasks;
    tasks(id) = toc;
end

function printstuck()
global tasks;
for i=1:length(tasks)
    t = toc;
    if t - tasks(i) > 400
        fprintf("---Task %d seems to be stuck\n", i);
        tasks(i) = t;
    end
end 
end