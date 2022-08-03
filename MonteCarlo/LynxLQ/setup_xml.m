function xml = setup_xml(xml_name, run_setup)
    xml = xml2struct(xml_name);

    xml = setsimoptions(xml, run_setup.opt);

    xml = setscalarparam(xml, "opt.guidance_disable", string(~run_setup.enable_guidance));
    xml = setscalarparam(xml, "opt.drogue_enable", string(run_setup.enable_drogue));
    xml = setscalarparam(xml, "opt.main_enable", string(run_setup.enable_main));

    xml = setscalarparam(xml, "opt.guidance_enable_met", run_setup.control_start_met);
    xml = setscalarparam(xml, "opt.guidance_disable_met", run_setup.control_stop_met);
    % xml = setscalarparam(xml, "opt.launch_azimuth", azimuth);
    % xml = setscalarparam(xml, "opt.launch_elevation", elevation);
end