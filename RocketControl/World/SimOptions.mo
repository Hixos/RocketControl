within RocketControl.World;

model SimOptions
  parameter Integer samplePeriodMs = 20;
  parameter SI.Angle launch_azimuth = from_deg(10);
  parameter SI.Angle launch_elevation = from_deg(84);
  parameter SI.Angle roll_target_heading = from_deg(90);
  parameter Boolean guidance_disable = false;
  parameter Boolean drogue_enable = false;
  parameter Boolean main_enable = false;
  parameter Integer global_seed = 1234;
  
  parameter SI.Duration guidance_enable_met = 1;
  parameter SI.Time guidance_disable_met = 10;
  
  annotation(defaultComponentPrefixes="inner", defaultComponentName = "opt",
    Icon(graphics = {Ellipse(fillColor = {255, 164, 250}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-5, 10}, extent = {{-81, 56}, {81, -56}}, textString = "opt")}));
end SimOptions;
