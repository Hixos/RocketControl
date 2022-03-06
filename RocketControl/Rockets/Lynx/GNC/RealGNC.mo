within RocketControl.Rockets.Lynx.GNC;

model RealGNC
extends RocketControl.Icons.Guidance;

outer World.SimOptions opt;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Navigation.Navigation navigation annotation(
    Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Guidance.ConstantFlightPathGuidanceDiscrete constantFlightPathGuidanceDiscrete(dt = opt.samplePeriodMs / 1000, flightpathangle_0 (displayUnit = "deg") = 1.466076571675237, heading = opt.launch_azimuth, int_lim = 60, k = 1, kint = 4)  annotation(
    Placement(visible = true, transformation(origin = {-40, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.AccelerationRollRateControl accelerationRollRateControl(Qvec = {0, 100, 100, 0, 0, 0, 0, 0, 0, 400} * 0.01, Rvec = {1, 1, 1} * 300)  annotation(
    Placement(visible = true, transformation(origin = {50, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.FeedbackIntegrator feedbackIntegrator(kint = 1,ts = opt.samplePeriodMs / 1000, useEnablePort = true)  annotation(
    Placement(visible = true, transformation(origin = {-12, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Guidance.RollDirectionGuidance rollDirectionGuidance(k = 0.5, rollrate_max = from_deg(10), target_heading = 2.268928027592628)  annotation(
    Placement(visible = true, transformation(origin = {-52, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.RealSensors realSensors annotation(
    Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.SampledTrueSensors sampledTrueSensors annotation(
    Placement(visible = true, transformation(origin = {-10, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(bus, constantFlightPathGuidanceDiscrete.bus) annotation(
    Line(points = {{100, 0}, {100, -100}, {-70, -100}, {-70, -8}, {-50, -8}}, thickness = 0.5));
  connect(bus.w_est[1], feedbackIntegrator.feedback[1]) annotation(
    Line(points = {{100, 0}, {100, -100}, {-12, -100}, {-12, -62}}, thickness = 0.5));
  connect(accelerationRollRateControl.bus, bus) annotation(
    Line(points = {{60, -14}, {100, -14}, {100, 0}}, thickness = 0.5));
  connect(constantFlightPathGuidanceDiscrete.acc_err_int, accelerationRollRateControl.acc_err_int) annotation(
    Line(points = {{-28, -8}, {38, -8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(feedbackIntegrator.err_int[1], accelerationRollRateControl.rollrate_err_int) annotation(
    Line(points = {{0, -50}, {18, -50}, {18, -20}, {38, -20}}, color = {0, 0, 127}));
  connect(rollDirectionGuidance.rollrate_ref, feedbackIntegrator.ref[1]) annotation(
    Line(points = {{-40, -50}, {-24, -50}}, color = {0, 0, 127}));
  connect(rollDirectionGuidance.bus, constantFlightPathGuidanceDiscrete.bus) annotation(
    Line(points = {{-62, -50}, {-70, -50}, {-70, -8}, {-50, -8}}, thickness = 0.5));
  connect(bus.velocity_guidace, feedbackIntegrator.enable) annotation(
    Line(points = {{100, 0}, {-12, 0}, {-12, -40}}, color = {255, 0, 255}));
  connect(frame_a, realSensors.frame_a) annotation(
    Line(points = {{-100, 0}, {-88, 0}, {-88, 90}, {-20, 90}}));
  connect(navigation.bus, bus) annotation(
    Line(points = {{0, 30}, {100, 30}, {100, 0}}, thickness = 0.5));
  connect(sampledTrueSensors.bus, bus) annotation(
    Line(points = {{0, 58}, {100, 58}, {100, 0}}, color = {255, 204, 51}, thickness = 0.5));
  connect(frame_a, sampledTrueSensors.frame_a) annotation(
    Line(points = {{-100, 0}, {-88, 0}, {-88, 58}, {-20, 58}}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end RealGNC;
