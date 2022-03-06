within RocketControl.Rockets.Lynx.GNC;

model RealGNCAng
extends RocketControl.Icons.Guidance;

outer World.SimOptions opt;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Navigation.Navigation navigation annotation(
    Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.FeedbackIntegrator feedbackIntegrator(kint = 1,ts = opt.samplePeriodMs / 1000, useEnablePort = true)  annotation(
    Placement(visible = true, transformation(origin = {-10, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Guidance.RollDirectionGuidance rollDirectionGuidance(k = 0.5, rollrate_max = from_deg(10), target_heading = 2.268928027592628)  annotation(
    Placement(visible = true, transformation(origin = {-50, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.RealSensors realSensors annotation(
    Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.SampledTrueSensors sampledTrueSensors annotation(
    Placement(visible = true, transformation(origin = {-10, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.AngularRateControl angularRateControl(Qvec = {0, 1, 1, 0, 0, 0, 0, 0, 0, 1000, 1000, 1000}, Rvec = {1, 1, 1} * 500)  annotation(
    Placement(visible = true, transformation(origin = {50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.FeedbackIntegrator feedbackIntegrator1(kint = 5, n = 2, ts = opt.samplePeriodMs / 1000, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {-20, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Guidance.AngularRateVelocityTrack angularRateVelocityTrack(k = 5, w_max = from_deg(20))  annotation(
    Placement(visible = true, transformation(origin = {-72, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Guidance.ParabolicVref parabolicVref(flightpathangle_0 = opt.launch_elevation, heading = opt.launch_azimuth, target_apogee = 1500)  annotation(
    Placement(visible = true, transformation(origin = {-142, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(bus.w_est[1], feedbackIntegrator.feedback[1]) annotation(
    Line(points = {{100, 0}, {100, -100}, {-10, -100}, {-10, -86}}, thickness = 0.5));
  connect(rollDirectionGuidance.rollrate_ref, feedbackIntegrator.ref[1]) annotation(
    Line(points = {{-39, -74}, {-23, -74}}, color = {0, 0, 127}));
  connect(frame_a, realSensors.frame_a) annotation(
    Line(points = {{-100, 0}, {-88, 0}, {-88, 90}, {-20, 90}}));
  connect(navigation.bus, bus) annotation(
    Line(points = {{0, 30}, {100, 30}, {100, 0}}, thickness = 0.5));
  connect(frame_a, sampledTrueSensors.frame_a) annotation(
    Line(points = {{-100, 0}, {-88, 0}, {-88, 58}, {-20, 58}}));
  connect(bus, angularRateControl.bus) annotation(
    Line(points = {{100, 0}, {100, -50}, {60, -50}}, thickness = 0.5));
  connect(bus.w_est[2:3], feedbackIntegrator1.feedback) annotation(
    Line(points = {{100, 0}, {100, -100}, {-20, -100}, {-20, -40}}, thickness = 0.5));
  connect(feedbackIntegrator1.err_int, angularRateControl.qr_err_int) annotation(
    Line(points = {{-9, -28}, {14.5, -28}, {14.5, -46}, {38, -46}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.velocity_guidace, feedbackIntegrator1.enable) annotation(
    Line(points = {{100, 0}, {-20, 0}, {-20, -19}}, color = {255, 0, 255}));
  connect(bus, angularRateVelocityTrack.bus) annotation(
    Line(points = {{100, 0}, {14, 0}, {14, -18}, {-82, -18}}, thickness = 0.5));
  connect(bus, rollDirectionGuidance.bus) annotation(
    Line(points = {{100, 0}, {100, -100}, {-72, -100}, {-72, -74}, {-60, -74}}, thickness = 0.5));
  connect(angularRateVelocityTrack.w_ref, feedbackIntegrator1.ref) annotation(
    Line(points = {{-61, -28}, {-32, -28}}, color = {0, 0, 127}, thickness = 0.5));
  connect(parabolicVref.V_ref, angularRateVelocityTrack.v_ref) annotation(
    Line(points = {{-131, -28}, {-84, -28}}, color = {0, 0, 127}));
  connect(feedbackIntegrator.err_int[1], angularRateControl.p_err_int) annotation(
    Line(points = {{2, -74}, {28, -74}, {28, -56}, {38, -56}}, color = {0, 0, 127}));
  connect(bus.roll_guidance, feedbackIntegrator.enable) annotation(
    Line(points = {{100, 0}, {-10, 0}, {-10, -64}}, color = {255, 0, 255}));
  connect(bus, realSensors.bus) annotation(
    Line(points = {{100, 0}, {104, 0}, {104, 90}, {0, 90}}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end RealGNCAng;
