within RocketControl.Rockets.Lynx.GNC.GuidanceControl;

model AngularRateGC
extends RocketControl.Icons.Guidance;
outer RocketControl.World.SimOptions opt;
  RocketControl.GNC.Guidance.ParabolicVref parabolic_traj(flightpathangle_0 = opt.launch_elevation, heading = opt.launch_azimuth, t0 = 1.5, target_apogee = 1614) annotation(
    Placement(visible = true, transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Guidance.RollDirectionGuidance roll_guidance(k = 1, rollrate_max = from_deg(20), rollrate_ref(displayUnit = "rad/s"), target_heading = opt.roll_target_heading) annotation(
    Placement(visible = true, transformation(origin = {-50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.FeedbackIntegrator feedbackIntegrator(kint = 1, ts = opt.samplePeriodMs / 1000, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.AngularRateControl angularRateControl(Qvec = {0, 0, 0, 5, 5, 0, 0, 0, 10, 10, 10}, Rvec = {1, 1, 1} * 100) annotation(
    Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Guidance.AngularRateVelocityTrack wref_guidance(k = 5, w_max = from_deg(20)) annotation(
    Placement(visible = true, transformation(origin = {-40, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.FeedbackIntegrator feedbackIntegrator1(kint = 12, n = 2, saturation = 0, ts = opt.samplePeriodMs / 1000, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  connect(bus, angularRateControl.bus) annotation(
    Line(points = {{100, 0}, {80, 0}}, thickness = 0.5));
  connect(wref_guidance.w_ref, feedbackIntegrator1.ref) annotation(
    Line(points = {{-29, 30}, {-2, 30}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.w_est[1], feedbackIntegrator.feedback[1]) annotation(
    Line(points = {{100, 0}, {100, -56}, {10, -56}, {10, -42}}, thickness = 0.5));
  connect(feedbackIntegrator.err_int[1], angularRateControl.p_err_int) annotation(
    Line(points = {{21, -30}, {31, -30}, {31, -6}, {58, -6}}, color = {0, 0, 127}));
  connect(parabolic_traj.V_ref, wref_guidance.v_ref) annotation(
    Line(points = {{-79, 30}, {-52, 30}}, color = {0, 0, 127}));
  connect(roll_guidance.rollrate_ref, feedbackIntegrator.ref[1]) annotation(
    Line(points = {{-39, -30}, {-2, -30}}, color = {0, 0, 127}));
  connect(bus.w_est[2:3], feedbackIntegrator1.feedback) annotation(
    Line(points = {{100, 0}, {100, -56}, {6, -56}, {6, 6}, {10, 6}, {10, 18}}, thickness = 0.5));
  connect(bus, roll_guidance.bus) annotation(
    Line(points = {{100, 0}, {100, -56}, {-80, -56}, {-80, -30}, {-60, -30}}, thickness = 0.5));
  connect(bus, wref_guidance.bus) annotation(
    Line(points = {{100, 0}, {100, -56}, {-80, -56}, {-80, 0}, {-50, 0}, {-50, 20}}, thickness = 0.5));
  connect(feedbackIntegrator1.err_int, angularRateControl.qr_err_int) annotation(
    Line(points = {{22, 30}, {32, 30}, {32, 4}, {58, 4}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.velocity_guidace, feedbackIntegrator1.enable) annotation(
    Line(points = {{100, 0}, {100, 60}, {10, 60}, {10, 40}}, color = {255, 0, 255}));
  connect(bus.roll_guidance, feedbackIntegrator.enable) annotation(
    Line(points = {{100, 0}, {100, 60}, {10, 60}, {10, -20}}, color = {255, 0, 255}));
  connect(parabolic_traj.V_ref, bus.v_ref) annotation(
    Line(points = {{-78, 30}, {-64, 30}, {-64, 82}, {100, 82}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(graphics = {Text(origin = {10, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end AngularRateGC;
