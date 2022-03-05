within RocketControl.Rockets.Lynx.GNC.Sensors;

model SampledTrueSensors
  outer World.SimOptions opt;
  extends Rockets.Internal.PartialSensorsPackage;
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.SensorModels.RealGyroscope realGyroscope(bias(each displayUnit = "rad/s") = {0, 0, 0}, fixedLocalSeed = {10, 100, 100}, noisy = true, samplePeriodMs = opt.samplePeriodMs, sigmaARW(displayUnit = "rad/s") = 0, sigmaRRW(displayUnit = "rad/s2") = 0) annotation(
    Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.SensorModels.RealAccelerometer realAccelerometer(bias = {0, 0, 0}, fixedLocalSeed = {11, 101, 1001}, noisy = true, samplePeriodMs = opt.samplePeriodMs, sigmaBiasInstability = 0, sigmaNoise = 0) annotation(
    Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.SensorModels.RealMagnetometer realMagnetometer(b_max(displayUnit = "T") = 0, bias(displayUnit = "T") = 0, fixedLocalSeed = {1211, 10211, 100211}, misalignement(each displayUnit = "rad") = {0, 0, 0}, noisy = true, samplePeriodMs = opt.samplePeriodMs, sigmaNoise(displayUnit = "T") = 0) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.SensorModels.RealBarometer realBarometer(bias(displayUnit = "Pa") = 0, fixedLocalSeed = 14, noisy = true, p_max(displayUnit = "Pa") = 0, samplePeriodMs = opt.samplePeriodMs, sigmaNoise(displayUnit = "Pa") = 0) annotation(
    Placement(visible = true, transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.SensorModels.RealGNSS realGNSS(fixedLocalSeed = {13, 103, 1003}, noisy = true, samplePeriodMs = opt.samplePeriodMs, sigmaNoise_vxy = 0, sigmaNoise_vz = 0, sigmaNoise_xy = 0, sigmaNoise_z = 0, sin_error_freq = 0) annotation(
    Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Sensors.SensorModels.RealAngularAccel realAngularAccel annotation(
    Placement(visible = true, transformation(origin = {2, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_a, realGyroscope.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, 80}, {-10, 80}}));
  connect(frame_a, realAccelerometer.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, 40}, {-10, 40}}));
  connect(frame_a, realMagnetometer.frame_a) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  connect(frame_a, realGNSS.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, -40}, {-10, -40}}));
  connect(frame_a, realBarometer.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, -70}, {-10, -70}}));
  connect(realGyroscope.w, bus.w_meas) annotation(
    Line(points = {{11, 80}, {60, 80}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(realAccelerometer.a, bus.a_meas) annotation(
    Line(points = {{10, 40}, {60, 40}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(realMagnetometer.b, bus.b_meas) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(realGNSS.x, bus.x_meas) annotation(
    Line(points = {{12, -36}, {60, -36}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(realGNSS.v, bus.v_meas) annotation(
    Line(points = {{12, -44}, {60, -44}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(realBarometer.p, bus.p_meas) annotation(
    Line(points = {{11, -70}, {60, -70}, {60, 0}, {100, 0}}, color = {0, 0, 127}));
  connect(frame_a, realAngularAccel.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, -92}, {-8, -92}}));
  connect(realAngularAccel.w_dot, bus.w_dot_meas) annotation(
    Line(points = {{12, -92}, {60, -92}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end SampledTrueSensors;
