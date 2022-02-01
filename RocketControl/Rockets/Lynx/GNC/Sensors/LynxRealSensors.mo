within RocketControl.Rockets.Lynx.GNC.Sensors;

model LynxRealSensors
  outer World.SimOptions opt;
  extends Rockets.Internal.PartialSensorsPackage;
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealGyroscope realGyroscope(noisy = true, biased = true, quantized = true, limited = true, bias(each displayUnit = "rad/s") = {0.00174532925199433, -0.0008726646259971648, -0.0003490658503988659}, bits = 16, fixedLocalSeed = {21, 201, 2001}, rate_max(displayUnit = "rad/s") = 4.363323129985824, samplePeriodMs = opt.samplePeriodMs, sigmaARW(displayUnit = "rad/s") = 0.008726646259971648, sigmaRRW(displayUnit = "rad/s2") = 0.0174532925199433) annotation(
    Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealAccelerometer realAccelerometer(noisy = true, biased = true, quantized = true, limited = true, acc_max = 156.96, bias = {0.02, 0.04, -0.03}, bits = 16, fixedLocalSeed = {22, 202, 2002}, samplePeriodMs = opt.samplePeriodMs, sigmaBiasInstability = 0.01, sigmaNoise = 0.1) annotation(
    Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealMagnetometer realMagnetometer(noisy = true, biased = true, quantized = true, limited = true, b_max = 2500000 * 1e-9, bias(displayUnit = "nT") = 250 * 1e-9, bits = 16, fixedLocalSeed = {23, 203, 2003}, misalignement(each displayUnit = "deg") = {from_deg(0.2), from_deg(-0.1), from_deg(0.3)}, samplePeriodMs = opt.samplePeriodMs, sigmaNoise(displayUnit = "nT") = 1.000000000000001e-08) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealBarometer realBarometer(noisy = true, biased = true, quantized = true, limited = true, bias(displayUnit = "Pa") = 300, bits = 24, fixedLocalSeed = 24, p_max(displayUnit = "Pa") = 110000, p_min(displayUnit = "Pa") = 999.9999999999999, samplePeriodMs = opt.samplePeriodMs, sigmaNoise(displayUnit = "Pa") = 50) annotation(
    Placement(visible = true, transformation(origin = {0, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.RealSensors.RealGNSS realGNSS(noisy = true, fixedLocalSeed = {2511, 20511, 200511}, samplePeriodMs = opt.samplePeriodMs, sigmaNoise_vxy = 3, sigmaNoise_vz = 5, sigmaNoise_xy = 15, sigmaNoise_z = 30, sin_error_amplitude = {30, 30, 50}, sin_error_freq = 0.005, sin_error_phase = from_deg({20, 30, 40})) annotation(
    Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
    Line(points = {{-100, 0}, {-40, 0}, {-40, -80}, {-10, -80}}));
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
    Line(points = {{10, -80}, {60, -80}, {60, 0}, {100, 0}}, color = {0, 0, 127}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end LynxRealSensors;
