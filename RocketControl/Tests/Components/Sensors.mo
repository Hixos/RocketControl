within RocketControl.Tests.Components;

package Sensors
  model SensorTestSuite
    RocketControl.World.FlatWorld myWorld(n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.Body body(m = 1, r_CM = {0, 0, 0}) annotation(
      Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(r_rel_a_1(fixed = true), r_rel_a_2(fixed = true), r_rel_a_3(fixed = true), use_r = true, use_v = true, use_w = true, v_rel_a_1(fixed = true), v_rel_a_2(fixed = true, start = 20), v_rel_a_3(fixed = true, start = -400), w_rel_b_1(fixed = true, start = 1), w_rel_b_2(fixed = true, start = 2), w_rel_b_3(fixed = true, start = 3)) annotation(
      Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Components.Sensors.RealSensors.RealGyroscope realGyroscope(bias(each displayUnit = "rad/s") = {1, 2, 3}, biased = true, bits = 14, limited = true, noisy = true, quantized = true, rate_max(displayUnit = "rad/s") = 10, samplePeriodMs = 20, sigmaARW(displayUnit = "rad/s") = 1, sigmaRRW(displayUnit = "rad/s2") = 0.1) annotation(
      Placement(visible = true, transformation(origin = {30, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Components.Sensors.RealSensors.RealAccelerometer realAccelerometer(acc_max = 140, bias = {1, 2, 3}, biased = true, limited = true, noisy = true, quantized = true, samplePeriodMs = 20, sigmaBiasInstability = 0.1, sigmaNoise = 1) annotation(
      Placement(visible = true, transformation(origin = {30, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Components.Sensors.RealSensors.RealMagnetometer realMagnetometer(b_max(displayUnit = "nT") = 0.0001000000000000001, bias(displayUnit = "nT") = 3.000000000000003e-07, biased = true, bits = 16, limited = true, misalignement(each displayUnit = "rad") = {0.1, 0.2, 0.3}, noisy = true, quantized = true, samplePeriodMs = 20, sigmaNoise(displayUnit = "T") = 3.000000000000002e-08) annotation(
      Placement(visible = true, transformation(origin = {30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Components.Sensors.RealSensors.RealBarometer realBarometer(bias(displayUnit = "Pa") = 323, biased = true, bits = 12, limited = true, noisy = true, p_max(displayUnit = "Pa") = 100000, quantized = true, samplePeriodMs = 20, sigmaNoise(displayUnit = "Pa") = 10) annotation(
      Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    RocketControl.Components.Sensors.RealSensors.RealGNSS realGNSS(noisy = true, samplePeriodMs = 20, sigmaNoise_vxy = 0, sigmaNoise_vz = 0, sigmaNoise_xy = 0, sigmaNoise_z = 0, sin_error_freq = 0) annotation(
      Placement(visible = true, transformation(origin = {34, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(freeMotionScalarInit.frame_a, myWorld.frame_b) annotation(
      Line(points = {{-80, 0}, {-80, -90}}));
    connect(freeMotionScalarInit.frame_b, body.frame_a) annotation(
      Line(points = {{-60, 0}, {-30, 0}, {-30, 20}}, color = {95, 95, 95}));
    connect(realGyroscope.frame_a, body.frame_a) annotation(
      Line(points = {{20, 88}, {-8, 88}, {-8, 0}, {-30, 0}, {-30, 20}}, color = {95, 95, 95}));
    connect(realAccelerometer.frame_a, body.frame_a) annotation(
      Line(points = {{20, 50}, {-10, 50}, {-10, 0}, {-30, 0}, {-30, 20}}, color = {95, 95, 95}));
    connect(realMagnetometer.frame_a, body.frame_a) annotation(
      Line(points = {{20, 10}, {-8, 10}, {-8, 0}, {-30, 0}, {-30, 20}}, color = {95, 95, 95}));
    connect(realBarometer.frame_a, body.frame_a) annotation(
      Line(points = {{20, -30}, {-10, -30}, {-10, 0}, {-30, 0}, {-30, 20}}, color = {95, 95, 95}));
    connect(body.frame_a, realGNSS.frame_a) annotation(
      Line(points = {{-30, 20}, {-30, 0}, {-8, 0}, {-8, -68}, {24, -68}}));
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end SensorTestSuite;

  model ADEffect
    RocketControl.Components.Sensors.Internal.ADeffects aDeffects(biased = true, noisy = false, samplePeriodMs = 20) annotation(
      Placement(visible = true, transformation(origin = {6, 10}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    Modelica.Blocks.Sources.Sine sine(amplitude = 10, f = 1) annotation(
      Placement(visible = true, transformation(origin = {-70, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(sine.y, aDeffects.u) annotation(
      Line(points = {{-59, 10}, {-1, 10}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end ADEffect;

  model FinPosSensorTest
  RocketControl.Components.Sensors.SensorModels.RealFinPositionSensor realFinPositionSensor(angle_max = from_deg(10), bias = {0.000174532925199433, 0.000174532925199433, 0.000174532925199433, 0.000174532925199433}, biased = true, bits = 12, limited = true, noisy = true, quantized = true, samplePeriodMs = 10, sigma_noise = 0.00174532925199433)  annotation(
      Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine(amplitude = 10, f = 1) annotation(
      Placement(visible = true, transformation(origin = {-58, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine1(amplitude = 10, f = 1) annotation(
      Placement(visible = true, transformation(origin = {-58, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine2(amplitude = 10, f = 1) annotation(
      Placement(visible = true, transformation(origin = {-58, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine3(amplitude = 10, f = 1) annotation(
      Placement(visible = true, transformation(origin = {-58, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(sine.y, realFinPositionSensor.fin_pos_true[1]) annotation(
      Line(points = {{-46, 60}, {-30, 60}, {-30, 0}, {-12, 0}}, color = {0, 0, 127}));
    connect(sine1.y, realFinPositionSensor.fin_pos_true[2]) annotation(
      Line(points = {{-46, 20}, {-30, 20}, {-30, 0}, {-12, 0}}, color = {0, 0, 127}));
  connect(sine2.y, realFinPositionSensor.fin_pos_true[3]) annotation(
      Line(points = {{-46, -20}, {-30, -20}, {-30, 0}, {-12, 0}}, color = {0, 0, 127}));
  connect(sine3.y, realFinPositionSensor.fin_pos_true[4]) annotation(
      Line(points = {{-46, -60}, {-30, -60}, {-30, 0}, {-12, 0}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end FinPosSensorTest;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Sensors;
