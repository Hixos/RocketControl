within RocketControl.Components.Sensors.SensorModels;

model RealGNSS
  extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
  parameter Integer samplePeriodMs(min = 1) "Sample period in milliseconds" annotation(
    Dialog(group = "Sampling and noise"));
  parameter Boolean noisy = false "= true, if output should be superimposed with noise" annotation(
    Evaluate = true,
    choices(checkBox = true),
    Dialog(group = "Sampling and noise"));
  parameter SI.Position sigmaNoise_xy "Position noise standard deviation on the horizontal plane" annotation(
    Evaluate = true,
    Dialog(group = "Sampling and noise"));
  parameter SI.Position sigmaNoise_z "Position noise standard deviation on the vertical plane" annotation(
    Evaluate = true,
    Dialog(group = "Sampling and noise"));
  parameter SI.Velocity sigmaNoise_vxy "Velocity noise standard deviation on the horizontal plane" annotation(
    Evaluate = true,
    Dialog(group = "Sampling and noise"));
  parameter SI.Velocity sigmaNoise_vz "Velocity noise standard deviation on the vertical plane" annotation(
    Evaluate = true,
    Dialog(group = "Sampling and noise"));
  parameter SI.Position sin_error_amplitude[3] = {0, 0, 0} annotation(
    Dialog(group = "Bias"));
  parameter SI.Frequency sin_error_freq annotation(
    Dialog(group = "Bias"));
  parameter SI.Angle sin_error_phase[3](each displayUnit = "deg") = {0, 0, 0} annotation(
    Dialog(group = "Bias"));
  parameter Integer fixedLocalSeed[3] = {10, 100, 1000} "Local seed for each of the accelerometer axes" annotation(
    Evaluate = true,
    Dialog(group = "Sampling and noise"));
  Modelica.Blocks.Interfaces.RealOutput x[3](each final quantity = "Position", each final unit = "m") annotation(
    Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock periodicClock1(factor = samplePeriodMs) annotation(
    Placement(visible = true, transformation(origin = {-94, -94}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects samplePX(samplePeriodMs = samplePeriodMs, biased = false, limited = false, noise(fixedLocalSeed = fixedLocalSeed[1], sigma = sigmaNoise_xy, useAutomaticLocalSeed = false), noisy = noisy, quantized = false) annotation(
    Placement(visible = true, transformation(origin = {60, 100}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects samplePY(samplePeriodMs = samplePeriodMs, biased = false, limited = false, noise(fixedLocalSeed = fixedLocalSeed[2], sigma = sigmaNoise_xy, useAutomaticLocalSeed = false), noisy = noisy, quantized = false) annotation(
    Placement(visible = true, transformation(origin = {60, 60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects samplePZ(samplePeriodMs = samplePeriodMs, biased = false, limited = false, noise(fixedLocalSeed = fixedLocalSeed[3], sigma = sigmaNoise_z, useAutomaticLocalSeed = false), noisy = noisy, quantized = false) annotation(
    Placement(visible = true, transformation(origin = {60, 20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects sampleVX(samplePeriodMs = samplePeriodMs, biased = false, limited = false, noise(fixedLocalSeed = fixedLocalSeed[1] + 1, sigma = sigmaNoise_vxy, useAutomaticLocalSeed = false), noisy = noisy, quantized = false) annotation(
    Placement(visible = true, transformation(origin = {60, -20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects sampleVY(samplePeriodMs = samplePeriodMs, biased = false, limited = false, noise(fixedLocalSeed = fixedLocalSeed[3] + 1, sigma = sigmaNoise_vz, useAutomaticLocalSeed = false), noisy = noisy, quantized = false) annotation(
    Placement(visible = true, transformation(origin = {60, -100}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects sampleVZ(samplePeriodMs = samplePeriodMs, biased = false, limited = false, noise(fixedLocalSeed = fixedLocalSeed[2] + 1, sigma = sigmaNoise_vxy, useAutomaticLocalSeed = false), noisy = noisy, quantized = false) annotation(
    Placement(visible = true, transformation(origin = {60, -60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput v[3](each quantity = "Velocity") annotation(
    Placement(visible = true, transformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vector.VectorAdd vectorAdd annotation(
    Placement(visible = true, transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vector.VectorSine vectorSine(A = sin_error_amplitude, f = sin_error_freq, phase = sin_error_phase) annotation(
    Placement(visible = true, transformation(origin = {-40, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vector.VectorSine vectorSine1(A = sin_error_amplitude * 2 * pi * sin_error_freq, f = sin_error_freq, phase = sin_error_phase + from_deg({90, 90, 90})) annotation(
    Placement(visible = true, transformation(origin = {-68, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vector.VectorAdd vectorAdd1 annotation(
    Placement(visible = true, transformation(origin = {10, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.IdealSensors.IdealGNSS idealGNSS annotation(
    Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(vectorAdd.vc[1], samplePX.u) annotation(
    Line(points = {{11, 60}, {28, 60}, {28, 100}, {52, 100}}, color = {0, 0, 127}));
  connect(vectorAdd.vc[2], samplePY.u) annotation(
    Line(points = {{12, 60}, {52, 60}}, color = {0, 0, 127}));
  connect(vectorAdd.vc[3], samplePZ.u) annotation(
    Line(points = {{12, 60}, {28, 60}, {28, 20}, {52, 20}}, color = {0, 0, 127}));
  connect(vectorSine.y, vectorAdd.v1) annotation(
    Line(points = {{-28, 90}, {-24, 90}, {-24, 64}, {-12, 64}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorSine1.y, vectorAdd1.v1) annotation(
    Line(points = {{-57, -32}, {-24, -32}, {-24, -56}, {-2, -56}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorAdd1.vc[1], sampleVX.u) annotation(
    Line(points = {{21, -60}, {28, -60}, {28, -20}, {52, -20}}, color = {0, 0, 127}));
  connect(vectorAdd1.vc[2], sampleVY.u) annotation(
    Line(points = {{21, -60}, {52, -60}}, color = {0, 0, 127}));
  connect(vectorAdd1.vc[3], sampleVZ.u) annotation(
    Line(points = {{21, -60}, {28, -60}, {28, -100}, {52, -100}}, color = {0, 0, 127}));
  connect(samplePX.y, x[1]) annotation(
    Line(points = {{66, 100}, {80, 100}, {80, 60}, {110, 60}}, color = {0, 0, 127}));
  connect(samplePY.y, x[2]) annotation(
    Line(points = {{66, 60}, {110, 60}}, color = {0, 0, 127}));
  connect(samplePZ.y, x[3]) annotation(
    Line(points = {{66, 20}, {80, 20}, {80, 60}, {110, 60}}, color = {0, 0, 127}));
  connect(sampleVX.y, v[1]) annotation(
    Line(points = {{66, -20}, {80, -20}, {80, -60}, {110, -60}}, color = {0, 0, 127}));
  connect(sampleVY.y, v[2]) annotation(
    Line(points = {{66, -60}, {110, -60}}, color = {0, 0, 127}));
  connect(sampleVZ.y, v[3]) annotation(
    Line(points = {{66, -100}, {80, -100}, {80, -60}, {110, -60}}, color = {0, 0, 127}));
  connect(frame_a, idealGNSS.frame_a) annotation(
    Line(points = {{-100, 0}, {-80, 0}}));
  connect(idealGNSS.x, vectorAdd.v2) annotation(
    Line(points = {{-58, 4}, {-42, 4}, {-42, 56}, {-12, 56}}, color = {0, 0, 127}, thickness = 0.5));
  connect(idealGNSS.v, vectorAdd1.v2) annotation(
    Line(points = {{-58, -4}, {-40, -4}, {-40, -64}, {-2, -64}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(graphics = {Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "GNSS"), Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Line(origin = {90, 0}, points = {{10, 40}, {-10, 40}, {-10, -40}, {10, -40}}), Line(origin = {75, 0}, points = {{5, 0}, {-5, 0}})}));
end RealGNSS;
