within RocketControl.Components.Sensors.SensorModels;

model RealAccelerometer
  extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
  parameter Integer samplePeriodMs(min = 1) "Sample period in milliseconds" annotation(
    Dialog(group = "Sampling and noise"));
  parameter Boolean noisy = false "= true, if output should be superimposed with noise" annotation(
    Evaluate = true,
    choices(checkBox = true),
    Dialog(group = "Sampling and noise"));
  parameter Boolean biased = false "= true, if output should be biased" annotation(
    Evaluate = true,
    choices(checkBox = true),
    Dialog(group = "Bias"));
  parameter Boolean limited = false "= true, if output is limited" annotation(
    Evaluate = true,
    choices(checkBox = true),
    Dialog(group = "Limiting and quantization"));
  parameter Boolean quantized = false "= true, if output quantization effects included" annotation(
    Evaluate = true,
    choices(checkBox = true),
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter SI.Acceleration bias[3] "Measurement bias for each axis" annotation(
    Dialog(enable = biased, group = "Bias"));
  parameter SI.Acceleration acc_max "Acceleration measurement upper limit" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter SI.Acceleration acc_min = -acc_max "Acceleration measurement lower limit" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter Integer bits(min = 1) = 8 "Resolution in bits" annotation(
    Dialog(enable = limited and quantized, group = "Limiting and quantization"));
  parameter Integer fixedLocalSeed[3] = {10, 100, 1000} "Local seed for each of the accelerometer axes" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
  parameter SI.Acceleration sigmaNoise "Noise standard deviation" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
  parameter SI.Jerk sigmaBiasInstability "Bias instability standard deviation" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
  RocketControl.Components.Sensors.TrueSensors.TrueAccelerometer trueAccelerometer annotation(
    Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects sampleX(samplePeriodMs = samplePeriodMs, redeclare RocketControl.Components.Sensors.Internal.Noise.ClockedSensorNoise noise(sigmaNoise = sigmaNoise, sigmaRW = sigmaBiasInstability, useAutomaticLocalSeed = false, fixedLocalSeed = fixedLocalSeed[1]), bias = bias[1], biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, yMax = acc_max, yMin = acc_min) annotation(
    Placement(visible = true, transformation(origin = {0, 40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects sampleY(samplePeriodMs = samplePeriodMs, redeclare RocketControl.Components.Sensors.Internal.Noise.ClockedSensorNoise noise(sigmaNoise = sigmaNoise, sigmaRW = sigmaBiasInstability, useAutomaticLocalSeed = false, fixedLocalSeed = fixedLocalSeed[2]), bias = bias[1], biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, yMax = acc_max, yMin = acc_min) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects sampleZ(samplePeriodMs = samplePeriodMs, redeclare RocketControl.Components.Sensors.Internal.Noise.ClockedSensorNoise noise(sigmaNoise = sigmaNoise, sigmaRW = sigmaBiasInstability, useAutomaticLocalSeed = false, fixedLocalSeed = fixedLocalSeed[3]), bias = bias[1], biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, yMax = acc_max, yMin = acc_min) annotation(
    Placement(visible = true, transformation(origin = {0, -40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput a[3](each final unit = "m/s2", each final quantity = "Acceleration") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_a, trueAccelerometer.frame_a) annotation(
    Line(points = {{-100, 0}, {-70, 0}}));
  connect(trueAccelerometer.a[1], sampleX.u) annotation(
    Line(points = {{-50, 0}, {-29, 0}, {-29, 40}, {-7, 40}}, color = {0, 0, 127}));
  connect(trueAccelerometer.a[2], sampleY.u) annotation(
    Line(points = {{-50, 0}, {-8, 0}}, color = {0, 0, 127}));
  connect(trueAccelerometer.a[3], sampleZ.u) annotation(
    Line(points = {{-50, 0}, {-28, 0}, {-28, -40}, {-8, -40}}, color = {0, 0, 127}));
  connect(sampleX.y, a[1]) annotation(
    Line(points = {{6, 40}, {40, 40}, {40, 0}, {106, 0}}, color = {0, 0, 127}));
  connect(sampleY.y, a[2]) annotation(
    Line(points = {{6, 0}, {106, 0}}, color = {0, 0, 127}));
  connect(sampleZ.y, a[3]) annotation(
    Line(points = {{6, -40}, {40, -40}, {40, 0}, {106, 0}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "m/s^2"), Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name")}));
end RealAccelerometer;
