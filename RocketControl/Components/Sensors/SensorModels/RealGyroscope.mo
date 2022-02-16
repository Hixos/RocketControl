within RocketControl.Components.Sensors.SensorModels;

model RealGyroscope "Implementation of a real gyroscope, affected by startup random bias, bias instability (Rate Random Walk) and Normal Noise (Angle Random Walk)"
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
  parameter Types.AngularVelocity bias[3] "Measurement bias for each axis" annotation(
    Dialog(enable = biased, group = "Bias"));
  parameter Types.AngularVelocity rate_max "Angular velocity measurement upper limit" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter Types.AngularVelocity rate_min = -rate_max "Angular velocity measurement lower limit" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter Integer bits(min = 1) = 8 "Resolution in bits" annotation(
    Dialog(enable = limited and quantized, group = "Limiting and quantization"));
  parameter Integer fixedLocalSeed[3] = {10, 100, 1000} "Local seed for each of the accelerometer axes" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
  parameter Types.AngularVelocity sigmaARW "Angular random walk standard deviation" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
  parameter SI.AngularAcceleration sigmaRRW(displayUnit = "deg/s2") "Rate random walk standard deviation" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
  RocketControl.Components.Sensors.Internal.ADeffects sampleX(samplePeriodMs = samplePeriodMs, redeclare RocketControl.Components.Sensors.Internal.Noise.ClockedSensorNoise noise(sigmaNoise = sigmaARW, sigmaRW = sigmaRRW, useAutomaticLocalSeed = false, fixedLocalSeed = fixedLocalSeed[1]), bias = bias[1], biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, yMax = rate_max, yMin = rate_min) annotation(
    Placement(visible = true, transformation(origin = {0, 40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects sampleZ(samplePeriodMs = samplePeriodMs, redeclare RocketControl.Components.Sensors.Internal.Noise.ClockedSensorNoise noise(sigmaNoise = sigmaARW, sigmaRW = sigmaRRW, useAutomaticLocalSeed = false, fixedLocalSeed = fixedLocalSeed[3]), bias = bias[3], biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, yMax = rate_max, yMin = rate_min) annotation(
    Placement(visible = true, transformation(origin = {0, -40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueGyroscope trueGyroscope annotation(
    Placement(visible = true, transformation(origin = {-58, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput w[3](each final unit = "rad/s", each final quantity = "AngularVelocity", each displayUnit = "deg/s") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Internal.ADeffects sampleY(bias = bias[2], biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, samplePeriodMs = samplePeriodMs, yMax = rate_max, yMin = rate_min) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
equation
  connect(sampleX.y, w[1]) annotation(
    Line(points = {{6, 40}, {40, 40}, {40, 0}, {106, 0}}, color = {0, 0, 127}));
  connect(sampleZ.y, w[3]) annotation(
    Line(points = {{6, -40}, {40, -40}, {40, 0}, {106, 0}}, color = {0, 0, 127}));
  connect(trueGyroscope.w[1], sampleX.u) annotation(
    Line(points = {{-47, 0}, {-30, 0}, {-30, 40}, {-8, 40}}, color = {0, 0, 127}));
  connect(trueGyroscope.w[3], sampleZ.u) annotation(
    Line(points = {{-47, 0}, {-32, 0}, {-32, -40}, {-8, -40}}, color = {0, 0, 127}));
  connect(frame_a, trueGyroscope.frame_a) annotation(
    Line(points = {{-100, 0}, {-68, 0}}));
  connect(trueGyroscope.w[2], sampleY.u) annotation(
    Line(points = {{-47, 0}, {-8, 0}}, color = {0, 0, 127}));
  connect(sampleY.y, w[2]) annotation(
    Line(points = {{106, 0}, {6, 0}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-127, 77}, {134, 125}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "w")}),
    Diagram);
end RealGyroscope;
