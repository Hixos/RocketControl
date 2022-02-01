within RocketControl.Components.Sensors.SensorModels;

model RealBarometer
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
  parameter SI.Pressure bias "Measurement bias" annotation(
    Dialog(enable = biased, group = "Bias"));
  parameter SI.Pressure p_max "Pressure measurement upper limit" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter SI.Pressure p_min = -p_max "Pressure measurement lower limit" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter Integer bits(min = 1) = 8 "Resolution in bits" annotation(
    Dialog(enable = limited and quantized, group = "Limiting and quantization"));
  parameter Integer fixedLocalSeed = 10 "Local seed" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
  parameter SI.Pressure sigmaNoise "Noise standard deviation" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
  RocketControl.Components.Sensors.TrueSensors.TrueBarometer trueBarometer annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects sample(samplePeriodMs = samplePeriodMs, noise(sigma = sigmaNoise, useAutomaticLocalSeed = false, fixedLocalSeed = fixedLocalSeed), bias = bias, biased = biased, bits = bits, limited = limited, noisy = noisy, quantized = quantized, yMax = p_max, yMin = p_min) annotation(
    Placement(visible = true, transformation(origin = {40, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput p(final unit = "Pa", final quantity = "Pressure", displayUnit = "kPa") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_a, trueBarometer.frame_a) annotation(
    Line(points = {{-100, 0}, {-60, 0}}));
  connect(trueBarometer.p, sample.u) annotation(
    Line(points = {{-40, 0}, {32, 0}}, color = {0, 0, 127}));
  connect(sample.y, p) annotation(
    Line(points = {{46, 0}, {106, 0}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "Pa"), Text(lineColor = {0, 0, 255}, extent = {{-127, 77}, {134, 125}}, textString = "%name"), Line(origin = {81, 0}, points = {{-11, 0}, {11, 0}})}));
end RealBarometer;
