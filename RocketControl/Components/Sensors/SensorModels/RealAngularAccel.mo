within RocketControl.Components.Sensors.SensorModels;

model RealAngularAccel
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
  parameter SI.AngularAcceleration bias "Measurement bias" annotation(
    Dialog(enable = biased, group = "Bias"));
  parameter SI.AngularAcceleration w_dot_max "Magnetic field measurement upper limit" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter SI.AngularAcceleration w_dot_min = -w_dot_max "Magnetic field measurement lower limit" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter Integer bits(min = 1) = 8 "Resolution in bits" annotation(
    Dialog(enable = limited and quantized, group = "Limiting and quantization"));
  parameter Integer fixedLocalSeed[3] = {10, 100, 1000} "Local seed for each of the axes" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
  parameter  SI.AngularAcceleration sigmaNoise "Noise standard deviation" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
  
  
  RocketControl.Components.Sensors.Internal.ADeffects sampleX(samplePeriodMs = samplePeriodMs, bias = 0, biased = biased, bits = bits, limited = limited, noise(fixedLocalSeed = fixedLocalSeed[1], sigma = sigmaNoise, useAutomaticLocalSeed = false), noisy = noisy, quantized = quantized, yMax = w_dot_max, yMin = w_dot_min) annotation(
    Placement(visible = true, transformation(origin = {16, 40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects sampleY(samplePeriodMs = samplePeriodMs, bias = 0, biased = biased, bits = bits, limited = limited, noise(fixedLocalSeed = fixedLocalSeed[2], sigma = sigmaNoise, useAutomaticLocalSeed = false), noisy = noisy, quantized = quantized, yMax = w_dot_max, yMin = w_dot_min) annotation(
    Placement(visible = true, transformation(origin = {16, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects sampleZ(samplePeriodMs = samplePeriodMs, bias = 0, biased = biased, bits = bits, limited = limited, noise(fixedLocalSeed = fixedLocalSeed[3], sigma = sigmaNoise, useAutomaticLocalSeed = false), noisy = noisy, quantized = quantized, yMax = w_dot_max, yMin = w_dot_min) annotation(
    Placement(visible = true, transformation(origin = {16, -40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
 RocketControl.Components.Sensors.TrueSensors.TrueAngularAccel trueAngularAccel annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Blocks.Interfaces.RealOutput w_dot[3](each displayUnit = "deg/s2", each quantity = "AngularAcceleration", each unit = "rad/s2") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_a, trueAngularAccel.frame_a) annotation(
    Line(points = {{-100, 0}, {-60, 0}}));
 connect(trueAngularAccel.w_dot[1], sampleX.u) annotation(
    Line(points = {{-40, 0}, {-20, 0}, {-20, 40}, {8, 40}}, color = {0, 0, 127}));
 connect(trueAngularAccel.w_dot[2], sampleY.u) annotation(
    Line(points = {{-40, 0}, {8, 0}}, color = {0, 0, 127}));
 connect(trueAngularAccel.w_dot[3], sampleZ.u) annotation(
    Line(points = {{-40, 0}, {-20, 0}, {-20, -40}, {8, -40}}, color = {0, 0, 127}));
 connect(sampleX.y, w_dot[1]) annotation(
    Line(points = {{22, 40}, {60, 40}, {60, 0}, {110, 0}}, color = {0, 0, 127}));
 connect(sampleY.y, w_dot[2]) annotation(
    Line(points = {{22, 0}, {110, 0}}, color = {0, 0, 127}));
 connect(sampleZ.y, w_dot[3]) annotation(
    Line(points = {{22, -40}, {60, -40}, {60, 0}, {110, 0}}, color = {0, 0, 127}));

annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end RealAngularAccel;
