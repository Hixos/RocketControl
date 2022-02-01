within RocketControl.Components.Sensors.SensorModels;

model RealMagnetometer "Magnetometer sensor model with bias, gaussian noise and quantization effects"
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
  parameter Types.MagneticFluxDensity bias "Measurement bias" annotation(
    Dialog(enable = biased, group = "Bias"));
  parameter SI.Angle misalignement[3] "Magnetic vector measurement miasalignment" annotation(
    Dialog(enable = biased, group = "Bias"));
  parameter Types.MagneticFluxDensity b_max "Magnetic field measurement upper limit" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter Types.MagneticFluxDensity b_min = -b_max "Magnetic field measurement lower limit" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter Integer bits(min = 1) = 8 "Resolution in bits" annotation(
    Dialog(enable = limited and quantized, group = "Limiting and quantization"));
  parameter Integer fixedLocalSeed[3] = {10, 100, 1000} "Local seed for each of the axes" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
  parameter Types.MagneticFluxDensity sigmaNoise "Noise standard deviation" annotation(
    Dialog(enable = noisy, group = "Sampling and noise"));
  RocketControl.Components.Sensors.Internal.ADeffects sampleX(samplePeriodMs = samplePeriodMs, bias = 0, biased = biased, bits = bits, limited = limited, noise(fixedLocalSeed = fixedLocalSeed[1], sigma = sigmaNoise, useAutomaticLocalSeed = false), noisy = noisy, quantized = quantized, yMax = b_max, yMin = b_min) annotation(
    Placement(visible = true, transformation(origin = {16, 40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects sampleY(samplePeriodMs = samplePeriodMs, bias = 0, biased = biased, bits = bits, limited = limited, noise(fixedLocalSeed = fixedLocalSeed[2], sigma = sigmaNoise, useAutomaticLocalSeed = false), noisy = noisy, quantized = quantized, yMax = b_max, yMin = b_min) annotation(
    Placement(visible = true, transformation(origin = {16, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.ADeffects sampleZ(samplePeriodMs = samplePeriodMs, bias = 0, biased = biased, bits = bits, limited = limited, noise(fixedLocalSeed = fixedLocalSeed[3], sigma = sigmaNoise, useAutomaticLocalSeed = false), noisy = noisy, quantized = quantized, yMax = b_max, yMin = b_min) annotation(
    Placement(visible = true, transformation(origin = {16, -40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueMagnetometer trueMagnetometer annotation(
    Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput b[3](each final unit = "T", each final quantity = "MagneticFluxDensity", each displayUnit = "nT") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  final parameter Modelica.Mechanics.MultiBody.Frames.Orientation R_mis = Modelica.Mechanics.MultiBody.Frames.axesRotations({3, 2, 1}, misalignement, {0, 0, 0}) annotation(
    Evaluate = true);
  Types.MagneticFluxDensity b_mis[3];
  Types.NanoTesla b_biased[3];
equation
  b_biased = trueMagnetometer.b + trueMagnetometer.b * bias / norm(trueMagnetometer.b);
  b_mis = Modelica.Mechanics.MultiBody.Frames.resolve2(R_mis, b_biased);
  sampleX.u = b_mis[1];
  sampleY.u = b_mis[2];
  sampleZ.u = b_mis[3];
  connect(frame_a, trueMagnetometer.frame_a) annotation(
    Line(points = {{-100, 0}, {-70, 0}}));
  connect(sampleX.y, b[1]) annotation(
    Line(points = {{23, 40}, {40, 40}, {40, 0}, {106, 0}}, color = {0, 0, 127}));
  connect(sampleY.y, b[2]) annotation(
    Line(points = {{23, 0}, {106, 0}}, color = {0, 0, 127}));
  connect(sampleZ.y, b[3]) annotation(
    Line(points = {{23, -40}, {40, -40}, {40, 0}, {106, 0}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-127, 77}, {134, 125}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "rad/s")}),
    Diagram);
end RealMagnetometer;
