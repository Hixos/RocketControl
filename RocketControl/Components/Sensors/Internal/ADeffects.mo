within RocketControl.Components.Sensors.Internal;

block ADeffects "Sample with (simulated) Analog-Digital converter effects including noise"
  extends Modelica.Clocked.RealSignals.Interfaces.PartialSISOSampler;
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
  parameter Real yMax = 1 "Upper limit of output (if limited = true)" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter Real yMin = -yMax "Lower limit of output (if limited = true)" annotation(
    Dialog(enable = limited, group = "Limiting and quantization"));
  parameter Integer bits(min = 1) = 8 "Number of bits of quantization (if quantized = true)" annotation(
    Dialog(enable = limited and quantized, group = "Limiting and quantization"));
  parameter Real bias "Signal bias" annotation(
    Dialog(enable = biased, group = "Bias"));
  replaceable RocketControl.Components.Sensors.Internal.Noise.ClockedNormalNoise noise if noisy annotation(
    Placement(visible = true, transformation(extent = {{-54, -6}, {-42, 6}}, rotation = 0))) constrainedby Modelica.Clocked.RealSignals.Interfaces.PartialNoise "Noise model" annotation(
     Dialog(enable = noisy, group = "Sampling and noise"),
     Placement(transformation(extent = {{-54, -6}, {-42, 6}})));
  Modelica.Clocked.RealSignals.Sampler.Utilities.Internal.Limiter limiter(uMax = yMax, uMin = yMin) if limited annotation(
    Placement(visible = true, transformation(extent = {{22, -8}, {38, 8}}, rotation = 0)));
  Modelica.Clocked.RealSignals.Sampler.Utilities.Internal.Quantization quantization(quantized = quantized, yMax = yMax, yMin = yMin, bits = bits) if quantized and limited annotation(
    Placement(visible = true, transformation(extent = {{60, -8}, {76, 8}}, rotation = 0)));
  Modelica.Blocks.Math.Add add if biased annotation(
    Placement(visible = true, transformation(origin = {-12, -1.33227e-15}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant bias_k(k = bias) if biased annotation(
    Placement(visible = true, transformation(origin = {-48, -30}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock periodicClock1(factor = samplePeriodMs) annotation(
    Placement(visible = true, transformation(origin = {-88, -60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Clocked.RealSignals.Sampler.SampleClocked sample1 annotation(
    Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
protected
  Modelica.Blocks.Interfaces.RealInput uFeedthrough1 if not noisy annotation(
    Placement(transformation(extent = {{-58, 12}, {-42, 28}})));
  Modelica.Blocks.Interfaces.RealInput uFeedthrough2 if not biased annotation(
    Placement(visible = true, transformation(extent = {{-20, 12}, {-4, 28}}, rotation = 0), iconTransformation(extent = {{-26, 12}, {-10, 28}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput uFeedthrough3 if not limited annotation(
    Placement(visible = true, transformation(extent = {{20, 12}, {36, 28}}, rotation = 0), iconTransformation(extent = {{-26, 12}, {-10, 28}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput uFeedthrough4 if not quantized or not limited annotation(
    Placement(visible = true, transformation(extent = {{58, 12}, {74, 28}}, rotation = 0), iconTransformation(extent = {{12, 12}, {28, 28}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y1 "Connector with a Real output signal" annotation(
    Placement(transformation(extent = {{-61, -1}, {-59, 1}})));
  Modelica.Blocks.Interfaces.RealOutput y3 annotation(
    Placement(visible = true, transformation(extent = {{11, -1}, {13, 1}}, rotation = 0), iconTransformation(extent = {{-35, -1}, {-33, 1}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y4 annotation(
    Placement(visible = true, transformation(extent = {{49, -1}, {51, 1}}, rotation = 0), iconTransformation(extent = {{3, -1}, {5, 1}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y5 annotation(
    Placement(visible = true, transformation(extent = {{87, -1}, {89, 1}}, rotation = 0), iconTransformation(extent = {{41, -1}, {43, 1}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y2 annotation(
    Placement(visible = true, transformation(extent = {{-31, -1}, {-29, 1}}, rotation = 0), iconTransformation(extent = {{-35, -1}, {-33, 1}}, rotation = 0)));
equation
  connect(y1, uFeedthrough1) annotation(
    Line(points = {{-50, 20}, {-58, 20}, {-58, 0}, {-60, 0}}, color = {0, 0, 127}));
  connect(y1, noise.u) annotation(
    Line(points = {{-60, 0}, {-55, 0}}, color = {0, 0, 127}));
  connect(sample1.y, y1) annotation(
    Line(points = {{-74, 0}, {-60, 0}}, color = {0, 0, 127}));
  connect(y3, limiter.u) annotation(
    Line(points = {{12, 0}, {20.4, 0}}, color = {0, 0, 127}));
  connect(y3, uFeedthrough3) annotation(
    Line(points = {{12, 0}, {16, 0}, {16, 20}, {28, 20}}, color = {0, 0, 127}));
  connect(limiter.y, y4) annotation(
    Line(points = {{38.8, 0}, {50, 0}}, color = {0, 0, 127}));
  connect(y4, quantization.u) annotation(
    Line(points = {{50, 0}, {58.4, 0}}, color = {0, 0, 127}));
  connect(y4, uFeedthrough4) annotation(
    Line(points = {{50, 0}, {54, 0}, {54, 20}, {66, 20}}, color = {0, 0, 127}));
  connect(quantization.y, y5) annotation(
    Line(points = {{76.8, 0}, {88, 0}}, color = {0, 0, 127}));
  connect(uFeedthrough4, y5) annotation(
    Line(points = {{66, 20}, {84, 20}, {84, 0}, {88, 0}}, color = {0, 0, 127}));
  connect(uFeedthrough3, y4) annotation(
    Line(points = {{28, 20}, {46, 20}, {46, 0}, {50, 0}}, color = {0, 0, 127}));
  connect(y5, y) annotation(
    Line(points = {{88, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(noise.y, y2) annotation(
    Line(points = {{-41, 0}, {-30, 0}}, color = {0, 0, 127}));
  connect(uFeedthrough1, y2) annotation(
    Line(points = {{-50, 20}, {-36, 20}, {-36, 0}, {-30, 0}}, color = {0, 0, 127}));
  connect(y2, uFeedthrough2) annotation(
    Line(points = {{-30, 0}, {-28, 0}, {-28, 20}, {-12, 20}}, color = {0, 0, 127}));
  connect(uFeedthrough2, y3) annotation(
    Line(points = {{-12, 20}, {4, 20}, {4, 0}, {12, 0}}, color = {0, 0, 127}));
  connect(add.u1, y2) annotation(
    Line(points = {{-22, 4}, {-26, 4}, {-26, 0}, {-30, 0}}, color = {0, 0, 127}));
  connect(add.y, y3) annotation(
    Line(points = {{-4, 0}, {12, 0}}, color = {0, 0, 127}));
  connect(bias_k.y, add.u2) annotation(
    Line(points = {{-41, -30}, {-32, -30}, {-32, -4}, {-22, -4}}, color = {0, 0, 127}));
  connect(u, sample1.u) annotation(
    Line(points = {{-120, 0}, {-88, 0}}, color = {0, 0, 127}));
  connect(periodicClock1.y, sample1.clock) annotation(
    Line(points = {{-82, -60}, {-80, -60}, {-80, -8}}, color = {175, 175, 175}));
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.06), graphics = {Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{0, -22}, {-6, -38}, {6, -38}, {0, -22}}), Line(points = {{0, -100}, {0, -38}}, color = {192, 192, 192}), Line(points = {{-40, -72}, {40, -72}}, color = {192, 192, 192}), Polygon(origin = {48, -72}, rotation = -90, lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{0, 8}, {-6, -8}, {6, -8}, {0, 8}}), Line(points = {{-30, -92}, {-10, -92}, {-10, -72}, {10, -72}, {10, -52}, {30, -52}}, color = {0, 0, 127}), Text(lineColor = {0, 0, 255}, extent = {{-150, 90}, {150, 50}}, textString = "%name")}),
    Diagram(coordinateSystem(extent = {{-140, 40}, {120, -80}})));
end ADeffects;
