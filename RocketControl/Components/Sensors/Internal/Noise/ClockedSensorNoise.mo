within RocketControl.Components.Sensors.Internal.Noise;

model ClockedSensorNoise
  extends Modelica.Clocked.RealSignals.Interfaces.PartialNoise;
  parameter Boolean enableNoise = globalSeed.enableNoise "= true: y = u + noise, otherwise y = u" annotation(
    choices(checkBox = true),
    Dialog(tab = "Advanced", group = "Noise generation"));
  // Advanced dialog menu: Initialization
  parameter Boolean useGlobalSeed = true "= true: use global seed, otherwise ignore it" annotation(
    choices(checkBox = true),
    Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise));
  parameter Boolean useAutomaticLocalSeed = true "= true: use automatic local seed, otherwise use fixedLocalSeed" annotation(
    choices(checkBox = true),
    Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise));
  parameter Integer fixedLocalSeed = 1 "Local seed for the normal noise (any Integer number)" annotation(
    Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise and not useAutomaticLocalSeed));
  parameter Integer fixedLocalSeedRW = fixedLocalSeed + 1 "Local seed for the random walk noise(any Integer number)" annotation(
    Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise and not useAutomaticLocalSeed));
  parameter Real sigmaNoise = 1 "Standard Deviation of the white noise";
  parameter Real sigmaRW = 1 "Standard Deviation of the white noise generating the Random Walk";
  RocketControl.Components.Sensors.Internal.Noise.ClockedNormalNoise rwNoise(sigma = sigmaRW, useAutomaticLocalSeed = useAutomaticLocalSeed, useGlobalSeed = useGlobalSeed, enableNoise = enableNoise, fixedLocalSeed = fixedLocalSeedRW) annotation(
    Placement(visible = true, transformation(origin = {-30, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator annotation(
    Placement(visible = true, transformation(origin = {10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add annotation(
    Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(
    Placement(visible = true, transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Internal.Noise.ClockedNormalNoise normalNoise(sigma = sigmaNoise, useAutomaticLocalSeed = useAutomaticLocalSeed, useGlobalSeed = useGlobalSeed, enableNoise = enableNoise, fixedLocalSeed = fixedLocalSeed) annotation(
    Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  outer Modelica.Blocks.Noise.GlobalSeed globalSeed "Definition of global seed via inner/outer";
equation
  connect(rwNoise.y, integrator.u) annotation(
    Line(points = {{-19, 50}, {-2, 50}}, color = {0, 0, 127}));
  connect(add.y, y) annotation(
    Line(points = {{72, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(integrator.y, add.u1) annotation(
    Line(points = {{22, 50}, {40, 50}, {40, 6}, {48, 6}}, color = {0, 0, 127}));
  connect(const.y, rwNoise.u) annotation(
    Line(points = {{-59, 50}, {-42, 50}}, color = {0, 0, 127}));
  connect(u, normalNoise.u) annotation(
    Line(points = {{-120, 0}, {-42, 0}}, color = {0, 0, 127}));
  connect(normalNoise.y, add.u2) annotation(
    Line(points = {{-19, 0}, {40, 0}, {40, -6}, {48, -6}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-38, -33}, {-32, -39}}, endAngle = 360), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{-81, 90}, {-89, 68}, {-73, 68}, {-81, 90}}), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-70, 3}, {-64, -3}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-28, -15}, {-22, -21}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{30, 53}, {36, 47}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-84, -13}, {-78, -19}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{48, -47}, {54, -53}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{40, 19}, {46, 13}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-46, 59}, {-40, 53}}, endAngle = 360), Line(points = {{-90, -23}, {82, -23}}, color = {192, 192, 192}), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{20, -19}, {26, -25}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-54, -23}, {-48, -29}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-2, 53}, {4, 47}}, endAngle = 360), Line(points = {{-81, 78}, {-81, -90}}, color = {192, 192, 192}), Line(points = {{-35, 25}, {-35, -35}, {-25, -35}, {-25, -17}, {-15, -17}, {-15, -45}, {-5, -45}, {-5, 37}, {1, 37}, {1, 51}, {7, 51}, {7, -5}, {17, -5}, {17, 7}, {23, 7}, {23, -23}, {33, -23}, {33, 49}, {43, 49}, {43, 15}, {51, 15}, {51, -51}, {61, -51}}, color = {0, 0, 127}, pattern = LinePattern.Dot), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{14, 9}, {20, 3}}, endAngle = 360), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{91, -22}, {69, -14}, {69, -30}, {91, -22}}), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-8, 39}, {-2, 33}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-62, -47}, {-56, -53}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-18, -41}, {-12, -47}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{4, -1}, {10, -7}}, endAngle = 360), Line(points = {{-81, -17}, {-67, -17}, {-67, -1}, {-59, -1}, {-59, -49}, {-51, -49}, {-51, -27}, {-43, -27}, {-43, 57}, {-35, 57}, {-35, 25}}, color = {0, 0, 127}, pattern = LinePattern.Dot), Text(lineColor = {175, 175, 175}, extent = {{-66, 92}, {94, 66}}, textString = "sigmaRW=sigmaRW"), Text(lineColor = {175, 175, 175}, extent = {{-70, -68}, {94, -96}}, textString = "sigma=%sigma"), Line(origin = {5, 0.35}, points = {{-86.0123, -23.0123}, {-54.0123, -9.01233}, {-40.0123, -17.0123}, {-14.0123, 2.98767}, {9.98767, -5.01233}, {31.9877, 10.9877}, {45.9877, -1.01233}, {63.9877, 4.98767}}, color = {255, 0, 0})}));
end ClockedSensorNoise;
