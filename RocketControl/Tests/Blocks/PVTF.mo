within RocketControl.Tests.Blocks;

model PVTF
  RocketControl.Blocks.Math.ParameterVaryingTransferFunction parameterVaryingTF(n = 1)  annotation(
    Placement(visible = true, transformation(origin = {2, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant vectorConstant(k = {1, 2 * 3.14 * 2}, n = 2)  annotation(
    Placement(visible = true, transformation(origin = {-152, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant vectorConstant1(k = {0, 2 * 3.14 * 2}, n = 2) annotation(
    Placement(visible = true, transformation(origin = {-148, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorAdd vectorAdd(n = 2)  annotation(
    Placement(visible = true, transformation(origin = {-86, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Noise.BandLimitedWhiteNoise bandLimitedWhiteNoise(noisePower = 0.01, samplePeriod = 0.01, useAutomaticLocalSeed = true, useGlobalSeed = true)  annotation(
    Placement(visible = true, transformation(origin = {-108, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine(amplitude = 3, f = 1)  annotation(
    Placement(visible = true, transformation(origin = {-144, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0)  annotation(
    Placement(visible = true, transformation(origin = {-208, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(vectorConstant.v, parameterVaryingTF.a) annotation(
    Line(points = {{-140, -26}, {-10, -26}, {-10, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorAdd.vc, parameterVaryingTF.b) annotation(
    Line(points = {{-75, 12}, {-43.5, 12}, {-43.5, 16}, {-10, 16}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConstant1.v, vectorAdd.v2) annotation(
    Line(points = {{-136, 10}, {-108, 10}, {-108, 8}, {-98, 8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(sine.y, vectorAdd.v1[2]) annotation(
    Line(points = {{-132, 62}, {-122, 62}, {-122, 16}, {-98, 16}}, color = {0, 0, 127}));
  connect(const.y, vectorAdd.v1[1]) annotation(
    Line(points = {{-196, 34}, {-98, 34}, {-98, 16}}, color = {0, 0, 127}));
  connect(bandLimitedWhiteNoise.y, parameterVaryingTF.u) annotation(
    Line(points = {{-96, -76}, {-52, -76}, {-52, 8}, {-10, 8}}, color = {0, 0, 127}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end PVTF;
