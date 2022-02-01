within RocketControl.Tests.Blocks;

model EulerRatesTest
  RocketControl.Components.Blocks.EulerRates eulerRates annotation(
    Placement(visible = true, transformation(origin = {80, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Continuous.IdealAsset idealAsset annotation(
    Placement(visible = true, transformation(origin = {4, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
    Placement(visible = true, transformation(origin = {-90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.Continuous.IdealGyroscope idealGyroscope annotation(
    Placement(visible = true, transformation(origin = {0, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 1, r_CM = {0, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-32, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(
    Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(angle_1(fixed = true), angle_2(fixed = true, start = from_deg(90)), angle_3(fixed = true), sequence_start = {3, 2, 1}, use_w = true, w_rel_b_1(fixed = true, start = from_deg(45)), w_rel_b_2(fixed = true, start = from_deg(45)), w_rel_b_3(fixed = true, start = 0)) annotation(
    Placement(visible = true, transformation(origin = {-60, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Math.Blocks.Quaternion2Euler quaternion2Euler annotation(
    Placement(visible = true, transformation(origin = {52, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {1, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-12, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(idealAsset.q, eulerRates.q) annotation(
    Line(points = {{15, -18}, {41, -18}, {41, 12}, {67, 12}}, color = {0, 0, 127}, thickness = 0.5));
  connect(idealGyroscope.w, eulerRates.w) annotation(
    Line(points = {{10.2, 56}, {44.2, 56}, {44.2, 24}, {68.2, 24}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixed.frame_b, freeMotionScalarInit.frame_a) annotation(
    Line(points = {{-80, -30}, {-70, -30}}, color = {95, 95, 95}));
  connect(freeMotionScalarInit.frame_b, body.frame_a) annotation(
    Line(points = {{-50, -30}, {-42, -30}}, color = {95, 95, 95}));
  connect(idealAsset.frame_a, body.frame_a) annotation(
    Line(points = {{-6, -18}, {-42, -18}, {-42, -30}}));
  connect(idealGyroscope.frame_a, body.frame_a) annotation(
    Line(points = {{-10, 56}, {-42, 56}, {-42, -30}}));
  connect(idealAsset.q, quaternion2Euler.q) annotation(
    Line(points = {{16, -18}, {28, -18}, {28, -62}, {40, -62}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedTranslation.frame_a, body.frame_a) annotation(
    Line(points = {{-22, 12}, {-42, 12}, {-42, -30}}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end EulerRatesTest;
