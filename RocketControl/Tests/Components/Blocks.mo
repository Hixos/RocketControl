within RocketControl.Tests.Components;

package Blocks
  model Track
    RocketControl.Components.Blocks.Track track annotation(
      Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const(k = 0) annotation(
      Placement(visible = true, transformation(origin = {-50, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Ramp ramp(duration = 10, height = 2 * pi) annotation(
      Placement(visible = true, transformation(origin = {-90, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Sin sin annotation(
      Placement(visible = true, transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Cos cos annotation(
      Placement(visible = true, transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Product product annotation(
      Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Sine sine(amplitude = 300, f = 2, offset = 300) annotation(
      Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Product product1 annotation(
      Placement(visible = true, transformation(origin = {-10, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.UnitConversions.To_deg track_true annotation(
      Placement(visible = true, transformation(origin = {72, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.WrapAngle wrapAngle annotation(
      Placement(visible = true, transformation(origin = {30, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(ramp.y, sin.u) annotation(
      Line(points = {{-78, 12}, {-70, 12}, {-70, -10}, {-62, -10}}, color = {0, 0, 127}));
    connect(ramp.y, cos.u) annotation(
      Line(points = {{-78, 12}, {-70, 12}, {-70, 30}, {-62, 30}}, color = {0, 0, 127}));
    connect(sin.y, product1.u2) annotation(
      Line(points = {{-38, -10}, {-22, -10}, {-22, -16}}, color = {0, 0, 127}));
    connect(cos.y, product.u2) annotation(
      Line(points = {{-38, 30}, {-22, 30}, {-22, 24}}, color = {0, 0, 127}));
    connect(product1.u1, product.u1) annotation(
      Line(points = {{-22, -4}, {-22, 36}}, color = {0, 255, 255}));
    connect(product.y, track.v[1]) annotation(
      Line(points = {{2, 30}, {38, 30}, {38, 0}}, color = {0, 0, 127}));
    connect(product1.y, track.v[2]) annotation(
      Line(points = {{2, -10}, {38, -10}, {38, 0}}, color = {0, 0, 127}));
    connect(const.y, track.v[3]) annotation(
      Line(points = {{-38, -42}, {38, -42}, {38, 0}}, color = {0, 0, 127}));
    connect(ramp.y, wrapAngle.u) annotation(
      Line(points = {{-78, 12}, {-68, 12}, {-68, 50}, {18, 50}}, color = {0, 0, 127}));
    connect(wrapAngle.y, track_true.u) annotation(
      Line(points = {{42, 50}, {60, 50}}, color = {0, 0, 127}));
    connect(sine.y, product.u1) annotation(
      Line(points = {{-78, 90}, {-22, 90}, {-22, 36}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Track;

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
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Blocks;
