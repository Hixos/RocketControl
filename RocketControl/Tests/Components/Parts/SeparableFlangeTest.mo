within RocketControl.Tests.Components.Parts;

model SeparableFlangeTest
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
    Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 2, r_CM = {0, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {6, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Body body1(m = 2, r_CM = {0, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(angle_1(fixed = true), angle_2(fixed = true), angle_3(fixed = true),r_rel_a_1(fixed = true), r_rel_a_2(fixed = true), r_rel_a_3(fixed = true), use_angle = true, use_r = true)  annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.SeparableFlange separableFlange(separation_force = 200, separation_duration = 0.3)  annotation(
    Placement(visible = true, transformation(origin = {38, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y = time > 1)  annotation(
    Placement(visible = true, transformation(origin = {12, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(
    Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit1(angle_1(fixed = true), angle_2(fixed = true), angle_3(fixed = true), r_rel_a_1(fixed = true), r_rel_a_2(fixed = true), r_rel_a_3(fixed = true), use_angle = true, use_r = true) annotation(
    Placement(visible = true, transformation(origin = {42, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(fixed.frame_b, freeMotionScalarInit.frame_a) annotation(
    Line(points = {{-80, 0}, {-60, 0}}));
  connect(freeMotionScalarInit.frame_b, body.frame_a) annotation(
    Line(points = {{-40, 0}, {16, 0}}));
  connect(body.frame_a, separableFlange.frame_a) annotation(
    Line(points = {{16, 0}, {28, 0}}, color = {95, 95, 95}));
  connect(separableFlange.frame_b, body1.frame_a) annotation(
    Line(points = {{48, 0}, {60, 0}}, color = {95, 95, 95}));
  connect(booleanExpression.y, separableFlange.enable) annotation(
    Line(points = {{24, 52}, {38, 52}, {38, 10}}, color = {255, 0, 255}));
  connect(separableFlange.frame_a, freeMotionScalarInit1.frame_a) annotation(
    Line(points = {{28, 0}, {26, 0}, {26, -36}, {32, -36}}));
  connect(freeMotionScalarInit1.frame_b, body1.frame_a) annotation(
    Line(points = {{52, -36}, {60, -36}, {60, 0}}, color = {95, 95, 95}));

annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end SeparableFlangeTest;
