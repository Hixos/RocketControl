within RocketControl.Components.LaunchPad;

model LaunchLug
  extends Interfaces.PartialConditionalEnablePort;
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  parameter Modelica.Units.SI.ModulusOfElasticity c_ax = 1 "Axial spring stiffness";
  parameter Modelica.Units.SI.DampingCoefficient d_ax = 1 "Axial dampening coefficient";
  parameter Modelica.Units.SI.ModulusOfElasticity c_norm = 1 "Normal spring stiffness";
  parameter Modelica.Units.SI.DampingCoefficient d_norm = 1 "Normal dampening coefficient";
  parameter SI.Length lug_length;
  parameter Real n_ax[3] = {0, 0, 1} "Axial direction of the launch lug (constrained movement)";
  parameter Real n_free[3] = {1, 0, 0} "Direction where the relative movement of frame_a and frame_b is not constrained (with respect to frame_a)";
  parameter SI.Length s_0_ax = 0 "Undeformed axial spring length";
  parameter SI.Length s_start_ax = 0 "Inital axial spring deformation";
  parameter Boolean s_fixed_ax = false;
  parameter SI.Length s_0_norm = 0 "Undeformed axial spring length";
  parameter SI.Length s_start_norm = 0 "Inital axial spring deformation";
  parameter Boolean s_fixed_norm = false;
  parameter Boolean s_fixed_norm2 = false;
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lugTranslation1(r = -n_ax * lug_length) annotation(
    Placement(visible = true, transformation(origin = {20, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Components.Parts.LinearSpringDamperParallel normalRailSpring1(c = c_norm / 2, d = d_norm / 2, n = n_norm, s_0 = s_0_norm, s_start = s_start_norm, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Components.Parts.LinearSpringDamperParallel normalRailSpring2(c = c_norm / 2, d = d_norm / 2, n = n_norm, s_0 = s_0_norm, s_fixed = s_fixed_norm2, s_start = s_start_norm, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Components.Parts.LinearSpringDamperParallel axialRailSpring(c = c_ax, d = d_ax, n = n_ax, s_0 = s_0_ax, s_start = s_start_ax, s_fixed = s_fixed_ax, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lugTranslation2(r = n_ax * lug_length) annotation(
    Placement(visible = true, transformation(origin = {20, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  parameter Real n_norm[3] = cross(n_ax, n_free) "Normal direction of the launch lug";
equation
  connect(frame_a, axialRailSpring.frame_a) annotation(
    Line(points = {{-100, 0}, {-80, 0}, {-80, 40}, {-60, 40}}));
  connect(normalRailSpring1.frame_b, lugTranslation1.frame_a) annotation(
    Line(points = {{50, 10}, {50, 40}, {30, 40}}, color = {95, 95, 95}));
  connect(lugTranslation1.frame_b, normalRailSpring2.frame_b) annotation(
    Line(points = {{10, 40}, {-10, 40}, {-10, 10}}, color = {95, 95, 95}));
  connect(axialRailSpring.frame_b, lugTranslation1.frame_b) annotation(
    Line(points = {{-40, 40}, {10, 40}}));
  connect(frame_a, lugTranslation2.frame_a) annotation(
    Line(points = {{-100, 0}, {-80, 0}, {-80, -40}, {10, -40}}));
  connect(lugTranslation2.frame_b, normalRailSpring1.frame_a) annotation(
    Line(points = {{30, -40}, {50, -40}, {50, -10}}, color = {95, 95, 95}));
  connect(lugTranslation2.frame_a, normalRailSpring2.frame_a) annotation(
    Line(points = {{10, -40}, {-10, -40}, {-10, -10}}));
  connect(normalRailSpring2.enable, enable) annotation(
    Line(points = {{-20, 0}, {-28, 0}, {-28, 60}, {0, 60}, {0, 106}}, color = {255, 0, 255}));
  connect(axialRailSpring.enable, enable) annotation(
    Line(points = {{-50, 50}, {-50, 60}, {0, 60}, {0, 106}}, color = {255, 0, 255}));
  connect(normalRailSpring1.enable, enable) annotation(
    Line(points = {{40, 0}, {20, 0}, {20, 20}, {-28, 20}, {-28, 60}, {0, 60}, {0, 106}}, color = {255, 0, 255}));
  connect(lugTranslation1.frame_a, frame_b) annotation(
    Line(points = {{30, 40}, {80, 40}, {80, 0}, {100, 0}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {Polygon(origin = {-7, 2}, fillColor = {139, 139, 139}, fillPattern = FillPattern.HorizontalCylinder, points = {{101, 28}, {101, -28}, {-53, -28}, {-53, -74}, {-89, -24}, {-89, 18}, {-53, 68}, {-53, 28}, {9, 28}, {101, 28}}), Text(lineColor = {0, 0, 255}, extent = {{-150, 85}, {150, 45}}, textString = "%name")}));
end LaunchLug;
