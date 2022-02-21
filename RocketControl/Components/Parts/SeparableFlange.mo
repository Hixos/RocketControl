within RocketControl.Components.Parts;

model SeparableFlange
extends Interfaces.PartialConditionalEnablePort(useEnablePort=true);
extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;

  parameter Modelica.Units.SI.ModulusOfElasticity c = 10000;
  parameter Modelica.Units.SI.DampingCoefficient d = 300;
  parameter SI.Distance flange_radius = 0.075;
  
  parameter SI.Force separation_force = 0;
  parameter SI.Duration separation_duration = 0;
  
  RocketControl.Components.Parts.SpringDamperParallel springDamperParallel(c = c, d = d, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.SpringDamperParallel springDamperParallel2(c = c, d = d, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, cos(from_deg(0)), sin(from_deg(0))} * flange_radius) annotation(
    Placement(visible = true, transformation(origin = {-40, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation2(r = {0, cos(from_deg(240)), sin(from_deg(240))} * flange_radius) annotation(
    Placement(visible = true, transformation(origin = {-40, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation5(r = {0, cos(from_deg(120)), sin(from_deg(120))} * flange_radius) annotation(
    Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation4(r = {0, cos(from_deg(0)), sin(from_deg(0))} * flange_radius) annotation(
    Placement(visible = true, transformation(origin = {40, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation3(r = {0, cos(from_deg(240)), sin(from_deg(240))} * flange_radius) annotation(
    Placement(visible = true, transformation(origin = {40, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  RocketControl.Components.Parts.SpringDamperParallel springDamperParallel1(c = c, d = d, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = {0, cos(from_deg(120)), sin(from_deg(120))} * flange_radius) annotation(
    Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.Force force annotation(
    Placement(visible = true, transformation(origin = {0, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  
  SI.Time separation_time;
  Modelica.Blocks.MathBoolean.Not not1 annotation(
    Placement(visible = true, transformation(origin = {0, 64}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
initial equation
  separation_time = 0;
equation
  when enable then
    separation_time = time;
  end when;
  
  if not enable then
    force.force = {0,0,0};
  elseif enable and time < separation_time + separation_duration then
    force.force = {separation_force,0,0};
  else
    force.force = {0,0,0};
  end if;
  
  connect(fixedTranslation2.frame_b, springDamperParallel2.frame_a) annotation(
    Line(points = {{-30, -30}, {-10, -30}}));
  connect(fixedTranslation3.frame_b, springDamperParallel2.frame_b) annotation(
    Line(points = {{30, -30}, {10, -30}}));
  connect(springDamperParallel2.enable, springDamperParallel1.enable) annotation(
    Line(points = {{0, -20.8}, {0, 9.2}}, color = {255, 0, 255}));
  connect(fixedTranslation.frame_b, springDamperParallel.frame_a) annotation(
    Line(points = {{-30, 30}, {-10, 30}}, color = {95, 95, 95}));
  connect(fixedTranslation5.frame_b, springDamperParallel1.frame_b) annotation(
    Line(points = {{30, 0}, {10, 0}}));
  connect(fixedTranslation1.frame_b, springDamperParallel1.frame_a) annotation(
    Line(points = {{-30, 0}, {-10, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation4.frame_b, springDamperParallel.frame_b) annotation(
    Line(points = {{30, 30}, {10, 30}}));
  connect(springDamperParallel1.enable, springDamperParallel.enable) annotation(
    Line(points = {{0, 9.2}, {0, 39.2}}, color = {255, 0, 255}));
  connect(frame_a, fixedTranslation.frame_a) annotation(
    Line(points = {{-100, 0}, {-80, 0}, {-80, 30}, {-50, 30}}));
  connect(frame_a, fixedTranslation1.frame_a) annotation(
    Line(points = {{-100, 0}, {-50, 0}}));
  connect(frame_a, fixedTranslation2.frame_a) annotation(
    Line(points = {{-100, 0}, {-80, 0}, {-80, -30}, {-50, -30}}));
  connect(fixedTranslation4.frame_a, frame_b) annotation(
    Line(points = {{50, 30}, {80, 30}, {80, 0}, {100, 0}}));
  connect(fixedTranslation5.frame_a, frame_b) annotation(
    Line(points = {{50, 0}, {100, 0}}));
  connect(fixedTranslation3.frame_a, frame_b) annotation(
    Line(points = {{50, -30}, {80, -30}, {80, 0}, {100, 0}}));
  connect(frame_a, force.frame_a) annotation(
    Line(points = {{-100, 0}, {-80, 0}, {-80, -80}, {-10, -80}}));
  connect(force.frame_b, frame_b) annotation(
    Line(points = {{10, -80}, {80, -80}, {80, 0}, {100, 0}}));
  connect(enable, not1.u) annotation(
    Line(points = {{0, 106}, {0, 70}}, color = {255, 0, 255}));
  connect(not1.y, springDamperParallel.enable) annotation(
    Line(points = {{0, 60}, {0, 40}}, color = {255, 0, 255}));
  annotation(
    Icon(graphics = {Rectangle(origin = {-32, 0}, fillColor = {175, 175, 175}, fillPattern = FillPattern.Solid, extent = {{-20, 66}, {20, -66}}), Rectangle(origin = {32, 0}, fillColor = {175, 175, 175}, fillPattern = FillPattern.Solid, extent = {{-20, 66}, {20, -66}}), Rectangle(fillColor = {255, 170, 0}, fillPattern = FillPattern.Solid, extent = {{-12, 66}, {12, -66}}), Line(origin = {-74, 0}, points = {{-22, 0}, {22, 0}}), Line(origin = {74, 0}, points = {{-22, 0}, {22, 0}}), Text(origin = {0, 8}, lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Text(origin = {0, -79}, extent = {{-100, 21}, {100, -21}}, textString = "f=%separation_force")}));
end SeparableFlange;
