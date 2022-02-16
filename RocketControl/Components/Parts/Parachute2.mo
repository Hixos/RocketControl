within RocketControl.Components.Parts;

model Parachute2
  extends RocketControl.Interfaces.PartialConditionalEnablePort(final useEnablePort = true);
  
  parameter SI.Length line_length;
  parameter Modelica.Units.SI.ModulusOfElasticity c;
  parameter Modelica.Units.SI.DampingCoefficient d;
  
  parameter Modelica.Units.SI.ModulusOfElasticity c_stowed;
  parameter Modelica.Units.SI.DampingCoefficient d_stowed;
    
  parameter SI.Mass mass;
  parameter SI.Area surface;
  parameter SI.Area initial_surface;
  parameter SI.Duration opening_transient_duration;
  
  parameter SI.Length stow_radius;
  parameter Real Cd;
  
  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a chute_link annotation(
    Placement(visible = true, transformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0), iconTransformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a stowed_position annotation(
    Placement(visible = true, transformation(extent = {{-116, -76}, {-84, -44}}, rotation = 0), iconTransformation(extent = {{-116, -82}, {-84, -50}}, rotation = 0)));
  RocketControl.Components.Parts.ParachuteLine2 parachuteLine2(c = c, d = d, s_unstretched = line_length)  annotation(
    Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b chute annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body(animation = false, m = mass, r_CM = {0, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {20, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  RocketControl.Components.Parts.SpringDamperParallel springDamperParallel(c = c_stowed, d = d_stowed, useEnablePort = true)  annotation(
    Placement(visible = true, transformation(origin = {-10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, cos(from_deg(0)), sin(from_deg(0))} * stow_radius)  annotation(
    Placement(visible = true, transformation(origin = {-50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = {0, cos(from_deg(120)), sin(from_deg(120))} * stow_radius) annotation(
    Placement(visible = true, transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation2(r = {0, cos(from_deg(240)), sin(from_deg(240))} * stow_radius) annotation(
    Placement(visible = true, transformation(origin = {-50, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation3(r = {0, cos(from_deg(240)), sin(from_deg(240))} * stow_radius) annotation(
    Placement(visible = true, transformation(origin = {30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation4(r = {0, cos(from_deg(0)), sin(from_deg(0))} * stow_radius) annotation(
    Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation5(r = {0, cos(from_deg(120)), sin(from_deg(120))} * stow_radius) annotation(
    Placement(visible = true, transformation(origin = {30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  RocketControl.Components.Parts.SpringDamperParallel springDamperParallel1(c = c_stowed, d = d_stowed, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {-10, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.SpringDamperParallel springDamperParallel2(c = c_stowed, d = d_stowed, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {-10, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.MathBoolean.Not not1 annotation(
    Placement(visible = true, transformation(origin = {-10, 66}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
  RocketControl.Aerodynamics.ParachuteAerodynamics parachuteAerodynamics(Cd = Cd,max_area = surface, min_area = initial_surface, opening_transient_duration = opening_transient_duration)  annotation(
    Placement(visible = true, transformation(origin = {90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Aerodynamics.AeroStateSensor aeroStateSensor annotation(
    Placement(visible = true, transformation(origin = {72, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
equation
  connect(chute_link, parachuteLine2.frame_a) annotation(
    Line(points = {{-100, 0}, {-75, 0}, {-75, 10}, {-60, 10}}));
  connect(parachuteLine2.frame_b, body.frame_a) annotation(
    Line(points = {{-40, 10}, {20, 10}, {20, 20}}, color = {95, 95, 95}));
  connect(body.frame_a, chute) annotation(
    Line(points = {{20, 20}, {20, 0}, {100, 0}}));
  connect(stowed_position, fixedTranslation.frame_a) annotation(
    Line(points = {{-100, -60}, {-80, -60}, {-80, -30}, {-60, -30}}));
  connect(stowed_position, fixedTranslation1.frame_a) annotation(
    Line(points = {{-100, -60}, {-60, -60}}));
  connect(stowed_position, fixedTranslation2.frame_a) annotation(
    Line(points = {{-100, -60}, {-80, -60}, {-80, -90}, {-60, -90}}));
  connect(fixedTranslation.frame_b, springDamperParallel.frame_a) annotation(
    Line(points = {{-40, -30}, {-20, -30}}, color = {95, 95, 95}));
  connect(fixedTranslation1.frame_b, springDamperParallel1.frame_a) annotation(
    Line(points = {{-40, -60}, {-20, -60}}, color = {95, 95, 95}));
  connect(fixedTranslation2.frame_b, springDamperParallel2.frame_a) annotation(
    Line(points = {{-40, -90}, {-20, -90}}));
  connect(body.frame_a, fixedTranslation4.frame_a) annotation(
    Line(points = {{20, 20}, {20, 0}, {52, 0}, {52, -30}, {40, -30}}, color = {95, 95, 95}));
  connect(body.frame_a, fixedTranslation5.frame_a) annotation(
    Line(points = {{20, 20}, {20, 0}, {52, 0}, {52, -60}, {40, -60}}, color = {95, 95, 95}));
  connect(body.frame_a, fixedTranslation3.frame_a) annotation(
    Line(points = {{20, 20}, {20, 0}, {52, 0}, {52, -90}, {40, -90}}, color = {95, 95, 95}));
  connect(fixedTranslation4.frame_b, springDamperParallel.frame_b) annotation(
    Line(points = {{20, -30}, {0, -30}}));
  connect(fixedTranslation5.frame_b, springDamperParallel1.frame_b) annotation(
    Line(points = {{20, -60}, {0, -60}}));
  connect(fixedTranslation3.frame_b, springDamperParallel2.frame_b) annotation(
    Line(points = {{20, -90}, {0, -90}}));
  connect(springDamperParallel2.enable, springDamperParallel1.enable) annotation(
    Line(points = {{-10, -80}, {-10, -50}}, color = {255, 0, 255}));
  connect(springDamperParallel1.enable, springDamperParallel.enable) annotation(
    Line(points = {{-10, -50}, {-10, -20}}, color = {255, 0, 255}));
  connect(enable, not1.u) annotation(
    Line(points = {{0, 106}, {0, 76}, {-10, 76}, {-10, 72}}, color = {255, 0, 255}));
  connect(not1.y, springDamperParallel.enable) annotation(
    Line(points = {{-10, 61}, {-10, -20}}, color = {255, 0, 255}));
  connect(parachuteAerodynamics.frame_b, body.frame_a) annotation(
    Line(points = {{80, 50}, {52, 50}, {52, 0}, {20, 0}, {20, 20}}, color = {95, 95, 95}));
  connect(enable, parachuteAerodynamics.deployed) annotation(
    Line(points = {{0, 106}, {0, 76}, {84, 76}, {84, 60}}, color = {255, 0, 255}));
  connect(parachuteLine2.stretched, parachuteAerodynamics.open) annotation(
    Line(points = {{-39, 3}, {46, 3}, {46, 60}, {94, 60}}, color = {255, 0, 255}));
  connect(aeroStateSensor.aeroStateOutput, parachuteAerodynamics.aeroStateInput) annotation(
    Line(points = {{72, 28}, {72, 44}, {80, 44}}));
  connect(body.frame_a, aeroStateSensor.frame_a) annotation(
    Line(points = {{20, 20}, {20, 0}, {72, 0}, {72, 8}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {Ellipse(origin = {-89, -75}, fillPattern = FillPattern.Solid, extent = {{-7, 7}, {7, -7}}), Polygon(origin = {32, 43}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Sphere, points = {{-82, 31}, {-36, 17}, {-2, -13}, {18, -55}, {22, -101}, {60, -55}, {68, -3}, {46, 37}, {6, 55}, {-36, 55}, {-82, 31}}), Line(origin = {-41.9419, -46.9462}, points = {{92.6532, 35.7916}, {-47.3468, -28.2084}, {96.6532, -10.2084}}), Line(origin = {-100.514, -2.19848}, points = {{51.7615, 76.6624}, {9.7615, -73.3376}, {97.761, 62.6624}}), Text(origin = {0, 14}, lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name")}));
end Parachute2;
