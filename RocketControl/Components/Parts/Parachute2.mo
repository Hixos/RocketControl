within RocketControl.Components.Parts;

model Parachute2
  extends RocketControl.Interfaces.PartialConditionalEnablePort(final useEnablePort = true);
  
  // Flange
  parameter Modelica.Units.SI.ModulusOfElasticity c_flange;
  parameter Modelica.Units.SI.DampingCoefficient d_flange;
  parameter SI.Length flange_radius;
  parameter SI.Force separation_force = 0;
  parameter SI.Duration separation_duration = 0.2;
  
  // Line
  parameter SI.Length line_length;
  parameter Modelica.Units.SI.ModulusOfElasticity c_line;
  parameter Modelica.Units.SI.DampingCoefficient d_line;
  
  
  // Parachute
  parameter SI.Mass mass;
  parameter SI.Area surface;
  parameter SI.Area initial_surface;
  parameter SI.Duration opening_transient_duration;
  parameter Real Cd;
  
  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a chute_link annotation(
    Placement(visible = true, transformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0), iconTransformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a flange_position annotation(
    Placement(visible = true, transformation(extent = {{-116, -76}, {-84, -44}}, rotation = 0), iconTransformation(extent = {{-116, -82}, {-84, -50}}, rotation = 0)));
  RocketControl.Components.Parts.ParachuteLine2 parachuteLine(c = c_line, d = d_line, s_unstretched = line_length)  annotation(
    Placement(visible = true, transformation(origin = {-52, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b chute annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body chute_body(animation = false, m = mass, r_CM = {0, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {0, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  RocketControl.Aerodynamics.ParachuteAerodynamics parachuteAerodynamics(Cd = Cd,max_area = surface, min_area = initial_surface, opening_transient_duration = opening_transient_duration)  annotation(
    Placement(visible = true, transformation(origin = {70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Aerodynamics.AeroStateSensor aeroStateSensor annotation(
    Placement(visible = true, transformation(origin = {28, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.SeparableFlange separableFlange(c = c_flange, d = d_flange, flange_radius = flange_radius, separation_duration = separation_duration, separation_force = separation_force)  annotation(
    Placement(visible = true, transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit1(angle_1(fixed = true), angle_2(fixed = true), angle_3(fixed = true), r_rel_a_1(fixed = true), r_rel_a_2(fixed = true), r_rel_a_3(fixed = true), use_angle = true, use_r = true) annotation(
    Placement(visible = true, transformation(origin = {-50, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(chute_link, parachuteLine.frame_a) annotation(
    Line(points = {{-100, 0}, {-62, 0}}));
  connect(flange_position, separableFlange.frame_a) annotation(
    Line(points = {{-100, -60}, {-60, -60}}));
  connect(parachuteLine.frame_b, chute_body.frame_a) annotation(
    Line(points = {{-42, 0}, {0, 0}, {0, 4}}));
  connect(separableFlange.frame_b, chute_body.frame_a) annotation(
    Line(points = {{-40, -60}, {0, -60}, {0, 4}}));
  connect(chute_body.frame_a, aeroStateSensor.frame_a) annotation(
    Line(points = {{0, 4}, {0, -50}, {18, -50}}, color = {95, 95, 95}));
  connect(aeroStateSensor.aeroStateOutput, parachuteAerodynamics.aeroStateInput) annotation(
    Line(points = {{38, -50}, {48, -50}, {48, -36}, {60, -36}}));
  connect(chute_body.frame_a, chute) annotation(
    Line(points = {{0, 4}, {0, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(parachuteLine.extended, parachuteAerodynamics.extended) annotation(
    Line(points = {{-40, -6}, {74, -6}, {74, -20}}, color = {255, 0, 255}));
  connect(enable, parachuteAerodynamics.deployed) annotation(
    Line(points = {{0, 106}, {0, 30}, {64, 30}, {64, -20}}, color = {255, 0, 255}));
  connect(chute_body.frame_a, parachuteAerodynamics.frame_b) annotation(
    Line(points = {{0, 4}, {0, -30}, {60, -30}}));
  connect(enable, separableFlange.enable) annotation(
    Line(points = {{0, 106}, {0, 30}, {-72, 30}, {-72, -40}, {-50, -40}, {-50, -50}}, color = {255, 0, 255}));
  connect(separableFlange.frame_a, freeMotionScalarInit1.frame_a) annotation(
    Line(points = {{-60, -60}, {-76, -60}, {-76, -90}, {-60, -90}}, color = {95, 95, 95}));
  connect(separableFlange.frame_b, freeMotionScalarInit1.frame_b) annotation(
    Line(points = {{-40, -60}, {-22, -60}, {-22, -90}, {-40, -90}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {Ellipse(origin = {-89, -75}, fillPattern = FillPattern.Solid, extent = {{-7, 7}, {7, -7}}), Polygon(origin = {32, 43}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Sphere, points = {{-82, 31}, {-36, 17}, {-2, -13}, {18, -55}, {22, -101}, {60, -55}, {68, -3}, {46, 37}, {6, 55}, {-36, 55}, {-82, 31}}), Line(origin = {-41.9419, -46.9462}, points = {{92.6532, 35.7916}, {-47.3468, -28.2084}, {96.6532, -10.2084}}), Line(origin = {-100.514, -2.19848}, points = {{51.7615, 76.6624}, {9.7615, -73.3376}, {97.761, 62.6624}}), Text(origin = {0, 14}, lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name")}));
end Parachute2;
