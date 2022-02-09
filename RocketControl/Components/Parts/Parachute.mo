within RocketControl.Components.Parts;

model Parachute
  extends RocketControl.Interfaces.PartialConditionalEnablePort;
  
  parameter SI.Length line_length;
  parameter Modelica.Units.SI.ModulusOfElasticity c;
  parameter Modelica.Units.SI.ModulusOfElasticity c_stowed;
  parameter Modelica.Units.SI.DampingCoefficient d_stowed;
    
  parameter SI.Mass mass;
  parameter SI.Area surface;
  parameter SI.Area initial_surface;
  parameter SI.Duration opening_transient_duration;
  parameter Real Cd;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0), iconTransformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body chute(animation = false,m = mass, r_CM = {0, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {20, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  RocketControl.Aerodynamics.AeroStateSensor aeroStateSensor annotation(
    Placement(visible = true, transformation(origin = {36, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.ParachuteLine parachuteLine(c = c, c_stowed = c_stowed, d_stowed = d_stowed, extended_length = line_length, r_rel(each fixed = true), useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a1 annotation(
    Placement(visible = true, transformation(extent = {{-116, -76}, {-84, -44}}, rotation = 0), iconTransformation(extent = {{-116, -82}, {-84, -50}}, rotation = 0)));
  Aerodynamics.ParachuteAerodynamics parachuteAerodynamics(Cd = Cd, max_area = surface, min_area = initial_surface, opening_transient_duration = opening_transient_duration) annotation(
    Placement(visible = true, transformation(origin = {64, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_a, parachuteLine.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}}));
  connect(parachuteLine.frame_b, chute.frame_a) annotation(
    Line(points = {{-20, 0}, {20, 0}, {20, 18}}));
  connect(enable, parachuteLine.enable) annotation(
    Line(points = {{0, 106}, {-2, 106}, {-2, 44}, {-30, 44}, {-30, 10}}, color = {255, 0, 255}));
  connect(frame_a1, chute.frame_a) annotation(
    Line(points = {{-100, -60}, {20, -60}, {20, 18}}));
  connect(parachuteLine.extended, parachuteAerodynamics.open) annotation(
    Line(points = {{-20, 8}, {38, 8}, {38, 20}, {68, 20}, {68, 10}}, color = {255, 0, 255}));
  connect(enable, parachuteAerodynamics.deployed) annotation(
    Line(points = {{0, 106}, {0, 44}, {58, 44}, {58, 10}}, color = {255, 0, 255}));
  connect(parachuteAerodynamics.frame_b, chute.frame_a) annotation(
    Line(points = {{54, 0}, {20, 0}, {20, 18}}, color = {95, 95, 95}));
  connect(chute.frame_a, aeroStateSensor.frame_a) annotation(
    Line(points = {{20, 18}, {20, -42}, {26, -42}}, color = {95, 95, 95}));
  connect(aeroStateSensor.aeroStateOutput, parachuteAerodynamics.aeroStateInput) annotation(
    Line(points = {{46, -42}, {54, -42}, {54, -6}}));
  annotation(
    Icon(graphics = {Line(origin = {-41.9419, -46.9462}, points = {{92.6532, 35.7916}, {-47.3468, -28.2084}, {96.6532, -10.2084}}), Line(origin = {-100.514, -2.19848}, points = {{51.7615, 76.6624}, {9.7615, -73.3376}, {97.761, 62.6624}}), Polygon(origin = {32, 43}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Sphere, points = {{-82, 31}, {-36, 17}, {-2, -13}, {18, -55}, {22, -101}, {60, -55}, {68, -3}, {46, 37}, {6, 55}, {-36, 55}, {-82, 31}}), Ellipse(origin = {-89, -75}, fillPattern = FillPattern.Solid, extent = {{-7, 7}, {7, -7}}), Text(origin = {0, 14},lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name")}),
    Diagram);
end Parachute;
