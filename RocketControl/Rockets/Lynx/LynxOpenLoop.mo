within RocketControl.Rockets.Lynx;

model LynxOpenLoop
  extends Rockets.Internal.PartialRocket;
  outer RocketControl.World.SimOptions opt;
  parameter SI.Duration start_delay = 0.5;
  replaceable RocketControl.Rockets.Lynx.AerodynamicsWithCanards.Aerodynamics aerodynamics annotation(
    Placement(visible = true, transformation(origin = {70, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation to_parachute_link(animation = false, r = {0.61, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {10, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation to_lug_aft(animation = false, r = {-0.408, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation to_lug_bow(animation = false, r = {-0.009, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {-70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Body fuselage(I_11 = 0.06, I_21 = 0, I_22 = 6.436, I_31 = 0, I_32 = 0, I_33 = 6.436, animation = false, enforceStates = false, m = 18.362, r_CM = {0, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
    Placement(visible = true, transformation(origin = {-40, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation nozzle_trans(animation = false, r = {-1.150, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-40, -48}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Components.Parachutes.SimpleParachute drogue(CLL_p = -5, Cd = 0.9, d = 0.15, initial_surface = 0.01, opening_delay = 0.5, opening_transient_duration = 0.2, surface = 0.6, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {60, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parachutes.SimpleParachute main(CLL_p = -5, Cd = 0.9, d = 0.15, initial_surface = 0.1, opening_delay = 1, opening_transient_duration = 0.7, surface = 10.6, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {80, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Propulsion.L1350 l1350(start_delay = start_delay) annotation(
    Placement(visible = true, transformation(origin = {-40, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Blocks.Math.Vectors.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4) annotation(
    Placement(visible = true, transformation(origin = {4, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanConstant booleanConstant(k = false)  annotation(
    Placement(visible = true, transformation(origin = {50, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
//  for i in 1:4 loop
//    sample2.u[i] = der(deflection2Control.control[i]);
//  end for;
  connect(to_lug_bow.frame_b, frame_lug_bow) annotation(
    Line(points = {{-80, -10}, {-100, -10}, {-100, 60}}, color = {95, 95, 95}));
  connect(ref_center, to_lug_aft.frame_a) annotation(
    Line(points = {{100, 0}, {-10, 0}, {-10, -30}, {-60, -30}}));
  connect(to_lug_aft.frame_b, frame_lug_aft) annotation(
    Line(points = {{-80, -30}, {-100, -30}, {-100, -40}}, color = {95, 95, 95}));
  connect(to_lug_aft.frame_a, to_lug_bow.frame_a) annotation(
    Line(points = {{-60, -30}, {-50, -30}, {-50, -10}, {-60, -10}}, color = {95, 95, 95}));
  connect(to_lug_aft.frame_a, nozzle_trans.frame_a) annotation(
    Line(points = {{-60, -30}, {-40, -30}, {-40, -38}}, color = {95, 95, 95}));
  connect(to_parachute_link.frame_a, to_lug_aft.frame_a) annotation(
    Line(points = {{0, -60}, {-20, -60}, {-20, -30}, {-60, -30}}, color = {95, 95, 95}));
  connect(aerodynamics.frame_b, ref_center) annotation(
    Line(points = {{60, 80}, {28, 80}, {28, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(to_parachute_link.frame_b, drogue.frame_a) annotation(
    Line(points = {{20, -60}, {40, -60}, {40, -30}, {50, -30}}, color = {95, 95, 95}));
  connect(to_parachute_link.frame_b, main.frame_a) annotation(
    Line(points = {{20, -60}, {40, -60}, {40, -70}, {70, -70}}, color = {95, 95, 95}));
  connect(nozzle_trans.frame_a, fuselage.frame_a) annotation(
    Line(points = {{-40, -38}, {-40, 60}}));
  connect(nozzle_trans.frame_b, l1350.frame_b) annotation(
    Line(points = {{-40, -58}, {-40, -72}}));
  connect(vectorConstant.v, aerodynamics.finDeflection) annotation(
    Line(points = {{16, 60}, {46, 60}, {46, 74}, {60, 74}}, color = {0, 0, 127}, thickness = 0.5));
  connect(booleanConstant.y, drogue.enable) annotation(
    Line(points = {{62, 32}, {60, 32}, {60, -20}}, color = {255, 0, 255}));
  connect(booleanConstant.y, main.enable) annotation(
    Line(points = {{62, 32}, {80, 32}, {80, -60}}, color = {255, 0, 255}));
protected
  annotation(
    Icon(coordinateSystem(grid = {2, 0})),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
    Diagram(graphics = {Text(origin = {-31, -21}, extent = {{-19, 7}, {19, -7}}, textString = "ref_center", textStyle = {TextStyle.Italic})}));
end LynxOpenLoop;
