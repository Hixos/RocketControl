within RocketControl.Rockets.Lynx.GNC.Navigation;

model LynxIdealNavigation
  extends Rockets.Internal.PartialNavigationSystem;
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Quaternion2Euler quaternion2Euler annotation(
    Placement(visible = true, transformation(origin = {10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Navigation.FlightPathBlocks.Track track annotation(
    Placement(visible = true, transformation(origin = {10, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Navigation.FlightPathBlocks.ClimbAngle climbAngle annotation(
    Placement(visible = true, transformation(origin = {10, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Navigation.FlightPathBlocks.Downrange downrange annotation(
    Placement(visible = true, transformation(origin = {10, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  Modelica.Blocks.Interfaces.RealInput v_est[3] annotation(
    Placement(visible = true, transformation(extent = {{-8, 72}, {8, 88}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput x_est[3] annotation(
    Placement(visible = true, transformation(extent = {{-8, 52}, {8, 68}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput w_est[3] annotation(
    Placement(visible = true, transformation(extent = {{-8, 32}, {8, 48}}, rotation = 0)));
equation
  connect(bus.v_meas, v_est) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 80}, {0, 80}}, thickness = 0.5));
  connect(v_est, bus.v_est) annotation(
    Line(points = {{0, 80}, {60, 80}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.x_meas, x_est) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 60}, {0, 60}}, thickness = 0.5));
  connect(x_est, bus.x_est) annotation(
    Line(points = {{0, 60}, {60, 60}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.w_meas, w_est) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 40}, {0, 40}}, thickness = 0.5));
  connect(w_est, bus.w_est) annotation(
    Line(points = {{0, 40}, {60, 40}, {60, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.v_est, track.v) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -20}, {-2, -20}}, thickness = 0.5));
  connect(bus.q_est, quaternion2Euler.q) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 10}, {-2, 10}}, thickness = 0.5));
  connect(track.v, climbAngle.v) annotation(
    Line(points = {{-2, -20}, {-14, -20}, {-14, -50}, {-2, -50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(climbAngle.v, downrange.x) annotation(
    Line(points = {{-2, -50}, {-14, -50}, {-14, -80}, {-2, -80}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end LynxIdealNavigation;
