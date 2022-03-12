within RocketControl.Rockets.Lynx.GNC;

model RealGNCAng
extends RocketControl.Icons.Guidance;

outer World.SimOptions opt;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 44}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Navigation.Navigation navigation annotation(
    Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.RealSensors realSensors annotation(
    Placement(visible = true, transformation(origin = {-10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.GuidanceControl.AngularRateGC guidance_control annotation(
    Placement(visible = true, transformation(origin = {-12, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_a, realSensors.frame_a) annotation(
    Line(points = {{-100, 0}, {-88, 0}, {-88, 50}, {-20, 50}}));
  connect(navigation.bus, bus) annotation(
    Line(points = {{0, 0}, {100, 0}}, thickness = 0.5));
  connect(bus, guidance_control.bus) annotation(
    Line(points = {{100, 0}, {100, -50}, {-2, -50}}, thickness = 0.5));
  connect(bus, realSensors.bus) annotation(
    Line(points = {{100, 0}, {100, 50}, {0, 50}}, thickness = 0.5));
  annotation(
    Icon(graphics = {Text(origin = {-10, -254}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name")}));
end RealGNCAng;
