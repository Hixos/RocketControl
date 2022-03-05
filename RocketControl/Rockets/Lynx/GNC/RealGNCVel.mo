within RocketControl.Rockets.Lynx.GNC;

model RealGNCVel
extends RocketControl.Icons.Guidance;

outer World.SimOptions opt;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Navigation.Navigation navigation annotation(
    Placement(visible = true, transformation(origin = {-10, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Guidance.ConstantFlightPathGuidanceDiscrete constantFlightPathGuidanceDiscrete(dt = opt.samplePeriodMs / 1000, flightpathangle (displayUnit = "deg") = 1.396263401595464, heading = 0, int_lim = 60, k = 0.3, kint = 4)  annotation(
    Placement(visible = true, transformation(origin = {-40, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant pqr_zero(k = {0, 0, 0}, n = 3) annotation(
    Placement(visible = true, transformation(origin = {-40, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.RealSensors realSensors annotation(
    Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.AccelerationRollRateControlVel accelerationRollRateControlVel annotation(
    Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(bus, constantFlightPathGuidanceDiscrete.bus) annotation(
    Line(points = {{100, 0}, {100, -100}, {-70, -100}, {-70, -8}, {-50, -8}}, thickness = 0.5));
  connect(frame_a, realSensors.frame_a) annotation(
    Line(points = {{-100, 0}, {-88, 0}, {-88, 90}, {-20, 90}}));
  connect(realSensors.bus, bus) annotation(
    Line(points = {{0, 90}, {100, 90}, {100, 0}}, thickness = 0.5));
  connect(navigation.bus, bus) annotation(
    Line(points = {{0, 60}, {100, 60}, {100, 0}}, thickness = 0.5));
  connect(constantFlightPathGuidanceDiscrete.acc_err_int, accelerationRollRateControlVel.acc_err_int) annotation(
    Line(points = {{-28, -8}, {-6, -8}, {-6, -4}, {18, -4}}, color = {0, 0, 127}, thickness = 0.5));
  connect(accelerationRollRateControlVel.accelerationVelOutput.bus, bus) annotation(
    Line(points = {{40, -10}, {100, -10}, {100, 0}}, color = {255, 204, 51}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end RealGNCVel;
