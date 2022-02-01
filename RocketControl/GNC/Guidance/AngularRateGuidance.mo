within RocketControl.GNC.Guidance;

model AngularRateGuidance
  Components.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput w_ref(displayUnit = "deg/s", quantity = "AngularVelocity", unit = "rad/s") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation

  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end AngularRateGuidance;
