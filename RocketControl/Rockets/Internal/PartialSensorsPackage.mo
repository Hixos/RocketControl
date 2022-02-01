within RocketControl.Rockets.Internal;

partial model PartialSensorsPackage
  extends Modelica.Icons.SensorsPackage;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
equation

  annotation(
    Icon(graphics = {Text(origin = {-10, -254}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name")}));
end PartialSensorsPackage;
