within RocketControl.Rockets.Internal;

partial model PartialRocketBody
  extends Icon;
  extends Interfaces.PartialLaunchMount;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a ref_center annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
equation

  annotation(
    Icon(graphics = {Line(origin = {-57, -40}, points = {{-37, -20}, {17, -20}, {17, 20}, {37, 20}}), Line(origin = {-57, 41}, points = {{-37, 19}, {17, 19}, {17, -19}, {37, -19}}), Line(origin = {58, 0}, points = {{38, 0}, {-38, 0}}), Text(origin = {469.111, 26}, lineColor = {128, 128, 128}, extent = {{-423.111, -41}, {-311.111, -82}}, textString = "ref_center")}));
end PartialRocketBody;
