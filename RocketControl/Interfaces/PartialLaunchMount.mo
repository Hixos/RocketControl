within RocketControl.Interfaces;

model PartialLaunchMount
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_lug_bow annotation(
    Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_lug_aft annotation(
    Placement(visible = true, transformation(origin = {-100, -40}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
equation

  annotation(
    Icon(coordinateSystem(grid = {2, 0}), graphics = {Text(origin = {18, 66}, lineColor = {128, 128, 128}, extent = {{-136, -25}, {-100, -50}}, textString = "bow"), Text(origin = {16, -54}, lineColor = {128, 128, 128}, extent = {{-136, -25}, {-100, -50}}, textString = "aft")}));
end PartialLaunchMount;
