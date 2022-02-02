within RocketControl.Components.Parts;

model StaticFrame
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-104, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {104, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
equation
  frame_b.r_0 = {0,0,0};
  frame_b.R = frame_a.R;

  /* Force and torque balance */
  zeros(3) = frame_a.f;
  zeros(3) = frame_a.t;
  
annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end StaticFrame;
