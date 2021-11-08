within RocketControl;

package Types
  type AngularRandomWalk = Real(final quantity = "AngularRandomWalk", unit = "deg2/h");

  type NanoTesla
= Real(final quantity = "MagneticFluxDensity", final unit = "nT");
  
  type LapseRate = Real(unit = "K/m");


  type AngularVelocity_degs
  = Real(final quantity = "AngularVelocity", final unit = "deg/s");

  block quat2eul
  parameter Integer sequence[3] = {3,2,1};
  parameter Real guessAngle = 0;
  Modelica.Blocks.Interfaces.RealInput q[4] annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput angles[3] annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Frames.Orientation R;
  equation
  R = Modelica.Mechanics.MultiBody.Frames.from_Q(q, {0,0,0});
  angles = to_deg(Modelica.Mechanics.MultiBody.Frames.axesRotationsAngles(R, sequence, guessAngle));
    annotation(
      Icon(coordinateSystem(grid = {2, 0}), graphics = {Text(extent = {{-100, 100}, {100, -100}}, textString = "q2e")}));
  end quat2eul;

  model quaternion
  Modelica.Blocks.Interfaces.RealOutput q[4] annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  equation
  frame_a.f = {0,0,0};
  frame_a.t = {0,0,0};
  
  q = Modelica.Mechanics.MultiBody.Frames.Quaternions.from_T(frame_a.R.T);
    annotation(
      Icon(coordinateSystem(grid = {2, 0}), graphics = {Text(extent = {{-100, 100}, {100, -100}}, textString = "quat")}));
  end quaternion;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Types;
