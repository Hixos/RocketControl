within RocketControl.Blocks.Math.Quaternion;

block resolve1
  extends Internal.QuaternionIcon;
  Modelica.Blocks.Interfaces.RealInput q[4] "Quaternion representing the rotation from frame 1 to frame 2" annotation(
    Placement(visible = true, transformation(origin = {-120, 58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput x2[3] "Vector expressed in frame 2" annotation(
    Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput x1[3] "Vector expressed in frame 1" annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //        Modelica.Mechanics.MultiBody.Frames.Quaternions.Orientation R;
equation
//  R = Modelica.Mechanics.MultiBody.Frames.Quaternions.Orientation(q, {0,0,0});
  x1 = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve1(q, x2);
  annotation(
    Icon(graphics = {Rectangle(lineColor = {0, 170, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(origin = {0.79, 1.69}, points = {{-38.7932, 12.1393}, {-28.7932, 30.1393}, {-6.7932, 40.1393}, {17.2068, 36.1393}, {31.2068, 24.1393}, {41.2068, 4.13926}, {37.2068, -17.8607}, {21.2068, -37.8607}, {-4.7932, -41.8607}}, thickness = 1.25, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 15, smooth = Smooth.Bezier), Text(origin = {-70, 0}, extent = {{-30, 60}, {30, -60}}, textString = "2"), Text(origin = {64, 0}, extent = {{-30, 60}, {30, -60}}, textString = "1"), Text(origin = {-130, 20}, lineColor = {102, 102, 102}, fillColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "q"), Text(origin = {-130, -100}, lineColor = {102, 102, 102}, fillColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "x2"), Text(origin = {110, -22}, lineColor = {102, 102, 102}, fillColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "x1")}));
end resolve1;
