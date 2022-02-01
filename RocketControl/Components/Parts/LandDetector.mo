within RocketControl.Components.Parts;

model LandDetector
    extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
    parameter SI.Position groundlevel = 0;
  equation
    frame_a.f = zeros(3);
    frame_a.t = zeros(3);
    when frame_a.r_0[3] > groundlevel and time > 10 then
      terminate("Simulation terminated successfully");
    end when;
    annotation(
      Icon(graphics = {Polygon(origin = {-3, -53}, lineColor = {0, 170, 127}, fillColor = {85, 255, 127}, fillPattern = FillPattern.HorizontalCylinder, points = {{-53, 41}, {-73, 31}, {-83, -19}, {-73, -39}, {67, -39}, {87, -27}, {97, 21}, {87, 41}, {-53, 41}}), Ellipse(origin = {6, -51}, fillColor = {0, 134, 98}, fillPattern = FillPattern.Solid, extent = {{38, 13}, {-38, -13}}, endAngle = 360), Polygon(origin = {10, -10}, fillColor = {0, 170, 255}, fillPattern = FillPattern.Solid, points = {{-6, -42}, {18, -34}, {6, -24}, {-16, 38}, {-24, 48}, {-24, 34}, {-4, -28}, {-6, -42}}), Polygon(origin = {38, 61}, fillColor = {255, 170, 0}, fillPattern = FillPattern.Solid, points = {{-32, 25}, {20, -35}, {28, -25}, {32, -7}, {28, 13}, {18, 29}, {0, 35}, {-20, 33}, {-32, 25}}), Line(origin = {6.18874, 57.3515}, points = {{-0.188744, 28.6485}, {-12.1887, -29.3515}, {11.8113, 14.6485}, {11.8113, 14.6485}}), Line(origin = {18.3124, 43.2299}, points = {{11.6876, 14.7701}, {-24.3124, -15.2299}, {23.6876, 0.770139}}), Line(origin = {26.4447, 31.0503}, points = {{21.5553, 6.94969}, {-32.4447, -3.05031}, {31.5553, -5.05031}, {31.5553, -5.05031}}), Text(origin = {6, -204}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Text(origin = {-52, 74}, extent = {{-46, 28}, {46, -28}}, textString = "z > 0")}));
  end LandDetector;