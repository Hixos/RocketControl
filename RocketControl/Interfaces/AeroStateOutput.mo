within RocketControl.Interfaces;

connector AeroStateOutput
  output SI.Position altitude;
  output SI.MachNumber mach;
  output SI.Angle alpha(displayUnit = "deg");
  output SI.AngularVelocity alpha_dot(displayUnit = "deg/s");
  output SI.Angle beta(displayUnit = "deg");
  
  output SI.Velocity v[3];
  output SI.AngularVelocity w[3](each displayUnit = "deg/s");
  annotation(
    Diagram(graphics = {Polygon(origin = {20, 0}, fillColor = {135, 221, 245}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}})}),
    Icon(graphics = {Polygon(origin = {20, 0}, fillColor = {135, 221, 245}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}})}));
end AeroStateOutput;
