within RocketControl.Interfaces;

connector AeroStateInput
input SI.Position altitude;
input SI.MachNumber mach;
input SI.Angle alpha(displayUnit = "deg");
input SI.AngularVelocity alpha_dot(displayUnit = "deg/s");
input SI.Angle beta(displayUnit = "deg");

input SI.Velocity v[3];
input SI.AngularVelocity w[3](each displayUnit = "deg/s");

annotation(
    Diagram(graphics = {Polygon(origin = {20, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}})}),
    Icon(graphics = {Polygon(origin = {20, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-80, -100}, {80, -50}, {80, 50}, {-80, 100}})}));
end AeroStateInput;
