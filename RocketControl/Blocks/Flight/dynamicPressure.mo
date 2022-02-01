within RocketControl.Blocks.Flight;

model dynamicPressure
extends Icon;
  outer World.Atmosphere atmosphere;
  Modelica.Blocks.Interfaces.RealInput vel[3] annotation(
    Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput q annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput pos[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  q = 0.5 * atmosphere.density(-pos[3]) * (vel[1] ^ 2 + vel[2] ^ 2 + vel[3] ^ 2);
  annotation(
    Icon(graphics = {Text(origin = {-122, 20}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "pos"), Text(origin = {-122, -80}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "vel"), Text(origin = {110, -20}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "q"), Text(origin = {0, -131}, lineColor = {0, 0, 255}, extent = {{-180, 29}, {180, -29}}, textString = "%name"), Text(origin = {-4, 9}, extent = {{-82, 87}, {82, -87}}, textString = "qdyn")}));
end dynamicPressure;
