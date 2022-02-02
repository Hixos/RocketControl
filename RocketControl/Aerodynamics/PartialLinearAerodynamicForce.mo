within RocketControl.Aerodynamics;

partial model PartialLinearAerodynamicForce
extends Icons.AerodynamicsIcon; 

  outer World.Atmosphere atmosphere;
  outer World.Interfaces.WorldBase world;
  
  parameter SI.Area S = pi * (0.15 / 2) ^ 2;
  parameter SI.Length d = 0.15;
  
  Real CA;
  Real CN;
  Real CY;
  
  Real CLL;
  Real CLM;
  Real CLN;
  
  SI.Velocity v_norm;
  
  SI.Pressure q(displayUnit = "Pa");
  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Interfaces.AeroStateInput aeroState annotation(
    Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-110, -10}, {-90, 10}}, rotation = 0)));
    
    SI.Torque ma[3];
    SI.Force fa[3];
equation
  v_norm = norm(aeroState.v);
  q = 0.5 * atmosphere.density(world.altitude(frame_b.r_0)) * v_norm^2;
  
  fa[1] = - q*S*CA;
  fa[2] = q*S*CY;
  fa[3] = - q*S*CN;
  
  ma[1] = q*S*d*CLL;
  ma[2] = q*S*d*CLM;
  ma[3] = q*S*d*CLN;
  
  frame_b.f = -fa;
  frame_b.t = -ma;
annotation(
    Icon(graphics = {Text(origin = {1, -49}, extent = {{-89, 23}, {89, -23}}, textString = "linear")}));
end PartialLinearAerodynamicForce;
