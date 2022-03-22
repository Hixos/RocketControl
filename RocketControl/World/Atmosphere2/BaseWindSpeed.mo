within RocketControl.World.Atmosphere2;

model BaseWindSpeed
outer RocketControl.World.Interfaces.WorldBase world;
  outer RocketControl.World.Atmosphere atmosphere;
  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0), iconTransformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput v_wind[3] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));    
    protected
     SI.Position h;
equation
     h = world.altitude(frame_a.r_0);
     v_wind = atmosphere.windSpeed(h);
     
     frame_a.f = zeros(3);
  frame_a.t = zeros(3);
annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end BaseWindSpeed;
