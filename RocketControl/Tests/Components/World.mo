within RocketControl.Tests.Components;

package World
  model MagneticField
    RocketControl.World.MyWorld world(altitude_0 = 100, latitude_0 = 45, longitude_0 = 8) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Real b[3];
  equation
    b = world.magneticField({500000, 500000, 500000});
    annotation(
      Icon(coordinateSystem(grid = {2, 0})),
      experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
  end MagneticField;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end World;
