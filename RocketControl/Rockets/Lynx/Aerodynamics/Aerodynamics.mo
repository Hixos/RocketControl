within RocketControl.Rockets.Lynx.Aerodynamics;

model Aerodynamics
  extends RocketControl.Aerodynamics.PartialAerodynamics(redeclare AerodynamicForce aerodynamicForce);
equation

  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Aerodynamics;
