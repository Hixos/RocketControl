within RocketControl.Rockets.Lynx.Aerodynamics;

model AerodynamicForce
  extends RocketControl.Aerodynamics.PartialAerodynamicForce;
  class AeroData = RocketControl.Aerodynamics.AeroData(redeclare type State = State);
  AeroData.ExternalAeroData aerodata = AeroData.ExternalAeroData();
  Real state_arr[State];
equation
  state_arr[State.alpha] = Modelica.Units.Conversions.to_deg(aeroState.alpha);
  state_arr[State.beta] = Modelica.Units.Conversions.to_deg(aeroState.beta);
  state_arr[State.mach] = aeroState.mach;
  state_arr[State.altitude] = aeroState.altitude;
  coeffs = AeroData.getData(aerodata, state_arr);
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end AerodynamicForce;
