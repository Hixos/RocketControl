within RocketControl.Rockets.Lynx.AerodynamicsWithCanards;

model AerodynamicForce
  extends RocketControl.Aerodynamics.PartialAerodynamicForce;
  
  class AeroData = RocketControl.Aerodynamics.AeroData(redeclare type State = State);
  
  AeroData.ExternalAeroData aerodata = AeroData.ExternalAeroData(states_file, coeffs_file);
  
  Real state_arr[State] "Aerodynamic state array";
  
  Modelica.Blocks.Interfaces.RealInput finDeflection[4](each final unit = "rad", each final quantity="Angle", each displayUnit="deg") annotation(
    Placement(visible = true, transformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  constant String states_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/fins_states.npz");
  constant String coeffs_file = Modelica.Utilities.Files.loadResource("modelica://RocketControl/Resources/fins_coeffs.npz");
equation
  state_arr[State.alpha] = to_deg(aeroState.alpha);
  state_arr[State.beta] = to_deg(aeroState.beta);
  state_arr[State.mach] = aeroState.mach;
  state_arr[State.altitude] = aeroState.altitude;
  state_arr[State.fin2delta1] = to_deg(finDeflection[1]);
  state_arr[State.fin2delta2] = to_deg(finDeflection[2]);
  state_arr[State.fin2delta3] = to_deg(finDeflection[3]);
  state_arr[State.fin2delta4] = to_deg(finDeflection[4]);
  coeffs = AeroData.getData(aerodata, state_arr);
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end AerodynamicForce;
