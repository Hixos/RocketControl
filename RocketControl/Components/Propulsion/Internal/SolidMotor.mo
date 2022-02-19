within RocketControl.Components.Propulsion.Internal;

model SolidMotor
  parameter Modelica.Units.SI.Length Dext "External diameter";
  parameter Modelica.Units.SI.Length Din_0 "Initial internal diameter";
  parameter Modelica.Units.SI.Length h "Height";
  
  
  extends PartialRocketMotor(final r_cm = {h/2,0,0}, redeclare Inertia.SolidPropellantInertia inertia(Dext = Dext, Din_0=Din_0, h=h));

end SolidMotor;
