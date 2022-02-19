within RocketControl.Components.Propulsion.Internal.Inertia;

model SolidPropellantInertia
  extends PartialPropellantInertia;
  import Modelica.Constants.pi;
  parameter Modelica.Units.SI.Length Dext "External diameter";
  parameter Modelica.Units.SI.Length Din_0 "Initial internal diameter";
  parameter Modelica.Units.SI.Length h "Height";
  final parameter SI.Volume V_0 = pi * ((Dext / 2) ^ 2 - (Din_0 / 2) ^ 2) * h;
  final parameter SI.Density rho = m_0 / V_0;
  Modelica.Units.SI.Length Din "Internal diameter";
  Modelica.Units.SI.Length Din2 "Internal diameter";
  SI.Volume V "Propellant volume";
equation
  V = massOut.m / rho;
  Din = sqrt(Dext ^ 2 - 4 * V / (h * pi));
  Din2 = sqrt(Dext ^ 2 - 4 * V / ( pi));
  massOut.I[1, 1] = 1 / 2 * massOut.m * ((Dext / 2) ^ 2 + (Din / 2) ^ 2);
  massOut.I[2, 2] = 1 / 12 * massOut.m * (3 * ((Dext / 2) ^ 2 + (Din / 2) ^ 2) + h ^ 2);
  massOut.I[3, 3] = 1 / 12 * massOut.m * (3 * ((Dext / 2) ^ 2 + (Din / 2) ^ 2) + h ^ 2);
  massOut.I[1, 2] = 0;
  massOut.I[1, 3] = 0;
  massOut.I[2, 1] = 0;
  massOut.I[2, 3] = 0;
  massOut.I[3, 1] = 0;
  massOut.I[3, 2] = 0;
  annotation(
    Icon);
end SolidPropellantInertia;
