within RocketControl;

package Types
  extends Modelica.Icons.TypesPackage;
  type NanoTesla
= Real(final quantity = "MagneticFluxDensity", final unit = "nT");
  
  type LapseRate = Real(unit = "K/m");


  type AngularVelocity_degs
  = Real(final quantity = "AngularVelocity", final unit = "deg/s");
  
  type AngularVelocity
  = Real(final quantity = "AngularVelocity", final unit = "rad/s", displayUnit = "deg/s");
  
  type MagneticFluxDensity  = Real(final quantity = "MagneticFluxDensity", final unit = "T", displayUnit = "nT");
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Types;
