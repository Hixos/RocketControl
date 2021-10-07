within RocketControl;

package Types
  type AngularRandomWalk = Real(final quantity = "AngularRandomWalk", unit = "deg2/h");

  type NanoTesla
= Real(final quantity = "MagneticFluxDensity", final unit = "nT");

  type AngularVelocity_degs
  = Real(final quantity = "AngularVelocity", final unit = "deg/s");
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Types;
