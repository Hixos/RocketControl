within RocketControl.World.Interfaces;

partial model WorldBase
    extends Modelica.Mechanics.MultiBody.World;

    replaceable partial function altitude
      input SI.Position x[3];
      output SI.Position altitude;
    end altitude;

    partial function magneticField
      input SI.Position x[3];
      output SI.MagneticFluxDensity b[3];
      annotation(
        Inline = true,
        Icon(coordinateSystem(grid = {2, 0})));
    end magneticField;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end WorldBase;
