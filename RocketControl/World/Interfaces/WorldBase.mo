within RocketControl.World.Interfaces;

partial model WorldBase
    extends Modelica.Mechanics.MultiBody.World;

    replaceable partial function altitude
extends Modelica.Icons.Function;
      input SI.Position x[3];
      output SI.Position altitude;
    end altitude;

    partial function magneticField
    extends Modelica.Icons.Function;
      input SI.Position x[3];
      output SI.MagneticFluxDensity b[3];
      annotation(
        Inline = true,
        Icon(coordinateSystem(grid = {2, 0})));
    end magneticField;

  function altitude_agl
  extends Modelica.Icons.Function;
    input SI.Position x[3];
        output SI.Position altitude;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end altitude_agl;
    annotation(defaultComponentPrefixes="inner", defaultComponentName = "world",
      Icon(coordinateSystem(grid = {2, 0})));
  end WorldBase;
