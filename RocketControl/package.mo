package RocketControl

  extends RocketControl.Icons.Navigation;
  import Modelica.Math.Vectors.norm;
  import Modelica.Math.Matrices.solve;
  import Modelica.Math.Matrices.solve2;
  import Modelica.Math.Matrices.inv;
  import Modelica.Units.SI;
  import Modelica.Units.NonSI;
  import Modelica.Mechanics.MultiBody;
  import Modelica.Units.Conversions.from_deg;
  import Modelica.Units.Conversions.to_deg;
  import Modelica.Constants.pi;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end RocketControl;
