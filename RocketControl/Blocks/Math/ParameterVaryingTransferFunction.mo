within RocketControl.Blocks.Math;

block ParameterVaryingTransferFunction "Linear transfer function"
  import Modelica.Blocks.Types.Init;
  extends Modelica.Blocks.Interfaces.SISO;

  parameter Integer n(min = 1) annotation(Evaluate = true);

  parameter Modelica.Blocks.Types.Init initType=Modelica.Blocks.Types.Init.NoInit
    "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
                                     annotation(Evaluate=true, Dialog(group=
          "Initialization"));
  parameter Real x_start[n]=zeros(nx)
    "Initial or guess values of states"
    annotation (Dialog(group="Initialization"));
  parameter Real y_start=0
    "Initial value of output (derivatives of y are zero up to nx-1-th derivative)"
    annotation(Dialog(enable=initType == Init.InitialOutput, group=
          "Initialization"));
  output Real x[n](start=x_start)
    "State of transfer function from controller canonical form";
  Modelica.Blocks.Interfaces.RealInput a[n+1] annotation(
    Placement(visible = true, transformation(extent = {{-140, -70}, {-100, -30}}, rotation = 0), iconTransformation(extent = {{-140, -100}, {-100, -60}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput b[n+1] annotation(
    Placement(visible = true, transformation(extent = {{-140, -110}, {-100, -70}}, rotation = 0), iconTransformation(extent = {{-140, 60}, {-100, 100}}, rotation = 0)));
protected
  parameter Integer na=n+1 "Size of Denominator of transfer function.";
  parameter Integer nb=n+1 "Size of Numerator of transfer function.";
  parameter Integer nx=n;
  Real bb[n+1];
  Real d;
  Real a_end;
  Real x_scaled[n] "Scaled vector x";

initial equation
  if initType == Init.SteadyState then
    der(x_scaled) = zeros(nx);
  elseif initType == Init.InitialState then
    x_scaled = x_start*a_end;
  elseif initType == Init.InitialOutput then
    y = y_start;
    der(x_scaled[2:nx]) = zeros(nx-1);
  end if;
equation
  assert(size(b,1) <= size(a,1), "Transfer function is not proper");
  bb[:] = b;
  d = bb[1]/a[1];
  a_end = if a[end] > 100*Modelica.Constants.eps*sqrt(a*a) then a[end] else 1.0;
  
  assert(size(b,1) <= size(a,1), "Transfer function is not proper");
  if nx == 0 then
     y = d*u;
  else
     der(x_scaled[1])    = (-a[2:na]*x_scaled + a_end*u)/a[1];
     der(x_scaled[2:nx]) = x_scaled[1:nx-1];
     y = ((bb[2:na] - d*a[2:na])*x_scaled)/a_end + d*u;
     x = x_scaled/a_end;
  end if;
  annotation(
    Documentation(info = "<html>
<p>
This block defines the transfer function between the input
u and the output y
as (nb = dimension of b, na = dimension of a):
</p>
<blockquote><pre>
      b[1]*s^[nb-1] + b[2]*s^[nb-2] + ... + b[nb]
y(s) = --------------------------------------------- * u(s)
      a[1]*s^[na-1] + a[2]*s^[na-2] + ... + a[na]
</pre></blockquote>
<p>
State variables <strong>x</strong> are defined according to <strong>controller canonical</strong>
form. Internally, vector <strong>x</strong> is scaled to improve the numerics (the states in versions before version 3.0 of the Modelica Standard Library have been not scaled). This scaling is
not visible from the outside of this block because the non-scaled vector <strong>x</strong>
is provided as output signal and the start value is with respect to the non-scaled
vector <strong>x</strong>.
Initial values of the states <strong>x</strong> can be set via parameter <strong>x_start</strong>.
</p>

<p>
Example:
</p>
<blockquote><pre>
TransferFunction g(b = {2,4}, a = {1,3});
</pre></blockquote>
<p>
results in the following transfer function:
</p>
<blockquote><pre>
   2*s + 4
y = --------- * u
    s + 3
</pre></blockquote>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-80, 0}, {80, 0}}, color = {0, 0, 127}), Text(lineColor = {0, 0, 127}, extent = {{-90, 10}, {90, 90}}, textString = "b(s)"), Text(lineColor = {0, 0, 127}, extent = {{-90, -90}, {90, -10}}, textString = "a(s)")}));
end ParameterVaryingTransferFunction;
