within RocketControl.Components.Sensors.Internal.Noise;

partial block PartialClockedNoise "Partial noise generator"
  import generator = Modelica.Math.Random.Generators.Xorshift128plus;
  import Modelica.Math.Random.Utilities.automaticLocalSeed;
  extends Modelica.Clocked.RealSignals.Interfaces.PartialNoise;
  // Advanced dialog menu: Noise generation
  parameter Boolean enableNoise = globalSeed.enableNoise "= true: y = u + noise, otherwise y = u" annotation(
    choices(checkBox = true),
    Dialog(tab = "Advanced", group = "Noise generation"));
  // Advanced dialog menu: Initialization
  parameter Boolean useGlobalSeed = true "= true: use global seed, otherwise ignore it" annotation(
    choices(checkBox = true),
    Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise));
  parameter Boolean useAutomaticLocalSeed = true "= true: use automatic local seed, otherwise use fixedLocalSeed" annotation(
    choices(checkBox = true),
    Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise));
  parameter Integer fixedLocalSeed = 1 "Local seed (any Integer number)" annotation(
    Dialog(tab = "Advanced", group = "Initialization", enable = enableNoise and not useAutomaticLocalSeed));
  final parameter Integer localSeed(fixed = false) "The actual localSeed";
  
  Real r "Random number according to the desired distribution";
protected
  outer Modelica.Blocks.Noise.GlobalSeed globalSeed "Definition of global seed via inner/outer";
  parameter Integer actualGlobalSeed = if useGlobalSeed then globalSeed.seed else 0 "The global seed, which is actually used";
  parameter Boolean generateNoise = enableNoise and globalSeed.enableNoise "= true, if noise shall be generated, otherwise no noise";
  // Declare state and random number variables
  Integer state[generator.nState](start = generator.initialState(localSeed, actualGlobalSeed)) "Internal state of random number generator";
  
  Real r_raw "Uniform random number in the range (0,1]";
initial equation
  localSeed = if useAutomaticLocalSeed then automaticLocalSeed(getInstanceName()) else fixedLocalSeed;
//   pre(state) = generator.initialState(localSeed, actualGlobalSeed);
//   r_raw = generator.random(pre(state));
equation
// Draw random number at sample times
//  when firstTick(clock) then
//    r_raw = generator.random(previous(state));
//  end when;
  when Clock() then
    (r_raw, state) = generator.random(pre(state));
// Generate noise if requested
    y = if not generateNoise then u else u + r;
  end when;
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-81, -17}, {-67, -17}, {-67, -1}, {-59, -1}, {-59, -49}, {-51, -49}, {-51, -27}, {-43, -27}, {-43, 57}, {-35, 57}, {-35, 25}}, color = {0, 0, 127}, pattern = LinePattern.Dot), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{40, 19}, {46, 13}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-54, -23}, {-48, -29}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-18, -41}, {-12, -47}}, endAngle = 360), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{91, -22}, {69, -14}, {69, -30}, {91, -22}}), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-62, -47}, {-56, -53}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-28, -15}, {-22, -21}}, endAngle = 360), Line(points = {{-90, -23}, {82, -23}}, color = {192, 192, 192}), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{14, 9}, {20, 3}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{4, -1}, {10, -7}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-8, 39}, {-2, 33}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{48, -47}, {54, -53}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-2, 53}, {4, 47}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{30, 53}, {36, 47}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-46, 59}, {-40, 53}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{20, -19}, {26, -25}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-84, -13}, {-78, -19}}, endAngle = 360), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-38, -33}, {-32, -39}}, endAngle = 360), Line(points = {{-35, 25}, {-35, -35}, {-25, -35}, {-25, -17}, {-15, -17}, {-15, -45}, {-5, -45}, {-5, 37}, {1, 37}, {1, 51}, {7, 51}, {7, -5}, {17, -5}, {17, 7}, {23, 7}, {23, -23}, {33, -23}, {33, 49}, {43, 49}, {43, 15}, {51, 15}, {51, -51}, {61, -51}}, color = {0, 0, 127}, pattern = LinePattern.Dot), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-70, 3}, {-64, -3}}, endAngle = 360), Line(points = {{-81, 78}, {-81, -90}}, color = {192, 192, 192}), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{-81, 90}, {-89, 68}, {-73, 68}, {-81, 90}})}),
    Documentation(info = "<html>
  <p>
  Partial base class of noise generators defining the common features
  of noise blocks.
  </p>
  </html>", revisions = "<html>
  <table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
  <tr><th>Date</th> <th align=\"left\">Description</th></tr>
  
  <tr><td> June 22, 2015 </td>
    <td>
  
  <table border=\"0\">
  <tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
  </td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
  </td></tr></table>
  </td></tr>
  
  </table>
  </html>"));
end PartialClockedNoise;
