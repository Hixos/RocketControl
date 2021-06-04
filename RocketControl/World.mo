within RocketControl;

package World
  model MyWorld
    extends Modelica.Mechanics.MultiBody.World;

    function altitude
      input SI.Position pos_ecef[3];
      output SI.Position altitude;
    algorithm
      altitude := -pos_ecef[3];
    end altitude;
  equation

  end MyWorld;

  model Atmosphere
    outer MyWorld world;
    type LapseRate = Real(unit = "K/m");
      
    import Modelica.Constants.R;
    
    parameter SI.Temperature T0 = 288.15;
    parameter SI.Pressure P0 = 101325;
    parameter LapseRate lapse_rate = -0.0065;
    parameter SI.Density rho0 = 1.2250;
    parameter SI.MolarMass M = 0.0289644;
    parameter SI.Acceleration g0 = 9.80665;
    
    function density
      input SI.Position[3] r_0;
      output SI.Density density;
    protected
      SI.Position h;
    algorithm
      h := world.altitude(r_0);
      density := rho0 * (T0 / (T0 + h * lapse_rate))^(1+(g0*M)/(R*lapse_rate));
    end density;
    
    function temperature
      input SI.Position[3] r_0;
      output SI.Temperature temp;
    protected
      SI.Position h;
    algorithm
      h := world.altitude(r_0);
      temp := T0 + h * lapse_rate;
    end temperature;
    
    function pressure
      input SI.Position[3] pos_ecef;
      output SI.Pressure press;
    protected
      SI.Position h;
    algorithm
      h := world.altitude(pos_ecef);
      press := P0 * ((T0 + h * lapse_rate) / T0)^(-(g0*M)/(R*lapse_rate));
    end pressure;
  
    function speedOfSound
      input SI.Position[3] r_0;
      output SI.Velocity speedOfSound;
    protected
      SI.Temperature T;
    algorithm
      T := temperature(r_0);
      speedOfSound := sqrt(1.4*R*T/M);
    end speedOfSound;
  equation
  
  annotation(
      Icon(graphics = {Ellipse(origin = {-53, -46}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}, endAngle = 360), Ellipse(origin = {-19, 0}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}, endAngle = 360), Ellipse(origin = {-3, -58}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}, endAngle = 360), Ellipse(origin = {61, -44}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}, endAngle = 360), Ellipse(origin = {43, -10}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}, endAngle = 360), Ellipse(origin = {-65, 56}, lineColor = {255, 255, 255}, fillColor = {255, 255, 0}, fillPattern = FillPattern.Sphere, extent = {{-21, 20}, {21, -20}}, endAngle = 360), Line(origin = {-63.0786, 24.1207}, points = {{6, 9}, {12, -9}}, color = {255, 255, 0}, thickness = 1), Line(origin = {-85.3258, 26.8173}, points = {{6, 9}, {-6, -9}}, color = {255, 255, 0}, thickness = 1), Line(origin = {-74.2022, 23.1095}, points = {{6, 9}, {2, -9}}, color = {255, 255, 0}, thickness = 1), Line(origin = {-44.2022, 45.3567}, points = {{6, 9}, {24, 5}}, color = {255, 255, 0}, thickness = 1), Line(origin = {-49.2584, 30.5252}, points = {{6, 9}, {18, -5}}, color = {255, 255, 0}, thickness = 1), Line(origin = {-46.5617, 59.514}, points = {{6, 9}, {26, 11}}, color = {255, 255, 0}, thickness = 1), Text(origin = {4, -4}, lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name")}));end Atmosphere;
end World;
