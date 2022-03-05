within RocketControl.World;

model Atmosphere
  import Modelica.Constants.R;
  import RocketControl.Types.LapseRate;
  parameter SI.Temperature T0 = 288.15;
  parameter SI.Pressure P0 = 101325;
  parameter LapseRate lapse_rate = -0.0065;
  parameter SI.Density rho0 = 1.2250;
  parameter SI.MolarMass M = 0.0289644;
  parameter SI.Acceleration g0 = 9.80665;
  
  parameter Integer num_wind_layers(min = 1) = 1;
  
  parameter SI.Length wind_layer_height[num_wind_layers] = {100000};
  parameter SI.Angle wind_direction[num_wind_layers](each displayUnit = "deg") = {0};
  parameter SI.Velocity wind_magnitude[num_wind_layers] = {0};

  function density
    input SI.Position h;
    output SI.Density density;
  
  algorithm
    density := rho0 * (T0 / (T0 + h * lapse_rate)) ^ (1 + g0 * M / (R * lapse_rate));
  end density;

  function temperature
    input SI.Position h;
    output SI.Temperature temp;
  algorithm
    temp := T0 + h * lapse_rate;
  end temperature;

  function pressure
    input SI.Position h;
    output SI.Pressure press;
  algorithm
    press := P0 * ((T0 + h * lapse_rate) / T0) ^ (-g0 * M / (R * lapse_rate));
    annotation(Inline = true);
  end pressure;

  function speedOfSound
    input SI.Position h;
    output SI.Velocity speedOfSound;
  protected
    SI.Temperature T;
  algorithm
    T := temperature(h);
    speedOfSound := sqrt(1.4 * R * T / M);
  end speedOfSound;

  function windSpeed
    input SI.Position h;
    output SI.Velocity[3] windSpeed;
  protected
   SI.Length cum_height = 0;
  algorithm
   for i in 1:num_wind_layers loop
    cum_height := cum_height + wind_layer_height[i];
    if h <= cum_height then
      windSpeed := wind_magnitude[i]*{cos(wind_direction[i]), sin(wind_direction[i]), 0};
      break;
    end if;
   end for;
   
   if h > cum_height then
     windSpeed := wind_magnitude[end]*{cos(wind_direction[end]), sin(wind_direction[end]), 0};
   end if;
  end windSpeed;

  package Blocks
    model Density
    extends Icon;
    outer RocketControl.World.Atmosphere atmosphere;
    Modelica.Blocks.Interfaces.RealInput h annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
Modelica.Blocks.Interfaces.RealOutput rho annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
rho = atmosphere.density(h);
      annotation(
        Icon(graphics = {Text(origin = {1, 11}, extent = {{-65, 67}, {65, -67}}, textString = "rho"), Text(origin = {-121, -42}, textColor = {102, 102, 102}, extent = {{-21, 22}, {21, -22}}, textString = "h"), Text(origin = {0, -223},textColor = {0, 0, 255}, extent = {{-119, 124}, {119, 84}}, textString = "%name")}));
    end Density;

    model Icon
    equation

      annotation(
        Icon(coordinateSystem(grid = {2, 0}), graphics = {Rectangle(fillColor = {228, 251, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20)}));
    end Icon;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Blocks;
equation

  annotation(defaultComponentPrefixes="inner", defaultComponentName = "atmosphere",
    Icon(graphics = {Rectangle(fillColor = {202, 244, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Ellipse(origin = {-53, -46}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}), Ellipse(origin = {-19, 0}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}), Ellipse(origin = {-3, -58}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}), Ellipse(origin = {61, -44}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}), Ellipse(origin = {43, -10}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}), Ellipse(origin = {-65, 56}, lineColor = {255, 255, 255}, fillColor = {255, 255, 0}, fillPattern = FillPattern.Sphere, extent = {{-21, 20}, {21, -20}}), Line(origin = {-63.08, 24.12}, points = {{6, 9}, {12, -9}}, color = {255, 255, 0}, arrowSize = 1), Line(origin = {-85.33, 26.82}, points = {{6, 9}, {-6, -9}}, color = {255, 255, 0}, arrowSize = 1), Line(origin = {-74.2, 23.11}, points = {{6, 9}, {2, -9}}, color = {255, 255, 0}, arrowSize = 1), Line(origin = {-44.2, 45.36}, points = {{6, 9}, {24, 5}}, color = {255, 255, 0}), Line(origin = {-49.26, 30.53}, points = {{6, 9}, {18, -5}}, color = {255, 255, 0}), Line(origin = {-46.56, 59.51}, points = {{6, 9}, {26, 11}}, color = {255, 255, 0}), Text(origin = {4, 20}, lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name")}));
end Atmosphere;
