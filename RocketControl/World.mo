within RocketControl;

package World
  model MyWorld
    extends Interfaces.WorldBase;
    import RocketControl.Types.NanoTesla;
    parameter NonSI.Angle_deg latitude_0 = 41.808682 "Latitude of the origin of the world frame";
    parameter NonSI.Angle_deg longitude_0 = 14.054440 "Longitude of the origin of the world frame";
    parameter SI.Position altitude_0 = 1416 "Altitude of the origin of the world frame";
    parameter SI.Length a = 6371000 "Planetary mean radius";
    parameter NonSI.Angle_deg pole_lat = 80.59 "North geomagnetic pole latitude";
    parameter NonSI.Angle_deg pole_lon = -72.64 "North geomagnetic pole longitude";
    parameter NanoTesla g10 = -29404.8 "IGRF gauss coefficient (n=1, m=0)";
    parameter NanoTesla g11 = -1450.9 "IGRF gauss coefficient (n=1, m=1)";
    parameter NanoTesla h11 = 4652.5 "IGRF gauss coefficient (n=1, m=1)";
    parameter SI.Position x_0_ecef[3] = lla2ecef(latitude_0, longitude_0, altitude_0) "Coordinates of the origin of the NED (world) frame in the ecef frame";
    final parameter Real T[3, 3] = ecef2nedMatrix(x_0_ecef) "Transformation matrix from world frame to ned frame";
  //    final parameter Real rpole[3] = Coordinates.lla2ecef(pole_lat, pole_lon, 0) "Magnetic north dipole pole coordinates in ecef frame (IGRF 2020)";
    //    final parameter Real m[3] = -rpole / norm(rpole) "Magnetic dipole axis in ecef frame (IGRF 2020)";
    final parameter SI.Length R = 6371200 "IGRF Earth radius";
    final parameter SI.MagneticFluxDensity H0 = sqrt(g10 ^ 2 + g11 ^ 2 + h11 ^ 2) / 1e9;
    final parameter Real m_vec[3] = -lla2ecef(pole_lat, pole_lon, 0);
    final parameter Real m[3] = m_vec / norm(m_vec);
    
    redeclare function extends altitude
      algorithm
        altitude := -x[3] + altitude_0;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end altitude;
  
    redeclare function extends magneticField "Calculates the Earth's magnetic field intensity based on the IGRF model with N = 1. Default parameter values refer to IGRF-13 year 2020"
        protected
          Real r_vec[3];
          Real r_norm;
          Real r[3];
  
      algorithm
          r_vec := x_0_ecef + transpose(T)*x;
          r_norm := norm(r_vec);
          r := r_vec / r_norm;
          b := T*(R ^ 3 * H0 * (3 * (m * r) * r - m) / r_norm ^ 3);
      annotation(
        Inline = true,
        Icon(coordinateSystem(grid = {2, 0})));
    end magneticField;
    
  function ned2ecefPosition
  input SI.Position[3] x_ned;
  output SI.Position[3] x_ecef;
  algorithm
  x_ecef := x_0_ecef + transpose(T)*x_ned;
  end ned2ecefPosition;
  
  function lla2ecef
    input NonSI.Angle_deg lat;
    input NonSI.Angle_deg lon;
    input SI.Distance alt;
    output SI.Position[3] x_ecef;
  protected
    Real lat_rad;
    Real lon_rad;
  algorithm
    lat_rad := from_deg(lat);
    lon_rad := from_deg(lon);
    x_ecef[1] := cos(lat_rad) * cos(lon_rad) * (alt + a);
    x_ecef[2] := cos(lat_rad) * sin(lon_rad) * (alt + a);
    x_ecef[3] := sin(lat_rad) * (alt + a);
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end lla2ecef;
      function ecef2nedMatrix
        input SI.Position x_ecef[3] "Position in ECEF coordinates";
        output Real T[3, 3];
      protected
        Real D[3];
        Real E[3];
      algorithm
        D := -x_ecef / norm(x_ecef);
        if abs(D * {0, 0, 1}) > 1 - 1e-10 then
          E := {0, 1, 0};
        else
          E := cross(D, {0, 0, 1});
          E := E / norm(E);
        end if;
        T[1, :] := cross(E, D);
        T[2, :] := E;
        T[3, :] := D;
//  R := Modelica.Mechanics.MultiBody.Frames.from_T(T, {0,0,0});
        annotation(Inline = true,
          Icon(coordinateSystem(grid = {2, 0})));
      end ecef2nedMatrix;
    //  class Coordinates
  //    function ecef2nedMatrix
  //      input SI.Position x_ecef[3] "Position in ECEF coordinates";
  //      output Real T[3, 3];
  //    protected
  //      Real D[3];
  //      Real E[3];
  //    algorithm
  //      D := -x_ecef / norm(x_ecef);
  //      if abs(D * {0, 0, 1}) > 1 - 1e-10 then
  //        E := {0, 1, 0};
  //      else
  //        E := cross(D, {0, 0, 1});
  //        E := E / norm(E);
  //      end if;
  //      T[1, :] := cross(E, D);
  //      T[2, :] := E;
  //      T[3, :] := D;
  ////  R := Modelica.Mechanics.MultiBody.Frames.from_T(T, {0,0,0});
  //      annotation(Inline = true,
  //        Icon(coordinateSystem(grid = {2, 0})));
  //    end ecef2nedMatrix;

//    function ecef2ned
  //      input SI.Position v_ecef[3];
  //      output SI.Position v_ned[3];
  //    algorithm
  //      v_ned := T * v_ecef;
  //      annotation(
  //        Inline = true,
  //        Icon(coordinateSystem(grid = {2, 0})));
  //    end ecef2ned;
  //    function lla2ecef
  //      input NonSI.Angle_deg lat;

//      input NonSI.Angle_deg lon;
  //      input SI.Distance alt;
  //      output SI.Position[3] x_ecef;
  //    protected
  //      Real lat_rad;
  //      Real lon_rad;
  //    algorithm
  //      lat_rad := from_deg(lat);
  //      lon_rad := from_deg(lon);
  //      x_ecef[1] := cos(lat_rad) * cos(lon_rad) * (alt + a);
  //      x_ecef[2] := cos(lat_rad) * sin(lon_rad) * (alt + a);
  //      x_ecef[3] := sin(lat_rad) * (alt + a);
  //      annotation(
  //        Icon(coordinateSystem(grid = {2, 0})));
  //    end lla2ecef;
  //    function ned2ecefPosition "Converts a position from the NED frame to the ECEF frame"
  //      input SI.Position x_ned[3];
  //      output SI.Position x_ecef[3];
  //    algorithm

//      x_ecef := x_0_ecef + transpose(T) * x_ned;
  //      annotation(
  //        Icon(coordinateSystem(grid = {2, 0})));
  //    end ned2ecefPosition;
  //    function getNadir
  //      input SI.Position x[3] "Position in ECEF coordinates";
  //      output Real n[3] "Unit vector pointing in the nadir direction";
  //    algorithm
  //      n := -x / norm(x);
  //      annotation(

//        Icon(coordinateSystem(grid = {2, 0})));
  //    end getNadir;
  //    function getEast
  //      input Real nadir[3];
  //      output Real east[3];
  //    algorithm
  //      if abs(nadir * {0, 0, 1}) > 1 - 1e-10 then
  //        east := {0, 1, 0};
  //      else
  //        east := cross(nadir, {0, 0, 1});

//        east := east / norm(east);
  //      end if;
  //      annotation(
  //        Icon(coordinateSystem(grid = {2, 0})));
  //    end getEast;
  //    annotation(
  //      Icon(coordinateSystem(grid = {2, 0})));
  //  end Coordinates;
  equation
  
  end MyWorld;

  model Atmosphere
    outer MyWorld world;
    import Modelica.Constants.R;
    import RocketControl.Types.LapseRate;
    parameter SI.Temperature T0 = 288.15;
    parameter SI.Pressure P0 = 101325;
    parameter LapseRate lapse_rate = -0.0065;
    parameter SI.Density rho0 = 1.2250;
    parameter SI.MolarMass M = 0.0289644;
    parameter SI.Acceleration g0 = 9.80665;

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
    algorithm
//if r_0[3] < -1000 then
//windSpeed := {0, 30, 0};
//else
      windSpeed := {0, 0, 0};
//  end if;
    end windSpeed;
  equation

    annotation(
      Icon(graphics = {Ellipse(origin = {-53, -46}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}, endAngle = 360), Ellipse(origin = {-19, 0}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}, endAngle = 360), Ellipse(origin = {-3, -58}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}, endAngle = 360), Ellipse(origin = {61, -44}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}, endAngle = 360), Ellipse(origin = {43, -10}, lineColor = {255, 255, 255}, fillColor = {195, 195, 195}, fillPattern = FillPattern.Sphere, extent = {{-37, 36}, {37, -36}}, endAngle = 360), Ellipse(origin = {-65, 56}, lineColor = {255, 255, 255}, fillColor = {255, 255, 0}, fillPattern = FillPattern.Sphere, extent = {{-21, 20}, {21, -20}}, endAngle = 360), Line(origin = {-63.0786, 24.1207}, points = {{6, 9}, {12, -9}}, color = {255, 255, 0}, thickness = 1), Line(origin = {-85.3258, 26.8173}, points = {{6, 9}, {-6, -9}}, color = {255, 255, 0}, thickness = 1), Line(origin = {-74.2022, 23.1095}, points = {{6, 9}, {2, -9}}, color = {255, 255, 0}, thickness = 1), Line(origin = {-44.2022, 45.3567}, points = {{6, 9}, {24, 5}}, color = {255, 255, 0}, thickness = 1), Line(origin = {-49.2584, 30.5252}, points = {{6, 9}, {18, -5}}, color = {255, 255, 0}, thickness = 1), Line(origin = {-46.5617, 59.514}, points = {{6, 9}, {26, 11}}, color = {255, 255, 0}, thickness = 1), Text(origin = {4, -4}, lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name")}));
  end Atmosphere;

  package Interfaces
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
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Interfaces;
  annotation(
    Icon(graphics = {Ellipse(lineColor = {255, 255, 255}, fillColor = {170, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, lineThickness = 0, extent = {{-90, 90}, {90, -90}}, endAngle = 360), Ellipse(fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-80, 80}, {80, -80}}, endAngle = 360), Polygon(origin = {25, -7}, fillColor = {0, 170, 0}, fillPattern = FillPattern.Solid, points = {{-29, 21}, {-39, 7}, {-25, -3}, {-15, 9}, {-7, -1}, {-9, -9}, {1, -5}, {-9, 11}, {15, -9}, {-5, -17}, {-31, -13}, {-53, -9}, {-51, -47}, {-33, -73}, {-7, -71}, {-1, -21}, {13, -43}, {33, -35}, {21, -17}, {51, -15}, {53, 21}, {49, 35}, {33, 61}, {21, 71}, {-5, 73}, {-19, 71}, {-21, 43}, {-5, 43}, {-5, 63}, {-3, 41}, {-29, 37}, {-29, 21}}), Polygon(origin = {-62, 5}, fillColor = {0, 170, 0}, fillPattern = FillPattern.Solid, points = {{12, 57}, {18, 41}, {2, 15}, {-12, 13}, {-16, -11}, {-2, -21}, {2, -57}, {-6, -47}, {-16, -23}, {-18, -7}, {-16, 11}, {-8, 33}, {4, 49}, {12, 57}}), Polygon(origin = {-3, 71}, fillColor = {170, 255, 255}, fillPattern = FillPattern.Solid, points = {{-35, -5}, {-21, -9}, {17, -3}, {37, 1}, {27, 5}, {9, 9}, {-15, 7}, {-35, -1}, {-35, -5}})}));
end World;
