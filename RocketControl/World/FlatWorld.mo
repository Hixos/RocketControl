within RocketControl.World;

model FlatWorld
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
  final parameter Real T[3, 3] = ecef2nedMatrix(x_0_ecef) "Transformation matrix from world frame to ned frame" annotation(Evaluate = true);
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

end FlatWorld;
