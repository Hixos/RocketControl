within RocketControl;

package World
  model MyWorld
    extends Modelica.Mechanics.MultiBody.World;

    function altitude
      input SI.Position pos_ecef[3];
      output SI.Position altitude;
    protected
    algorithm
      (, , altitude) := WGS84.ecef2lla(pos_ecef);
    end altitude;

    class WGS84
      parameter SI.Distance a = 6371000 "Earth mean radius";

      function lla2ecef
        import Modelica.Units.Conversions.from_deg;
        input NonSI.Angle_deg lat;
        input NonSI.Angle_deg lon;
        input SI.Distance alt;
        output SI.Position[3] pos_ecef;
      protected
        Real lat_rad;
        Real lon_rad;
      algorithm
        lat_rad := from_deg(lat);
        lon_rad := from_deg(lon);
        pos_ecef[1] := cos(lat_rad) * cos(lon_rad) * (alt + a);
        pos_ecef[2] := cos(lat_rad) * sin(lon_rad) * (alt + a);
        pos_ecef[3] := sin(lat_rad) * (alt + a);
        annotation(
          Inline = true,
          Icon(coordinateSystem(grid = {2, 0})));
      end lla2ecef;

      function ecef2lla
        input SI.Position[3] x_ecef;
        output NonSI.Angle_deg lat;
        output NonSI.Angle_deg lon;
        output SI.Distance alt;
      protected
        Real N[3];
        Real X[3];
        SI.Position x_norm;
        SI.Position x_eq[3];
        SI.Position x_det;
      algorithm
        N := {0, 0, 1};
        X := {1, 0, 0};
        x_norm := norm(x_ecef);
        lat := 90 - rad2deg(acos(dot(N, x_ecef) / x_norm));
        if abs(lat) > 90 - 1e-8 then
          lon := 0;
        else
          x_eq := {x_ecef(1), x_ecef(2), 0};
          x_det := dot(N, cross(X, x_eq));
          lon := rad2deg(atan2(x_det, dot(X, x_eq)));
        end if;
        alt := x_norm - a;
        annotation(
          Inline = true,
          Icon(coordinateSystem(grid = {2, 0})));
      end ecef2lla;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end WGS84;

    function eci2ecefTransform
      input SI.Time t;
      output Real T[3, 3];
      output SI.AngularVelocity w[3];
    algorithm
      w := {0, 0, 72.92115e-6};
      T := [cos(w[3] * t), sin(w[3] * t), 0; -sin(w[3] * t), cos(w[3] * t), 0; 0, 0, 1];
      annotation(
        Inline = true,
        Icon(coordinateSystem(grid = {2, 0})));
    end eci2ecefTransform;

    function eci2nedTransform
      input SI.Position x[3] "Position in ECI coordinates";
      input SI.Velocity v[3] "Velocity in ECI coordinates";
      output Real T[3,3];
      output SI.AngularVelocity w[3];
    protected
      Real xnorm;
      Real xvers[3];
      Real D[3];
      Real E[3];
    //  SI.AngularVelocity w[3];
      Real vp[3];
    algorithm
      xnorm := norm(x);
      xvers := x / xnorm;
      D := -x / xnorm;
      E := cross(D, {0, 0, 1});
      E := E / Modelica.Math.Vectors.norm(E);
      T[1, :] := cross(E, D);
      T[2, :] := E;
      T[3, :] := D;
      vp := v - v * xvers * xvers; // We need only the component perpendicular to x
      w := T*(cross(x, vp) / xnorm ^ 2);
      annotation(
        Inline = true,
        Icon(coordinateSystem(grid = {2, 0})));
    end eci2nedTransform;

    function eci2ecef
      input Real v[3] "Vector expressed in ECI frame";
      input SI.Time t;
      output Real v_ecef[3] "v expressed in the ECEF frame";
    protected
      Real T[3, 3];
      SI.AngularVelocity w;
    algorithm
      w := 72.92115e-6;
      T := [cos(w * t), sin(w * t), 0; -sin(w * t), cos(w * t), 0; 0, 0, 1];
      v_ecef := T * v;
      annotation(
        Inline = true,
        Icon(coordinateSystem(grid = {2, 0})));
    end eci2ecef;

    function eci2ecefOrientation
      input SI.Time t;
      output Modelica.Mechanics.MultiBody.Frames.Orientation R;
    protected
      constant SI.AngularVelocity w = 72.92115e-6;
      Real T[3, 3];
    algorithm
      T := [cos(w * t), sin(w * t), 0; -sin(w * t), cos(w * t), 0; 0, 0, 1];
      R := Modelica.Mechanics.MultiBody.Frames.from_T(T, {0, 0, w});
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end eci2ecefOrientation;

    function eci2nedOrientation
  input SI.Position x[3] "Position in ECI coordinates";
      input SI.Velocity v[3] "Velocity in ECI coordinates";
      output Modelica.Mechanics.MultiBody.Frames.Orientation R;
    protected
      Real xnorm;
      Real xvers[3];
      Real D[3];
      Real E[3];
      SI.AngularVelocity w[3];
      Real T[3, 3];
      Real vp[3];
    algorithm
      xnorm := norm(x);
      xvers := x / xnorm;
      D := -x / xnorm;
      E := cross(D, {0, 0, 1});
      E := E / Modelica.Math.Vectors.norm(E);
      T[1, :] := cross(E, D);
      T[2, :] := E;
      T[3, :] := D;
    //  T := T;
      vp := v - v * xvers * xvers;
// We need only the component perpendicular to x
      w := cross(x, vp) / xnorm ^ 2;
      R := Modelica.Mechanics.MultiBody.Frames.Orientation(T, T * w);
      annotation(
        Inline = true,
        Icon(coordinateSystem(grid = {2, 0})));
    end eci2nedOrientation;
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
      density := rho0 * (T0 / (T0 + h * lapse_rate)) ^ (1 + g0 * M / (R * lapse_rate));
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
      press := P0 * ((T0 + h * lapse_rate) / T0) ^ (-g0 * M / (R * lapse_rate));
    end pressure;

    function speedOfSound
      input SI.Position[3] r_0;
      output SI.Velocity speedOfSound;
    protected
      SI.Temperature T;
    algorithm
      T := temperature(r_0);
      speedOfSound := sqrt(1.4 * R * T / M);
    end speedOfSound;

    function windSpeed
      input SI.Position[3] r_0;
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

  model MyWorldWGS
    extends Modelica.Mechanics.MultiBody.World;

    function altitude
      input SI.Position pos_ecef[3];
      output SI.Position altitude;
    protected
    algorithm
      (, , altitude) := WGS84.ecef2lla(pos_ecef);
    end altitude;

    class WGS84
      constant SI.Distance a = 6378137 "WGS84 equatorial radius";
      constant SI.Distance b = 6356752.3142518 "WGS84 Polar radius";
      constant Real e2 = 1 - b ^ 2 / a ^ 2;

      function lla2ecef
        import Modelica.Units.Conversions.from_deg;
        input NonSI.Angle_deg lat;
        input NonSI.Angle_deg lon;
        input SI.Distance alt;
        output SI.Position[3] pos_ecef;
      protected
        Real N;
        Real lat_rad;
        Real lon_rad;
      algorithm
        lat_rad := from_deg(lat);
        lon_rad := from_deg(lon);
        N := a / sqrt(1 - e2 * sin(lat_rad) ^ 2);
        pos_ecef[1] := (N + alt) * cos(lat_rad) * cos(lon_rad);
        pos_ecef[2] := (N + alt) * cos(lat_rad) * sin(lon_rad);
        pos_ecef[3] := (b ^ 2 / a ^ 2 * N + alt) * sin(lat_rad);
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end lla2ecef;

      function ecef2lla
        input SI.Position[3] pos_ecef;
        output NonSI.Angle_deg lat;
        output NonSI.Angle_deg lon;
        output SI.Distance alt;
      protected
        SI.Position x;
        SI.Position y;
        SI.Position z;
        Real k0;
        Real k;
        Real p;
        Real c;
      algorithm
        x := pos_ecef[1];
        y := pos_ecef[2];
        z := pos_ecef[3];
        k0 := 1 / (1 - e2);
        k := k0;
        p := sqrt(x ^ 2 + y ^ 2);
        for i in 1:2 loop
          c := (p ^ 2 + (1 - e2) * z ^ 2 * k ^ 2) ^ (3 / 2) / (a * e2);
          k := 1 + (p ^ 2 + (1 - e2) * z ^ 2 * k ^ 3) / (c - p ^ 2);
        end for;
        lon := Modelica.Units.Conversions.to_deg(atan2(y, x));
        alt := 1 / e2 * (1 / k - 1 / k0) * sqrt(p ^ 2 + z ^ 2 * k ^ 2);
        lat := Modelica.Units.Conversions.to_deg(atan(k * z / p));
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end ecef2lla;

      function getNadir
        input SI.Position x[3] "Position in ECEF coordinates";
        output Real n[3] "Unit vector pointing in the nadir direction";
      protected
        SI.Position xeq[3];
        SI.Position neqmod;
        Real xeq1[3];
        NonSI.Angle_deg lat;
        NonSI.Angle_deg lon;
      algorithm
        xeq := {x[1], x[2], 0};
// Close to the poles
        if abs(x[3]) > 1000 then
          if abs(xeq * xeq / x[3]) < 1e-5 then
            n := -x ./ norm(x);
            return;
          end if;
        end if;
        (lat, lon) := ecef2lla(x);
// Singularity at  the equator: tan(90-lat) = inf
        if abs(lat) < 0.01 then
          n := -x ./ norm(x);
          return;
        end if;
        neqmod := x[3] * tan(from_deg(90 - lat));
        n := {cos(from_deg(lon)), sin(from_deg(lon)), 0} * neqmod;
        n[3] := x[3];
        n := -n / norm(n);
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end getNadir;

      function getEast
        input Real nadir[3];
        output Real east[3];
      algorithm
        if abs(nadir * {0, 0, 1}) > 1 - 1e-10 then
          east := {0, 1, 0};
        else
          east := cross(nadir, {0, 0, 1});
          east := east / norm(east);
        end if;
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end getEast;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end WGS84;

    function eci2ecefTransform
      input SI.Time t;
      output Real T[3, 3];
      output SI.AngularVelocity w[3];
    algorithm
      w := {0, 0, 72.92115e-6};
      T := [cos(w[3] * t), sin(w[3] * t), 0; -sin(w[3] * t), cos(w[3] * t), 0; 0, 0, 1];
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end eci2ecefTransform;

    function ecef2nedTransform
      input SI.Position x[3] "Position in ECEF coordinates";
      output Real T[3, 3];
      //  output Modelica.Mechanics.MultiBody.Frames.Orientation R;
    protected
      Real D[3];
      Real E[3];
      //  Real T[3,3];
    algorithm
      D := WGS84.getNadir(x);
      E := WGS84.getEast(D);
      T[1, :] := cross(E, D);
      T[2, :] := E;
      T[3, :] := D;
//  R := Modelica.Mechanics.MultiBody.Frames.from_T(T, {0,0,0});
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end ecef2nedTransform;

    function eci2ecef
      input Real v[3] "Vector expressed in ECI frame";
      input SI.Time t;
      output Real v_ecef[3] "v expressed in the ECEF frame";
    protected
      Real T[3, 3];
    algorithm
      T := eci2ecefTransform(t);
      v_ecef := T * v;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end eci2ecef;

    function eci2ecefOrientation
      input SI.Time t;
      output Modelica.Mechanics.MultiBody.Frames.Orientation R;
    protected
      constant SI.AngularVelocity w = 72.92115e-6;
      Real T[3, 3];
    algorithm
      T := [cos(w * t), sin(w * t), 0; -sin(w * t), cos(w * t), 0; 0, 0, 1];
      R := Modelica.Mechanics.MultiBody.Frames.from_T(T, {0, 0, w});
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end eci2ecefOrientation;
  equation

  end MyWorldWGS;
end World;
