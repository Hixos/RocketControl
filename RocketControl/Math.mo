within RocketControl;

package Math
  function quat2euler "Converts a quaternion to the equivalent ZYX euler angle representation"
    input Real q[4];
    output SI.Angle eul[3];
  algorithm
    eul[1] := atan2(2*q[4]*q[3] + 2*q[1]*q[2], q[4]^2 + q[1]^2 - q[2]^2 - q[3]^2);
    eul[2] := asin(2*q[4]*q[2] - 2*q[1]*q[3]);
    eul[3] := atan2(2*q[4]*q[1] + 2*q[2]*q[3], q[4]^2 - q[1]^2 - q[2]^2 + q[3]^2);
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end quat2euler;

  function quatmolt
  input Real[4] q1;
  input Real[4] q2;
  output Real[4] qm;
  algorithm
 qm[1:3] := q2[4]*q1[1:3] + q1[4]*q2[1:3] - cross(q1[1:3], q2[1:3]);
  qm[4] := q1[4]*q2[4] - q1[1:3]*q2[1:3];
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end quatmolt;

  function quatinv
  input Real[4] q;
  
  output Real[4] qinv;
  algorithm
  qinv[1:3] := -q[1:3];
  qinv[4] := q[4];
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end quatinv;

  function sdaCare "Return solution X of the continuous-time algebraic Riccati equation A'*X + X*A - X*B*inv(R)*B'*X + Q = 0 (care) using a structured doubling algorithm"
    extends Modelica.Icons.Function;
    import Modelica.Math.Matrices;
    import Modelica.Math.Matrices.inv;
    
    input Real A[:, size(A, 1)] "Square matrix A in CARE";
    input Real B[size(A, 1), :] "Matrix B in CARE";
    input Real R[size(B, 2), size(B, 2)]=identity(size(B, 2))
      "Matrix R in CARE";
    input Real Q[size(A, 1), size(A, 1)]=identity(size(A, 1))
      "Matrix Q in CARE";
      
    input Real g = -1 "Scalar gamma used by the Cayley transform";
    input Real tol = 1e-13 "Tolerance";
    input Integer kmax(min = 1) = 50 "Maximum number of iterations";
    
    output Real X[size(A, 1), size(A, 2)] "Stabilizing solution of CARE";
    output Integer ki "Number of iterations";
  protected
  Real S[size(A, 1), size(A, 1)];
  Integer n = size(A,1);
  Integer k = 0;
  
  Real err = 1;
  
  Real Ai[size(A, 1), size(A, 1)];
  Real R1[size(A, 1), size(A, 1)];
  Real S1[size(A, 1), size(A, 1)];
  Real E[size(A, 1), size(A, 1)];
  Real G[size(A, 1), size(A, 1)];
  Real P[size(A, 1), size(A, 1)];
  
  Real IGP[size(A, 1), size(A, 1)];
  Real Z[size(A, 1)*2, size(A, 1)];
  Real E1[size(A, 1), size(A, 1)];
  Real P1[size(A, 1), size(A, 1)];
  algorithm
// Initialization
    S := B * inv(R) * transpose(B);
    Ai := inv(A+g*identity(n));
    R1 := S*transpose(Ai)*Q;
    S1 := inv(A+g*identity(n)+R1);
    E := S1*(A-g*identity(n)+R1);
    R1 := identity(n) - Ai*(A-g*identity(n));
    G := S1*S*transpose(R1);
    P := -transpose(S1)*Q*R1;
// SDA step
    while err > tol and k < kmax loop
      IGP := identity(n) - G * P;
      Z := [E; transpose(P)] * inv(IGP);
      E1 := Z[1:n, :];
      P1 := Z[n + 1:end, :];
      G := G + E1 * G * transpose(E);
      P := P + transpose(E) * transpose(P1) * E;
      E := E1 * E;
      err := Matrices.norm(E, 1);
      k := k + 1;
    end while;
    X := P;
    ki := k;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end sdaCare;
  
  function sdaDare "Return solution of discrete-time algebraic Riccati equation A'*X*A - X - A'*X*B*inv(R + B'*X*B)*B'*X*A + Q = 0 (dare) using a structured doubling algorithm"
    extends Modelica.Icons.Function;
    import Modelica.Math.Matrices;
    import Modelica.Math.Matrices.inv;
    
    input Real A[:, size(A, 1)] "Square matrix A in DARE";
    input Real B[size(A, 1), :] "Matrix B in DARE";
    input Real R[size(B, 2), size(B, 2)]=identity(size(B, 2))
      "Matrix R in DARE";
    input Real Q[size(A, 1), size(A, 1)]=identity(size(A, 1))
      "Matrix Q in DARE";
      
    input Real s = 1 "A real positive number such that R+s B'B is nonsingular";
    input Real tol = 1e-13 "Tolerance";
    input Integer kmax(min = 1) = 30 "Maximum number of iterations";
    
    output Real X[size(A, 1), size(A, 2)] "Stabilizing solution of DARE";
    output Integer ki "Number of iterations";
  protected
  
  Integer n = size(A,1);
  Integer k = 0;
  Real err = 1;
  
  Real W[size(A, 1), size(A, 1)];
  Real G[size(A, 1), size(A, 1)];
  Real E[size(A, 1), size(A, 1)];
  Real P[size(A, 1), size(A, 1)];
  Real Z[size(A, 1)*2, size(A, 1)];
  
  Real E1[size(A, 1), size(A, 1)];
  Real P1[size(A, 1), size(A, 1)];
  algorithm
// Initialization
    W := s * identity(n);
    G := -B*inv(R+transpose(B)*W*B)*transpose(B);
    E := (identity(n) + G*W)*A;
    P := Q - W + transpose(A)*W*E;
// SDA step
    while err > tol and k < kmax loop
      Z := [E; transpose(P)] * inv(identity(n) - G * P);
      E1 := Z[1:n, :];
      P1 := Z[n + 1:end, :];
      G := G + E1 * G * transpose(E);
      P := P + transpose(E) * P1 * E;
      E := E1 * E;
      err := Matrices.norm(E, 1);
      k := k + 1;
    end while;
    X := P;
    ki := k;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end sdaDare;

  package Blocks
    model Quaternion2Euler "Converts a quaternion to the equivalent ZYX euler angle representation"
      extends Modelica.Units.Icons.Conversion;
      Modelica.Blocks.Interfaces.RealOutput eul[3](each final quantity = "Angle", each final unit = "rad", each displayUnit = "deg") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput q[4] annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      eul = quat2euler(q);
      annotation(
        Icon(graphics = {Text(origin = {7, 75}, extent = {{-93, 25}, {93, -25}}, textString = "quat", horizontalAlignment = TextAlignment.Left), Text(origin = {-4, -69}, extent = {{-96, 25}, {96, -25}}, textString = "y,p,r", horizontalAlignment = TextAlignment.Right)}));
    end Quaternion2Euler;

    model UnwrapAngle "Unwraps the input angle(s)"
      parameter Integer n(min = 1) = 1 annotation(
        Evaluate = true);
      parameter Boolean inputRadians = true "Wheter the input u is in radians";
      parameter Boolean internalClock = false;
      final parameter Real a180 = if inputRadians then pi else 180;
      parameter Real threshold = a180 / 8;
      Modelica.Blocks.Interfaces.RealInput angle[n](each quantity = "Angle", each unit = "rad", each displayUnit = "deg") annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput angle_unwrapped[n](each quantity = "Angle", each unit = "rad", each displayUnit = "deg") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Real offset[n](each start = 0);
      Real uc[n](each start = 0);
      Clock c = if internalClock then Clock(20, 1000) else Clock();
    equation
      when c then
        if internalClock then
          for i in 1:n loop
            uc[i] = sample(angle[i], c);
          end for;
        else
          uc = angle;
        end if;
        for i in 1:n loop
          if previous(uc[i]) * uc[i] < 0 and a180 - abs(previous(uc[i])) < threshold and a180 - abs(uc[i]) < threshold then
            offset[i] = previous(offset[i]) + previous(uc[i]) - uc[i];
          else
            offset[i] = previous(offset[i]);
          end if;
          angle_unwrapped[i] = uc[i] + offset[i];
        end for;
      end when;
      annotation(
        Icon(graphics = {Rectangle(origin = {0, -0.02}, fillColor = {243, 243, 243}, fillPattern = FillPattern.Solid, extent = {{-100, 99.98}, {100, -99.98}}, radius = 30), Line(origin = {-70, -3}, points = {{0, 79}, {0, -83}}, arrow = {Arrow.Open, Arrow.None}, arrowSize = 20), Line(origin = {-2, 10.38}, points = {{-68, -42}, {32, 18}, {72, 58}}, color = {0, 170, 127}, thickness = 0.5), Line(origin = {5.02, -32.62}, points = {{-75.0184, 1}, {-39.0184, 23}, {-39.0184, -25}, {22.9816, 25}, {22.9816, -23}, {60.9816, 25}}, color = {255, 170, 0}, thickness = 0.5), Text(origin = {-2, -226}, lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 120}}, textString = "%name"), Line(origin = {5, -31.62}, points = {{-75, 0}, {75, 0}}, arrow = {Arrow.None, Arrow.Open}, arrowSize = 20)}));
    end UnwrapAngle;

    model SmallestAngleDiff
      Modelica.Blocks.Interfaces.RealInput a1(quantity = "Angle", unit = "rad", displayUnit = "deg") annotation(
        Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput a2(quantity = "Angle", unit = "rad", displayUnit = "deg") annotation(
        Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput amin(quantity = "Angle", unit = "rad", displayUnit = "deg") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.Angle diff;
    equation
      diff = a2 - a1;
      if noEvent(abs(diff) > pi) then
        amin = -((-2 * pi * sign(diff)) + diff);
      else
        amin = -diff;
      end if;
      annotation(
        Icon(graphics = {Ellipse(fillColor = {255, 255, 127}, fillPattern = FillPattern.Solid, extent = {{-80, 80}, {80, -80}}, endAngle = 360), Line(origin = {-14.87, 25}, points = {{34.8675, 21}, {22.8675, 25}, {8.86754, 25}, {-5.13246, 21}, {-19.1325, 13}, {-27.1325, 3}, {-33.1325, -9}, {-35.1325, -17}, {-35.1325, -25}}, color = {255, 0, 0}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 15), Line(origin = {-19.04, 31}, points = {{-60.9611, -31}, {19.0389, -31}, {49.0389, 43}}, thickness = 0.75, arrow = {Arrow.Open, Arrow.Open}, arrowSize = 10), Text(origin = {46, 29}, extent = {{-26, 21}, {26, -21}}, textString = "a1"), Text(origin = {-48, -21}, extent = {{-26, 21}, {26, -21}}, textString = "a2"), Text(origin = {10, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
    end SmallestAngleDiff;

    block resolve1
      Modelica.Blocks.Interfaces.RealInput q[4] "Quaternion representing the rotation from frame 1 to frame 2" annotation(
        Placement(visible = true, transformation(origin = {-120, 58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput x2[3] "Vector expressed in frame 2" annotation(
        Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput x1[3] "Vector expressed in frame 1" annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      //        Modelica.Mechanics.MultiBody.Frames.Quaternions.Orientation R;
    equation
//  R = Modelica.Mechanics.MultiBody.Frames.Quaternions.Orientation(q, {0,0,0});
      x1 = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve1(q, x2);
      annotation(
        Icon(graphics = {Rectangle(lineColor = {0, 170, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(origin = {0.79, 1.69}, points = {{-38.7932, 12.1393}, {-28.7932, 30.1393}, {-6.7932, 40.1393}, {17.2068, 36.1393}, {31.2068, 24.1393}, {41.2068, 4.13926}, {37.2068, -17.8607}, {21.2068, -37.8607}, {-4.7932, -41.8607}}, thickness = 1.25, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 15, smooth = Smooth.Bezier), Text(origin = {-70, 0}, extent = {{-30, 60}, {30, -60}}, textString = "2"), Text(origin = {64, 0}, extent = {{-30, 60}, {30, -60}}, textString = "1"), Text(origin = {-130, 20}, lineColor = {102, 102, 102}, fillColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "q"), Text(origin = {-130, -100}, lineColor = {102, 102, 102}, fillColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "x2"), Text(origin = {110, -22}, lineColor = {102, 102, 102}, fillColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "x1")}));
    end resolve1;

    package Vector
      extends Internal.VectorIcon;

      model VectorAdd
        extends Internal.VectorIcon;
        parameter Integer n(min = 1) = 3 annotation(Evaluate = true);
        
        parameter Real gain[2] = {1, 1};
        Modelica.Blocks.Interfaces.RealOutput vc[n] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput v2[n] annotation(
          Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput v1[n] annotation(
          Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      equation
        vc = gain[1] * v1 + gain[2] * v2;
        annotation(
          Icon(graphics = {Line(points = {{-60, 0}, {60, 0}}, thickness = 2), Line(points = {{0, 60}, {0, -60}}, thickness = 2), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
      end VectorAdd;

      block VectorGain
        extends Internal.VectorIcon;
        parameter Integer n(min = 1) = 3 annotation(Evaluate = true);
        parameter Boolean external_gain = true;
        
        parameter Real gain;
        Modelica.Blocks.Interfaces.RealInput v[n] annotation(
          Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput k  annotation(
          Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput vk[n]  annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        if not external_gain then
         k = gain;
        end if;
        vk = k*v;
        annotation(
          Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {-130, 20}, lineColor = {118, 118, 118}, fillColor = {118, 118, 118}, extent = {{-30, 20}, {30, -20}}, textString = "v"), Text(origin = {-130, -100}, lineColor = {118, 118, 118}, extent = {{-30, 20}, {30, -20}}, textString = "k"), Text(extent = {{-100, 100}, {100, -100}}, textString = "kv")}));
      end VectorGain;
      
      model VectorConcatenate
        extends Internal.VectorIcon;
        parameter Integer n1(min = 1) = 3 annotation(Evaluate = true);
        parameter Integer n2(min = 1) = 3 annotation(Evaluate = true);
        
        Modelica.Blocks.Interfaces.RealOutput vc[n1+n2] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput v2[n2] annotation(
          Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput v1[n1] annotation(
          Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      equation
        vc = cat(1, v1, v2);
        annotation(
          Icon(graphics = {Text(origin = {0, -250}, textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {1, 5}, extent = {{-85, 59}, {85, -59}}, textString = "{v1,v2}")}));
      end VectorConcatenate;

      block VectorConstant
        extends Internal.VectorIcon;
        parameter Integer n(min = 1) = 3 annotation(Evaluate = true);
        
        parameter Real[n] k;
        Modelica.Blocks.Interfaces.RealOutput v[n] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        v = k;
        annotation(
          Icon(graphics = {Text(origin = {-6, 10}, extent = {{-86, 78}, {86, -78}}, textString = "{k}"), Text(origin = {10, -10}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {10, -250}, extent = {{-150, 150}, {150, 110}}, textString = "%k")}));
      end VectorConstant;

      model UnitVector
        extends Internal.VectorIcon;
        
        parameter Integer n(min = 1) = 3 annotation(Evaluate = true);
        
        parameter Real v_small = 1e-6;
        Modelica.Blocks.Interfaces.RealOutput vu[n] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput v[n] annotation(
          Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Real vnorm;
      equation
        vnorm = norm(v);
        if noEvent(vnorm < v_small) then
          vu = {1, 0, 0};
        else
          vu = v / vnorm;
        end if;
        annotation(
          Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(extent = {{-100, 100}, {100, -100}}, textString = "1")}));
      end UnitVector;

      model CrossProduct
        extends Internal.VectorIcon;
        Modelica.Blocks.Interfaces.RealOutput vc[3] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput v2[3] annotation(
          Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput v1[3] annotation(
          Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      equation
        vc = cross(v1, v2);
        annotation(
          Icon(graphics = {Line(points = {{-60, 60}, {60, -60}, {60, -60}}, thickness = 2), Line(points = {{60, 60}, {-60, -60}}, thickness = 2), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
      end CrossProduct;

      block ProjectOnPlane
        extends Internal.VectorIcon;
        parameter Real n[3];
        Modelica.Blocks.Interfaces.RealInput v[3] annotation(
          Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput vp[3] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        vp = v - v * n * n;
        annotation(
          Icon(graphics = {Polygon(origin = {-2, -30}, fillColor = {156, 227, 249}, fillPattern = FillPattern.Solid, points = {{-90, -38}, {-42, 38}, {90, 38}, {44, -38}, {30, -38}, {-90, -38}}), Line(origin = {-9.72603, 10.5616}, points = {{-26, -51}, {38, 73}}, thickness = 0.75, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Line(origin = {28.3562, 44.7123}, points = {{0, 39}, {0, -71}}, pattern = LinePattern.Dash, thickness = 0.75), Line(origin = {-9.43836, -28.1507}, points = {{-26, -12}, {38, 2}}, color = {255, 0, 0}, thickness = 0.75, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {10, -250}, extent = {{-150, 150}, {150, 110}}, textString = "%n"), Line(origin = {-36, -0.5}, points = {{0, -39.5}, {0, 8.5}, {0, 26.5}}, color = {170, 0, 255}, thickness = 0.5, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Text(origin = {-58, 43}, extent = {{-8, 13}, {8, -13}}, textString = "n")}));
      end ProjectOnPlane;

      block VectorSine
        parameter Real[3] A;
        parameter SI.Frequency f;
        parameter SI.Angle[3] phase(each displayUnit = "deg") = {0, 0, 0};
        parameter Real[3] b = {0, 0, 0};
        extends Internal.VectorIcon;
        Modelica.Blocks.Interfaces.RealOutput y[3] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Sine sine(amplitude = A[1], f = f, offset = b[1], phase = phase[1]) annotation(
          Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Sine sine1(amplitude = A[2], f = f, offset = b[2], phase = phase[2]) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Sine sine2(amplitude = A[3], f = f, offset = b[3], phase = phase[3]) annotation(
          Placement(visible = true, transformation(origin = {0, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(sine.y, y[1]) annotation(
          Line(points = {{12, 50}, {60, 50}, {60, 0}, {110, 0}}, color = {0, 0, 127}));
        connect(sine1.y, y[2]) annotation(
          Line(points = {{12, 0}, {110, 0}}, color = {0, 0, 127}));
        connect(sine2.y, y[3]) annotation(
          Line(points = {{12, -50}, {60, -50}, {60, 0}, {110, 0}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Line(origin = {-37.87, 0.22}, points = {{-49.9838, 21.8869}, {-43.9838, 1.88686}, {-33.9838, -18.1131}, {-19.9838, -30.1131}, {-5.98375, -34.1131}, {8.0162, -30.1131}, {20.0162, -18.1131}, {32.0162, 1.8869}, {38.0162, 21.8869}}, color = {255, 0, 0}, thickness = 0.5), Line(origin = {38.28, 44.34}, rotation = 180, points = {{-49.9838, 21.8869}, {-43.9838, 1.88686}, {-33.9838, -18.1131}, {-19.9838, -30.1131}, {-5.98375, -34.1131}, {8.0162, -30.1131}, {20.0162, -18.1131}, {32.0162, 1.8869}, {38.0162, 21.8869}}, color = {255, 0, 0}, thickness = 0.5), Line(origin = {1.95, -40.82}, points = {{-87, 0}, {87, 0}}, arrow = {Arrow.Open, Arrow.Open}, arrowSize = 7), Line(origin = {1.64, -17.2}, points = {{-87, 0}, {87, 0}}, color = {134, 134, 134}, arrowSize = 7), Line(origin = {0, 2.89}, points = {{0, -20}, {0, 20}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 5), Text(origin = {25, 4}, extent = {{-23, 18}, {23, -18}}, textString = "b"), Text(origin = {-1, -62}, extent = {{-23, 18}, {23, -18}}, textString = "2 pi f")}));
      end VectorSine;

      model VectorNorm
        extends Internal.VectorIcon;
        parameter Integer n(min = 1) = 3 annotation(Evaluate = true);
        
        Modelica.Blocks.Interfaces.RealInput v[n] annotation(
          Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(extent = {{-140, -20}, {-100, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput vnorm annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
      equation
        vnorm = norm(v);
        annotation(
          Icon(graphics = {Text(origin = {-1, 9}, extent = {{-73, 81}, {73, -81}}, textString = "|v|"), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
      end VectorNorm;

      package Internal
        model VectorIcon
        equation

          annotation(
            Icon(coordinateSystem(grid = {2, 0}), graphics = {Rectangle(lineColor = {170, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}})}));
        end VectorIcon;
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Internal;
      
      block VectorIntegrate
        extends Internal.VectorIcon;
        parameter Integer n(min = 1) = 3 annotation(Evaluate = true);
        
        parameter Real k = 1;
        Modelica.Blocks.Interfaces.RealInput v[n] annotation(
          Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput vi[n] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        der(vi) = k * v;
        annotation(
          Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {23, 10}, extent = {{-51, 64}, {51, -64}}, textString = "kv"), Line(origin = {-42.4, 11.56}, points = {{12.006, 67.944}, {-1.994, 69.944}, {-13.994, 51.944}, {14.006, -60.056}, {-1.994, -76.056}, {-17.994, -72.056}}, thickness = 1.25, smooth = Smooth.Bezier)}));
      end VectorIntegrate;
      annotation(
        Icon(graphics = {Text(origin = {0, 19}, extent = {{-100, 81}, {100, -81}}, textString = "v")}));
    end Vector;

    package Quaternion
      extends Internal.QuaternionIcon;

      package Internal
        model QuaternionIcon
        equation

          annotation(
            Icon(graphics = {Rectangle(lineColor = {0, 170, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {0, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
        end QuaternionIcon;
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Internal;

      model Quaternion2Euler "Converts a quaternion to the equivalent ZYX euler angle representation"
        extends Internal.QuaternionIcon;
        Modelica.Blocks.Interfaces.RealOutput eul[3](each final quantity = "Angle", each final unit = "rad", each displayUnit = "deg") annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput q[4] annotation(
          Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      equation
        eul = quat2euler(q);
        annotation(
          Icon(graphics = {Text(origin = {7, 75}, extent = {{-93, 25}, {93, -25}}, textString = "quat", horizontalAlignment = TextAlignment.Left), Text(origin = {-4, -69}, extent = {{-96, 25}, {96, -25}}, textString = "y,p,r", horizontalAlignment = TextAlignment.Right), Line(origin = {-5, 0}, points = {{-75, 0}, {83, 0}}, thickness = 0.5, arrow = {Arrow.None, Arrow.Open}, arrowSize = 15)}));
      end Quaternion2Euler;

      model QuaternionError
        extends Internal.QuaternionIcon;
        import RocketControl.Math.*;
        Modelica.Blocks.Interfaces.RealInput q1[4] annotation(
          Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 42}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput q2[4] annotation(
          Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput qerr[4] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        qerr = quatmolt(q1, quatinv(q2));
        annotation(
          Icon(graphics = {Text(origin = {2, 12}, extent = {{-86, 60}, {86, -60}}, textString = "q q*"), Ellipse(origin = {-24, -4}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Text(origin = {-28, -41}, extent = {{-16, 17}, {16, -17}}, textString = "1"), Text(origin = {48, -43}, extent = {{-16, 17}, {16, -17}}, textString = "2")}));
      end QuaternionError;

      block resolve1
        extends Internal.QuaternionIcon;
        Modelica.Blocks.Interfaces.RealInput q[4] "Quaternion representing the rotation from frame 1 to frame 2" annotation(
          Placement(visible = true, transformation(origin = {-120, 58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput x2[3] "Vector expressed in frame 2" annotation(
          Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput x1[3] "Vector expressed in frame 1" annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        //        Modelica.Mechanics.MultiBody.Frames.Quaternions.Orientation R;
      equation
//  R = Modelica.Mechanics.MultiBody.Frames.Quaternions.Orientation(q, {0,0,0});
        x1 = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve1(q, x2);
        annotation(
          Icon(graphics = {Rectangle(lineColor = {0, 170, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(origin = {0.79, 1.69}, points = {{-38.7932, 12.1393}, {-28.7932, 30.1393}, {-6.7932, 40.1393}, {17.2068, 36.1393}, {31.2068, 24.1393}, {41.2068, 4.13926}, {37.2068, -17.8607}, {21.2068, -37.8607}, {-4.7932, -41.8607}}, thickness = 1.25, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 15, smooth = Smooth.Bezier), Text(origin = {-70, 0}, extent = {{-30, 60}, {30, -60}}, textString = "2"), Text(origin = {64, 0}, extent = {{-30, 60}, {30, -60}}, textString = "1"), Text(origin = {-130, 20}, lineColor = {102, 102, 102}, fillColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "q"), Text(origin = {-130, -100}, lineColor = {102, 102, 102}, fillColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "x2"), Text(origin = {110, -22}, lineColor = {102, 102, 102}, fillColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "x1")}));
      end resolve1;
      annotation(
        Icon(graphics = {Text(origin = {0, 19}, extent = {{-100, 81}, {100, -81}}, textString = "q")}));
    end Quaternion;

    package Matrix
    extends Internal.MatrixIcon;
    
  model MatrixConstant
  extends Internal.MatrixIcon;
  parameter Integer n(min = 1) = 1 annotation(Evaluate = true);
  parameter Integer m(min = 1) = n annotation(Evaluate = true);
  
  parameter Real[n,m] val;
  
  Modelica.Blocks.Interfaces.RealOutput k[n, m] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
k = val;
        annotation(
          Icon(graphics = {Text(origin = {0, 10}, extent = {{-86, 78}, {86, -78}}, textString = "[k]"), Text(origin = {10, -10}, textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
      end MatrixConstant;

      model MatrixMult
      extends Internal.MatrixIcon;
      parameter Integer n(min = 1) = 1 annotation(Evaluate = true);
      parameter Integer m(min = 1) = n annotation(Evaluate = true);
      
  Modelica.Blocks.Interfaces.RealInput x[m] annotation(
          Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(extent = {{-140, -80}, {-100, -40}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput M[n, m] annotation(
          Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(extent = {{-140, 40}, {-100, 80}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Mx[n] annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
      equation
  Mx = M*x;
        annotation(
          Icon(graphics = {Text(origin = {-5, 2}, extent = {{-81, 74}, {81, -74}}, textString = "M*x"), Text(origin = {-73, 61}, lineColor = {102, 102, 102}, extent = {{-27, 21}, {27, -21}}, textString = "M"), Text(origin = {-73, -61}, lineColor = {102, 102, 102}, extent = {{-27, 21}, {27, -21}}, textString = "x"), Text(origin = {10, -10}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
      end MatrixMult;

      package Internal
        model MatrixIcon
        equation

          annotation(
            Icon(coordinateSystem(grid = {2, 0}), graphics = {Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20)}));
        end MatrixIcon;
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Internal;
      annotation(
        Icon(graphics = {Text(origin = {0, 19}, extent = {{-100, 81}, {100, -81}}, textString = "m")}));
    end Matrix;
  equation

    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Blocks;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})),
    Diagram(coordinateSystem(extent = {{0, 0}, {20, -20}})));
end Math;
