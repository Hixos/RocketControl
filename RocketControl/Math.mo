within RocketControl;

package Math
  function quat2euler
  input Real q[4];
  output SI.Angle eul[3];
  algorithm
  eul[1] := atan2(2*(q[1]*q[2]+q[3]*q[4]), q[4]^2 + q[1]^2 - q[2]^2 - q[3]^2);
    eul[2] := asin(-2*(q[1]*q[3]-q[2]*q[4]));
    eul[3] := atan2(2*(q[2]*q[3] + q[1]*q[4]), q[4]^2-q[1]^2-q[2]^2 + q[3]^2);
  annotation(
      Icon(coordinateSystem(grid = {2, 0})));end quat2euler;

  model Blocks
    model Quaternion
    Modelica.Blocks.Interfaces.RealOutput q[4] annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
        Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    equation
    frame_a.f = {0,0,0};
    frame_a.t = {0,0,0};
    
    q = Modelica.Mechanics.MultiBody.Frames.Quaternions.from_T(frame_a.R.T);
      annotation(
        Icon(coordinateSystem(grid = {2, 0}), graphics = {Text(extent = {{-100, 100}, {100, -100}}, textString = "quat")}));
    end Quaternion;

    model Quaternion2Euler
     extends Modelica.Units.Icons.Conversion;
    Modelica.Blocks.Interfaces.RealOutput eul[3](each final quantity = "Angle", each final unit = "deg") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput q[4] annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
  eul = to_deg(quat2euler(q));
      annotation(
        Icon(graphics = {Text(origin = {7, 75}, extent = {{-93, 25}, {93, -25}}, textString = "quat", horizontalAlignment = TextAlignment.Left), Text(origin = {-4, -69}, extent = {{-96, 25}, {96, -25}}, textString = "y,p,r", horizontalAlignment = TextAlignment.Right)}));
    end Quaternion2Euler;

    model UnwrapAngle
    parameter Integer n(min = 1) = 1 annotation(Evaluate = true);
    parameter Boolean inputRadians = false "Wheter the input u is in radians";
    parameter Boolean internalClock = false;
    
    final parameter Real a180 = if inputRadians then pi else 180;
    parameter Real threshold = a180/8;
    
    Modelica.Blocks.Interfaces.RealInput u[n] annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y[n] annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
      
        Real offset[n](each start = 0);
        Real uc[n](each start = 0);
        Clock c = if internalClock then Clock(20, 1000) else Clock();
    equation
        when c then
          if internalClock then
            for i in 1:n loop
              uc[i] = sample(u[i], c);
            end for;
          else        
              uc = u;   
          end if;
          for i in 1:n loop
            if previous(uc[i])*uc[i] < 0 and a180 - abs(previous(uc[i])) < threshold and  a180 - abs(uc[i]) < threshold then
              offset[i] = previous(offset[i]) + (previous(uc[i]) - uc[i]);
            else
              offset[i] = previous(offset[i]);
            end if;
            y[i] = uc[i] + offset[i];
          end for;
        end when;
      
      annotation(
        Icon(graphics = {Rectangle(origin = {0, -0.02}, fillColor = {243, 243, 243}, fillPattern = FillPattern.Solid, extent = {{-100, 99.98}, {100, -99.98}}, radius = 30), Line(origin = {-70, -3}, points = {{0, 79}, {0, -83}}, arrow = {Arrow.Open, Arrow.None}, arrowSize = 20), Line(origin = {-2, 10.38}, points = {{-68, -42}, {32, 18}, {72, 58}}, color = {0, 170, 127}, thickness = 0.5), Line(origin = {5.02, -32.62}, points = {{-75.0184, 1}, {-39.0184, 23}, {-39.0184, -25}, {22.9816, 25}, {22.9816, -23}, {60.9816, 25}}, color = {255, 170, 0}, thickness = 0.5), Text(origin = {-2, -226}, lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 120}}, textString = "%name"), Line(origin = {5, -31.62}, points = {{-75, 0}, {75, 0}}, arrow = {Arrow.None, Arrow.Open}, arrowSize = 20)}));
    end UnwrapAngle;
  equation

    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Blocks;

  function unwrapAngle
  input SI.Angle angle;
  input SI.Angle tol = 0;
  
  output SI.Angle angle_unwrap;
  algorithm

    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end unwrapAngle;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end Math;
