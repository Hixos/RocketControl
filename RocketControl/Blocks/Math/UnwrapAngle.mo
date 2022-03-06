within RocketControl.Blocks.Math;

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
