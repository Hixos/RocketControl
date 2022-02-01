within RocketControl.Rockets.Internal;

partial model PartialRocket
  extends PartialRocketBody;
equation

  annotation(
    Icon(graphics = {Line(origin = {46, 47}, points = {{54, 23}, {-46, -47}}, color = {0, 255, 0}), Line(origin = {-1, -7}, points = {{-11, 61}, {15, 61}, {1, 61}, {1, -61}, {-15, -61}, {1, -61}, {1, -23}, {15, -23}, {1, -23}, {1, 7}, {-13, 7}}, color = {85, 255, 0}), Text(origin = {-10, -254}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name")}));
end PartialRocket;
