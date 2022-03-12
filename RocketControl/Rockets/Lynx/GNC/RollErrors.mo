within RocketControl.Rockets.Lynx.GNC;

model RollErrors
  parameter SI.Angle target_heading(displayUnit = "deg") = 0;
  parameter Real z_small = 1e-6;
  parameter Real ref_dir_body[3] = {0,0,1};
  final parameter Real target_dir[3] = {cos(target_heading), sin(target_heading), 0};
  Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    Real target_dir_body[3];  
    Real target_dir_body_lat[3];  
    SI.Angle err;
equation
  target_dir_body = resolve2(bus.q_est, target_dir);
  
  target_dir_body_lat = cat(1, {0}, target_dir_body[2:3]); // Remove longitudinal component
  
  
  err = RocketControl.Math.Vectors.angle(ref_dir_body, target_dir_body_lat, z_small);

annotation(
    Icon(graphics = {Rectangle(fillColor = {240, 255, 244}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 12), Text(origin = {2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Polygon(origin = {-54, -64}, fillColor = {156, 212, 255}, fillPattern = FillPattern.VerticalCylinder, points = {{10, 18}, {20, 22}, {16, 12}, {-4, -8}, {-6, -18}, {-20, -4}, {-10, -2}, {10, 18}}), Line(origin = {20.87, 26.51}, points = {{-47, -57}, {47, 57}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 12), Line(origin = {9.01, 13.07}, points = {{-23.005, 26.9323}, {-35.005, 12.9323}, {-33.005, -7.06774}, {-17.005, -21.0677}, {2.99497, -25.0677}, {22.995, -19.0677}, {34.995, -7.06774}, {34.995, 6.93226}, {24.995, 18.9323}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 12)}));
end RollErrors;
