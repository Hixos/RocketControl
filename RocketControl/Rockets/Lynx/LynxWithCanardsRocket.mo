within RocketControl.Rockets.Lynx;

model LynxWithCanardsRocket
  extends Rockets.Internal.PartialRocket;
  RocketControl.Rockets.Lynx.LynxBody lynxBody annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.AerodynamicsWithCanards.Aerodynamics aerodynamics annotation(
    Placement(visible = true, transformation(origin = {-10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_lug_bow, lynxBody.frame_lug_bow) annotation(
    Line(points = {{-100, 60}, {-76, 60}, {-76, 6}, {-60, 6}}));
  connect(frame_lug_aft, lynxBody.frame_lug_aft) annotation(
    Line(points = {{-100, -40}, {-76, -40}, {-76, -6}, {-60, -6}}));
  connect(lynxBody.ref_center, ref_center) annotation(
    Line(points = {{-40, 0}, {100, 0}}));
  connect(aerodynamics.frame_b, lynxBody.ref_center) annotation(
    Line(points = {{-20, 50}, {-30, 50}, {-30, 0}, {-40, 0}}, color = {95, 95, 95}));
  connect(bus.fin_setpoint, aerodynamics.finDeflection) annotation(
    Line(points = {{100, 90}, {98, 90}, {98, 30}, {-26, 30}, {-26, 44}, {-20, 44}}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end LynxWithCanardsRocket;
