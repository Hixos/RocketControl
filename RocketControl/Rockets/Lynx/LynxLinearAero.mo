within RocketControl.Rockets.Lynx;

model LynxLinearAero
  extends Rockets.Internal.PartialRocket;
  RocketControl.Rockets.Lynx.LynxBody lynxBody annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.LinearAerodynamicsWithCanards.LinearAerodynamics linearAerodynamics annotation(
    Placement(visible = true, transformation(origin = {8, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_lug_bow, lynxBody.frame_lug_bow) annotation(
    Line(points = {{-100, 60}, {-76, 60}, {-76, 6}, {-60, 6}}));
  connect(frame_lug_aft, lynxBody.frame_lug_aft) annotation(
    Line(points = {{-100, -40}, {-76, -40}, {-76, -6}, {-60, -6}}));
  connect(lynxBody.ref_center, ref_center) annotation(
    Line(points = {{-40, 0}, {100, 0}}));
  connect(linearAerodynamics.frame_b, lynxBody.ref_center) annotation(
    Line(points = {{-2, 56}, {-40, 56}, {-40, 0}}, color = {95, 95, 95}));
  connect(linearAerodynamics.finDeflection, bus.fin_setpoint) annotation(
    Line(points = {{-2, 50}, {-34, 50}, {-34, 32}, {100, 32}, {100, 90}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end LynxLinearAero;
