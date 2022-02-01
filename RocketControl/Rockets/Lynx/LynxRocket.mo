within RocketControl.Rockets.Lynx;

model LynxRocket
  extends Rockets.Internal.PartialRocket;
  RocketControl.Rockets.Lynx.LynxBody lynxBody annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Aerodynamics.Aerodynamics aerodynamics annotation(
    Placement(visible = true, transformation(origin = {12, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 98}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant fin_pos(k = {0, 0, 0, 0}, n = 4)  annotation(
    Placement(visible = true, transformation(origin = {30, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_lug_bow, lynxBody.frame_lug_bow) annotation(
    Line(points = {{-100, 60}, {-76, 60}, {-76, 6}, {-60, 6}}));
  connect(frame_lug_aft, lynxBody.frame_lug_aft) annotation(
    Line(points = {{-100, -40}, {-76, -40}, {-76, -6}, {-60, -6}}));
  connect(lynxBody.ref_center, ref_center) annotation(
    Line(points = {{-40, 0}, {100, 0}}));
  connect(aerodynamics.frame_b, lynxBody.ref_center) annotation(
    Line(points = {{2, 46}, {-12, 46}, {-12, 0}, {-40, 0}}, color = {95, 95, 95}));
  connect(fin_pos.v, bus.fin_true_position) annotation(
    Line(points = {{42, 88}, {100, 88}, {100, 98}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end LynxRocket;
