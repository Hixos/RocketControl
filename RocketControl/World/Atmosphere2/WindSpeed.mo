within RocketControl.World.Atmosphere2;

model WindSpeed
  outer RocketControl.World.Atmosphere atmosphere;
  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0), iconTransformation(extent = {{-116, -16}, {-84, 16}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput v_wind[3] annotation(
    Placement(visible = true, transformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.World.Atmosphere2.BaseWindSpeed baseWindSpeed annotation(
    Placement(visible = true, transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.World.Atmosphere2.WindTurbolence windTurbolence(b = 0.15)  annotation(
    Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput w_wind[3] annotation(
    Placement(visible = true, transformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorAdd vectorAdd annotation(
    Placement(visible = true, transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));    protected
equation
  connect(baseWindSpeed.v_wind, windTurbolence.v_wind) annotation(
    Line(points = {{-38, 30}, {-36, 30}, {-36, -8}, {-22, -8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(frame_a, baseWindSpeed.frame_a) annotation(
    Line(points = {{-100, 0}, {-80, 0}, {-80, 30}, {-60, 30}}));
  connect(frame_a, windTurbolence.frame_a) annotation(
    Line(points = {{-100, 0}, {-20, 0}}));
  connect(windTurbolence.w_turb, w_wind) annotation(
    Line(points = {{2, -4}, {40, -4}, {40, -50}, {110, -50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorAdd.vc, v_wind) annotation(
    Line(points = {{62, 50}, {110, 50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(baseWindSpeed.v_wind, vectorAdd.v1) annotation(
    Line(points = {{-38, 30}, {0, 30}, {0, 54}, {38, 54}}, color = {0, 0, 127}, thickness = 0.5));
  connect(windTurbolence.v_turb, vectorAdd.v2) annotation(
    Line(points = {{2, 6}, {20, 6}, {20, 46}, {38, 46}}, color = {0, 0, 127}, thickness = 0.5));

annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end WindSpeed;
