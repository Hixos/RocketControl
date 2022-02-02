within RocketControl.Components.LaunchPad;

model LaunchRail
  parameter SI.Angle azimuth;
  parameter SI.Angle elevation;
  parameter SI.Distance lug_length;
  parameter SI.Distance rail_length;
  parameter SI.ModulusOfElasticity c_x;
  parameter SI.DampingCoefficient d_x;
  parameter SI.ModulusOfElasticity c_y;
  parameter SI.DampingCoefficient d_y;
  parameter SI.ModulusOfElasticity c_z;
  parameter SI.DampingCoefficient d_z;
  parameter SI.Position r_rel[3] = {0, 0, 0} "Relative position between the ramp base and the aft flange";
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(angle_1(fixed = true), angle_2(fixed = true), angle_3(fixed = true), angle_d_1(fixed = false), angle_d_2(fixed = false), angle_d_3(fixed = false), animation = false, r_rel_a_1(fixed = true, start = r_rel[1]), r_rel_a_2(fixed = true, start = r_rel[2]), r_rel_a_3(fixed = true, start = r_rel[3]), sequence_start = {3, 2, 1}, use_angle = true, use_r = true, use_v = true, use_w = true, v_rel_a_1(fixed = true), v_rel_a_2(fixed = true), v_rel_a_3(fixed = true), w_rel_b_1(fixed = true), w_rel_b_2(fixed = true), w_rel_b_3(fixed = true)) annotation(
    Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  LaunchRailPresenceSensor launchPadSensorBase(n = {1, 0, 0}, rail_length = 1e-8) annotation(
    Placement(visible = true, transformation(origin = {10, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  LaunchLug lug_aft(c_ax = c_z / 2, c_norm = c_y / 2, d_ax = d_z / 2, d_norm = d_y / 2, lug_length = lug_length, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  LaunchLug lug_bow(c_ax = c_z / 2, c_norm = c_y / 2, d_ax = d_z / 2, d_norm = d_y / 2, lug_length = lug_length, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {-10, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Parts.LinearSpringDamperParallel linearSpringDamperParallel(c = c_x, d = d_x, n = {1, 0, 0}, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  LaunchRailPresenceSensor launchPadSensorAft(n = {1, 0, 0}, rail_length = rail_length) annotation(
    Placement(visible = true, transformation(origin = {-70, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation launchRailAngle(angles = to_deg({azimuth, elevation, 0}), animation = false, rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.PlanarRotationSequence, sequence = {3, 2, 1}) annotation(
    Placement(visible = true, transformation(origin = {-62, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  LaunchRailPresenceSensor launchPadSensorBow(n = {1, 0, 0}, rail_length = rail_length) annotation(
    Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(origin = {-100, -50}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b_lug_bow annotation(
    Placement(visible = true, transformation(origin = {100, 80}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 60}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b_lug_aft annotation(
    Placement(visible = true, transformation(origin = {100, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Blocks.Logical.Not not1 annotation(
    Placement(visible = true, transformation(origin = {50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.BooleanOutput liftoff annotation(
    Placement(visible = true, transformation(origin = {108, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, -104}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
equation
  connect(frame_a, launchRailAngle.frame_a) annotation(
    Line(points = {{-100, -50}, {-72, -50}}));
  connect(freeMotionScalarInit.frame_a, launchRailAngle.frame_b) annotation(
    Line(points = {{20, -10}, {-40, -10}, {-40, -50}, {-52, -50}}, color = {95, 95, 95}));
  connect(launchRailAngle.frame_b, launchPadSensorAft.frame_rail) annotation(
    Line(points = {{-52, -50}, {-40, -50}, {-40, -20}, {-70, -20}, {-70, 0}}));
  connect(launchRailAngle.frame_b, linearSpringDamperParallel.frame_a) annotation(
    Line(points = {{-52, -50}, {-40, -50}, {-40, -90}, {70, -90}, {70, -80}}, color = {95, 95, 95}));
  connect(launchPadSensorBase.frame_rail, launchRailAngle.frame_b) annotation(
    Line(points = {{10, -80}, {10, -90}, {-40, -90}, {-40, -50}, {-52, -50}}));
  connect(linearSpringDamperParallel.frame_b, frame_b_lug_aft) annotation(
    Line(points = {{70, -60}, {70, 20}, {100, 20}}));
  connect(freeMotionScalarInit.frame_b, frame_b_lug_aft) annotation(
    Line(points = {{40, -10}, {60, -10}, {60, 20}, {100, 20}}));
  connect(lug_bow.frame_a, launchRailAngle.frame_b) annotation(
    Line(points = {{-20, 68}, {-40, 68}, {-40, -50}, {-52, -50}}, color = {95, 95, 95}));
  connect(lug_bow.frame_b, frame_b_lug_bow) annotation(
    Line(points = {{0, 68}, {50, 68}, {50, 80}, {100, 80}}));
  connect(launchPadSensorBow.frame_rail, launchRailAngle.frame_b) annotation(
    Line(points = {{-70, 60}, {-70, 30}, {-40, 30}, {-40, -50}, {-52, -50}}, color = {95, 95, 95}));
  connect(launchPadSensorBow.frame_lug, lug_bow.frame_b) annotation(
    Line(points = {{-60, 66}, {-31, 66}, {-31, 68}, {0, 68}}));
  connect(launchPadSensorBow.y, lug_bow.enable) annotation(
    Line(points = {{-60, 77}, {-44, 77}, {-44, 84}, {-10, 84}, {-10, 78}}, color = {255, 0, 255}));
  connect(launchPadSensorAft.frame_lug, lug_aft.frame_b) annotation(
    Line(points = {{-60, 6}, {14, 6}, {14, 20}, {0, 20}}, color = {95, 95, 95}));
  connect(lug_aft.frame_b, frame_b_lug_aft) annotation(
    Line(points = {{0, 20}, {100, 20}}, color = {95, 95, 95}));
  connect(launchPadSensorBase.y, linearSpringDamperParallel.enable) annotation(
    Line(points = {{20, -62}, {34, -62}, {34, -70}, {61, -70}}, color = {255, 0, 255}));
  connect(launchPadSensorBase.frame_lug, linearSpringDamperParallel.frame_b) annotation(
    Line(points = {{20, -74}, {38, -74}, {38, -54}, {70, -54}, {70, -60}}));
  connect(launchPadSensorAft.y, lug_aft.enable) annotation(
    Line(points = {{-60, 17}, {-32, 17}, {-32, 36}, {-10, 36}, {-10, 30}}, color = {255, 0, 255}));
  connect(lug_aft.frame_a, launchRailAngle.frame_b) annotation(
    Line(points = {{-20, 20}, {-40, 20}, {-40, -50}, {-52, -50}}));
  connect(not1.u, launchPadSensorBase.y) annotation(
    Line(points = {{38, -40}, {20, -40}, {20, -62}}, color = {255, 0, 255}));
  connect(not1.y, liftoff) annotation(
    Line(points = {{62, -40}, {108, -40}}, color = {255, 0, 255}));
  annotation(
    Icon(graphics = {Polygon(origin = {-20, -6}, fillColor = {85, 122, 162}, fillPattern = FillPattern.VerticalCylinder, points = {{-28, -42}, {-14, -50}, {-14, -42}, {26, 38}, {28, 50}, {20, 42}, {-20, -38}, {-28, -42}}), Polygon(origin = {-9, 8}, fillColor = {222, 222, 222}, fillPattern = FillPattern.Forward, points = {{17, 88}, {-71, -78}, {-71, -88}, {71, -88}, {49, -80}, {-55, -80}, {17, 62}, {17, 88}}), Rectangle(origin = {6, 20}, fillPattern = FillPattern.Solid, extent = {{-2, 4}, {2, -4}}), Rectangle(origin = {-22, -36}, fillPattern = FillPattern.Solid, extent = {{-2, 4}, {2, -4}}), Text(lineColor = {0, 0, 255}, extent = {{-150, 80}, {150, 120}}, textString = "%name"), Text(origin = {-760.66, 76}, lineColor = {128, 128, 128}, extent = {{566.66, -29}, {770.66, -58}}, textString = "a"), Text(origin = {-566.66, 74}, lineColor = {128, 128, 128}, extent = {{566.66, -29}, {770.66, -58}}, textString = "bow"), Text(origin = {-227.774, 14}, lineColor = {128, 128, 128}, extent = {{277.774, -29}, {377.774, -58}}, textString = "aft"), Line(origin = {40, -48}, points = {{-62, 12}, {20, 12}, {20, -12}, {62, -12}}), Line(origin = {53, 40}, points = {{-47, -20}, {7, -20}, {7, 20}, {47, 20}})}));
end LaunchRail;
