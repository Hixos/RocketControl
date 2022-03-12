within RocketControl.Rockets.Lynx.StateMachines;

model FlightModeManager
  outer RocketControl.World.Interfaces.WorldBase world;
  outer RocketControl.World.SimOptions opt;
  Modelica.StateGraph.Alternative alternative(nBranches = 2)  annotation(
    Placement(visible = true, transformation(origin = {14, 92}, extent = {{-102, -68}, {102, 68}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.StateGraph.InitialStep on_ramp_state(nOut = 1, nIn = 1)  annotation(
    Placement(visible = true, transformation(origin = {-170, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.StateGraph.TransitionWithSignal liftoff(waitTime = 0)  annotation(
    Placement(visible = true, transformation(origin = {-142, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.StateGraph.Parallel ascent_state annotation(
    Placement(visible = true, transformation(origin = {8, 70}, extent = {{-46, -44}, {46, 44}}, rotation = 0)));
  Modelica.StateGraph.StepWithSignal roll_control_state(nIn = 1, nOut = 1)  annotation(
    Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.StateGraph.StepWithSignal v_control_state(nIn = 1, nOut = 1) annotation(
    Placement(visible = true, transformation(origin = {-13, 93}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  Modelica.StateGraph.TransitionWithSignal stable_flight annotation(
    Placement(visible = true, transformation(origin = {7, 99}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  Modelica.StateGraph.StepWithSignal apogee_control_state(nIn = 1, nOut = 1) annotation(
    Placement(visible = true, transformation(origin = {25, 93}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  Modelica.StateGraph.TransitionWithSignal apogee annotation(
    Placement(visible = true, transformation(origin = {112, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.StateGraph.StepWithSignal drogue_descent_state(nIn = 1, nOut = 1) annotation(
    Placement(visible = true, transformation(origin = {40, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.StateGraph.StepWithSignal main_descent_state(nIn = 1, nOut = 1) annotation(
    Placement(visible = true, transformation(origin = {-78, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.StateGraph.TransitionWithSignal main_altitude annotation(
    Placement(visible = true, transformation(origin = {-38, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.StateGraph.Transition landed(condition = false, enableTimer = false)  annotation(
    Placement(visible = true, transformation(origin = {-166, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.StateGraph.TransitionWithSignal terminal_ascent_1 annotation(
    Placement(visible = true, transformation(origin = {68, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.Timer met annotation(
    Placement(visible = true, transformation(origin = {-16, -22}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression v_control_enable(y = met.y > opt.guidance_disable_met)  annotation(
    Placement(visible = true, transformation(origin = {-10, 74}, extent = {{-10, -8}, {10, 8}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression termina_ascent_expr(y = hold(bus.v_est[3]) > (-30)) annotation(
    Placement(visible = true, transformation(origin = {107, 15}, extent = {{-27, -7}, {27, 7}}, rotation = 180)));
  Modelica.Blocks.Sources.BooleanExpression apogee_expr(y = hold(bus.v_est[3]) > 1) annotation(
    Placement(visible = true, transformation(origin = {55, -39}, extent = {{-19, -7}, {19, 7}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression main_alt_expr(y = hold(world.altitude_agl(bus.x_est)) < 350) annotation(
    Placement(visible = true, transformation(origin = {-100, -45}, extent = {{-34, -7}, {34, 7}}, rotation = 0)));
 inner Modelica.StateGraph.StateGraphRoot stateGraphRoot annotation(
    Placement(visible = true, transformation(origin = {-136, 136}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Blocks.Sources.BooleanExpression main_deploy(y = main_descent_state.active) annotation(
    Placement(visible = true, transformation(origin = {-108, -17}, extent = {{-24, -7}, {24, 7}}, rotation = 0)));
 Modelica.Blocks.Sources.BooleanExpression drogue_deploy(y = drogue_descent_state.active or main_descent_state.active) annotation(
    Placement(visible = true, transformation(origin = {-108, -29}, extent = {{-24, -7}, {24, 7}}, rotation = 0)));
 Modelica.StateGraph.Step terminal_ascent_state(nIn = 1, nOut = 1)  annotation(
    Placement(visible = true, transformation(origin = {128, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
 Modelica.Clocked.BooleanSignals.Sampler.SampleClocked sample1 annotation(
    Placement(visible = true, transformation(origin = {0, 2.22045e-16}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
 Modelica.Clocked.BooleanSignals.Sampler.SampleClocked sampleClocked annotation(
    Placement(visible = true, transformation(origin = {16, 2.22045e-16}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
 Modelica.Clocked.BooleanSignals.Sampler.SampleClocked sampleClocked1 annotation(
    Placement(visible = true, transformation(origin = {30, 2.22045e-16}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
 Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock periodicClock1(factor = opt.samplePeriodMs)  annotation(
    Placement(visible = true, transformation(origin = {-22, 0}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
 Modelica.Blocks.Logical.And and1 annotation(
    Placement(visible = true, transformation(origin = {-49, -51}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
 Modelica.Blocks.Logical.And and2 annotation(
    Placement(visible = true, transformation(origin = {91, -45}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
 Modelica.StateGraph.Step no_guidance_state(nIn = 1, nOut = 1) annotation(
    Placement(visible = true, transformation(origin = {10, 138}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.StateGraph.Step liftoff_state(nIn = 1, nOut = 1) annotation(
    Placement(visible = true, transformation(origin = {-110, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.StateGraph.TransitionWithSignal guidance_disable(waitTime = 0) annotation(
    Placement(visible = true, transformation(origin = {-38, 138}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Blocks.Sources.BooleanExpression booleanExpression(y = opt.guidance_disable) annotation(
    Placement(visible = true, transformation(origin = {-98, 113}, extent = {{-24, -7}, {24, 7}}, rotation = 0)));
 Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y = opt.main_enable) annotation(
    Placement(visible = true, transformation(origin = {-118, -59}, extent = {{-24, -7}, {24, 7}}, rotation = 0)));
 Modelica.Blocks.Sources.BooleanExpression booleanExpression2(y = opt.drogue_enable) annotation(
    Placement(visible = true, transformation(origin = {36, -55}, extent = {{-24, -7}, {24, 7}}, rotation = 0)));
 Modelica.StateGraph.TransitionWithSignal terminal_ascent_2 annotation(
    Placement(visible = true, transformation(origin = {68, 138}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.StateGraph.TransitionWithSignal guidance_enable(waitTime = 0) annotation(
    Placement(visible = true, transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Blocks.Sources.BooleanExpression booleanExpression3(y = met.y >= opt.guidance_enable_met) annotation(
    Placement(visible = true, transformation(origin = {-104, 23}, extent = {{-24, -7}, {24, 7}}, rotation = 0)));
equation
  connect(on_ramp_state.outPort[1], liftoff.inPort) annotation(
    Line(points = {{-159.5, 84}, {-146, 84}}));
  connect(roll_control_state.inPort[1], ascent_state.split[1]) annotation(
    Line(points = {{-11, 40}, {-14.5, 40}, {-14.5, 70}, {-28, 70}}));
  connect(v_control_state.inPort[1], ascent_state.split[2]) annotation(
    Line(points = {{-20.7, 93}, {-28, 93}, {-28, 70}}));
  connect(roll_control_state.outPort[1], ascent_state.join[1]) annotation(
    Line(points = {{10.5, 40}, {39.25, 40}, {39.25, 70}, {44, 70}}));
  connect(v_control_state.outPort[1], stable_flight.inPort) annotation(
    Line(points = {{-5.65, 93}, {1.35, 93}, {1.35, 99}, {4.35, 99}}));
  connect(stable_flight.outPort, apogee_control_state.inPort[1]) annotation(
    Line(points = {{8.05, 99}, {19.55, 99}, {19.55, 93}, {17.05, 93}}));
  connect(apogee_control_state.outPort[1], ascent_state.join[2]) annotation(
    Line(points = {{32.35, 93}, {44, 93}, {44, 70}}));
  connect(apogee.outPort, drogue_descent_state.inPort[1]) annotation(
    Line(points = {{110.5, -80}, {51, -80}}));
  connect(drogue_descent_state.outPort[1], main_altitude.inPort) annotation(
    Line(points = {{29.5, -80}, {-34, -80}}));
  connect(main_descent_state.inPort[1], main_altitude.outPort) annotation(
    Line(points = {{-67, -80}, {-39.5, -80}}));
  connect(landed.inPort, main_descent_state.outPort[1]) annotation(
    Line(points = {{-162, -80}, {-88.5, -80}}));
  connect(stable_flight.condition, v_control_enable.y) annotation(
    Line(points = {{7, 90.6}, {5, 90.6}, {5, 74}, {1, 74}}, color = {255, 0, 255}));
  connect(landed.outPort, on_ramp_state.inPort[1]) annotation(
    Line(points = {{-167.5, -80}, {-185.5, -80}, {-185.5, 84}, {-181, 84}}));
  connect(bus.liftoff, met.u) annotation(
    Line(points = {{0, -30}, {-26, -30}, {-26, -22}, {-21, -22}}, color = {255, 0, 255}));
  connect(drogue_deploy.y, bus.drogue_deploy) annotation(
    Line(points = {{-82, -29}, {-36, -29}, {-36, -30}, {0, -30}}, color = {255, 0, 255}));
  connect(main_deploy.y, bus.main_deploy) annotation(
    Line(points = {{-82, -17}, {-36, -17}, {-36, -30}, {0, -30}}, color = {255, 0, 255}));
  connect(terminal_ascent_state.outPort[1], apogee.inPort) annotation(
    Line(points = {{128, -20.5}, {128, -80.5}, {116, -80.5}}));
  connect(roll_control_state.active, sample1.u) annotation(
    Line(points = {{0, 29}, {0, 4}}, color = {255, 0, 255}));
  connect(sample1.y, bus.roll_guidance) annotation(
    Line(points = {{0, -4}, {0, -30}}, color = {255, 0, 255}));
  connect(sampleClocked.y, bus.velocity_guidace) annotation(
    Line(points = {{16, -4}, {16, -30}, {0, -30}}, color = {255, 0, 255}));
  connect(sampleClocked1.y, bus.apogee_guidance) annotation(
    Line(points = {{30, -4}, {30, -30}, {0, -30}}, color = {255, 0, 255}));
  connect(apogee_control_state.active, sampleClocked1.u) annotation(
    Line(points = {{25, 85}, {30, 85}, {30, 4}}, color = {255, 0, 255}));
  connect(sampleClocked.u, v_control_state.active) annotation(
    Line(points = {{16, 4}, {16, 72}, {-13, 72}, {-13, 85}}, color = {255, 0, 255}));
  connect(periodicClock1.y, sample1.clock) annotation(
    Line(points = {{-20, 0}, {-4, 0}}, color = {175, 175, 175}));
  connect(periodicClock1.y, sampleClocked.clock) annotation(
    Line(points = {{-20, 0}, {12, 0}}, color = {175, 175, 175}));
  connect(periodicClock1.y, sampleClocked1.clock) annotation(
    Line(points = {{-20, 0}, {26, 0}}, color = {175, 175, 175}));
  connect(main_alt_expr.y, and1.u1) annotation(
    Line(points = {{-63, -45}, {-57, -45}, {-57, -51}, {-55, -51}}, color = {255, 0, 255}));
  connect(and1.y, main_altitude.condition) annotation(
    Line(points = {{-43.5, -51}, {-38, -51}, {-38, -68}}, color = {255, 0, 255}));
  connect(apogee_expr.y, and2.u1) annotation(
    Line(points = {{76, -39}, {84.9, -39}, {84.9, -46}}, color = {255, 0, 255}));
  connect(and2.y, apogee.condition) annotation(
    Line(points = {{96.5, -45}, {112, -45}, {112, -68}}, color = {255, 0, 255}));
  connect(bus.liftoff, liftoff.condition) annotation(
    Line(points = {{0, -30}, {-6, -30}, {-6, -10}, {-142, -10}, {-142, 72}}, color = {255, 0, 255}));
  connect(termina_ascent_expr.y, terminal_ascent_1.condition) annotation(
    Line(points = {{77, 15}, {68, 15}, {68, 58}}, color = {255, 0, 255}));
  connect(liftoff.outPort, liftoff_state.inPort[1]) annotation(
    Line(points = {{-140.5, 84}, {-121, 84}}));
  connect(liftoff_state.outPort[1], alternative.inPort) annotation(
    Line(points = {{-99.5, 84}, {-91.25, 84}, {-91.25, 92}, {-91, 92}}));
  connect(guidance_disable.inPort, alternative.split[1]) annotation(
    Line(points = {{-42, 138}, {-67, 138}, {-67, 92}}));
  connect(booleanExpression.y, guidance_disable.condition) annotation(
    Line(points = {{-72, 113}, {-38, 113}, {-38, 126}}, color = {255, 0, 255}));
  connect(booleanExpression1.y, and1.u2) annotation(
    Line(points = {{-92, -58}, {-54, -58}, {-54, -54}}, color = {255, 0, 255}));
  connect(booleanExpression2.y, and2.u2) annotation(
    Line(points = {{62, -54}, {86, -54}, {86, -48}}, color = {255, 0, 255}));
  connect(guidance_disable.outPort, no_guidance_state.inPort[1]) annotation(
    Line(points = {{-36.5, 138}, {-1, 138}}));
  connect(ascent_state.outPort, terminal_ascent_1.inPort) annotation(
    Line(points = {{54, 70}, {64, 70}}));
  connect(terminal_ascent_1.outPort, alternative.join[2]) annotation(
    Line(points = {{70, 70}, {95, 70}, {95, 92}}));
  connect(terminal_ascent_2.condition, terminal_ascent_1.condition) annotation(
    Line(points = {{68, 126}, {68, 58}}, color = {255, 0, 255}));
  connect(no_guidance_state.outPort[1], terminal_ascent_2.inPort) annotation(
    Line(points = {{20, 138}, {64, 138}}));
  connect(terminal_ascent_2.outPort, alternative.join[1]) annotation(
    Line(points = {{70, 138}, {95, 138}, {95, 92}}));
  connect(alternative.outPort, terminal_ascent_state.inPort[1]) annotation(
    Line(points = {{118, 92}, {128, 92}, {128, 2}}));
 connect(ascent_state.inPort, guidance_enable.outPort) annotation(
    Line(points = {{-40, 70}, {-48.5, 70}}));
 connect(guidance_enable.inPort, alternative.split[2]) annotation(
    Line(points = {{-54, 70}, {-66, 70}, {-66, 92}}));
 connect(booleanExpression3.y, guidance_enable.condition) annotation(
    Line(points = {{-78, 24}, {-50, 24}, {-50, 58}}, color = {255, 0, 255}));
  annotation(
    Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Rectangle(origin = {-30, 61}, extent = {{-30, 19}, {30, -19}}), Rectangle(origin = {50, 1}, extent = {{-30, 19}, {30, -19}}), Rectangle(origin = {-30, -59}, extent = {{-30, 19}, {30, -19}}), Line(origin = {25, 40}, points = {{-25, 20}, {25, 20}, {25, -20}}), Line(origin = {25, -39}, points = {{25, 21}, {25, -21}, {-25, -21}}), Rectangle(origin = {-30, 61}, extent = {{-28, 17}, {28, -17}}), Rectangle(origin = {50, 1}, extent = {{-28, 17}, {28, -17}}), Rectangle(origin = {-30, -59}, extent = {{-28, 17}, {28, -17}}), Text(origin = {-10, -254}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name")}),
    Diagram(coordinateSystem(extent = {{-200, 160}, {140, -100}})));
end FlightModeManager;
