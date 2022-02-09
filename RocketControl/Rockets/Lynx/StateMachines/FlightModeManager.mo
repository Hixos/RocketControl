within RocketControl.Rockets.Lynx.StateMachines;

model FlightModeManager
  outer RocketControl.World.Interfaces.WorldBase world;
 
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.StateGraph.InitialStep liftoff_state(nOut = 1, nIn = 1)  annotation(
    Placement(visible = true, transformation(origin = {-80, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.StateGraph.TransitionWithSignal guidance_enable(waitTime = 1)  annotation(
    Placement(visible = true, transformation(origin = {-50, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.StateGraph.Parallel ascent_state annotation(
    Placement(visible = true, transformation(origin = {6, 58}, extent = {{-46, -44}, {46, 44}}, rotation = 0)));
  Modelica.StateGraph.StepWithSignal roll_control_state(nIn = 1, nOut = 1)  annotation(
    Placement(visible = true, transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.StateGraph.StepWithSignal v_control_state(nIn = 1, nOut = 1) annotation(
    Placement(visible = true, transformation(origin = {-13, 83}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  Modelica.StateGraph.TransitionWithSignal stable_flight annotation(
    Placement(visible = true, transformation(origin = {7, 89}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  Modelica.StateGraph.StepWithSignal apogee_control_state(nIn = 1, nOut = 1) annotation(
    Placement(visible = true, transformation(origin = {25, 83}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  Modelica.StateGraph.TransitionWithSignal apogee annotation(
    Placement(visible = true, transformation(origin = {74, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.StateGraph.StepWithSignal drogue_descent_state(nIn = 1, nOut = 1) annotation(
    Placement(visible = true, transformation(origin = {40, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.StateGraph.StepWithSignal main_descent_state(nIn = 1, nOut = 1) annotation(
    Placement(visible = true, transformation(origin = {-40, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.StateGraph.TransitionWithSignal main_altitude annotation(
    Placement(visible = true, transformation(origin = {0, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.StateGraph.Transition landed(condition = false, enableTimer = false)  annotation(
    Placement(visible = true, transformation(origin = {-80, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.StateGraph.TransitionWithSignal terminal_ascent annotation(
    Placement(visible = true, transformation(origin = {80, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.Timer met annotation(
    Placement(visible = true, transformation(origin = {-14, -6}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression v_control_enable(y = met.y > 10)  annotation(
    Placement(visible = true, transformation(origin = {-10, 64}, extent = {{-10, -8}, {10, 8}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression termina_ascent_expr(y = bus.v_est[3] > (-30)) annotation(
    Placement(visible = true, transformation(origin = {61, 23}, extent = {{-19, -7}, {19, 7}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression apogee_expr(y = bus.v_est[3] > 1) annotation(
    Placement(visible = true, transformation(origin = {35, -53}, extent = {{-19, -7}, {19, 7}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression main_alt_expr(y = world.altitude_agl(bus.x_est) < 400) annotation(
    Placement(visible = true, transformation(origin = {-62, -61}, extent = {{-34, -7}, {34, 7}}, rotation = 0)));
 inner Modelica.StateGraph.StateGraphRoot stateGraphRoot annotation(
    Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Blocks.Sources.BooleanExpression main_deploy(y = main_descent_state.active) annotation(
    Placement(visible = true, transformation(origin = {-74, 3}, extent = {{-24, -7}, {24, 7}}, rotation = 0)));
 Modelica.Blocks.Sources.BooleanExpression drogue_deploy(y = drogue_descent_state.active or main_descent_state.active) annotation(
    Placement(visible = true, transformation(origin = {-74, -21}, extent = {{-24, -7}, {24, 7}}, rotation = 0)));
 Modelica.StateGraph.Step terminal_ascent_state(nIn = 1, nOut = 1)  annotation(
    Placement(visible = true, transformation(origin = {90, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
equation
  connect(liftoff_state.outPort[1], guidance_enable.inPort) annotation(
    Line(points = {{-69.5, 58}, {-54, 58}}));
  connect(guidance_enable.outPort, ascent_state.inPort) annotation(
    Line(points = {{-48.5, 58}, {-41, 58}}));
  connect(roll_control_state.inPort[1], ascent_state.split[1]) annotation(
    Line(points = {{-11, 30}, {-14.5, 30}, {-14.5, 58}, {-30, 58}}));
  connect(v_control_state.inPort[1], ascent_state.split[2]) annotation(
    Line(points = {{-20.7, 83}, {-30, 83}, {-30, 58}}));
  connect(roll_control_state.outPort[1], ascent_state.join[1]) annotation(
    Line(points = {{10.5, 30}, {39.25, 30}, {39.25, 58}, {42, 58}}));
  connect(v_control_state.outPort[1], stable_flight.inPort) annotation(
    Line(points = {{-5.65, 83}, {1.35, 83}, {1.35, 89}, {4.35, 89}}));
  connect(stable_flight.outPort, apogee_control_state.inPort[1]) annotation(
    Line(points = {{8.05, 89}, {19.55, 89}, {19.55, 83}, {17.05, 83}}));
  connect(apogee_control_state.outPort[1], ascent_state.join[2]) annotation(
    Line(points = {{32.35, 83}, {42, 83}, {42, 58}}));
  connect(apogee.outPort, drogue_descent_state.inPort[1]) annotation(
    Line(points = {{72.5, -80}, {51, -80}}));
  connect(drogue_descent_state.outPort[1], main_altitude.inPort) annotation(
    Line(points = {{29.5, -80}, {4, -80}}));
  connect(main_descent_state.inPort[1], main_altitude.outPort) annotation(
    Line(points = {{-29, -80}, {-2, -80}}));
  connect(landed.inPort, main_descent_state.outPort[1]) annotation(
    Line(points = {{-76, -80}, {-50, -80}}));
  connect(ascent_state.outPort, terminal_ascent.inPort) annotation(
    Line(points = {{53, 58}, {76, 58}}));
  connect(termina_ascent_expr.y, terminal_ascent.condition) annotation(
    Line(points = {{82, 23}, {82, 33.5}, {80, 33.5}, {80, 46}}, color = {255, 0, 255}));
  connect(stable_flight.condition, v_control_enable.y) annotation(
    Line(points = {{7, 80.6}, {5, 80.6}, {5, 64}, {1, 64}}, color = {255, 0, 255}));
  connect(main_alt_expr.y, main_altitude.condition) annotation(
    Line(points = {{-25, -61}, {0, -61}, {0, -68}}, color = {255, 0, 255}));
  connect(apogee_expr.y, apogee.condition) annotation(
    Line(points = {{56, -53}, {56, -52.5}, {74, -52.5}, {74, -68}}, color = {255, 0, 255}));
  connect(landed.outPort, liftoff_state.inPort[1]) annotation(
    Line(points = {{-82, -80}, {-100, -80}, {-100, 58}, {-90, 58}}));
  connect(roll_control_state.active, bus.roll_guidance) annotation(
    Line(points = {{0, 20}, {0, -30}}, color = {255, 0, 255}));
  connect(bus.liftoff, met.u) annotation(
    Line(points = {{0, -30}, {-26, -30}, {-26, -6}, {-18, -6}}, color = {255, 0, 255}));
  connect(bus.liftoff, guidance_enable.condition) annotation(
    Line(points = {{0, -30}, {-36, -30}, {-36, 36}, {-50, 36}, {-50, 46}}, color = {255, 0, 255}));
  connect(drogue_deploy.y, bus.drogue_deploy) annotation(
    Line(points = {{-48, -20}, {-36, -20}, {-36, -30}, {0, -30}}, color = {255, 0, 255}));
  connect(main_deploy.y, bus.main_deploy) annotation(
    Line(points = {{-48, 4}, {-36, 4}, {-36, -30}, {0, -30}}, color = {255, 0, 255}));
  connect(v_control_state.active, bus.velocity_guidace) annotation(
    Line(points = {{-12, 76}, {-12, 72}, {26, 72}, {26, -30}, {0, -30}}, color = {255, 0, 255}));
  connect(apogee_control_state.active, bus.apogee_guidance) annotation(
    Line(points = {{26, 76}, {26, -30}, {0, -30}}, color = {255, 0, 255}));
 connect(terminal_ascent.outPort, terminal_ascent_state.inPort[1]) annotation(
    Line(points = {{82, 58}, {90, 58}, {90, 2}}));
 connect(terminal_ascent_state.outPort[1], apogee.inPort) annotation(
    Line(points = {{90, -20}, {90, -80}, {78, -80}}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end FlightModeManager;
