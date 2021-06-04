within RocketControl;

package Components
  package Sensors
    model AeroAnglesSensor
      extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
      import Modelica.Mechanics.MultiBody.Frames;
      import Modelica.Math.Vectors;
      Modelica.Blocks.Interfaces.RealOutput aoa annotation(
        Placement(visible = true, transformation(origin = {102, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.Velocity v_b[3] "Velocity of frame_a expressed in frame_a (body frame)";
      SI.Velocity v_norm;
      parameter SI.Velocity v_small = 1e-7 "Prevent division by zero when velocity is too small";
      Modelica.Blocks.Interfaces.RealOutput sideslip annotation(
        Placement(visible = true, transformation(origin = {100, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      v_b = Frames.resolve2(frame_a.R, der(frame_a.r_0));
      v_norm = Vectors.norm(v_b);
  if abs(v_b[1]) > v_small then
//aoa = atan(sqrt(v_b[3]^2 + v_b[2]^2)/v_b[1]);
        aoa = atan(v_b[3] / v_b[1]);
//aoa = atan2(v_b[3], v_b[1]);
      elseif abs(v_b[3]) > v_small then
        aoa = Modelica.Units.Conversions.from_deg(90 * sign(v_b[3]));
      else
        aoa = 0;
      end if;
  if abs(v_norm) > v_small then
        sideslip = atan(v_b[2] / v_b[1]);
//sideslip = asin(v_b[2] / v_norm);
      elseif abs(v_b[2]) > v_small then
        sideslip = Modelica.Units.Conversions.from_deg(90 * sign(v_b[2]));
      else
        sideslip = 0;
      end if;
      frame_a.f = zeros(3);
      frame_a.t = zeros(3);
      annotation(
        Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name"), Text(origin = {-10, 51}, extent = {{-90, 19}, {90, -19}}, textString = "AoA", horizontalAlignment = TextAlignment.Right), Text(origin = {-8, -49}, extent = {{-90, 19}, {90, -19}}, textString = "beta", horizontalAlignment = TextAlignment.Right)}));
    end AeroAnglesSensor;

    model AeroStateSensor
      extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
      import Modelica.Mechanics.MultiBody.Frames;
      import Modelica.Math.Vectors;
      RocketControl.Components.Sensors.AeroAnglesSensor aeroAnglesSensor annotation(
        Placement(visible = true, transformation(origin = {0, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      outer World.Atmosphere atmosphere;
      outer World.MyWorld world;
      Modelica.Units.SI.Angle betaprime;
      RocketControl.Interfaces.AeroStateOutput aeroStateOutput annotation(
        Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
        Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      aeroStateOutput.alpha = aeroAnglesSensor.aoa;
      aeroStateOutput.beta = aeroAnglesSensor.sideslip;
      aeroStateOutput.mach = Vectors.norm(der(frame_a.r_0)) / atmosphere.speedOfSound(frame_a.r_0);
      aeroStateOutput.altitude = world.altitude(frame_a.r_0);
      aeroStateOutput.v = absoluteVelocity.v;
      aeroStateOutput.w = absoluteAngularVelocity.w;
      betaprime = atan2(absoluteVelocity.v[2], absoluteVelocity.v[1]);
      connect(aeroAnglesSensor.frame_a, frame_a) annotation(
        Line(points = {{-10, 34}, {-55, 34}, {-55, 0}, {-100, 0}}));
      connect(absoluteVelocity.frame_a, frame_a) annotation(
        Line(points = {{-10, 0}, {-100, 0}}));
      connect(frame_a, absoluteAngularVelocity.frame_a) annotation(
        Line(points = {{-100, 0}, {-54, 0}, {-54, -40}, {-10, -40}}));
      annotation(
        Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name"), Text(origin = {1, -40}, lineColor = {64, 64, 64}, extent = {{-101, 20}, {101, -20}}, textString = "aero"), Line(origin = {84.1708, -2.17082}, points = {{-14.1708, 2.17082}, {11.8292, 2.17082}, {13.8292, -1.82918}})}));
    end AeroStateSensor;

    model DirectionSensor
      extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
      import Modelica.Mechanics.MultiBody.Frames;
      //Real direction[3];
      Modelica.Blocks.Interfaces.RealOutput dir[3] annotation(
        Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      frame_a.f = zeros(3);
      frame_a.t = zeros(3);
      dir = Frames.resolve1(frame_a.R, {1, 0, 0});
      annotation(
        Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name"), Line(origin = {82, 0}, points = {{12, 0}, {-12, 0}}), Text(origin = {4, -142}, extent = {{-132, 76}, {129, 124}}, textString = "dir")}));
    end DirectionSensor;
  end Sensors;

  model BodyVariableMass "Rigid body with mass, inertia tensor and one frame connector (12 potential states)"
    import Modelica.Mechanics.MultiBody.Visualizers;
    import Modelica.Mechanics.MultiBody.Types;
    import Modelica.Mechanics.MultiBody.Frames;
    import Modelica.Units.Conversions.to_unit1;
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a "Coordinate system fixed at body" annotation(
      Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
    parameter Boolean animation = true "= true, if animation shall be enabled (show cylinder and sphere)";
    parameter SI.Position r_CM[3](start = {0, 0, 0}) "Vector from frame_a to center of mass, resolved in frame_a";
    SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_a" annotation(
      Dialog(tab = "Initialization", showStartAttribute = true));
    SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_a, resolved in world frame (= der(r_0))" annotation(
      Dialog(tab = "Initialization", showStartAttribute = true));
    SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_a resolved in world frame (= der(v_0))" annotation(
      Dialog(tab = "Initialization", showStartAttribute = true));
    parameter Boolean angles_fixed = false "= true, if angles_start are used as initial values, else as guess values" annotation(
      Evaluate = true,
      choices(checkBox = true),
      Dialog(tab = "Initialization"));
    parameter SI.Angle angles_start[3] = {0, 0, 0} "Initial values of angles to rotate world frame around 'sequence_start' axes into frame_a" annotation(
      Dialog(tab = "Initialization"));
    parameter Types.RotationSequence sequence_start = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a at initial time" annotation(
      Evaluate = true,
      Dialog(tab = "Initialization"));
    parameter Boolean w_0_fixed = false "= true, if w_0_start are used as initial values, else as guess values" annotation(
      Evaluate = true,
      choices(checkBox = true),
      Dialog(tab = "Initialization"));
    parameter SI.AngularVelocity w_0_start[3] = {0, 0, 0} "Initial or guess values of angular velocity of frame_a resolved in world frame" annotation(
      Dialog(tab = "Initialization"));
    parameter Boolean z_0_fixed = false "= true, if z_0_start are used as initial values, else as guess values" annotation(
      Evaluate = true,
      choices(checkBox = true),
      Dialog(tab = "Initialization"));
    parameter SI.AngularAcceleration z_0_start[3] = {0, 0, 0} "Initial values of angular acceleration z_0 = der(w_0)" annotation(
      Dialog(tab = "Initialization"));
    parameter SI.Diameter sphereDiameter = world.defaultBodyDiameter "Diameter of sphere" annotation(
      Dialog(tab = "Animation", group = "if animation = true", enable = animation));
    input Types.Color sphereColor = Modelica.Mechanics.MultiBody.Types.Defaults.BodyColor "Color of sphere" annotation(
      Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
    parameter SI.Diameter cylinderDiameter = sphereDiameter / Types.Defaults.BodyCylinderDiameterFraction "Diameter of cylinder" annotation(
      Dialog(tab = "Animation", group = "if animation = true", enable = animation));
    input Types.Color cylinderColor = sphereColor "Color of cylinder" annotation(
      Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
    input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(
      Dialog(tab = "Animation", group = "if animation = true", enable = animation));
    parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(
      Evaluate = true,
      Dialog(tab = "Advanced"));
    parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(
      Evaluate = true,
      Dialog(tab = "Advanced"));
    parameter Types.RotationSequence sequence_angleStates = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a around the 3 angles used as potential states" annotation(
      Evaluate = true,
      Dialog(tab = "Advanced", enable = not useQuaternions));
    SI.Mass m "Body Mass";
    SI.Inertia I[3, 3] "Inertia tensor";
    final parameter Frames.Orientation R_start = Modelica.Mechanics.MultiBody.Frames.axesRotations(sequence_start, angles_start, zeros(3)) "Orientation object from world frame to frame_a at initial time";
    SI.AngularVelocity w_a[3](start = Frames.resolve2(R_start, w_0_start), fixed = fill(w_0_fixed, 3), each stateSelect = if enforceStates then if useQuaternions then StateSelect.always else StateSelect.never else StateSelect.avoid) "Absolute angular velocity of frame_a resolved in frame_a";
    SI.AngularAcceleration z_a[3](start = Frames.resolve2(R_start, z_0_start), fixed = fill(z_0_fixed, 3)) "Absolute angular acceleration of frame_a resolved in frame_a";
    SI.Acceleration g_0[3] "Gravity acceleration resolved in world frame";
    RocketControl.Interfaces.MassPropertiesInput massInput annotation(
      Placement(visible = true, transformation(origin = {0, -96}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {40, -62}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  protected
    outer Modelica.Mechanics.MultiBody.World world;
    // Declarations for quaternions (dummies, if quaternions are not used)
    parameter Frames.Quaternions.Orientation Q_start = Frames.to_Q(R_start) "Quaternion orientation object from world frame to frame_a at initial time";
    Frames.Quaternions.Orientation Q(start = Q_start, each stateSelect = if enforceStates then if useQuaternions then StateSelect.prefer else StateSelect.never else StateSelect.avoid) "Quaternion orientation object from world frame to frame_a (dummy value, if quaternions are not used as states)";
    // Declaration for 3 angles
    parameter SI.Angle phi_start[3] = if sequence_start[1] == sequence_angleStates[1] and sequence_start[2] == sequence_angleStates[2] and sequence_start[3] == sequence_angleStates[3] then angles_start else Frames.axesRotationsAngles(R_start, sequence_angleStates) "Potential angle states at initial time";
    SI.Angle phi[3](start = phi_start, each stateSelect = if enforceStates then if useQuaternions then StateSelect.never else StateSelect.always else StateSelect.avoid) "Dummy or 3 angles to rotate world frame into frame_a of body";
    SI.AngularVelocity phi_d[3](each stateSelect = if enforceStates then if useQuaternions then StateSelect.never else StateSelect.always else StateSelect.avoid) "= der(phi)";
    SI.AngularAcceleration phi_dd[3] "= der(phi_d)";
    // Declarations for animation
    Visualizers.Advanced.Shape cylinder(shapeType = "cylinder", color = cylinderColor, specularCoefficient = specularCoefficient, length = if Modelica.Math.Vectors.length(r_CM) > sphereDiameter / 2 then Modelica.Math.Vectors.length(r_CM) - (if cylinderDiameter > 1.1 * sphereDiameter then sphereDiameter / 2 else 0) else 0, width = cylinderDiameter, height = cylinderDiameter, lengthDirection = to_unit1(r_CM), widthDirection = {0, 1, 0}, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
    Visualizers.Advanced.Shape sphere(shapeType = "sphere", color = sphereColor, specularCoefficient = specularCoefficient, length = sphereDiameter, width = sphereDiameter, height = sphereDiameter, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = r_CM - {1, 0, 0} * sphereDiameter / 2, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation and sphereDiameter > 0;
  initial equation
    if angles_fixed then
// Initialize positional variables
      if not Connections.isRoot(frame_a.R) then
// frame_a.R is computed somewhere else
        zeros(3) = Frames.Orientation.equalityConstraint(frame_a.R, R_start);
      elseif useQuaternions then
// frame_a.R is computed from quaternions Q
        zeros(3) = Frames.Quaternions.Orientation.equalityConstraint(Q, Q_start);
      else
// frame_a.R is computed from the 3 angles 'phi'
        phi = phi_start;
      end if;
    end if;
  equation
    if enforceStates then
      Connections.root(frame_a.R);
    else
      Connections.potentialRoot(frame_a.R);
    end if;
    r_0 = frame_a.r_0;
  if not Connections.isRoot(frame_a.R) then
// Body does not have states
// Dummies
      Q = {0, 0, 0, 1};
      phi = zeros(3);
      phi_d = zeros(3);
      phi_dd = zeros(3);
    elseif useQuaternions then
// Use Quaternions as states (with dynamic state selection)
      frame_a.R = Frames.from_Q(Q, Frames.Quaternions.angularVelocity2(Q, der(Q)));
      {0} = Frames.Quaternions.orientationConstraint(Q);
// Dummies
      phi = zeros(3);
      phi_d = zeros(3);
      phi_dd = zeros(3);
    else
// Use Cardan angles as states
      phi_d = der(phi);
      phi_dd = der(phi_d);
      frame_a.R = Frames.axesRotations(sequence_angleStates, phi, phi_d);
// Dummies
      Q = {0, 0, 0, 1};
    end if;
// gravity acceleration at center of mass resolved in world frame
    g_0 = world.gravityAcceleration(frame_a.r_0 + Frames.resolve1(frame_a.R, r_CM));
    m = massInput.m;
    I = massInput.I;
// translational kinematic differential equations
    v_0 = der(frame_a.r_0);
    a_0 = der(v_0);
// rotational kinematic differential equations
    w_a = Frames.angularVelocity2(frame_a.R);
    z_a = der(w_a);
/* Newton/Euler equations with respect to center of mass
              a_CM = a_a + cross(z_a, r_CM) + cross(w_a, cross(w_a, r_CM));
              f_CM = m*(a_CM - g_a);
              t_CM = I*z_a + cross(w_a, I*w_a);
         frame_a.f = f_CM
         frame_a.t = t_CM + cross(r_CM, f_CM);
      Inserting the first three equations in the last two results in:
    */
    frame_a.f = m * (Frames.resolve2(frame_a.R, a_0 - g_0) + cross(z_a, r_CM) + cross(w_a, cross(w_a, r_CM)));
    frame_a.t = I * z_a + cross(w_a, I * w_a) + cross(r_CM, frame_a.f);
    annotation(
      Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 24, 48}, fillColor = {0, 127, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, 30}, {-3, -30}}, radius = 10), Text(lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Ellipse(lineColor = {0, 24, 48}, fillColor = {0, 127, 255}, fillPattern = FillPattern.Sphere, extent = {{-20, 60}, {100, -60}}, endAngle = 360)}),
      Documentation(info = "<html>
  <p>
  <strong>Rigid body</strong> with mass and inertia tensor.
  All parameter vectors have to be resolved in frame_a.
  The <strong>inertia tensor</strong> has to be defined with respect to a
  coordinate system that is parallel to frame_a with the
  origin at the center of mass of the body.
  </p>
  <p>
  By default, this component is visualized by a <strong>cylinder</strong> located
  between frame_a and the center of mass and by a <strong>sphere</strong> that has
  its center at the center of mass. If the cylinder length is smaller as
  the radius of the sphere, e.g., since frame_a is located at the
  center of mass, the cylinder is not displayed. Note, that
  the animation may be switched off via parameter animation = <strong>false</strong>.
  </p>
  <p>
  <img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Parts/Body.png\" alt=\"Parts.Body\">
  </p>
  
  <p>
  <strong>States of Body Components</strong>
  </p>
  <p>
  Every body has potential states. If possible a tool will select
  the states of joints and not the states of bodies because this is
  usually the most efficient choice. In this case the position, orientation,
  velocity and angular velocity of frame_a of the body will be computed
  by the component that is connected to frame_a. However, if a body is moving
  freely in space, variables of the body have to be used as states. The potential
  states of the body are:
  </p>
  <ul>
  <li> The <strong>position vector</strong> frame_a.r_0 from the origin of the
       world frame to the origin of frame_a of the body, resolved in
       the world frame and the <strong>absolute velocity</strong> v_0 of the origin of
       frame_a, resolved in the world frame (= der(frame_a.r_0)).
  </li>
  <li> If parameter <strong>useQuaternions</strong> in the \"Advanced\" menu
       is <strong>true</strong> (this is the default), then <strong>4 quaternions</strong>
       are potential states. Additionally, the coordinates of the
       absolute angular velocity vector of the
       body are 3 potential states.<br>
       If <strong>useQuaternions</strong> in the \"Advanced\" menu
       is <strong>false</strong>, then <strong>3 angles</strong> and the derivatives of
       these angles are potential states. The orientation of frame_a
       is computed by rotating the world frame along the axes defined
       in parameter vector \"sequence_angleStates\" (default = {1,2,3}, i.e.,
       the Cardan angle sequence) around the angles used as potential states.
       For example, the default is to rotate the x-axis of the world frame
       around angles[1], the new y-axis around angles[2] and the new z-axis
       around angles[3], arriving at frame_a.
   </li>
  </ul>
  <p>
  The quaternions have the slight disadvantage that there is a
  non-linear constraint equation between the 4 quaternions.
  Therefore, at least one non-linear equation has to be solved
  during simulation. A tool might, however, analytically solve this
  simple constraint equation. Using the 3 angles as states has the
  disadvantage that there is a singular configuration in which a
  division by zero will occur. If it is possible to determine in advance
  for an application class that this singular configuration is outside
  of the operating region, the 3 angles might be used as potential
  states by setting <strong>useQuaternions</strong> = <strong>false</strong>.
  </p>
  <p>
  In text books about 3-dimensional mechanics often 3 angles and the
  angular velocity are used as states. This is not the case here, since
  3 angles and their derivatives are used as potential states
  (if useQuaternions = false). The reason
  is that for real-time simulation the discretization formula of the
  integrator might be \"inlined\" and solved together with the body equations.
  By appropriate symbolic transformation the performance is
  drastically increased if angles and their
  derivatives are used as states, instead of angles and the angular
  velocity.
  </p>
  <p>
  Whether or not variables of the body are used as states is usually
  automatically selected by the Modelica translator. If parameter
  <strong>enforceStates</strong> is set to <strong>true</strong> in the \"Advanced\" menu,
  then body variables are forced to be used as states according
  to the setting of parameters \"useQuaternions\" and
  \"sequence_angleStates\".
  </p>
  </html>"));
  end BodyVariableMass;

  package Motors
    model M2000R
      Modelica.Blocks.Sources.TimeTable thrustCurve(offset = 0, shiftTime = 0, startTime = 0, table = [0, 0; 0.1, 1500; 0.2, 2250; 2.1, 2400; 4, 1750; 4.6, 100; 4.7, 0; 500, 0], timeScale = 1) annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Motors.Internal.SolidMotorAndTank m2000r(Dext = 0.098, Din_0 = 0.03, Isp = 176, h = 0.732, m_0 = 5.368)  annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
        Placement(visible = true, transformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
    equation
      connect(thrustCurve.y, m2000r.thrust) annotation(
        Line(points = {{-59, 0}, {-10, 0}}, color = {0, 0, 127}));
  connect(m2000r.frame_b, frame_b) annotation(
        Line(points = {{0, 10}, {0, 98}}));
      annotation(
        Icon(graphics = {Text(origin = {2, -184}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Rectangle(origin = {0.51, 47.73}, fillColor = {85, 85, 85}, fillPattern = FillPattern.VerticalCylinder, extent = {{-59.94, 48.27}, {59.94, -48.27}}), Polygon(origin = {1, -30}, fillColor = {43, 88, 85}, fillPattern = FillPattern.VerticalCylinder, points = {{-21, 30}, {-51, 10}, {-71, -30}, {71, -30}, {49, 10}, {19, 30}, {5, 30}, {-21, 30}}), Text(origin = {1, 38}, lineColor = {255, 255, 255}, extent = {{-59, 38}, {59, -38}}, textString = "M2000R")}));
    end M2000R;

    package Internal
      model GenericMotor
        parameter SI.Time Isp = 180 "Specific impulse in seconds";
        Modelica.Blocks.Sources.Constant const(k = 0) annotation(
          Placement(visible = true, transformation(origin = {-34, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput thrust annotation(
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-94, -2}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Math.Gain gain(k = 1 / (Modelica.Constants.g_n * Isp)) annotation(
          Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Forces.WorldForce force(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
          Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
          Placement(visible = true, transformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 94}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
        Modelica.Blocks.Interfaces.RealOutput mdot annotation(
          Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(thrust, gain.u) annotation(
          Line(points = {{-100, 0}, {24, 0}}, color = {0, 0, 127}));
        connect(const.y, force.force[3]) annotation(
          Line(points = {{-23, -54}, {0, -54}, {0, 38}}, color = {0, 0, 127}));
        connect(const.y, force.force[2]) annotation(
          Line(points = {{-23, -54}, {0, -54}, {0, 38}}, color = {0, 0, 127}));
        connect(thrust, force.force[1]) annotation(
          Line(points = {{-100, 0}, {0, 0}, {0, 38}}, color = {0, 0, 127}));
        connect(force.frame_b, frame_b) annotation(
          Line(points = {{0, 60}, {0, 100}}, color = {95, 95, 95}));
        connect(gain.y, mdot) annotation(
          Line(points = {{48, 0}, {106, 0}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Polygon(origin = {0, 12}, fillColor = {107, 107, 107}, fillPattern = FillPattern.VerticalCylinder, points = {{-6, 78}, {-60, 62}, {-90, 2}, {-100, -78}, {100, -78}, {90, 2}, {60, 62}, {6, 78}, {-6, 78}}), Text(origin = {-10, -172},lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name")}));
      end GenericMotor;

      package Inertia
        partial model PropellantInertiaBase
          parameter Modelica.Units.SI.Mass m_0 "Initial mass";
          parameter Modelica.Units.SI.Mass m_small = 1e-3 "Smallest mass before setting mdot to 0";
          Interfaces.MassPropertiesOutput massOut annotation(
            Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput mdot annotation(
            Placement(visible = true, transformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-88, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        initial equation
          massOut.m = m_0;
        equation
          if massOut.m > m_small then
            der(massOut.m) = -mdot;
          else
            der(massOut.m) = 0;
          end if;
          annotation(
            Diagram,
            Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name")}));
        end PropellantInertiaBase;

        model SolidPropellantInertia
          extends RocketControl.Components.Motors.Internal.Inertia.PropellantInertiaBase;
          import Modelica.Constants.pi;
          
          parameter Modelica.Units.SI.Length Dext "External diameter";
          parameter Modelica.Units.SI.Length Din_0 "Initial internal diameter";
          parameter Modelica.Units.SI.Length h "Height";
          final parameter SI.Volume V_0 = pi * ((Dext / 2) ^ 2 - (Din_0 / 2) ^ 2) * h;
          final parameter SI.Density rho = m_0 / V_0;
          
          Modelica.Units.SI.Length Din "Internal diameter";
          SI.Volume V "Propellant volume";
          
        equation
          V = massOut.m / rho;
          Din = sqrt(Dext^2 - 4 * V / pi);
          
          massOut.I[1, 1] = 1 / 2 * massOut.m * ((Dext / 2) ^ 2 + (Din / 2) ^ 2);
          massOut.I[2, 2] = 1 / 12 * massOut.m * (3*((Dext / 2) ^ 2 + (Din / 2) ^ 2) + h ^ 2);
          massOut.I[3, 3] = 1 / 12 * massOut.m * (3*((Dext / 2) ^ 2 + (Din / 2) ^ 2) + h ^ 2);
          massOut.I[1, 2] = 0;
          massOut.I[1, 3] = 0;
          massOut.I[2, 1] = 0;
          massOut.I[2, 3] = 0;
          massOut.I[3, 1] = 0;
          massOut.I[3, 2] = 0;
        annotation(
            Icon(graphics = {Rectangle(origin = {0, -13}, fillColor = {113, 88, 25}, fillPattern = FillPattern.VerticalCylinder, extent = {{-60, 85}, {60, -85}})}));end SolidPropellantInertia;
      end Inertia;

      model SolidMotorAndTank
        parameter SI.Time Isp = 180 "Specific impulse in seconds";
        parameter Modelica.Units.SI.Length Dext "External diameter";
        parameter Modelica.Units.SI.Length Din_0 "Initial internal diameter";
        parameter Modelica.Units.SI.Length h "Height";
        parameter Modelica.Units.SI.Mass m_0 "Initial mass";
        
  RocketControl.Components.Motors.Internal.Inertia.SolidPropellantInertia solidPropellantInertia(Dext = Dext, Din_0 = Din_0, h = h, m_0 = m_0)  annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.BodyVariableMass bodyVariableMass(a_0(each fixed = false),enforceStates = false, r_0(each fixed = false), r_CM = {h / 2, 0, 0}, v_0(each fixed = false), w_a(each fixed = false), z_a(each fixed = false))  annotation(
          Placement(visible = true, transformation(origin = {40, 4}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
          Placement(visible = true, transformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput thrust annotation(
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Components.Motors.Internal.GenericMotor genericMotor(Isp = Isp)  annotation(
          Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(solidPropellantInertia.massOut, bodyVariableMass.massInput) annotation(
          Line(points = {{10, 0}, {34, 0}}));
        connect(bodyVariableMass.frame_a, frame_b) annotation(
          Line(points = {{40, 14}, {40, 60}, {0, 60}, {0, 98}}));
  connect(solidPropellantInertia.mdot, genericMotor.mdot) annotation(
          Line(points = {{-8, 0}, {-30, 0}}, color = {0, 0, 127}));
  connect(genericMotor.thrust, thrust) annotation(
          Line(points = {{-50, 0}, {-100, 0}}, color = {0, 0, 127}));
  connect(genericMotor.frame_b, frame_b) annotation(
          Line(points = {{-40, 10}, {-40, 60}, {0, 60}, {0, 98}}, color = {95, 95, 95}));
      annotation(
          Icon(graphics = {Text(origin = {1, 38}, lineColor = {255, 255, 255}, extent = {{-59, 38}, {59, -38}}, textString = "M2000R"), Text(origin = {2, -176}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Polygon(origin = {1, -30}, fillColor = {43, 88, 85}, fillPattern = FillPattern.VerticalCylinder, points = {{-21, 30}, {-51, 10}, {-71, -30}, {71, -30}, {49, 10}, {19, 30}, {5, 30}, {-21, 30}}), Rectangle(origin = {0.51, 47.73}, fillColor = {85, 85, 85}, fillPattern = FillPattern.VerticalCylinder, extent = {{-59.94, 48.27}, {59.94, -48.27}})}));end SolidMotorAndTank;
    end Internal;
  end Motors;
end Components;
