within RocketControl.GNC.Guidance;

model AngularRateVelocityTrack
  extends Icon;
  parameter Real k;
  Modelica.Blocks.Interfaces.RealOutput w_ref[3](each final unit = "rad/s", each final quantity = "AngularVelocity", each displayUnit = "deg/s") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput v_ref[3](each final unit = "m/s", each final quantity = "Velocity") annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Components.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  SI.Angle angle_err;
  Real rotation_vector[3];
  Real rotation_versor[3];
  SI.AngularVelocity[3] w_body;
equation
  if norm(bus.v_est) > 1 and norm(v_ref) > 1 then
    angle_err = acos(bus.v_est * v_ref / (norm(bus.v_est) * norm(v_ref)));
    rotation_vector = cross(bus.v_est, v_ref);
    if norm(rotation_vector) > 1e-6 then
      rotation_versor = rotation_vector / norm(rotation_vector);
    else
      rotation_versor = {0, 0, 0};
// Singularity: no rotation. TODO: Handle case where velocities are opposing
    end if;
  else
    angle_err = 0;
    rotation_vector = {0, 0, 0};
    rotation_versor = {0, 0, 0};
  end if;
  w_body = k * Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve2(bus.q_est, rotation_versor) * angle_err;
  w_ref = cat(1, {0}, w_body[2:3]);
  annotation(
    Icon(graphics = {Text(origin = {1, 6}, extent = {{-79, 52}, {79, -52}}, textString = "w_ref")}));
end AngularRateVelocityTrack;
