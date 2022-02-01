within RocketControl.Tests.Math;

model sdaDareTest
  parameter Real dt = 0.001;
  parameter Real At[4, 4] = [1, 0, 0.001, 0; 0, 1, 0, 0.001; -0.0654, 0.0654, 0.99673, 0.00327; 1.962, -62.784, 0.0981, 0.9019];
  parameter Real B[5, 1] = [0; 0; 1.66666666666667e-06; -5e-05; 0];
  parameter Real C[1, 4] = [1, 0, 0, 0];
  parameter Real A[5, 5] = [At, zeros(4, 1); -C * dt, 1];
  parameter Real Q[5, 5] = diagonal({0, 0, 0, 0, 1});
  parameter Real R[1, 1] = [0.001];
  parameter Real P[5, 5] = RocketControl.Math.sdaDare(A, B, R, Q);
equation

  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end sdaDareTest;
