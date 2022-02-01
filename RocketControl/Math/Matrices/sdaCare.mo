within RocketControl.Math.Matrices;

function sdaCare "Return solution X of the continuous-time algebraic Riccati equation A'*X + X*A - X*B*inv(R)*B'*X + Q = 0 (care) using a structured doubling algorithm"
  extends Modelica.Icons.Function;
  import Modelica.Math.Matrices;
  import Modelica.Math.Matrices.inv;
  input Real A[:, size(A, 1)] "Square matrix A in CARE";
  input Real B[size(A, 1), :] "Matrix B in CARE";
  input Real R[size(B, 2), size(B, 2)] = identity(size(B, 2)) "Matrix R in CARE";
  input Real Q[size(A, 1), size(A, 1)] = identity(size(A, 1)) "Matrix Q in CARE";
  input Real g = -1 "Scalar gamma used by the Cayley transform";
  input Real tol = 1e-13 "Tolerance";
  input Integer kmax(min = 1) = 50 "Maximum number of iterations";
  output Real X[size(A, 1), size(A, 2)] "Stabilizing solution of CARE";
  output Integer ki "Number of iterations";
protected
  Real S[size(A, 1), size(A, 1)];
  Integer n = size(A, 1);
  Integer k = 0;
  Real err = 1;
  Real Ai[size(A, 1), size(A, 1)];
  Real R1[size(A, 1), size(A, 1)];
  Real S1[size(A, 1), size(A, 1)];
  Real E[size(A, 1), size(A, 1)];
  Real G[size(A, 1), size(A, 1)];
  Real P[size(A, 1), size(A, 1)];
  Real IGP[size(A, 1), size(A, 1)];
  Real Z[size(A, 1) * 2, size(A, 1)];
  Real E1[size(A, 1), size(A, 1)];
  Real P1[size(A, 1), size(A, 1)];
algorithm
// Initialization
  S := B * inv(R) * transpose(B);
  Ai := inv(A + g * identity(n));
  R1 := S * transpose(Ai) * Q;
  S1 := inv(A + g * identity(n) + R1);
  E := S1 * (A - g * identity(n) + R1);
  R1 := identity(n) - Ai * (A - g * identity(n));
  G := S1 * S * transpose(R1);
  P := -transpose(S1) * Q * R1;
// SDA step
  while err > tol and k < kmax loop
    IGP := identity(n) - G * P;
    Z := [E; transpose(P)] * inv(IGP);
    E1 := Z[1:n, :];
    P1 := Z[n + 1:end, :];
    G := G + E1 * G * transpose(E);
    P := P + transpose(E) * transpose(P1) * E;
    E := E1 * E;
    err := Matrices.norm(E, 1);
    k := k + 1;
  end while;
  X := P;
  ki := k;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end sdaCare;
