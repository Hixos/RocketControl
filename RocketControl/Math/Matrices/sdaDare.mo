within RocketControl.Math.Matrices;

function sdaDare "Return solution of discrete-time algebraic Riccati equation A'*X*A - X - A'*X*B*inv(R + B'*X*B)*B'*X*A + Q = 0 (dare) using a structured doubling algorithm"
  extends Modelica.Icons.Function;
  import Modelica.Math.Matrices;
  import Modelica.Math.Matrices.inv;
  input Real A[:, size(A, 1)] "Square matrix A in DARE";
  input Real B[size(A, 1), :] "Matrix B in DARE";
  input Real R[size(B, 2), size(B, 2)] = identity(size(B, 2)) "Matrix R in DARE";
  input Real Q[size(A, 1), size(A, 1)] = identity(size(A, 1)) "Matrix Q in DARE";
  input Real s = 1 "A real positive number such that R+s B'B is nonsingular";
  input Real tol = 1e-13 "Tolerance";
  input Integer kmax(min = 1) = 30 "Maximum number of iterations";
  output Real X[size(A, 1), size(A, 2)] "Stabilizing solution of DARE";
  output Integer ki "Number of iterations";
protected
  Integer n = size(A, 1);
  Integer k = 0;
  Real err = 1;
  Real W[size(A, 1), size(A, 1)];
  Real G[size(A, 1), size(A, 1)];
  Real E[size(A, 1), size(A, 1)];
  Real P[size(A, 1), size(A, 1)];
  Real Z[size(A, 1) * 2, size(A, 1)];
  Real E1[size(A, 1), size(A, 1)];
  Real P1[size(A, 1), size(A, 1)];
algorithm
// Initialization
  W := s * identity(n);
  G := -B * inv(R + transpose(B) * W * B) * transpose(B);
  E := (identity(n) + G * W) * A;
  P := Q - W + transpose(A) * W * E;
// SDA step
  while err > tol and k < kmax loop
    Z := [E; transpose(P)] * inv(identity(n) - G * P);
    E1 := Z[1:n, :];
    P1 := Z[n + 1:end, :];
    G := G + E1 * G * transpose(E);
    P := P + transpose(E) * P1 * E;
    E := E1 * E;
    err := Matrices.norm(E, 1);
    k := k + 1;
  end while;
  X := P;
  ki := k;
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end sdaDare;
