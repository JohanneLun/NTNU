within ;
model test
  import SI = Modelica.SIunits;
  SI.Current i(start = 1);
  SI.Voltage u;
  parameter SI.Resistance R=2;
  parameter SI.Inductance L=1;
  parameter Real Kp = 1;
  parameter SI.Current iref = 2;

equation
  u = R*iref - Kp*(i-iref);

  L*der(i) + R*i = u;

  annotation (uses(Modelica(version="3.2.1")));
end test;
