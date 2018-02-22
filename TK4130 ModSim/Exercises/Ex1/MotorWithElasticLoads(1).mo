within ;
model MotorWithElasticLoads
  Modelica.Mechanics.Rotational.Sources.Torque torque 
    annotation (Placement(transformation(extent={{-66,34},{-46,54}})));
  Modelica.Mechanics.Rotational.Components.SpringDamper springDamper1(c=0.5, d=
        0.01) annotation (Placement(transformation(extent={{-8,34},{12,54}})));
  Modelica.Mechanics.Rotational.Components.Inertia motorInertia(J=1) 
    annotation (Placement(transformation(extent={{-36,34},{-16,54}})));
  Modelica.Mechanics.Rotational.Components.SpringDamper springDamper2(c=0.5, d=
        0.01) annotation (Placement(transformation(extent={{48,34},{68,54}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=1) 
    annotation (Placement(transformation(extent={{20,34},{40,54}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=1) 
    annotation (Placement(transformation(extent={{76,34},{96,54}})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
    annotation (Placement(transformation(extent={{106,34},{126,54}})));
  Modelica.Blocks.Interfaces.RealInput u
    annotation (Placement(transformation(extent={{-124,24},{-84,64}})));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{138,34},{158,54}})));
equation
  connect(torque.flange, motorInertia.flange_a) annotation (Line(
      points={{-46,44},{-36,44}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(motorInertia.flange_b, springDamper1.flange_a) annotation (Line(
      points={{-16,44},{-8,44}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(springDamper1.flange_b, inertia1.flange_a) annotation (Line(
      points={{12,44},{20,44}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(inertia1.flange_b, springDamper2.flange_a) annotation (Line(
      points={{40,44},{48,44}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(springDamper2.flange_b, inertia2.flange_a) annotation (Line(
      points={{68,44},{76,44}},
      color={0,0,0},
      smooth=Smooth.None));
  annotation (uses(Modelica(version="3.1")), Diagram(graphics));
  connect(inertia2.flange_b, speedSensor.flange) annotation (Line(
      points={{96,44},{106,44}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(torque.tau, u) annotation (Line(
      points={{-68,44},{-104,44}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(speedSensor.w, y) annotation (Line(
      points={{127,44},{148,44}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(y, y) annotation (Line(
      points={{148,44},{148,44}},
      color={0,0,127},
      smooth=Smooth.None));
end MotorWithElasticLoads;
