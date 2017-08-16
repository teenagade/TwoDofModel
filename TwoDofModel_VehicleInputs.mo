model TwoDofModel_VehicleInputs
  // Total Vehicle Ixx = 120, Iyy = 1060,
  // Sprung Mass M = 602, Ixx 47, Iyy = 660, Izz = 670
  
  TwoDofModel TwoDofModel1(massVeh = 725, Izz = 1150, cf = 10, cr = 10, wheelBase = 3.69, cgLocation = 0.46, vx0 = 5, vy0 = 0, wheelRadius = 0.326) annotation(
    Placement(visible = true, transformation(origin = {-2, 30}, extent = {{-40, -40}, {40, 40}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp steerInput(duration = 2, height = 10, offset = 0, startTime = 0.5)  annotation(
    Placement(visible = true, transformation(origin = {-90, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp driveTorque(duration = 2, height = 0, offset = 0, startTime = 0.5) annotation(
    Placement(visible = true, transformation(origin = {-90, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp brakeTorque(duration = 2, height = 0, offset = 0, startTime = 0.5) annotation(
    Placement(visible = true, transformation(origin = {-90, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(TwoDofModel1.driveTorque, driveTorque.y);
  connect(TwoDofModel1.brakeTorque, brakeTorque.y);
  connect(TwoDofModel1.delta, steerInput.y) annotation(
      Line(points = {{-42, 54}, {-79, 54}}, color = {0, 0, 127}));

  annotation(
    uses(Modelica(version = "3.2.2")));
  
end TwoDofModel_VehicleInputs;
