model TwoDofModel
  import SI = Modelica.SIunits;
  
  parameter SI.Mass massVeh "Mass of the vehicle";
//  parameter SI.Inertia Ixx "Ixx Total of the vehicle";
//  parameter SI.Inertia Iyy "Iyy Total of the vehicle";
  parameter SI.Inertia Izz "Izz Total of the vehicle";
  parameter SI.Distance wheelBase "Wheelbase of vehicle";
  parameter Real cgLocation "Distance from front axle";
  parameter Real cf "Front Cornering Stiffness";
  parameter Real cr "Rear Cornering Stiffness";
  parameter SI.Distance wheelRadius "Wheel Radius Rear";
    
  SI.Distance a = wheelBase*cgLocation;
  SI.Distance b = wheelBase*(1-cgLocation);
  SI.Velocity vx, vy, vx0, vy0;
  Real alphaf, alphar, omegaz, beta;
  Real Ybeta, Yr, Ydelta;
  Real Nbeta, Nr, Ndelta; 
  Real pathX, pathY, curvature;
  SI.Distance pathDistance; 
  
  Modelica.Blocks.Interfaces.RealInput delta annotation(Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput driveTorque annotation(
    Placement(visible = true, transformation(origin = {-100, 2}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput brakeTorque annotation(
    Placement(visible = true, transformation(origin = {-100, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
initial equation
  vx = vx0;
  vy = vy0;

equation
  alphaf = (vy + omegaz * a) / vx - delta;
  alphar = (vy + omegaz * b) / vx;
// Lateral Force Balance
  cf * alphaf + cr * alphar = massVeh * (omegaz * vx + der(vy));
// Yaw moment Balance
  a * cf * alphaf - b * cr * alphar = Izz * der(omegaz);
// Longitudinal Forces
  massVeh * der(vx) = vy * omegaz + driveTorque/wheelRadius - brakeTorque/wheelRadius;
// Beta
  beta = atan(vy / vx);
// Vehicle Path
  der(pathX) = vx;
  der(pathY) = vy;
  pathDistance = (pathX^2+pathY^2)^0.5;
// New bit before i break it  - need to sort out inital condition...
  if abs(vy) < 1e-5 then
    curvature = 0;
  else
    curvature = vx^2/der(vy);
  end if;
    
// Control and Stability Derivatives
  Ybeta = cf + cr;
  Yr = 1 / vx * (a * cf - b * cr);
  Ydelta = -cf;
  Nbeta = a * cf - b * cr;
  Nr = 1 / vx * (a ^ 2 * cf + b ^ 2 * cr);
  Ndelta = -a * cf;
  annotation(
    uses(Modelica(version = "3.2.2")));

end TwoDofModel;
