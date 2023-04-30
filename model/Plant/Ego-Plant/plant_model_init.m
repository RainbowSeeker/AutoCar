%% Constant Variable (for internal use)
PLANT_CONST.dt = 0.002;

%% init plant model parameter
plant_model.mass = 0.886;     % kg
plant_model.Jxx = 0.016;	% moment of inertia in body x axis
plant_model.Jyy = 0.016;	% moment of inertia in body y axis
plant_model.Jzz = 0.0274;	% moment of inertia in body z axis

plant_model.Cm = 1.982e-07;	% Rotor torque coefficient(kg.m^2)
plant_model.Ct = 1.239e-05;	% Rotor thrust coefficient(kg.m^2)
plant_model.R = 0.225; % Body radius(m)

plant_model.motorCr = 718.078; %Motor throttle-speed curve slope(rad/s)
plant_model.motorWb = 88.448;  %Motor speed-throttle curve constant term(rad/s)
plant_model.motorT = 0.02;  %Motor inertia time constant(s)
plant_model.motorJm = 0.0001287;    %Moment of inertia of motor rotor + propeller(kg.m^2)

%% Exported Value
PLANT_EXPORT_VALUE.period = uint32(PLANT_CONST.dt*1e3); 

PLANT_EXPORT = Simulink.Parameter(PLANT_EXPORT_VALUE);
PLANT_EXPORT.CoderInfo.StorageClass = 'ExportedGlobal'; 


%% init sensor model
run('sensor_model_init.m');

load("mission.mat");