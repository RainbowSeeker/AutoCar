CONTROL_CONST.dt = 0.02;
CONTROL_CONST.NumOfPoints = single(50);

%% Paramaters
CONTROL_PARAM_VALUE.KP = single(2);
CONTROL_PARAM_VALUE.KI = single(0.5);
CONTROL_PARAM_VALUE.CRUISE_SPEED = single(1);
CONTROL_PARAM_VALUE.VIEWSHED = single(2*CONTROL_CONST.NumOfPoints);
CONTROL_PARAM = Simulink.Parameter(CONTROL_PARAM_VALUE);
CONTROL_PARAM.CoderInfo.StorageClass = 'ExportedGlobal';



%% Exported Value
CONTROL_EXPORT_VALUE.period = uint32(CONTROL_CONST.dt*1e3); 

CONTROL_EXPORT = Simulink.Parameter(CONTROL_EXPORT_VALUE);
CONTROL_EXPORT.CoderInfo.StorageClass = 'ExportedGlobal'; 