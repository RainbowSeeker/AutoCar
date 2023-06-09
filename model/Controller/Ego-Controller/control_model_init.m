CONTROL_CONST.dt = 0.02;
CONTROL_CONST.WHEEL_BASE = single(0.28);
CONTROL_CONST.MAX_STEER = single(30);

%% Exported Value
CONTROL_EXPORT_VALUE.period = uint32(CONTROL_CONST.dt*1e3); 

CONTROL_EXPORT = Simulink.Parameter(CONTROL_EXPORT_VALUE);
CONTROL_EXPORT.CoderInfo.StorageClass = 'ExportedGlobal'; 

%% Paramaters
CONTROL_PARAM_VALUE.VEL_X_P = single(1.0);
CONTROL_PARAM_VALUE.VEL_X_I = single(0.02);
CONTROL_PARAM_VALUE.VEL_X_D = single(0.0);
CONTROL_PARAM_VALUE.VEL_X_I_MIN = single(-1);
CONTROL_PARAM_VALUE.VEL_X_I_MAX = single(1);
CONTROL_PARAM_VALUE.VEL_X_D_MIN = single(-1);
CONTROL_PARAM_VALUE.VEL_X_D_MAX = single(1);
CONTROL_PARAM_VALUE.DIR_KP = single(2);
CONTROL_PARAM_VALUE.CRUISE_SPEED = single(1);
CONTROL_PARAM = Simulink.Parameter(CONTROL_PARAM_VALUE);
CONTROL_PARAM.CoderInfo.StorageClass = 'ExportedGlobal';

