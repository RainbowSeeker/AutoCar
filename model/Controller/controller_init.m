
CONTROL_CONST.dt = 0.02;
%% Paramaters
CONTROL_PARAM_VALUE.KP = single(2);
CONTROL_PARAM_VALUE.KI = single(0.5);
CONTROL_PARAM_VALUE.dt = single(0.02);
CONTROL_PARAM = Simulink.Parameter(CONTROL_PARAM_VALUE);
CONTROL_PARAM.CoderInfo.StorageClass = 'ExportedGlobal';