function cellInfo = Controller_In_Bus(varargin) 
% CONTROLLER_IN_BUS returns a cell array containing bus object information 
% 
% Optional Input: 'false' will suppress a call to Simulink.Bus.cellToObject 
%                 when the MATLAB file is executed. 
% The order of bus element attributes is as follows:
%   ElementName, Dimensions, DataType, Complexity, SamplingMode, DimensionsMode, Min, Max, DocUnits, Description 

suppressObject = false; 
if nargin == 1 && islogical(varargin{1}) && varargin{1} == false 
    suppressObject = true; 
elseif nargin > 1 
    error('Invalid input argument(s) encountered'); 
end 

cellInfo = { ... 
  { ... 
    'Controller_In_Bus', ... 
    '', ... 
    '', ... 
    'Auto', ... 
    '-1', ... 
    '0', {... 
{'valid_num', 1, 'uint16', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'timestamp', 1, 'uint32', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'savePose', [50 3], 'single', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'lat_0', 1, 'int32', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'lon_0', 1, 'int32', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
}'; 

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo) 
end 