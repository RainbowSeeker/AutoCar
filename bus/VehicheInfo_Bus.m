function cellInfo = VehicheInfo_Bus(varargin) 
% VEHICHEINFO_BUS returns a cell array containing bus object information 
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
    'VehicheInfo_Bus', ... 
    '', ... 
    '', ... 
    'Auto', ... 
    '-1', ... 
    '0', {... 
{'lat', 1, 'int32', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'lon', 1, 'int32', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'direction', 1, 'single', 'real', 'Sample', 'Fixed', [], [], '', sprintf('+- 1')}; ...
{'currSteer', 1, 'single', 'real', 'Sample', 'Fixed', [], [], sprintf('deg'), ''}; ...
{'currR', 1, 'single', 'real', 'Sample', 'Fixed', [], [], '', sprintf('Psi Rate')}; ...
{'currVelocity', 1, 'single', 'real', 'Sample', 'Fixed', [], [], '', sprintf('vx forward speed')}; ...
{'currPose', [1 3], 'single', 'real', 'Sample', 'Fixed', [], [], '', sprintf('current pose ---> (x y psi)')}; ...
    } ...
  } ...
}'; 

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo) 
end 