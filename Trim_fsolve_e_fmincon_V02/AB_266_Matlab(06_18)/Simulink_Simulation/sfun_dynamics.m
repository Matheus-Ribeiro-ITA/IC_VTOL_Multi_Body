function [sys,X0,str,ts,simStateCompliance] = ...
    sfun_dynamics(t,X,U,flag,aircraft,X0)

%
% The following outlines the general structure of an S-function.
%
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,X0,str,ts,simStateCompliance]=mdlInitializeSizes(X0);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=dynamics(t,X,U,aircraft);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
      sys=[];
%     sys=mdlUpdate(t,X,U);
%     [~,Y]=dynamics(t,X,U,aircraft);
%     sys=[X
%         Y];
  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
      [~,Y]=dynamics(t,X,U,aircraft);
    sys=[X
        Y];

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=[];

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=[];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,X0,str,ts,simStateCompliance]=mdlInitializeSizes(X0)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 13;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 13+21; %Combinacao estados mais saidas
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

% %
% % initialize the initial conditions
% %
% X0  = [];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes
