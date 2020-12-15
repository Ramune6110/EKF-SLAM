% control parameters
V           = 3; % m/s
MAXG        = 30*pi/180; % radians, maximum steering angle (-MAXG < g < MAXG)
RATEG       = 20*pi/180; % rad/s, maximum rate of change in steer angle
WHEELBASE   = 4; % metres, vehicle wheel-base
DT_CONTROLS = 0.025; % seconds, time interval between control signals

% observation parameters
MAX_RANGE  = 10.0; % metres
DT_OBSERVE = 8*DT_CONTROLS; % seconds, time interval between observations

% data association innovation gates (Mahalanobis distances)
GATE_REJECT  = 4.0; % maximum distance for association
GATE_AUGMENT = 25.0; % minimum distance for creation of new feature

% waypoint proximity
AT_WAYPOINT  = 1.0; % metres, distance from current waypoint at which to switch to next waypoint
NUMBER_LOOPS = 2; % number of loops through the waypoint list

% data asosiation
SWITCH_ASSOCIATION_KNOWN = 0;
GIF_flag = 0;

% control noises
sigmaV= 0.3; % m/s
sigmaG= (3.0*pi/180); % radians
Q= [sigmaV^2 0; 0 sigmaG^2];

% observation noises
sigmaR= 0.1; % metres
sigmaB= (1.0*pi/180); % radians
R= [sigmaR^2 0; 0 sigmaB^2];