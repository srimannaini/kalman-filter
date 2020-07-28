%*************************************************************************
% BallPlateSim.m
%
% BallPlate (x-AXIS ONLY) simulation base program for xPos- & xDot-
% estimation via a 2nd- 3rd- or 4th-ORDER MODEL & OBSERVER
%
% 2020/05/25 KF_WS2019_BallPlate_Team1
%
% NOTES
%  Logfile ball position (XXXpos) measurements/estimates are scaled in [pxl]  
%  Logfile ball speed    (XXXdot) values/estimates       are scaled in [pxl/s]
%  They can be re-scaled into [m] and [m/s] via constant CM2PXL (see code).
%
% Format of logfile record:
%  1st column:   Abs. log time stamp          t_abs        in [s]
%  9th column:   Measured ball position       xPosMeas     in [pxl]
% 10th column:   Measured ball position       yPosMeas     in [pxl]
% 11th column:   Reference ball position      xPosRef      in [pxl]
% 12th column:   Reference ball position      yPosRef      in [pxl]
% 13th column:   Ball speed via num. dif.     xDotDifLog   in [pxl/s]
% 14th column:   Ball speed via num. dif.     yDotDifLog   in [pxl/s]
% 15th column:   Platform angle x             xAlfaPlatLog in [o]
% 16th column:   Platform angle y             yAlfaPlatLog in [o]
% 17th column:   Servo arm angle x            xThetServLog in [o]
% 18th column:   Servo arm angle y            yThetServLog in [o]
% 19th column:   Servo angle x PWM command    xThetPWMcmd  in [us]
% 20th column:   Servo angle y PWM command    yThetPWMcmd  in [us]
% 21st column:   Estimated ball x position    xPosEstLog   in [pxl]
% 22nd column:   Estimated ball x speed       xDotEstLog   in [pxl/s]
% Extended record option:
% +23rd column:  Estimated ball y position    yPosEstLog   in [pxl]
% +24th column:  Estimated ball y speed       yDotEstLog   in [pxl/s]
%
%   MatLab decimal number format uses '.' vs orig logfile uses ','  !! 
%   Original *.csv was converted to MatLab readable decimal format.
%
% Variable naming & suffix convention:
%  -Meas   real measured value
%  -Ref    real controller reference/setpoint value
%  -Dif    num. differentiation via backwards difference
%  -Pos    Position
%  -Dot    Position derivative == speed
%  -Est    Estimate (computed via Observer or KF)
%  -Obs    Estimate computed via Observer
%  -KF     Estimate computed via Kalman Filter
%
%  -Log    from log file == recorded from real system
%  -Sim    computed via this MATLAB simulation, to be compared with -Log values
%
% Changes
% 161025 Initial version, read 'BallPlateLog.cvs' & plot selected columns
% 171101 Detailed inline documentation of logfile record, added comments
% 181129 Line 206 changed sign of xThetServSim_v = - THETA_SERVO_MAX * ...
% 200203 Line 111 set PWM_X_OFFSET = -20 to compensate calibration offset
% 200524 Read 'SlowStep191216.csv',loaded 'BallPlateKFlib.c' via .Mex64 built, 
%        extended upto 4th order kalman discrete time, non-linear state space model (from line 474)
%        plotted Double vs Float figure 20 in order to see the precise
%        difference in C(float) & Matlab (Double) outputs @ 10e-7 
%*************************************************************************

clear all;
clear;
clc;
%% Build .mex file
%mex 'BallPlateKFlib.c'

%% Load KFlib C-library
if not(libisloaded('BallPlateKFlib')) 
    addpath('C:\Users\srima\OneDrive\Desktop\3rd sem\matlab\test1');     
    loadlibrary('BallPlateKFlib'); 
end 
  libfunctions BallPlateKFlib -full

%% Control section
LogFile = 'SlowStep191216.csv';
kMax=300;   
%kMax=500

% common general simulation parameters

T=50.0e-3;             % DEFAULT sample period @ 20Hz frame rate = 50ms
kSample_v=0:kMax-1;    % simulation sample index vector
SimTime_v=T*kSample_v; % simulation time vector

% sObsPole0 = -10.0;        
% sObsPole1 = -20.0;
% sObsPole2 = -50.0; 

sObsPole0 = -5.0;        % observer pole assignment in S-domain
sObsPole1 = -10.0;
sObsPole2 = -15.0;       % only used in 3rd order observer system
 
% Various flags/switches to control program behaviour 

bLogFileFormat24Columns = 1;% if 1: extended logfile record +yEst +yDotEst in cols 23&24 
bUseTrueSineTheta       = 0;% if 1: compute num. precise AlfaPlatSim with sin(ThetaServo) 
bPlotBasicLogFileData   = 1;% if 1: plot real system / basic logfile data 
bPlotSimulatedVsLogData = 0;% if 1: plot simulation vs real system data 

 bPlotSimObs2            = 1;% if 1: plot OBS2 simulation vs real system 
 bPlotSimObs3            = 0;% if 1: plot OBS3 simulation vs real system 
 bPlotSimObs23           = 0;% if 1: plot OBS3 vs OBS2 vs real system 
 bPlotKF3                = 0;% if 1: plot KF3  vs OBS3 & real system 
 bPlotKFn3               = 1;% if 1: plot KFn3(non-linear) vs KF3 & real system 
 bPlotKFn4               = 1;%if 1:  plot KFn4(non-linear) vs KFn3 & real system
 
 %% 
% INITIALIZATION SECTION

T22       = T*T/2;       % sample period in [s] 
CSQRT2    = sqrt(2.0);
CSQRT3    = sqrt(3.0);
C2PI      = 2.0 * pi;
CRAD2DEG  = 180.0 / pi;  % converts [rad] -> [o]  
CDEG2RAD  = pi / 180.0;  % converts [o] -> [rad]
  
CGEARTH   = 9.81;        % gravity constant in [m/s2] 
CM2PXL    = 1380;        % camera & lens scaling factor in [pxl] per [m]

% Plant model parameters

THETA_SERVO_MAX = 50;     % servo arm angle max amplitude        in [o]
PWM_CENTER   =  1500;     % PWM for servo neutral position       in [us]
PWM_AMPLTD   =   500;     % PWM amplitude for servo +/-max pos   in [us] 
PWM_X_OFFSET =   -20;     % PWM offset for plate horizontal pos. in [us]  -35 
PWM_Y_OFFSET =     0;     % PWM offset for plate horizontal pos. in [us]  -50

mBall     = 0.003;        % ball mass        in [kg]   NOT USED/NEEDED HERE ! 
rBall     = 0.002;        % ball radius      in [m]    NOT USED/NEEDED HERE !
dServo    = 0.030;        % servo arm length in [m]
dPlate    = 0.200;        % plate arm attachment radius in [m]

kbb = -0.6*CGEARTH*dServo/dPlate; % model/accel. constant in SI units
kga = kbb * CM2PXL * CDEG2RAD;    % ZiSo's model/acceleration constant:
                                  % requires input (servo-) angle in [o]
                                  % returns output scaled in [pxl/s2] 
                                  
% Time CONTINUOUS 2nd order plant model:   xdot(t) = A2*x(t) + b2*u(t)
A2=[0 1;0 0];                     %State Space Matrix
b2=[0 ;kga];                      %Input Vector
c2=[1 0];                         %Ouput Vector
Qb2=[c2; c2*A2];                  %observability
  
det( Qb2 );                        % observability check for CONTINUOUS system
if (det( Qb2 ) == 0)
 warning( 'det(Q_b2) = 0: 2nd ORDER CONTINUOUS SYSTEM NOT OBSERVABLE !' ); % message/break
end

% Time DISCRETE 2nd order plant model:   x[k+1] = F2*x[k] + g2*u[k]

F2 = [1,T;0,1];                   % continuous state to time-discrete
g2 = kga * [T22; T]; 
c2 = [1;0];
z20 = exp(sObsPole0*T); 
z21 = exp(sObsPole1*T);
q20 = z20*z21;
q21 = -(z20+z21);
hObs2 = [q21+2;(q20+q21+1)/T];    %observer gain

% Time CONTINUOUS 3rd order plant model:

taus = 0.3;                       % time constant
as = 1/taus;                       
A3 = [0,1,0;0,0,kga;0,0,-as];
b3 = [0;0;as];
c3 = [1;0;0];

Qb3c = [c3';c3'*A3;c3'*(A3*A3)];
              
if (det( Qb3c ) == 0)             % observability check for CONTINUOUS system
   warning( 'det(Q_b3c) = 0: 3rd ORDER CONTINUOUS SYSTEM NOT OBSERVABLE !' ); % message/break
end

% Time DISCRETE 3rd order plant model: 
 
F3=expm(A3*T);                     %continuous to discrete state matrix
syms x;
fun= expm(A3*x);
g3 = int(fun,x,0,T)*b3;
g3=eval(g3);
%g3=[-0.0014;-0.039;-0.1515];
Qb3=[c3';c3'*F3;c3'*F3^2];


% Time DISCRETE 3rd order observer gain design:

I3=[1 0 0;0 1 0;0 0 1];
z30 = exp(sObsPole0*T);
z31 = exp(sObsPole1*T);
z32 = exp(sObsPole2*T);
q30 = -(z30*z31*z32);
q31 = (z30*z31+z30*z32+z31*z32);
q32 = -(z30+z31+z32);

Qb3inv = inv(Qb3);    
f = Qb3inv(:,3);
hchk1 = q30*I3+q31*F3+q32*(F3*F3)+F3*F3*F3;
hObs3 = hchk1*f;                           % observer gain

%% DATA INPUT / LOGFILE READ SECTION

iLine = 2;   % line # to start reading (skip header & only LONG! lines)  
jCol0 = 0;   % column # to start reading
 

if ( bLogFileFormat24Columns == 1 ),  % use extended 24 column data record
  jCol1 = 23;  % last column index, extended (NEW) logfile has 24 data columns
else               % use 22 column data record                    
  jCol1 = 21;  % OLDER ballplate logfile may have only 22 data columns
end

% Read 1st data record/line from logfile (i.e. skip header text line !)
LogData1_t = dlmread( LogFile , ';', iLine, jCol0, [iLine jCol0 iLine jCol1] );

% Read desired range of kMax data records into one table/matrix LogData1_t
for k = 1:kMax-1
  iLine = iLine + 2;   % current line#/record to be read from logfile 
  LogRec1_t = dlmread( LogFile , ';', iLine, jCol0, [iLine jCol0 iLine jCol1] );
  %LogRec1_t           % test output of current file record     
  LogData1_t = [ LogData1_t
                 LogRec1_t  ];  % append new record to total table
end % for 

% Split up logfile table/matrix into individual variables -> column vectors XXX_v
AbsTime_v      = LogData1_t(:,1);   %  1st column:  absolute log time in [s]
xPosMeas_v     = LogData1_t(:,9);   %  9th column:  xPosMeas   in [pxl]
yPosMeas_v     = LogData1_t(:,10);  % 10th column:  yPosMeas   in [pxl]
xPosRef_v      = LogData1_t(:,11);  % 11th column:  xPosRef    in [pxl]
yPosRef_v      = LogData1_t(:,12);  % 12th column:  yPosRef    in [pxl]


xDotDifLog_v   = LogData1_t(:,13);  % 13th column:  xDotDifLog in [pxl/s]
yDotDifLog_v   = LogData1_t(:,14);  % 14th column:  yDotDifLog in [pxl/s]

xAlfaPlatLog_v = LogData1_t(:,15);  % 15th column:  platform angle x  in [o]
yAlfaPlatLog_v = LogData1_t(:,16);  % 16th column:  platform angle y  in [o] 
xThetServLog_v = LogData1_t(:,17);  % 17th column:  servo arm angle x in [o]
yThetServLog_v = LogData1_t(:,18);  % 18th column:  servo arm angle y in [o]


xThetPWMcmd_v = LogData1_t(:,19)-PWM_CENTER+PWM_X_OFFSET;  % 19th col: xThetPWMcmd in [us]
yThetPWMcmd_v = LogData1_t(:,20)-PWM_CENTER+PWM_Y_OFFSET;  % 20th col: yThetPWMcmd in [us]

xPosEstLog_v = LogData1_t(:,21);    % 21st column:  xPosEstLog    in [pxl] 
xDotEstLog_v = LogData1_t(:,22);   % 22nd column:  xDotEstLog    in [pxl/s]


if ( bLogFileFormat24Columns == 1 ),
  yPosEstLog_v = LogData1_t(:,23); % +23rd column: +yPosEstLog   in [pxl]
end

if ( bLogFileFormat24Columns == 1 ),
  yDotEstLog_v = LogData1_t(:,24);  % +24th column: +yDotEstLog   in [pxl/s]
end
% compute alfa platform simulation values (+/- numerically precise/true sin(Theta) )
xThetServSim_v = - THETA_SERVO_MAX * xThetPWMcmd_v / PWM_AMPLTD ;    

if ( bUseTrueSineTheta == 1 )  % use true sin(ThetaServo)
  xAlfaPlatSim_v = sin(xThetServSim_v*CDEG2RAD) * dServo / dPlate;
else                           % use linearization for sin(ThetaServo)
  xAlfaPlatSim_v = xThetServSim_v * dServo / dPlate;         
end


% if ( 0 ),
% % Plot xAlfaPlatLog vs xAlfaPlatSim
%   figure(1); clf;
%   plot(SimTime_v, [ xThetServLog_v  xThetServSim_v ] );
%   grid on;
%   xlabel('SimTime [s]')
%   ylabel('Theta  [o] [o]')
%   title('ThetaServLog(blu)  ThetaServSim(red)');
% 
%   figure(2); clf;
%   plot(SimTime_v, [ xAlfaPlatLog_v  xThetServSim_v  xAlfaPlatSim_v ] );
%   grid on;
%   xlabel('SimTime [s]')
%   ylabel('Alfa/Theta  [o] [o] [o]')
%   title('AlfaPlatLog(blu)  ThetaServSim(red) AlfaPlatSim(yel)');
% end

%*************************************************************************
% DATA PROCESSING & SIMULATION SECTION
%*************************************************************************
%% DATA PROCESSING & SIMULATION SECTION

% dimension & init simulation vars / plot vectors
xDotSimLPF_v   = zeros( kMax,1 );

% 2nd order system simulation
xPosSim2Obs_v  = zeros( kMax,1 );  % xPos2 via observer simulation
xDotSim2Obs_v  = zeros( kMax,1 );  % xDot2 via observer simulation

% dimension & init 2nd order state vector before starting sim loop
xState2Obsk  = [ xPosMeas_v(1)    % initialize xObsk(1) w measured value
                 0 ];
xState2Obsk1 = [ xPosMeas_v(1)    % initialize xObsk1(1) w measured value
                 0 ];
                          
xPosSim2Obs_v(1) = xPosMeas_v(1);

u     = xThetServLog_v;           % scalar control input variable (here: servo arm angle Theta in [o])

% MAIN SIMULATION LOOP
  
for k = 1:(kMax-1)                % sample index 
  % Time DISCRETE 2nd order observer design:
  %  x[k+1] = F2*x[k] + g2*u[k] + h2*( y[k] - c2'x[k] ) 
  xPosSim2Obs_v(k+1) = F2(1,1)*xPosSim2Obs_v(k)+ F2(1,2)*xDotSim2Obs_v(k) + g2(1)*u(k) ...
  + hObs2(1)*( xPosMeas_v(k) - (c2(1)*xPosSim2Obs_v(k)+c2(2)*xDotSim2Obs_v(k)));
  xDotSim2Obs_v(k+1) = F2(2,1)*xPosSim2Obs_v(k)+ F2(2,2)*xDotSim2Obs_v(k) + g2(2)*u(k) ...
  + hObs2(2)*( xPosMeas_v(k) - (c2(1)*xPosSim2Obs_v(k)+c2(2)*xDotSim2Obs_v(k)));
  
end % MAIN SIMULATION LOOP
%% 
% 3rd order Observer system simulation
xPosSim3Obs_v  = zeros( kMax,1 );  % xPos3 via observer simulation
xDotSim3Obs_v  = zeros( kMax,1 );  % xDot3 via observer simulation
xThetSim3Obs_v  = zeros( kMax,1 ); % xThet3 via observer simulation

% dimension & init 3nd order state vector before starting sim loop
xState3Obsk  = [ xPosMeas_v(1)    % initialize xObsk(1) w measured value
                 0 
                 0];
xState3Obsk1 = [ xPosMeas_v(1)    % initialize xObsk1(1) w measured value
                 0
                 0];
uk = 0;
yk = 0;

% MAIN SIMULATION LOOP
 
for k = 1:kMax            % sample index 
  % Time DISCRETE 3rd order observer design:
  uk = xThetServSim_v(k);
  yk = xPosMeas_v(k);
  % x[k+1] = F3*x[k] + g3*u[k] + h3*( y[k] - c3'x[k] )  
  xState3Obsk1 = F3*xState3Obsk + g3*uk + hObs3*( yk - c3'*xState3Obsk );
  
  xPosSim3Obs_v(k)  =  xState3Obsk1(1);
  xDotSim3Obs_v(k)  =  xState3Obsk1(2);
  xThetSim3Obs_v(k) =  xState3Obsk1(3)*10;
  
  xState3Obsk = xState3Obsk1;
  
  end % MAIN SIMULATION LOOP
 
%% 
% 3rd order KALMAN FILTER design for:          
%  xState3KFk       % optimal   state vector estimate at [k]
%  xState3KFk1      % predicted state vector estimate at [k+1]

  xk1KF3_v     = zeros( kMax,1 );  %KF gain matrix
  xk2KF3_v     = zeros( kMax,1 ); 
  xk3KF3_v     = zeros( kMax,1 ); 
    
  xP11KF3_v    = zeros( kMax,1 );  %estim. error cov matrix 
  xP22KF3_v    = zeros( kMax,1 ); 
  xP33KF3_v    = zeros( kMax,1 ); 

  xPosSim3KF_v  = zeros( kMax,1 ); 
  xDotSim3KF_v  = zeros( kMax,1 ); 
  xThetSim3KF_v = zeros( kMax,1 ); 
  
  
 sigmaXposPxl = 0.8;                         % measurement noise standard deviation
 rVar3        = sigmaXposPxl * sigmaXposPxl; %measurement noise cov matrix
 qVarXpos     = 5.0;                         %process noise cov diagonal elements initialization
 qVarXdot     = 400; 
 qVarTheta    = 5.0; 
 
 q11          = qVarXpos; 
 q22          = qVarXdot; 
 q33          = qVarTheta;
 Q3k  = diag( [ q11  q22  q33 ] );           % process noise covarience matrix
 P3k1  = diag( [ 0   0   0 ] );              % Initial est. error cov. matrix
 P3k= diag( [ 0   0  0   ] );                % Initial est. error cov. matrix(predicted)

 kKF3 =       [ 0   0   0 ]';                % initial Kalman filter gain vector
 xState3KFk  = [ xPosMeas_v(1)     
                 0 
                 0];
% MAIN SIMULATION LOOP

 for k=1:kMax; 
 uk = xThetServSim_v(k);
 yk = xPosMeas_v(k);
  
 %prediction
 P3k  = F3*P3k1*F3'+Q3k;                %estim. error covarience matrix(extrapolated) 
 xState3KFk1   = F3*xState3KFk + g3*uk; %state vector in predicted
     
 %measurement update
 kkF3= P3k*c3/(c3'*P3k*c3 + rVar3);     % KF gain vector
 P3k1  = (I3-kkF3*c3')*P3k;             % measurement estim. error covarience matrix 
 xState3KFk = xState3KFk1+kkF3*(yk-c3'*xState3KFk1); %state vector updated
 
 xk1KF3_v(k)     = kkF3(1);   
 xk2KF3_v(k)     = kkF3(2);
 xk3KF3_v(k)     = kkF3(3);
 
 xP11KF3_v(k)    = P3k1(1,1);
 xP22KF3_v(k)    = P3k1(2,2)/100;
 xP33KF3_v(k)    = P3k1(3,3);
 
 xPosSim3KF_v(k)  = xState3KFk(1);   
 xDotSim3KF_v(k)  = xState3KFk(2); 
 xThetSim3KF_v(k) = xState3KFk(3)*10; 
 end
 
 %% The 3rd order kalman discrete time, non-linear state space model  
  xk1KFn3_v     = zeros( kMax,1 );
  xk2KFn3_v     = zeros( kMax,1 ); 
  xk3KFn3_v     = zeros( kMax,1 ); 
    
  xP11KFn3_v    = zeros( kMax,1 );
  xP22KFn3_v    = zeros( kMax,1 ); 
  xP33KFn3_v    = zeros( kMax,1 ); 
  
  xPosSim3nKF_v  = zeros( kMax,1 ); 
  xDotSim3nKF_v  = zeros( kMax,1 ); 
  xThetSim3nKF_v = zeros( kMax,1 ); 
   
 sigmaXposPxl = 0.8;                    % measurement noise standard deviation
 rVar3        = sigmaXposPxl * sigmaXposPxl;  %measurement noise cov matrix
 qVarXpos     = 5.0;                  % process noise cov matrix
 qVarXdot     = 400; 
 qVarTheta    = 1.0; 
 
 q11          = qVarXpos; 
 q22          = qVarXdot; 
 q33          = qVarTheta;
 Q3k  = diag( [ q11  q22  q33 ] );     % process noise covarience matrix
 P3nk  = diag( [ 0   0   0 ] );        % Initial est. error cov. matrix
 P3nk1= diag( [  10 10 10 ] );         % Initial est. error cov. matrix
 knKF3 =       [ 0   0   0 ]';         % Kalman filter gain vector
 xState3KFnk  = [ xPosMeas_v(1)     
                 0 
                 0];
              
 for k=1:kMax;
     
     uk = xThetServSim_v(k);           
     yk = xPosMeas_v(k);
     zk(1)=0;                                 %initialization
     zk(k) = xState3KFnk(3);                  %estimated servo angle
     costhetas=cos( zk(k)*CDEG2RAD);          %coversion from [o] to [rad] 
 Fk=[1 T 0 ;0 1 kga*costhetas*T ; 0 0 1-as*T]; 
 gn3k=[0;kga*costhetas*T22*as;as*T-as*as*T22];
     
 %prediction
 P3nk  = Fk*P3nk1*Fk'+Q3k;
 xState3KFnk1   = Fk*xState3KFnk+gn3k*uk;
            
 %measurement update
 knkF3= P3nk*c3/(c3'*P3nk*c3 + rVar3);
 P3nk1  = (I3-knkF3*c3')*P3nk;
 xState3KFnk = xState3KFnk1+knkF3*(yk-c3'*xState3KFnk1);
 
 xk1KFn3_v(k)     = knkF3(1);
 xk2KFn3_v(k)     = knkF3(2);
 xk3KFn3_v(k)    = knkF3(3);
 
 xP11KFn3_v(k)   = P3nk1(1,1);
 xP22KFn3_v(k)    = P3nk1(2,2)/100;
 xP33KFn3_v(k)    = P3nk1(3,3);
 
 xPosSim3nKF_v(k)  = xState3KFnk(1) ; 
 xDotSim3nKF_v(k)  = xState3KFnk(2) ; 
 xThetSim3nKF_v(k) = xState3KFnk(3)*10 ; 
end    
 %% The 4th order discrete time, non-linear state space model  
 % Variable initiation for matlab script
  xk1KFn4_v     = zeros( kMax,1 );
  xk2KFn4_v     = zeros( kMax,1 ); 
  xk3KFn4_v     = zeros( kMax,1 );
  xk4KFn4_v     = zeros( kMax,1 );
    
  xP11KFn4_v    = zeros( kMax,1 );
  xP22KFn4_v    = zeros( kMax,1 ); 
  xP33KFn4_v    = zeros( kMax,1 ); 
  xP44KFn4_v    = zeros( kMax,1 );
  
  xPosSim4nKF_v  = zeros( kMax,1 ); 
  xDotSim4nKF_v  = zeros( kMax,1 ); 
  xThetSim4nKF_v = zeros( kMax,1 ); 
  xdThetSim4nKF_v = zeros( kMax,1 ); 
   
 sigmaXposPxl = 0.8;                             % measurement noise standard deviation
 rVar4        = sigmaXposPxl * sigmaXposPxl;   %measurement noise cov matrix
 qVarXpos     = 5.0;                           %process cov matrix diagonal elements
 qVarXdot     = 400; 
 qVarTheta    = 5.0; 
 qVardelta    = 0.1;
 q11          = qVarXpos; 
 q22          = qVarXdot; 
 q33          = qVarTheta;
 q44          = qVardelta;
 Q4k = diag( [ q11 q22 q33 q44 ] );           % process noise covarience matrix
 P4nk1  = diag( [ 0   0   0   0 ] );          % Initial est. error cov. matrix
 P4nk= diag( [ 10 10 10 1] );                 % Initial est. error cov. matrix
 knKF4 =       [ 0  0  0  0 ]';               % Kalman filter gain vector
 xState4KFnk1=zeros(4,1);
 xState4KFnk  = [ xPosMeas_v(1)     
                 0 
                 0
                 0];
c4=[1; 0; 0; 0];
I4 = [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
b4=[0 ;0; as; 0]; 

% Variable initiation for C-functions(same as matlab SCRIPT)
 gn4kc= [0,0,0,0];
 Q4kc = diag( [ q11 q22 q33 q44 ] );
 P4nk1c  = diag( [ 0   0   0   0 ] ); 
 P4nkc= diag( [ 10 10 10 1] );
 knKF4c =       [ 0  0  0  0 ]';
 xState4KFnk1c=zeros(4,1);
 xState4KFnkc  = [ xPosMeas_v(1)     
                 0 
                 0
                 0];
             
                       
    
%MAIN SIMULATION LOOP (4th order)
 for  k=1:kMax;                         % sample index 
     zk1(1)=0;
     zk(1)=0;
     zk(k) = xState4KFnk(3)*CDEG2RAD; % estimated servo angle
     zk1(k)= xState4KFnk(4)*CDEG2RAD; % estimated thetaservooffset
     theta4= zk(k)+zk1(k);
 %% Matlab-functions
 
 Fk=[1 T 0 0;0 1 kga*cos(theta4)*T kga*cos(theta4)*T; 0 0 1-as*T 0;0 0 0 1]; 
 gn4k=[0;kga*cos(theta4)*T22*as;as*T-as*as*T22;0];
 uk = xThetServSim_v(k);
 yk = xPosMeas_v(k);
 
 %prediction
 P4nk1  = Fk*P4nk*Fk'+Q4k;                  %estim. error covarience matrix 
 xState4KFnk1   = Fk*xState4KFnk+gn4k*uk;   %state vector in predicted
 %measurement update
 knKF4= P4nk1*c4/(c4'*P4nk1*c4 + rVar4);    % KF gain vector
 P4nk  = (I4-knKF4*c4')*P4nk1;              % measurement estim. error covarience matrix 
 xState4KFnk = xState4KFnk1+knKF4*(yk-c4'*xState4KFnk1); %state vector estimate
 %store matlab outputs for plot
 xk1KFn4_v(k)    = knKF4(1);
 xk2KFn4_v(k)    = knKF4(2);
 xk3KFn4_v(k)    = knKF4(3);
 xk4KFn4_v(k)    = knKF4(4);
 
 xP11KFn4_v(k)   = P4nk(1,1);
 xP22KFn4_v(k)   = P4nk(2,2)/100;
 xP33KFn4_v(k)   = P4nk(3,3);
 xP44KFn4_v(k)   = P4nk(4,4)/10;
 
 xP11KFn41_v(k)   = P4nk1(1,1);
 xP22KFn41_v(k)   = P4nk1(2,2)/100;
 xP33KFn41_v(k)   = P4nk1(3,3);
 xP44KFn41_v(k)   = P4nk1(4,4)/10;
 
 
 xPosSim4nKF_v(k)   = xState4KFnk(1) ; 
 xDotSim4nKF_v(k)   = xState4KFnk(2) ; 
 xThetSim4nKF_v(k)  = xState4KFnk(3)*10 ; 
 xdThetSim4nKF_v(k) = xState4KFnk(4);
 
 
 %% C-functions
 Fkc = [1 T 0 0;0 1 0 0; 0 0 1-as*T 0;0 0 0 1];
 Tkga=T*kga;
 FParams= [Tkga;zk(k);zk1(k)];
 Fkc=calllib( 'BallPlateKFlib', 'KFcomputeFMat',Fkc,Fkc',FParams');
 gn4kc=calllib( 'BallPlateKFlib','KFcomputeG',gn4kc',FParams',as,T,T22);
 ukc = xThetServSim_v(k);
 ykc = xPosMeas_v(k);
 
 %prediction
 Q4kc= [q11 q22 q33 q44];
 P4nk1c= calllib( 'BallPlateKFlib', 'KF_PTUPD', P4nk1c,Fkc',P4nkc',Q4kc' );
 xState4KFnk1c= calllib( 'BallPlateKFlib', 'KFpredictState', xState4KFnk1c,xState4KFnkc',Fkc', gn4kc',ukc );
 %measurement update
 knKF4c= calllib('BallPlateKFlib', 'KFcomputeKMat',knKF4c,P4nk1c', rVar3);
 P4nkc= calllib('BallPlateKFlib','KF_PMUPD',P4nkc,knKF4c', P4nk1c');
 xState4KFnkc= calllib('BallPlateKFlib','KF_xMUPD',xState4KFnkc, xState4KFnk1c', knKF4c', ykc);
 %store C-outputs for plot
 xk1KFn4_vc(k)    = knKF4c(1);
 xk2KFn4_vc(k)    = knKF4c(2);
 xk3KFn4_vc(k)    = knKF4c(3);
 xk4KFn4_vc(k)    = knKF4c(4);
 
 xP11KFn4_vc(k)   = P4nkc(1,1);
 xP22KFn4_vc(k)   = P4nkc(2,2)/100;
 xP33KFn4_vc(k)   = P4nkc(3,3);
 xP44KFn4_vc(k)   = P4nkc(4,4)/10;
 
 xPosSim4nKF_vc(k)   = xState4KFnkc(1) ; 
 xDotSim4nKF_vc(k)   = xState4KFnkc(2) ; 
 xThetSim4nKF_vc(k)  = xState4KFnkc(3)*10 ; 
 xdThetSim4nKF_vc(k) = xState4KFnkc(4);
 
 %% double vs float (difference = C_Output-Matlab_Output)
 diff_xk4KFn4(k)= xk4KFn4_vc(k)-xk4KFn4_v(k);
 diff_xdThetSim4nKF(k) = xdThetSim4nKF_vc(k)-xdThetSim4nKF_v(k);
 end

%% DATA OUTPUT / PLOTTING SECTION  

% 2nd order observer vs log data
 if( bPlotSimObs2 == 1); % if 1: plot OBS2 simulation vs real system 
    
figure(1); clf;
plot(SimTime_v,xPosRef_v,'r', SimTime_v, xPosMeas_v,'b', SimTime_v, xPosEstLog_v,'b', SimTime_v,xPosSim2Obs_v,'g' );
grid on;
xlabel('SimTime [s]')
ylabel('xPos  [pxl] [pxl] [pxl] [pxl] ')
legend('xPosRef(red)', 'xPosMeas(blue)', 'xPosEst(black)','xPosSim2(green)')
title('Log data vs Sim2Obs(Position)')
  
figure(2); clf;
plot(SimTime_v,xDotDifLog_v,'r',SimTime_v,xDotEstLog_v,'b',SimTime_v,xDotSim2Obs_v,'g' );
grid on;
xlabel('SimTime [s]')
ylabel('xDot  [pxl/s] ')
legend('xDotDifLog_v(red)','xDotEstLog_v(blue)','xDotSim2Obs_v(green)')
title('Log data vs Sim2Obs(Velocity)')

end;
%    
%3rd order observer vs log data
if( bPlotSimObs3 == 1 ); % if 1: plot OBS3 simulation vs real system
    
figure(3); clf;
plot( SimTime_v, xPosMeas_v,'r', SimTime_v, xPosEstLog_v,'b',SimTime_v,xPosSim3Obs_v,'g' );
grid on;
xlabel('SimTime [s]')
ylabel('xPos  [pxl] ')
legend( 'xPosMeas(red)', 'xPosEst(blue)','xPosSim3Obs_v(green)')
title('Log data vs Sim3Obs(Position)')
  
figure(4); clf;
plot(SimTime_v,xDotDifLog_v,'r',SimTime_v,xDotEstLog_v,'b',SimTime_v,xDotSim3Obs_v,'g' );
grid on;
xlabel('SimTime [s]')
ylabel('xDot  [pxl/s] ')
legend('xDotEstLog_v(r)','xDotDifLog_v(b)','xDotSim2Obs_v(g)')
title('Log data vs Sim3Obs(Velocity)')
%      
figure(5); clf;
plot(SimTime_v,xThetServLog_v*10,'r',SimTime_v,xThetPWMcmd_v,'b',SimTime_v,xThetSim3Obs_v,'g' );
grid on;
xlabel('SimTime [s]')
ylabel('xTheta[o]')
legend('xThetServLog_v*10(red)','xThetPWMcmd_v(blue)','xThetaSim3Obs_v(green)')
title('Log data vs Sim3Obs(Thetaservo)')

end;
%       
% 3rd order observer vs 2nd order vs log data
if( bPlotSimObs23 == 1 ); % if 1: plot OBS3 vs OBS2 vs real system 
    
figure(6); clf;
plot(SimTime_v, xPosMeas_v,'r', SimTime_v,xPosSim2Obs_v,'b',SimTime_v,xPosSim3Obs_v,'g' );
grid on;
xlabel('SimTime [s]')
ylabel('xPos  [pxl] ')
legend('xPosMeas(blue)','xPosSim2Obs_v(black)','xPosSim3Obs_v(cyan)')
title('Log data vs Sim2Obs vs Sim3Obs(Position)')
%   
figure(7); clf;
plot(SimTime_v,xDotDifLog_v,'r',SimTime_v,xDotSim2Obs_v,'b',SimTime_v,xDotSim3Obs_v,'g' );
grid on;
xlabel('SimTime [s]')
ylabel('xDot  [pxl/s] ')
legend('xDotDifLog_v(red)','xDotSim2Obs_v(blue)','xDotSim3Obs_v(green)')
title('Log data vs Sim2Obs vs Sim3Obs(Velocity)')

end;
%      
% 3rd order kalman filter vs 3rd order observer vs log data
if( bPlotKF3 == 1 ); % if 1: plot KF3 estimates vs OBS3 & real system 
    
figure(8); clf;
plot(SimTime_v, xPosMeas_v,'r',SimTime_v,xPosSim3Obs_v,'b',SimTime_v,xPosSim3KF_v,'-g' );
grid on;
xlabel('SimTime [s]')
ylabel('xPos  [pxl] ')
legend('xPosMeas(red)','xPosSim3Obs_v(blue)','xPosSim3KF_v(green)')
title('Log data vs Sim3Obs vs Sim3KF(Position)')
%   
figure(9); clf;
plot(SimTime_v,xDotDifLog_v,'r',SimTime_v,xDotSim3Obs_v,'b',SimTime_v,xDotSim3KF_v,'g' );
grid on;
xlabel('SimTime [s]')
ylabel('xDot  [pxl/s] ')
legend('xDotDifLog_v(red)','xDotSim3Obs_v(blue)','xDotSim3KF_v(green)')
title('Log data vs Sim3Obs vs Sim3KF(Velocity)')
%      
figure(10); clf;
plot(SimTime_v,xThetServLog_v*10,'r',SimTime_v,xThetSim3Obs_v,'b',SimTime_v,xThetSim3KF_v,'g' );
grid on;
xlabel('SimTime [s]')
ylabel('xTheta[o]')
legend('xThetServLog_v*10(red)','xThetaSim3Obs_v(blue)','xThetSim3KF_v(green)')
title('Log data vs Sim3KF  vs Sim3KF(Thetaservo)')

end;

%3rd order kalman (linear vs non-linear)
if( bPlotKFn3 == 1); % if 1: plot KFn3(non-linear) vs KF3 & real system 
    
figure(11); clf;
plot( SimTime_v, xPosMeas_v,'r',SimTime_v,xPosSim3KF_v,'b',SimTime_v, xPosSim3nKF_v,'-g'  );
grid on;
legend('xPosMeas(red)','xPosSim3KF_v(blue)','xPosSim3nKF_v(green)')
xlabel('SimTime [s]')
ylabel('xPos  [pxl] ')
title('measured vs Sim3KF vs Sim3nKF(position)')
%   
figure(12); clf;
plot( SimTime_v,xDotDifLog_v,'r',SimTime_v,xDotSim3KF_v,'b',SimTime_v, xDotSim3nKF_v,'g' );
grid on;
xlabel('SimTime [s]')
ylabel('xDot  [pxl/s] ')
legend( 'xDotDifLog_v(red)','xDotSim3KF_v(blue)','xDotSim3nKF_v(green)')
title('Log data vs Sim3KF vs Sim3nKF ')
%      
figure(13); clf;
plot(SimTime_v,xThetServLog_v*10,'r',SimTime_v,xThetSim3KF_v,'b',SimTime_v,xThetSim3nKF_v,'g' );
grid on;
xlabel('SimTime [s]')
ylabel('xTheta[o]')
legend('xThetServLog_v*10(red)','xThetSim3KF_v(blue)','xThetSim3nKF_v(green)')
title('Log data vs Sim3KF vs Sim3nKF(Thetaservo)')

end;
 
% 4th order kalman non-linear 
if( bPlotKFn4   == 1); % if 1: plot KFn4(non-linear) vs KFn3 vs KF3 & real system
    
figure(14); clf;
plot(SimTime_v, xPosMeas_v,'xr',SimTime_v,xPosSim4nKF_v,'og'  );
grid on;
legend('xPosMeas(red)','xPosSim4nKF_v(green)')
xlabel('SimTime [s]')
ylabel('xPos  [pxl] ')

title('xPosMeas vs xPosSim4nKF_v')
%   
figure(15); clf;
plot( SimTime_v,xDotDifLog_v,'r',SimTime_v,xDotSim3nKF_v,'b',SimTime_v, xDotSim4nKF_v,'-g' );
grid on;
xlabel('SimTime [s]')
ylabel('xDot  [pxl/s] ')
legend( 'xDotDifLog_v(red)','xDotSim3nKF_v(blue)','xDotSim4nKF_v(green)')
title(' xDotDifLog_v xDotSim3nKF_v xDotSim4nKF_v')
%      
figure(16); clf;
plot(SimTime_v,xThetServLog_v*10,'r',SimTime_v,xThetSim3nKF_v,'b',SimTime_v,xThetSim4nKF_v,'-g' );
grid on;
xlabel('SimTime [s]')
ylabel('xTheta [o]')
legend('xThetServLog_v*10(red)','xThetSim3nKF_v(blue)','xThetSim4nKF_v(green)')
title('xThetServLog_v*10 xThetSim3nKF_v xThetSim4nKF_v')
%
figure(17); clf;
plot(SimTime_v,xdThetSim4nKF_v,'g' );
grid on;
xlabel('SimTime [s]');
ylabel('xTheta [o]');
legend('xdThetSim4nKF_v(green)');
title('theta offset');


%
figure(18); clf;
plot(SimTime_v,xP11KFn4_v,'r',SimTime_v,xP22KFn4_v,'b',SimTime_v,xP33KFn4_v,'g',SimTime_v,xP44KFn4_v,'y' );
xlabel('SimTime [s]')
ylabel(' xP11KFn4_v xP22KFn4_v xP33KFn4_v xP44KFn4_v ')
legend('xP11KFn4_v(red)','xP22KFn4_v(blue)','xP33KFn4_v(green)','xP44KFn4_v(yellow)')
title('xP11KFn4_v xP22KFn4_v xP33KFn4_v xP44KFn4_v ')
%
figure(19); clf;
plot(SimTime_v,xk1KFn4_v,'r',SimTime_v,xk2KFn4_v,'b',SimTime_v,xk3KFn4_v,'g',SimTime_v,xk4KFn4_v,'y' );
grid on;
xlabel('SimTime [s]')
ylabel(' xk1KFn4_v xk2KFn4_v xk3KFn4_v  xk4KFn4_v')
legend('xk1KFn4_v(red)','xk2KFn4_v(blue)','xk3KFn4_v(green)','xk4KFn4_v(yellow)');
title(' non-linear kalman gain ')
%
figure(20);clf;
plot(SimTime_v,diff_xdThetSim4nKF,'r');
legend('diff_xdThetSim4nKF');
grid on;
xlabel('SimTime [s]');
ylabel('xdThetSim4nKF_vc(k)-xdThetSim4nKF_v(k)');
title(' Double vs Float ');
end;
%

% *************************************************************************
% End of BallPlateSim4.m
% *************************************************************************
% 
