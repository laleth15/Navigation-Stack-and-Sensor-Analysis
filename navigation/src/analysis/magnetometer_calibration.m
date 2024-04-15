clc;
close all;

%open bag file
bag = rosbag('../data/circles.bag');


imu = select(bag,'Topic','/imu');
msgStr = readMessages(imu,'DataFormat','struct');
magX = cellfun(@(m) double(m.MagField.MagneticField_.X),msgStr);
magY = cellfun(@(m) double(m.MagField.MagneticField_.Y),msgStr);
mag = [magX,magY];


% Ellipse fit
[a,b,orientationRad,x0,y0] = fitEllipse(magX,magY);

%Hard iron distortion
offsetX = x0;
offsetY = y0;
hardCorrectedX = magX - offsetX;
hardCorrectedY = magY - offsetY;

%soft iron distortion
theta = orientationRad;
R = [cos(theta) sin(theta);-sin(theta) cos(theta)];
magTransformed = (R*[hardCorrectedX,hardCorrectedY]')';

scaleFactor = b/a;
scaleMat= [scaleFactor 0;0 1];
magScaled = (scaleMat*magTransformed')';

theta = -theta;
R1 = [cos(theta) sin(theta);-sin(theta) cos(theta)];
magCorrected= (R1*magScaled')';

figure;
scatter(magCorrected(:,1),magCorrected(:,2), 10 ,"filled","DisplayName","After calibration");
hold on;
scatter(magX, magY, 10, "filled", "DisplayName", "Before calibration");
hold on;
xlabel("Magnetic Field X (Gauss)");
ylabel("Magnetic Field Y (Gauss)");
axis equal;

title('Plot - Magnetometer X-Y plot before and after hard and soft iron calibration');
legend;


SoftIronRotationBack = R1*scaleMat*R;


imuSec = cellfun(@(m) double(m.Header.Stamp.Sec),msgStr);
imuNanoSec = cellfun(@(m) double(m.Header.Stamp.Nsec),msgStr);
imuTimeTot = double(imuSec + ( imuNanoSec * 10^(-9)));
imuTime = imuTimeTot - imuTimeTot(1);


figure;
scatter(imuTime, magX, 10 ,"filled","DisplayName","Before Calibration");
hold on;
scatter(imuTime, magCorrected(:,1), 10, "filled", "DisplayName", "After calibration");
hold on;
xlabel("Time (s)");
ylabel("Magnetic Field X (Gauss)");
title('Plot - time series Magnetic Field X  before and after the correction');
legend;

figure;
scatter(imuTime, magY, 10 ,"filled","DisplayName","Before Calibration");
hold on;
scatter(imuTime, magCorrected(:,2), 10, "filled", "DisplayName", "After calibration");
hold on;
xlabel("Time (s)");
ylabel("Magnetic Field Y (Gauss)");
title('Plot - time series Magnetic Field Y before and after the correction');
legend;

%--------------------------------------------------------------------------------------------

function [a,b,orientationRad,X0,Y0] = fitEllipse(x,y,axis_handle)
% initialize
orientationTol = 1e-3;

% empty warning stack
warning( '' );

x = x(:);
y = y(:);

meanX = mean(x);
meanY = mean(y);
x = x-meanX;
y = y-meanY;

% the estimation for the conic equation of the ellipse
X = [x.^2, x.*y, y.^2, x, y ];
a = sum(X)/(X'*X);

% check for warnings
if ~isempty( lastwarn )
    disp( 'stopped because of a warning regarding matrix inversion' );
    ellipse_t = [];
    return
end

% extract parameters from the conic equation
[a,b,c,d,e] = deal( a(1),a(2),a(3),a(4),a(5) );

% remove the orientation from the ellipse
if ( min(abs(b/a),abs(b/c)) > orientationTol )
    
    orientationRad = 1/2 * atan( b/(c-a) );
    cosPhi = cos( orientationRad );
    sinPhi = sin( orientationRad );
    [a,b,c,d,e] = deal(...
        a*cosPhi^2 - b*cosPhi*sinPhi + c*sinPhi^2,...
        0,...
        a*sinPhi^2 + b*cosPhi*sinPhi + c*cosPhi^2,...
        d*cosPhi - e*sinPhi,...
        d*sinPhi + e*cosPhi );
    [meanX,meanY] = deal( ...
        cosPhi*meanX - sinPhi*meanY,...
        sinPhi*meanX + cosPhi*meanY );
else
    orientationRad = 0;
    cosPhi = cos( orientationRad );
    sinPhi = sin( orientationRad );
end

% check if conic equation represents an ellipse
test = a*c;
switch (1)
case (test>0),  status = '';
case (test==0), status = 'Parabola found';  warning( 'fit_ellipse: Did not locate an ellipse' );
case (test<0),  status = 'Hyperbola found'; warning( 'fit_ellipse: Did not locate an ellipse' );
end

% if we found an ellipse return it's data
if (test>0)
    
    % make sure coefficients are positive as required
    if (a<0), [a,c,d,e] = deal( -a,-c,-d,-e ); end
    
    % final ellipse parameters
    X0          = meanX - d/2/a;
    Y0          = meanY - e/2/c;
    F           = 1 + (d^2)/(4*a) + (e^2)/(4*c);
    [a,b]       = deal( sqrt( F/a ),sqrt( F/c ) );    
    longAxis   = 2*max(a,b);
    shortAxis  = 2*min(a,b);

    % rotate the axes backwards to find the center point of the original TILTED ellipse
    R           = [ cosPhi sinPhi; -sinPhi cosPhi ];
    P_in        = R * [X0;Y0];
    X0_in       = P_in(1);
    Y0_in       = P_in(2);
    
    % pack ellipse into a structure
    ellipse_t = struct( ...
        'a',a,...
        'b',b,...
        'phi',orientationRad,...
        'X0',X0,...
        'Y0',Y0,...
        'X0_in',X0_in,...
        'Y0_in',Y0_in,...
        'long_axis',longAxis,...
        'short_axis',shortAxis,...
        'status','' );
else
    % report an empty structure
    ellipse_t = struct( ...
        'a',[],...
        'b',[],...
        'phi',[],...
        'X0',[],...
        'Y0',[],...
        'X0_in',[],...
        'Y0_in',[],...
        'long_axis',[],...
        'short_axis',[],...
        'status',status );
end

% check if we need to plot an ellipse with it's axes.
if (nargin>2) & ~isempty( axis_handle ) & (test>0)
    
    % rotation matrix to rotate the axes with respect to an angle phi
    R = [ cosPhi sinPhi; -sinPhi cosPhi ];
    
    % the axes
    ver_line        = [ [X0 X0]; Y0+b*[-1 1] ];
    horz_line       = [ X0+a*[-1 1]; [Y0 Y0] ];
    new_ver_line    = R*ver_line;
    new_horz_line   = R*horz_line;
    
    % the ellipse
    theta_r         = linspace(0,2*pi);
    ellipse_x_r     = X0 + a*cos( theta_r );
    ellipse_y_r     = Y0 + b*sin( theta_r );
    rotated_ellipse = R * [ellipse_x_r;ellipse_y_r];
    
    % draw
    hold_state = get( axis_handle,'NextPlot' );
    set( axis_handle,'NextPlot','add' );
    plot( new_ver_line(1,:),new_ver_line(2,:),'r' );
    plot( new_horz_line(1,:),new_horz_line(2,:),'r' );
    plot( rotated_ellipse(1,:),rotated_ellipse(2,:),'r' );
    set( axis_handle,'NextPlot',hold_state );
end
end