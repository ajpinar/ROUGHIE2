% processing of 7/20/15 data
clc
clear

% 'header' is a list of strings that define what each column in 'data' is.
[ header, data ] = loadLog('7_20_2015_LOG_modified.CSV');

%% Extract each of the columns to a meaningful variable
time =       data(:,1);
time = (time - time(1)) / 1000; % puts time in seconds and removes bias
pressure =   data(:,2);
pitch =      data(:,3);
roll =       data(:,4);
drawWire =   data(:,5);
rotPos =     data(:,6);
linMassPos = data(:,7);
tp1 =        data(:,8);
yaw =        data(:,9);
rolld =      data(:,10);
pitchd =     data(:,11);
yawd =       data(:,12);

%% plot raw pressure reading (these are the numbers straight from the ADC)
figure
plot( time, pressure ); xlabel( 'Time (ms)' );
ylabel( 'Raw ressure reading' );

%% plot estimated depth reading using Tony's magical equation
% (0.0423062 * (getFiltAnalog(pressureSensorPin) - 102.3));
figure
plot( time, 0.0423062 * (pressure - 102.3) ); xlabel( 'Time (s)' );
ylabel( 'Estimated depth (feet)' );
title( 'Depth Estimate from Pressure Sensor Data' ); grid minor;
xlim([0 time(end)])
set( gca, 'Ydir', 'reverse' );

