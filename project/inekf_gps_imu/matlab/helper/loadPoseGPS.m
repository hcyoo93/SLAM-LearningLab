function [T_X, omega, accel, accel_b, T_GPS, XYZ_GPS] = loadPoseGPS()
%% Import data from text file.
% Script for importing data from the following text file:
%
%    H:\ETH_Fotokite\AGZ\Log Files\GroundTruthAGL.csv
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2015/10/20 16:37:39

%% Initialize variables.
F_GPS = '../data/OnboardGPS.csv';
F_X = '../data/OnboardPose.csv';

delimiter = ',';
startRow = 2;

%% Format string for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
%   column9: double (%f)
%	column10: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec_X = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
formatSpec_GPS = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID(1) = fopen(F_X,'r');
fileID(2) = fopen(F_GPS,'r');


%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray_X = textscan(fileID(1), formatSpec_X, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false);
dataArray_GPS = textscan(fileID(2), formatSpec_GPS, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false);

%% Close the text file.
fclose('all');

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
T_X = 1e-6 * dataArray_X{:,1};
omega_x = dataArray_X(:,2);
omega_y = dataArray_X(:,3);
omega_z = dataArray_X(:,4);
accel_x = dataArray_X(:,5);
accel_y = dataArray_X(:,6);
accel_z = dataArray_X(:,7);
acc_b_x = dataArray_X(:,11);
acc_b_y = dataArray_X(:,12);
acc_b_z = dataArray_X(:,13);
omega = [omega_x{:}, omega_y{:}, omega_z{:}];
accel = [accel_x{:}, accel_y{:}, accel_z{:}];
accel_b = [acc_b_x{:}, acc_b_y{:}, acc_b_z{:}];

T_GPS = dataArray_GPS(:,1);
T_GPS = 1e-6 * T_GPS{:};
LLA_GPS_LO = dataArray_GPS(:,3);
LLA_GPS_LA = dataArray_GPS(:,4);
LLA_GPS_A = dataArray_GPS(:,5);

XYZ_GPS = lla2ecef([LLA_GPS_LO{:}, LLA_GPS_LA{:}, LLA_GPS_A{:}]);
rz = rotz(-LLA_GPS_LA{1}(1)-90);
ry = roty(-LLA_GPS_LO{1}(1));
XYZ_GPS = (rz*ry*(XYZ_GPS - XYZ_GPS(1,:))')';
% XYZ_GPS = XYZ_GPS - XYZ_GPS(1,:);
GPS_unique = zeros(length(XYZ_GPS),1);
GPS_unique(1) = 1;
for i = 2:length(XYZ_GPS)
    GPS_unique(i) = XYZ_GPS(i,1) ~= XYZ_GPS(i-1,1);
end


% load the data into matlab 
[~, ~, ~, ~, x_gps, ~, y_gps, ~, z_gps, ~] = loadGroundTruthAGL();

% plot ground truth positions
%x_gt = x_gt - x_gt(1); y_gt = y_gt - y_gt(1); z_gt = z_gt - z_gt(1);
x_gps = x_gps - x_gps(1); y_gps = y_gps - y_gps(1); z_gps = z_gps - z_gps(1);

[TR, TT] = icp([x_gps,y_gps,z_gps]', XYZ_GPS');
XYZ_GPS = (TR * XYZ_GPS' + TT)';

% plot3(XYZ_GPS(:,1), XYZ_GPS(:,2), XYZ_GPS(:,3), '*k'); hold on;
% plot3(x_gt, y_gt, z_gt, '.')
% plot3(x_gps, y_gps, z_gps, 'or');
% plot gps positions


%%
XYZ_GPS = XYZ_GPS(logical(GPS_unique),:);
T_GPS = T_GPS(logical(GPS_unique));
%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans...
    omega_x omega_y omega_z accel_x accel_y accel_z acc_b_x acc_b_y acc_b_z...
    LLA_GPS_LO LLA_GPS_LA LLA_GPS_A;
end