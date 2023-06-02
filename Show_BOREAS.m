% Show_BOREAS is a MATLAB script that shows a series of LiDAR, RADAR, and
% Camera frames, obtained from the BOREAS Dataset (road car driving in
% adverse weather conditions). To use it, run the script after defining the
% sensor data locations after the cleanup section. This script can be used
% to study the sensor modalities using MATLAB inbuilt functions.
%
% Creative Commons License: CC BY-NC: This license allows reusers to 
%                           distribute, remix, adapt, and build upon
%                           the material in any medium or format for
%                           noncommercial purposes only, and only so
%                           long as attribution is given to the creator. 
%
% Please cite this work as follows:
% --------------------------
% Dunsten M. X. Dsouza, Nader J. Abu-Alrub, Nathir A. Rawashdeh, "A MATLAB
% Script for Visualizing and Processing BOREAS Dataset Road Driving Sensor
% Data: LiDAR, Radar, and Camera", Michigan Technological University, 
% DOI: 10.13140/RG.2.2.34186.24008., 2023, ResearchGate
%
% ----------------------------------
% @article{Dsouza2023_MATLAB_BOREAS,
%   title={A MATLAB Script for Visualizing and Processing BOREAS Dataset Road Driving Sensor Data: LiDAR, Radar, and Camera},
%   author={Dsouza, Dunsten M X and Abu-Alrub, Nader J and Rawashdeh, Nathir A},
%   journal={Michigan Technological University}
%   year={2023},
%   doi = {10.13140/RG.2.2.34186.24008},
%   publisher={ResearchGate}
% }



%% Clean up
clc, clear all, close all

%% Define sensor sub-folders or paths: LiDAR, RADAR, and Camera
path_2_lidar_scans =  'Boreas Dataset\lidar';
path_2_radar_scans =  'Boreas Dataset\radar\cart';
path_2_camera_scans = 'Boreas Dataset\camera';

%% Read sensor frame files: LiDAR, RADAR, and Camera
disp('LIDAR frames ...')
lidarfiles = dir([path_2_lidar_scans '\*.bin']);

disp('RADAR frames ...')
radarfiles = dir([path_2_radar_scans '\*.png']);

disp('CAEMRA frames ...')
camerafiles = dir([path_2_camera_scans '\*.png']);


%% Find times of FIRST frames in each sensor
file_name_lidar = lidarfiles(1).name; % first lidar frame
LidarUnixTimeinSTR = split(file_name_lidar,'.bin');
LidarUnixTimeinINT = str2double(string(LidarUnixTimeinSTR(1))) % unix time
LidarUnix2dateTime = datetime(LidarUnixTimeinINT,'ConvertFrom',...
    'epochtime','TicksPerSecond',1e6,'Format','dd-MMM-yyyy HH:mm:ss.SSS');

file_name_radar = radarfiles(1).name; % first radar frame
RadarUnixTimeinSTR = split(file_name_radar,'.png');
RadarUnixTimeinINT = str2double(string(RadarUnixTimeinSTR(1))) % unix time
RadarUnix2dateTime = datetime(RadarUnixTimeinINT,'ConvertFrom',...
    'epochtime','TicksPerSecond',1e6,'Format','dd-MMM-yyyy HH:mm:ss.SSS');

file_name_camera = camerafiles(1).name; % first camera frame
CameraUnixTimeinSTR = split(file_name_camera,'.png');
CameraUnixTimeinINT = str2double(string(CameraUnixTimeinSTR(1))) % unix time
CameraUnix2dateTime = datetime(CameraUnixTimeinINT,'ConvertFrom',...
    'epochtime','TicksPerSecond',1e6,'Format','dd-MMM-yyyy HH:mm:ss.SSS');

% Define display loop START time
t_start = min([LidarUnixTimeinINT,RadarUnixTimeinINT,CameraUnixTimeinINT])


%% Find times of LAST frames in each sensor
file_name_lidar = lidarfiles(length(lidarfiles)).name; % last lidar frame
LidarUnixTimeinSTR = split(file_name_lidar,'.bin');
LidarUnixTimeinINT = str2double(string(LidarUnixTimeinSTR(1))); % unix time
LidarUnix2dateTime = datetime(LidarUnixTimeinINT,'ConvertFrom',...
    'epochtime','TicksPerSecond',1e6,'Format','dd-MMM-yyyy HH:mm:ss.SSS');

file_name_radar = radarfiles(length(radarfiles)).name; % last radar frame
RadarUnixTimeinSTR = split(file_name_radar,'.png');
RadarUnixTimeinINT = str2double(string(RadarUnixTimeinSTR(1))); % unix time
RadarUnix2dateTime = datetime(RadarUnixTimeinINT,'ConvertFrom',...
    'epochtime','TicksPerSecond',1e6,'Format','dd-MMM-yyyy HH:mm:ss.SSS');

file_name_camera = camerafiles(length(camerafiles)).name; % last camera
CameraUnixTimeinSTR = split(file_name_camera,'.png');
CameraUnixTimeinINT = str2double(string(CameraUnixTimeinSTR(1))); % unix time
CameraUnix2dateTime = datetime(CameraUnixTimeinINT,'ConvertFrom',...
    'epochtime','TicksPerSecond',1e6,'Format','dd-MMM-yyyy HH:mm:ss.SSS');

% Define display loop END time
t_end = max([LidarUnixTimeinINT,RadarUnixTimeinINT,CameraUnixTimeinINT])

%% Time based display loop of sensor frames

% Define index of staritnf frames. These update based on time
lidar_index=1;
radar_index=1;
camera_index=1;

% open a large figure
figure('Position', [20 20 1000 700])

time_presicion = 100000 % Sets when to plot frames (depends on system)
time_step = 100000 % how fine to step over time (in Seconds).
% Time-loop from earliest to latest frame times
for t=t_start : time_step : t_end 

    % LiDAR frame reading
    try
        file_name_lidar = lidarfiles(lidar_index).name;

        fileID_lidar = fopen(strcat(path_2_lidar_scans,'\',file_name_lidar),'r');
        data = fread(fileID_lidar,'float32');
        fclose(fileID_lidar);
    catch
        disp('... no more Lidar frames')
    end
    length(data)/6;
    B_lidar = reshape(data,[6,length(data)/6])';
    ptCloud1 = pointCloud(B_lidar(:,1:3));
    % LiDAR frame time extraction
    LidarUnixTimeinSTR = split(file_name_lidar,'.bin');
    LidarUnixTimeinINT = str2double(string(LidarUnixTimeinSTR(1))); % unix time
    LidarUnix2dateTime = datetime(LidarUnixTimeinINT,'ConvertFrom',...
        'epochtime','TicksPerSecond',1e6,'Format','dd-MMM-yyyy HH:mm:ss.SSS');

    % RADAR frame reading
    try
        file_name_radar = radarfiles(radar_index).name;
        imr=imread([path_2_radar_scans '\' file_name_radar]);
    catch
        disp('... no more RADAR frames')
    end
    % RADAR frame time extraction
    RadarUnixTimeinSTR = split(file_name_radar,'.png');
    RadarUnixTimeinINT = str2double(string(RadarUnixTimeinSTR(1))); % unix time
    RadarUnix2dateTime = datetime(RadarUnixTimeinINT,'ConvertFrom',...
        'epochtime','TicksPerSecond',1e6,'Format','dd-MMM-yyyy HH:mm:ss.SSS');

    % Camera frame reading
    try
        file_name_camera = camerafiles(camera_index).name;
        imc=imread([path_2_camera_scans '\' file_name_camera]);
    catch
        disp('... no more Camera frames')
    end
    % Camera frame time extraction
    CameraUnixTimeinSTR = split(file_name_camera,'.png');
    CameraUnixTimeinINT = str2double(string(CameraUnixTimeinSTR(1))); % unix time
    CameraUnix2dateTime = datetime(CameraUnixTimeinINT,'ConvertFrom',...
        'epochtime','TicksPerSecond',1e6,'Format','dd-MMM-yyyy HH:mm:ss.SSS');

    % Plot lidar frame if the time within precision
    if abs(LidarUnixTimeinINT - t) < time_presicion

        figure(1), subplot(2,2,1), pcshow(ptCloud1);
        axis([0 100 0 100 0 20])
        title(['\color{white}LiDAR time: ' datestr(LidarUnix2dateTime)]);
        lidar_index = lidar_index+1;% increment frame index

    end

    % Plot radar frame if the time is within precision
    if abs(RadarUnixTimeinINT - t) < time_presicion
        figure(1), subplot(2,2,2), imagesc(imr)
        title(['\color{white}Radar time: ' datestr(RadarUnix2dateTime)]);
        radar_index = radar_index+1% increment frame index
    end

    % Plot camera frame if the time is within precision
    if abs(CameraUnixTimeinINT - t) < time_presicion
        figure(1), subplot(2,2,3), imshow(imc);
        title(['\color{white}Camera frame # ' datestr(CameraUnix2dateTime)]);
        camera_index = camera_index+1; % increment frame index
    end
% End time-loop
end

disp('Please see file comments for ciatation');
help Show_BOREAS



