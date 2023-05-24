function D = calibrateSensorData(data,calibration_info,sensorname)

% Extract data from table
gyro = table2array(data(:,8:10))/180*pi; % NB in rad/s
acc = table2array(data(:,2:4));
mag = table2array(data(:,5:7));
time = table2array(data(:,1));

% ------- remove gyro offset ------
gyro_offset = calibration_info.(sensorname).gyrobias/180*pi;
gyro = gyro - gyro_offset;

% ------- calibrate magnetometer -----
A       = calibration_info.(sensorname).magcal.A;
b       = calibration_info.(sensorname).magcal.b;
scale   = calibration_info.(sensorname).magcal.scale;

% calibrate
mag =(mag-b)*A;
% scale to have uT
mag = mag*scale;

% swap the axes x = -z; y = -y; z = -x
gyro=[-gyro(:,3) -gyro(:,2) -gyro(:,1)];
acc=[-acc(:,3) -acc(:,2) -acc(:,1)];
mag=[-mag(:,3) -mag(:,2) -mag(:,1)];

% Output as structure
D.gyro = gyro;
D.acc = acc;
D.mag = mag;
D.time = time;