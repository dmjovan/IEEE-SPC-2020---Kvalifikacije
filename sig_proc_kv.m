%% ilustracija kretanja drona (ne trazi se u zadatku)
clear all;
close all;
clc;

msg = DataExtract('03_Dataset with 5 normal experiments_17Jan2020\2020-01-17-11-32-12.bag','/mavros/global_position/raw/fix');
x = zeros(1,length(msg));
y = zeros(1,length(msg));
z = zeros(1,length(msg));

for i = 1:length(msg)

x(i) = msg{i}.Latitude; % geografska sirina prema GPS merenju
y(i) = msg{i}.Longitude; % geografska duzina prema GPS merenju
z(i) = msg{i}.Altitude; % apsolutna nadmorska visina prema GPS merenju

end

figure
    plot(x,y)
    text(x(1),y(1),'start')
    text(x(length(x)),y(length(y)),'finish')
    xlabel('geografska sirina [°]')
    ylabel('geografska duzina [°]')
    grid on
    title('GPS merenje - kretanje drona u ravni')
figure
    plot3(x,y,z)
    text(x(1),y(1),z(1), 'start')
    text(x(length(x)),y(length(y)),z(length(z)),'finish')
    xlabel('geografska sirina [°]')
    ylabel('geografska duzina [°]')
    zlabel('apsolutna nadmorska visina [m]')
    grid on
    title('GPS merenje - kretanje drona u prostoru')
    
   
%% vizuelizacija 
close all;
clc;

norm_string_1 = '03_Dataset with 5 normal experiments_17Jan2020\2020-01-17-11-32-12.bag';
norm_string_2 = '03_Dataset with 5 normal experiments_17Jan2020\2020-01-17-11-32-49.bag';
norm_string_3 = '03_Dataset with 5 normal experiments_17Jan2020\2020-01-17-11-33-26.bag';
norm_string_4 = '03_Dataset with 5 normal experiments_17Jan2020\2020-01-17-11-34-08.bag';
norm_string_5 = '03_Dataset with 5 normal experiments_17Jan2020\2020-01-17-11-34-43.bag';

abnorm_string_1 = '04_Dataset with 5 abnormal experiments_17Jan2020\2020-01-17-11-35-27.bag';
abnorm_string_2 = '04_Dataset with 5 abnormal experiments_17Jan2020\2020-01-17-11-36-03.bag';
abnorm_string_3 = '04_Dataset with 5 abnormal experiments_17Jan2020\2020-01-17-11-36-43.bag';
abnorm_string_4 = '04_Dataset with 5 abnormal experiments_17Jan2020\2020-01-17-11-37-25.bag';
abnorm_string_5 = '04_Dataset with 5 abnormal experiments_17Jan2020\2020-01-17-11-38-07.bag';

normal_flag = 1;
abnormal_flag = 0;

makeVideo(norm_string_1,normal_flag);
makeVideo(norm_string_2,normal_flag);
makeVideo(norm_string_3,normal_flag);
makeVideo(norm_string_4,normal_flag');
makeVideo(norm_string_5,normal_flag);

makeVideo(abnorm_string_1,abnormal_flag);
makeVideo(abnorm_string_2,abnormal_flag);
makeVideo(abnorm_string_3,abnormal_flag);
makeVideo(abnorm_string_4,abnormal_flag);
makeVideo(abnorm_string_5,abnormal_flag);

%% struktura podataka 
close all;
clc;

initial_01_string = '01_Initial data set for trainning_22 Nov 2019\IMU_camera Drone Synchronized training dataset_normal behabiour_no abnormalities.bag';
initial_02_string = '02_Initial data set for abnormalities training_2 Dec 2019\IMU_camera_Initial data set for abnormalities training_2 Dec 2019.bag';

glo_vel_mes_n = DataExtract(initial_01_string,'/mavros/global_position/raw/gps_vel');
v_glo_n = zeros(1,length(glo_vel_mes_n)); % intenzitet brzine normalnih eks. merene pomocu GPS-a (globalna)

for i = 1:length(glo_vel_mes_n)
    vx_n = glo_vel_mes_n{i}.Twist.Linear.X;
    vy_n = glo_vel_mes_n{i}.Twist.Linear.Y;
    vz_n = glo_vel_mes_n{i}.Twist.Linear.Z;
    v_glo_n(i) = sqrt(vx_n^2 + vy_n^2 + vz_n^2);
end

glo_pos_mes_n = DataExtract(initial_01_string,'/mavros/global_position/local');
pos_glo_n_x = zeros(1,length(glo_pos_mes_n)); % x-koordinata pozicije drona u abnormalnom eks. merena pomocu GPS-a (globalna)
pos_glo_n_y = zeros(1,length(glo_pos_mes_n)); % y-koordinata pozicije drona u abnormalnom eks. merena pomocu GPS-a (globalna)
pos_glo_n_z = zeros(1,length(glo_pos_mes_n)); % z-koordinata pozicije drona u abnormalnom eks. merena pomocu GPS-a (globalna)

for i = 1:length(glo_pos_mes_n)
    pos_glo_n_x(i) = glo_pos_mes_n{i}.Pose.Pose.Position.X;
    pos_glo_n_y(i) = glo_pos_mes_n{i}.Pose.Pose.Position.Y;
    pos_glo_n_z(i) = glo_pos_mes_n{i}.Pose.Pose.Position.Z;
end

loc_vel_mes_n = DataExtract(initial_01_string,'/mavros/local_position/velocity_body');
v_loc_n = zeros(1,length(loc_vel_mes_n)); % intenzitet brzine normalnih eks. merene pomocu senzora na dronu (lokalna)

for i = 1:length(loc_vel_mes_n)
    vx_n = loc_vel_mes_n{i}.Twist.Linear.X;
    vy_n = loc_vel_mes_n{i}.Twist.Linear.Y;
    vz_n = loc_vel_mes_n{i}.Twist.Linear.Z;
    v_loc_n(i) = sqrt(vx_n^2 + vy_n^2 + vz_n^2);
end

loc_pos_mes_n = DataExtract(initial_01_string,'/mavros/local_position/pose');
pos_loc_n_x = zeros(1,length(loc_pos_mes_n)); % x-koordinata pozicije drona u abnormalnom eks. merena pomocu senzora na dronu (lokalna)
pos_loc_n_y = zeros(1,length(loc_pos_mes_n)); % y-koordinata pozicije drona u abnormalnom eks. merena pomocu senzora na dronu (lokalna)
pos_loc_n_z = zeros(1,length(loc_pos_mes_n)); % z-koordinata pozicije drona u abnormalnom eks. merena pomocu senzora na dronu (lokalna)

for i = 1:length(loc_pos_mes_n)
    pos_loc_n_x(i) = loc_pos_mes_n{i}.Pose.Position.X;
    pos_loc_n_y(i) = loc_pos_mes_n{i}.Pose.Position.Y;
    pos_loc_n_z(i) = loc_pos_mes_n{i}.Pose.Position.Z;
end

glo_vel_mes_ab = DataExtract(initial_02_string,'/mavros/global_position/raw/gps_vel');
v_glo_ab = zeros(1,length(glo_vel_mes_ab)); % intenzitet brzine abnormalnih eks. merene pomocu GPS-a (globalna)

for i = 1:length(glo_vel_mes_ab)
    vx_ab = glo_vel_mes_ab{i}.Twist.Linear.X;
    vy_ab = glo_vel_mes_ab{i}.Twist.Linear.Y;
    vz_ab = glo_vel_mes_ab{i}.Twist.Linear.Z;
    v_glo_ab(i) = sqrt(vx_ab^2 + vy_ab^2 + vz_ab^2);
end

glo_pos_mes_ab = DataExtract(initial_02_string,'/mavros/global_position/local');
pos_glo_ab_x = zeros(1,length(glo_pos_mes_ab)); % x-koordinata pozicije drona u abnormalnom eks. merena pomocu GPS-a (globalna)
pos_glo_ab_y = zeros(1,length(glo_pos_mes_ab)); % y-koordinata pozicije drona u abnormalnom eks. merena pomocu GPS-a (globalna)
pos_glo_ab_z = zeros(1,length(glo_pos_mes_ab)); % z-koordinata pozicije drona u abnormalnom eks. merena pomocu GPS-a (globalna)

for i = 1:length(glo_pos_mes_ab)
    pos_glo_ab_x(i) = glo_pos_mes_ab{i}.Pose.Pose.Position.X;
    pos_glo_ab_y(i) = glo_pos_mes_ab{i}.Pose.Pose.Position.Y;
    pos_glo_ab_z(i) = glo_pos_mes_ab{i}.Pose.Pose.Position.Z;
end

loc_vel_mes_ab = DataExtract(initial_02_string,'/mavros/local_position/velocity_body');
v_loc_ab = zeros(1,length(loc_vel_mes_ab)); % intenzitet brzine abnormalnih eks. merene pomocu senzora na dronu (lokalna)

for i = 1:length(loc_vel_mes_ab)
    vx_ab = loc_vel_mes_ab{i}.Twist.Linear.X;
    vy_ab = loc_vel_mes_ab{i}.Twist.Linear.Y;
    vz_ab = loc_vel_mes_ab{i}.Twist.Linear.Z;
    v_loc_ab(i) = sqrt(vx_ab^2 + vy_ab^2 + vz_ab^2);
end

loc_pos_mes_ab = DataExtract(initial_02_string,'/mavros/local_position/pose');
pos_loc_ab_x = zeros(1,length(loc_pos_mes_ab)); % x-koordinata pozicije drona u abnormalnom eks. merena pomocu senzora na dronu (lokalna)
pos_loc_ab_y = zeros(1,length(loc_pos_mes_ab)); % y-koordinata pozicije drona u abnormalnom eks. merena pomocu senzora na dronu (lokalna)
pos_loc_ab_z = zeros(1,length(loc_pos_mes_ab)); % z-koordinata pozicije drona u abnormalnom eks. merena pomocu senzora na dronu (lokalna)

for i = 1:length(loc_pos_mes_ab)
    pos_loc_ab_x(i) = loc_pos_mes_ab{i}.Pose.Position.X;
    pos_loc_ab_y(i) = loc_pos_mes_ab{i}.Pose.Position.Y;
    pos_loc_ab_z(i) = loc_pos_mes_ab{i}.Pose.Position.Z;
end

figure
    subplot(211)
    plot3(pos_loc_n_x, pos_loc_n_y, pos_loc_n_z)
    text(pos_loc_n_x(1),pos_loc_n_y(1),pos_loc_n_z(1), 'start')
    text(pos_loc_n_x(length(pos_loc_n_x)),pos_loc_n_y(length(pos_loc_n_y)),pos_loc_n_z(length(pos_loc_n_z)),'finish')
    xlabel('x-coord [m]')
    ylabel('y-coord [m]')
    zlabel('z-coord [m]')
    grid on
    title('Local-data pozicija za regularne slucajeve')
    subplot(212)
    plot3(pos_glo_n_x,pos_glo_n_y,pos_glo_n_z)
    text(pos_glo_n_x(1),pos_glo_n_y(1),pos_glo_n_z(1), 'start')
    text(pos_glo_n_x(length(pos_glo_n_x)),pos_glo_n_y(length(pos_glo_n_y)),pos_glo_n_z(length(pos_glo_n_z)),'finish')
    xlabel('x-coord [m]')
    ylabel('y-coord [m]')
    zlabel('z-coord [m]')
    grid on
    title('Global-data pozicija za regularne slucajeve')
    
figure
    plot(v_loc_n)
    xlabel('time [s]')
    ylabel('velocity [m/s]')
    hold on
    plot(v_glo_n(1:length(v_loc_n)))
    hold off
    grid on
    title('Poredjenje global/local brzina za regularne slucajeve')
    legend('lokalna brzina', 'globalna brzina')
    
figure
    subplot(211)
    plot3(pos_loc_ab_x, pos_loc_ab_y, pos_loc_ab_z)
    text(pos_loc_ab_x(1),pos_loc_ab_y(1),pos_loc_ab_z(1), 'start')
    text(pos_loc_ab_x(length(pos_loc_ab_x)),pos_loc_ab_y(length(pos_loc_ab_y)),pos_loc_ab_z(length(pos_loc_ab_z)),'finish')
    xlabel('x-coord [m]')
    ylabel('y-coord [m]')
    zlabel('z-coord [m]')
    title('Local-data pozicija za neregularne slucajeve')
    grid on
    subplot(212)
    plot3(pos_glo_ab_x,pos_glo_ab_y,pos_glo_ab_z)
    text(pos_glo_ab_x(1),pos_glo_ab_y(1),pos_glo_ab_z(1), 'start')
    text(pos_glo_ab_x(length(pos_glo_ab_x)),pos_glo_ab_y(length(pos_glo_ab_y)),pos_glo_ab_z(length(pos_glo_ab_z)),'finish')
    xlabel('x-coord [m]')
    ylabel('y-coord [m]')
    zlabel('z-coord [m]')
    title('Global-data pozicija za neregularne slucajeve')
    grid on
    
figure
    plot(v_loc_ab)
    xlabel('time [s]')
    ylabel('velocity [m/s]')
    hold on
    plot(v_glo_ab(1:length(v_loc_ab)))
    hold off
    grid on
    title('Poredjenje global/local brzina za neregularne slucajeve')
    legend('lokalna brzina', 'globalna brzina')


%% klasifikacija 
close all;
clc;

% podaci za klasifikaciju sa IMU senzora

training_imu{1} = DataExtract(initial_01_string,'/mavros/imu/data');
training_imu{2} = DataExtract(initial_02_string,'/mavros/imu/data');
training_sensor{1} = DataExtract(initial_01_string,'/mavros/global_position/local');
training_sensor{2} = DataExtract(initial_02_string,'/mavros/global_position/local');

w_diff_tr = zeros(1,2);
w_diff_tr(1) = AngularVelocityDiff(training_imu{1});
w_diff_tr(2) = AngularVelocityDiff(training_imu{2});
ang_first_diff_tr = zeros(3,2);
ang_first_diff_tr(:,1) = AnglesFirstDiff(training_imu{1});
ang_first_diff_tr(:,2) = AnglesFirstDiff(training_imu{2});
ang_diff_tr = zeros(3,2);
ang_diff_tr(:,1) = AnglesDiff(training_imu{1});
ang_diff_tr(:,2) = AnglesDiff(training_imu{2});
acc_diff_tr = zeros(1,2);
acc_diff_tr(1) = LinearAccDiff(training_imu{1});
acc_diff_tr(2) = LinearAccDiff(training_imu{2});
v_diff_tr = zeros(1,2);
v_diff_tr(1) = SensorLinearVelocityDiff(training_sensor{1});
v_diff_tr(2) = SensorLinearVelocityDiff(training_sensor{2});

% normal
imu{1} = DataExtract(norm_string_1,'/mavros/imu/data');
imu{2} = DataExtract(norm_string_3,'/mavros/imu/data');
imu{3} = DataExtract(norm_string_3,'/mavros/imu/data');
imu{4} = DataExtract(norm_string_4,'/mavros/imu/data');
imu{5} = DataExtract(norm_string_5,'/mavros/imu/data');
% abnormal
imu{6} = DataExtract(abnorm_string_1,'/mavros/imu/data');
imu{7} = DataExtract(abnorm_string_2,'/mavros/imu/data');
imu{8} = DataExtract(abnorm_string_3,'/mavros/imu/data');
imu{9} = DataExtract(abnorm_string_4,'/mavros/imu/data');
imu{10} = DataExtract(abnorm_string_5,'/mavros/imu/data');

% podaci za klasifikaciju sa drugog senzora

% normal
sensor{1} = DataExtract(norm_string_1,'/mavros/global_position/local');
sensor{2} = DataExtract(norm_string_3,'/mavros/global_position/local');
sensor{3} = DataExtract(norm_string_3,'/mavros/global_position/local');
sensor{4} = DataExtract(norm_string_4,'/mavros/global_position/local');
sensor{5} = DataExtract(norm_string_5,'/mavros/global_position/local');
% abnormal
sensor{6} = DataExtract(abnorm_string_1,'/mavros/global_position/local');
sensor{7} = DataExtract(abnorm_string_1,'/mavros/global_position/local');
sensor{8} = DataExtract(abnorm_string_3,'/mavros/global_position/local');
sensor{9} = DataExtract(abnorm_string_4,'/mavros/global_position/local');
sensor{10} = DataExtract(abnorm_string_5,'/mavros/global_position/local');

% maksimalna apsolutna razlika izmedju uzastopnih ugaonih brzina (IMU)

w_diff = zeros(1,length(imu));
for i = 1:length(imu)
    w_diff(i) = AngularVelocityDiff(imu{i});
end

% maksimalna apsolutna ugaona odstupanja od pocetnog ugla(IMU)

ang_first_diff = zeros(3,length(imu));
for i = 1:length(imu)
    ang_first_diff(:,i) = AnglesFirstDiff(imu{i});
end

% maksimalna apsolutna razlika izmedju uzastopnih uglova orjentaciju(IMU)

ang_diff = zeros(3,length(imu));
for i = 1:length(imu)
    ang_diff(:,i) = AnglesDiff(imu{i});
end

% maksimalna apsolutna razlika izmedju uzastopnih linearnih ubrzanja (IMU)

acc_diff = zeros(1,length(imu));
for i = 1:length(imu)
    acc_diff(i) = LinearAccDiff(imu{i});
end

% maksimalna apsolutna razlika izmedju uzastopnih linearnih brzina (GPS merenje)

v_diff = zeros(1,length(sensor));
for i = 1:length(sensor)
    v_diff(i) = SensorLinearVelocityDiff(sensor{i});
end

% provera regularnosti snimaka na osnovu srednje vrednosti svih snimaka

angular_velocity_check = NormalityCheck(w_diff);
angle_first_1_check = NormalityCheck(ang_first_diff(1,:));
angle_first_2_check = NormalityCheck(ang_first_diff(2,:));
angle_first_3_check = NormalityCheck(ang_first_diff(3,:));
angle_1_check = NormalityCheck(ang_diff(1,:));
angle_2_check = NormalityCheck(ang_diff(2,:));
angle_3_check = NormalityCheck(ang_diff(3,:));
acceleration_check = NormalityCheck(acc_diff);
velocity_check = NormalityCheck(v_diff);

check_matrix = [angular_velocity_check;...
                angle_first_1_check;...
                angle_first_2_check;...
                angle_first_3_check;...
                angle_1_check;...
                angle_2_check;...
                angle_3_check;...
                acceleration_check;...
                velocity_check ];
            
check_vector_1 = zeros(1,length(check_matrix)); % vektor provere prema vecini normalnih
for i = 1: length(check_matrix)                 
    if mean(check_matrix(:,i)) >= 0.5
        check_vector_1(i) = 1; % 1 = regularan, 0 = neregularan
    end
end

check_vector_2 = zeros(1,length(check_matrix)); % vektor provere prema iskljucenju abnormalnih
for i = 1: length(check_matrix)                
    if prod(check_matrix(:,i)) == 1
        check_vector_2(i) = 1; % 1 = regularan, 0 = neregularan
    end
end

% provera regulranosti snimaka na osnovu apsolutne razlike merenja sa
% treninga i nasih merenja

angular_velocity_training_check = TrainingNormalityCheck(w_diff_tr,w_diff);
angle_first_1_training_check = TrainingNormalityCheck(ang_first_diff_tr(1,:),ang_first_diff(1,:));
angle_first_2_training_check = TrainingNormalityCheck(ang_first_diff_tr(2,:),ang_first_diff(2,:));
angle_first_3_training_check = TrainingNormalityCheck(ang_first_diff_tr(3,:),ang_first_diff(3,:));
angle_1_training_check = TrainingNormalityCheck(ang_diff_tr(1,:),ang_diff(1,:));
angle_2_training_check = TrainingNormalityCheck(ang_diff_tr(2,:),ang_diff(2,:));
angle_3_training_check = TrainingNormalityCheck(ang_diff_tr(3,:),ang_diff(3,:));
acceleration_training_check = TrainingNormalityCheck(acc_diff_tr,acc_diff);
velocity_training_check = TrainingNormalityCheck(v_diff_tr,v_diff);

check_matrix_training = [angular_velocity_training_check;...
                angle_first_1_training_check;...
                angle_first_2_training_check;...
                angle_first_3_training_check;...
                angle_1_training_check;...
                angle_2_training_check;...
                angle_3_training_check;...
                acceleration_training_check;...
                velocity_training_check ];
            
check_vector_1_training = zeros(1,length(check_matrix_training)); % vektor provere prema vecini normalnih i prema treningu
for i = 1: length(check_matrix_training)                         
    if mean(check_matrix_training(:,i)) >= 0.5
        check_vector_1_training(i) = 1; % 1 = regularan, 0 = neregularan
    end
end

check_vector_2_training = zeros(1,length(check_matrix_training)); % vektor provere prema iskljucenju abnormalnih i prema treningu
for i = 1: length(check_matrix_training)                         
    if prod(check_matrix_training(:,i)) == 1
        check_vector_2_training(i) = 1; % 1 = regularan, 0 = neregularan
    end
end

% uputstvo za citanje check_vector-a:
% svaki check_vector ima 1 vrtu i 10 kolona, od kojih svaka kolona
% predstavlja ispravnost jednog snimka (podataka iz jednog .bag fajla);
% prvih pet vrednosti se odnose na ispravnost podataka iz 03_Dataset
% foldera sa .bag fajlovima, ostalih pet vrednosti se odnose na ispravnost 
% podataka iz 04_Dataset foldera sa .bag fajlovima
