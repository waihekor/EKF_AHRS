% 基于EKF的IMU姿态解算(ADIS16470)
% 导航坐标：东北天坐标系(ENU)
% ADIS16470载体坐标系:X:右; Y:前; Z:上 ,逆时针欧拉角角度为正
% X:pitch; Y:roll; Z:yaw;
% 旋转顺序：z,x,y(偏航、俯仰，滚转)
% 备注：加速度数据以g为单位，不是以m/s2为单位
clc;
clear;
close all;  
addpath('utils');
addpath('datasets');
rad2deg = 180/pi;   
deg2rad = pi/180;   
%% 读取数据
filename = 'ADIS16465-rm3100-2020-01-11-18-57-yaw90.txt';
nav_data = load(filename);
gap=1;
data_num=round(size(nav_data,1));
j=0;
for i=1:gap:data_num-gap
    j=j+1;
    DATA_CNTR(j,1) = mean(nav_data(i:i+gap-1,1)); %取值范围：0-65535
    X_ACCL(j,1) = mean(nav_data(i:i+gap-1,2));    %单位:g
    Y_ACCL(j,1) = mean(nav_data(i:i+gap-1,3));
    Z_ACCL(j,1) = mean(nav_data(i:i+gap-1,4));
    X_GYRO(j,1) = mean(nav_data(i:i+gap-1,5))/4;
    Y_GYRO(j,1) = mean(nav_data(i:i+gap-1,6))/4;
    Z_GYRO(j,1) = mean(nav_data(i:i+gap-1,7))/4;
    TEMP_OUT(j,1) = mean(nav_data(i:i+gap-1,8));
    X_MAG(j,1) = mean(nav_data(i:i+gap-1,9));
    Y_MAG(j,1) = mean(nav_data(i:i+gap-1,10));
    Z_MAG(j,1) = mean(nav_data(i:i+gap-1,11));
    GAP_TIME(j,1) = mean(nav_data(i:i+gap-1,12))/1000000;  %单位：us
end

%% 初始化数据
L = size(DATA_CNTR,1);
Time=zeros(L,1);
Time(1,1)=0;

acc = [X_ACCL,Y_ACCL,Z_ACCL];
gyro_bias = [0,0,0];  
gyro = [(X_GYRO+gyro_bias(1))*deg2rad,(Y_GYRO+gyro_bias(2))*deg2rad,(Z_GYRO+gyro_bias(3))*deg2rad]; % gyro 单位：deg/sec
mag = [X_MAG,Y_MAG,Z_MAG];       

wn_var  = 1e-6 * ones(1,4);               
wbn_var = 1e-8 * ones(1,3);             
an_var  = 1e-1 * ones(1,3);              
Q = diag([wn_var, wbn_var]); 
R = diag(an_var);         

% EKF递推(AHRS)
g = 9.87;
acce_data0 = [X_ACCL(1),Y_ACCL(1),Z_ACCL(1)];
mag_data0=[X_MAG(1),Y_MAG(1),Z_MAG(1)];
[pitchEKF0,rollEKF0,yawEKF0] = Get_Init_AHRS(acce_data0,mag_data0);
[q0,q1,q2,q3]=quatfromeuler(pitchEKF0,rollEKF0,yawEKF0);

pitchEKF(1,1) = pitchEKF0*rad2deg;
rollEKF(1,1) = rollEKF0*rad2deg;
yawEKF(1,1) = yawEKF0*rad2deg;

pitch_gyro(1,1) = pitchEKF0*rad2deg;
roll_gyro(1,1) = rollEKF0*rad2deg;
yaw_gyro(1,1) = yawEKF0*rad2deg;
x_gyro = zeros(L,4); 
x_gyro(1,:) = [q0,q1,q2,q3]; 
q_gyro(1,:) = [q0,q1,q2,q3];  

x = zeros(L,7);   
Pk = zeros(7,7,L);
x(1,:) = [q0,q1,q2,q3,0,0,0];   
Pk(:,:,1) = eye(7); 

% 磁场融合参数
yaw_mag = zeros(L,1);
yaw_mag(1,1) = yawEKF(1,1);
z_bias = zeros(L,1);
P_mag = zeros(2,2,L);
Q_mag = diag([1e-6,1e-8]);
R_mag= 1e-3;
x_mag = zeros(L,2); 
%% 解算过程
for k=1:L-1
    T = abs(GAP_TIME(k+1,1));
    Time(k+1,1)=Time(k)+T; 
    fprintf("i = %d,process ratio = %f\n",k,k/L);
    
    tic;
    acc(k,:) = acc(k,:)/norm(acc(k,:),2);
    z(k,:) = acc(k,:); 

    if(k==1)
        x_gyro(k,:)=x(1,1:4);
    else
        
        A_gyro=[1,-(T/2)*gyro(k,1),-(T/2)*gyro(k,2),-(T/2)*gyro(k,3);
                (T/2)*gyro(k,1),1,(T/2)*gyro(k,3),-(T/2)*gyro(k,2);
                (T/2)*gyro(k,2),-(T/2)*gyro(k,3),1,(T/2)*gyro(k,1);
               (T/2)*gyro(k,3),(T/2)*gyro(k,2),-(T/2)*gyro(k,1),1];

        x_gyro(k,:)= (A_gyro*x_gyro(k-1,:)')';   
        x_gyro(k,:) = x_gyro(k,:)/norm(x_gyro(k,:),2); 
        [pitch_gyro(k,1),roll_gyro(k,1),yaw_gyro(k,1)] = quattoeuler(x_gyro(k,:));  
    end

    gyro_x_bias = gyro(k,1)-x(k,5);
    gyro_y_bias = gyro(k,2)-x(k,6);
    gyro_z_bias = gyro(k,3);
    
    A_11=[1,-(T/2)*gyro_x_bias,-(T/2)*gyro_y_bias,-(T/2)*gyro_z_bias;
            (T/2)*gyro_x_bias,1,(T/2)*gyro_z_bias,-(T/2)*gyro_y_bias;
            (T/2)*gyro_y_bias,-(T/2)*gyro_z_bias,1,(T/2)*gyro_x_bias;
           (T/2)*gyro_z_bias,(T/2)*gyro_y_bias,-(T/2)*gyro_x_bias,1];
    
    A_12=[0,0,0;
          0,0,0;
          0,0,0;
          0,0,0]; 
    A_21=[0,0,0,0;
          0,0,0,0;
          0,0,0,0];
    A_22=[1,0,0;
          0,1,0;
          0,0,1];   
    A=[A_11,A_12;A_21,A_22];
    
     Ak = eye(7)+T/2*...
         [0    -(gyro_x_bias)  -(gyro_y_bias) -(gyro_z_bias)   x(k,2) x(k,3)  x(k,4);
          (gyro_x_bias) 0    (gyro_z_bias)  -(gyro_y_bias)   -x(k,1) x(k,4)  -x(k,3);
          (gyro_y_bias) -(gyro_z_bias)  0  (gyro_x_bias)      -x(k,4) -x(k,1) x(k,2);
          (gyro_z_bias) (gyro_y_bias)   -(gyro_x_bias)  0     x(k,3) -x(k,2) -x(k,1);
          0 0 0 0 0 0 0;
          0 0 0 0 0 0 0;
          0 0 0 0 0 0 0];
      
    x_ = (A*x(k,:)')';  
    x_(1:4) = x_(1:4)/norm(x_(1:4),2); 
   
    Pk_ = Ak * Pk(:,:,k) * Ak' + Q; 
    hk = [2*(x_(2)*x_(4)-x_(1)*x_(3)) 2*(x_(1)*x_(2)+x_(3)*x_(4)) x_(1)^2+x_(4)^2-x_(2)^2-x_(3)^2]; 
    Hk = 2*[-x_(3)  x_(4) -x_(1)  x_(2) 0 0 0;
               x_(2)  x_(1)  x_(4)  x_(3) 0 0 0;
               x_(1) -x_(2) -x_(3)  x_(4) 0 0 0];

    Kk = Pk_ * Hk' * ((Hk * Pk_ * Hk' + R)^(-1));  
    x(k+1,:) = (x_' + Kk * (z(k,:) - hk)')';      
    x(k+1,1:4) = x(k+1,1:4)/norm(x(k+1,1:4),2);  
    Pk(:,:,k+1) = (eye(7) - Kk*Hk) * Pk_;       
    q=[x(k,1),x(k,2),x(k,3),x(k,4)];
    
   [pitchEKF(k,1),rollEKF(k,1),yawEKF(k,1)] = quattoeuler(q);
   mag_data = mag(k,:);
   yawEKF(k,1) = Get_Yaw(pitchEKF(k,1),rollEKF(k,1),mag_data);
   
   %加入磁力计的融合算法 
   A_2 = [1,-T;0,1];
   H_2 = [1,0];
   yaw_mag(k+1,1) = CalEndAngle_Zcoord(yaw_mag(k,1),(gyro(k,3)-z_bias(k,1))*T );
   z_bias(k+1,1) = z_bias(k,1); 
   x_mag(k+1,:) = [yaw_mag(k+1,1),z_bias(k+1,1)];
   P_22 = A_2 * P_mag(:,:,k) * A_2' + Q_mag; 
   
   z_2(k+1,1) = Get_Yaw(pitchEKF(k,1),rollEKF(k,1),mag_data); 
   
   K_2 = P_22 * H_2' * ((H_2 * P_22 * H_2' + R_mag)^(-1));            
   x_mag(k+1,:) = x_mag(k+1,:)+ (K_2 * (z_2(k+1,:) - x_mag(k+1,1)))';     
   P_mag(:,:,k+1) = (eye(2) - K_2*H_2) * P_22;      
    
    yaw_mag(k+1,1) = x_mag(k+1,1);
    z_bias(k+1,1) = x_mag(k+1,2);
    
   if(abs(z_bias(k+1,1))>0.1)
     z_bias(k+1,1)=0.1 ;
   end
   
end

%% 画图
end_num = min([size(Time,1),size(pitchEKF,1),size(yaw_mag,1)]);
gap_plot = 1;

figure;
pitch_ekf_plot = plot(Time(1:gap_plot:end_num),pitchEKF(1:gap_plot:end_num),'b-','LineWidth',2);
legend([pitch_ekf_plot],{'pitch-ekf'},'FontSize',10);
xlabel('t / s','FontSize',20)
ylabel('pitch','FontSize',20)
title('pitch','FontSize',20);
    
figure;
roll_ekf_plot = plot(Time(1:gap_plot:end_num),rollEKF(1:gap_plot:end_num),'b-','LineWidth',2);
legend([roll_ekf_plot],{'roll-ekf'},'FontSize',10);
xlabel('t / s','FontSize',20)
ylabel('roll','FontSize',20)
title('roll','FontSize',20);

figure;
yaw_gyro_plot = plot(Time(1:gap_plot:end_num),yaw_gyro(1:gap_plot:end_num),'r-','LineWidth',2);
hold on;
yaw_mag_plot = plot(Time(1:gap_plot:end_num),yawEKF(1:gap_plot:end_num),'g-','LineWidth',2);
hold on;
new_yaw_plot = plot(Time(1:gap_plot:end_num),yaw_mag(1:gap_plot:end_num),'b-','LineWidth',2);
legend([yaw_gyro_plot,yaw_mag_plot,new_yaw_plot],{'yaw-gyro-plot','yaw-mag-plot','new-yaw-plot'},'FontSize',10);
xlabel('t / s','FontSize',20)
ylabel('yaw','FontSize',20)
title('yaw','FontSize',20);
