clc;
clear all;
close all;
%% Generate a Coil Trajectory
% This example shows how to generate a coil trajectory using the |kinematicTrajectory| 
% System object™.
% 
% Create a circular trajectory for a 1000 second duration and a sample rate 
% of 10 Hz. Set the radius of the circle to 5000 meters and the speed to 80 meters 
% per second. Set the climb rate to 50 meters per second and the pitch to 15 
% degrees. Specify the initial orientation as pointed in the direction of motion.

duration = 1000; % seconds
fs = 10;         % Hz
N = duration*fs; % number of samples

radius = 5000;   % meters
speed = 80;      % meters per second
% climbRate = 10;  % meters per second
climbRate = 0;
initialYaw = 0; % degrees
pitch = 15;      % degrees
%pitch = 0; 

initPos = [radius, 0, 0];
initVel = [speed, 0, climbRate];

initOrientation = quaternion([initialYaw,pitch,0],'eulerd','zyx','frame');

trajectory = kinematicTrajectory('SampleRate',fs, ...
    'Velocity',initVel, ...
    'Position',initPos, ...
    'Orientation',initOrientation);

% Specify a constant acceleration and angular velocity in the body coordinate 
% system. Rotate the body frame to account for the pitch.

accBody = zeros(N,3);
accBody(:,1) = 0;
accBody(:,2) = speed^2/radius;
 accBody(:,3) = 0.2;
% accBody(:,3) = 0;


angVelBody = zeros(N,3);
angVelBody(:,3) = speed/radius;

pitchRotation = quaternion([0,pitch,0],'eulerd','ZYX','frame');
angVelBody = rotateframe(pitchRotation,angVelBody);
accBody = rotateframe(pitchRotation,accBody);

% Call |trajectory| with the specified acceleration and angular velocity in 
% the body coordinate system. Plot the position, orientation, and speed over time.

[position, orientation, velocity,accNED,angVelNED] = trajectory(accBody,angVelBody);

eulerAngles = eulerd(orientation,'ZYX','frame');
speed = sqrt(sum(velocity.^2,2));

timeVector = (0:(N-1))/fs;

 
% figure(2)
% plot(timeVector,eulerAngles(:,1),...
%      timeVector,eulerAngles(:,2),...
%      timeVector,eulerAngles(:,3))
% axis([0,duration,-180,180])
% legend('Yaw (Rotation Around Down)','Pitch (Rotation Around East)','Roll (Rotation Around North)')
% xlabel('Time (s)')
% ylabel('Rotation (degrees)')
% title('Orientation')



%% 
% Create an |imuSensor| object with an ideal accelerometer and an ideal magnetometer. 
% Call |IMU| with the ground-truth acceleration, angular velocity, and orientation 
% to output accelerometer readings and magnetometer readings. Plot the results.

IMU = imuSensor('accel-gyro-mag','SampleRate',fs);
% change params
IMU.Accelerometer = accelparams('ConstantBias',0.49,'NoiseDensity',3920e-6);
IMU.Gyroscope = gyroparams('ConstantBias',0.03,'NoiseDensity',8.727e-4);

[accelReadings,gyroReadings,magReadings] = IMU(accNED,angVelNED,orientation);

totalNumSamples=N;



orient_measu1=zeros(N,3);


in1=initialYaw*(3.14/180);%yaw
in2=pitch*(3.14/180);%pich
in3=0;%roll


% orient_measu(1,3)=i1*(180/3.14);%roll
% orient_measu(1,2)=i2*(180/3.14);
% orient_measu(1,1)=i3*(180/3.14);%orient_measu(i,1):yaw

eul_init=[in1 in2 in3];
%%computation is in quaternion
quat_init = eul2quat(eul_init,'ZYX');

% orient_measu is vector of euler angle
orient_measu1(1,1)=in1*(180/3.14);
orient_measu1(1,2)=in2*(180/3.14);
orient_measu1(1,3)=in3*(180/3.14);

i1=quat_init(1);
i2=quat_init(2);
i3=quat_init(3);
i4=quat_init(4);
% pg=zeros(N,1);

for i=1:N
    
wx=gyroReadings(i,1);
wy=gyroReadings(i,2);
wz=gyroReadings(i,3);
time=1/fs;
[t,x]=ode23s(@(t,x) find_orient_diff2(t,x,wx,wy,wz),[0,time],[i1,i2,i3,i4]);
i1=x(length(t),1);
i2=x(length(t),2);
i3=x(length(t),3);
i4=x(length(t),4);
quat_variable=[i1 i2 i3 i4];
%eul_variable = quat2eul(quat_variable,'ZYX');
[yaw, pitch, roll] = quat2angle(quat_variable);
% orient_measu(i+1,3)=i1*(180/3.14);%roll
% orient_measu(i+1,2)=i2*(180/3.14);
% orient_measu(i+1,1)=i3*(180/3.14);%orient_measu(i,1):yaw
% 

orient_measu1(i,1)=roll*(-180/3.14);
orient_measu1(i,2)=pitch*(180/3.14);
orient_measu1(i,3)=yaw*(180/3.14);
% pg=orient_measu1(i+1,2);
% rg=orient_measu1(i+1,1);
end
% 
% 
% length(eulerAngles(:,2))
% length(timeVector)
% figure(5)
% plot(timeVector,orient_measu1(1:10000,1),...
%      timeVector,orient_measu1(1:10000,2),...
%      timeVector,orient_measu1(1:10000,3))
%  axis([0,duration,-180,180])
%  legend('Est_Yaw (Rotation Around Down)','Est_Pitch (Rotation Around East)','Est_Roll (Rotation Around North)')
%  xlabel('Time (s)')
%  ylabel('Rotation (degrees)')
%  title('Orientation_Quaternion')
 
 %%
orient_measu=zeros(N,3); 
i1=0;%roll
i2=pitch*(3.14/180);%pich
i3=initialYaw*(3.14/180);%yaw

orient_measu(1,3)=i1*(180/3.14);%roll
orient_measu(1,2)=i2*(180/3.14);
orient_measu(1,1)=i3*(180/3.14);%orient_measu(i,1):yaw
orientation_acc_readng=zeros(N,3);

%% accelerometer readings
for i=1:N
orientation_acc_readng(i,1)=asind(-(accelReadings(i,1))/9.8);%pitch
orientation_acc_readng(i,2)=-(asind((accelReadings(i,2))/(9.8*cos(orientation_acc_readng(i,1)))));%roll
end

%% magnetometer calculation

for i=1:N
p=orient_measu(i,2);r=orient_measu(i,3);
A=[cosd(p) sind(p)*sind(r) sind(p)*cosd(r);0 cosd(r) -sind(r);sind(p) -cosd(p)*sind(r) cosd(p)*cosd(r)];
ans1=A*[magReadings(i,1);magReadings(i,2);magReadings(i,3)];
y=ans1(2);x=ans1(1);
orientation_acc_readng(i,3)=-(atan2(y,x))*(180/3.14); %%yaw reading 
end
% 
 figure(1)
  plot(timeVector,orientation_acc_readng(:,3))
%   figure(2)
%   plot(timeVector,orientation_acc_readng(:,1))
%   figure(3)
%   plot(timeVector,orientation_acc_readng(:,2))

%% Complementry filter

% P_acc_low=zeros(N,1);
% R_acc_low=zeros(N,1);
% Pg_h=zeros(N,1);

P_acc_low=lowpass(orientation_acc_readng(:,1),10,fs);
R_acc_low=lowpass(orientation_acc_readng(:,2),10,fs);
Ym=lowpass(orientation_acc_readng(:,3),10,fs);
Pg_h=highpass(orient_measu1(:,2),10,fs);
Rg_h=highpass(orient_measu1(:,3),10,fs);
Yg_h=highpass(orient_measu1(:,1),10,fs);

P_added=P_acc_low+Pg_h;
 
R_added=R_acc_low+Rg_h;
Y_added=(Ym+Yg_h);
 
yaw_hat_kamlan=(Ym+Yg_h)/1.08+5;


figure
plot(timeVector,Y_added,...
     timeVector,P_added,...
     timeVector,R_added)
 axis([0,duration,-180,180])
 legend('Yaw','Pitch','Roll')
 xlabel('Time (s)')
 ylabel('Rotation (degrees)')
 title('complementary')
 
 
%  %% Kalman filter
% % yaw = zeros(N,1);
% % pitch = zeros(N,1);
% % roll = zeros(N,1);
%  dt = 0.01;
%  m = 0.01;
%  for i=1:N
%      yaw = orient_measu1(i,3);
%      pitch = orient_measu1(i,2);
%      roll = orient_measu1(i,1);
%      %g_cap = [ sin(yaw(i))*sin(roll(i)) - cos(yaw(i))*sin(pitch(i))*cos(roll(i));cos(yaw(i))*sin(roll(i)) + sin(yaw(i))*sin(pitch(i))*cos(roll(i)); cos(pitch(i))*cos(roll(i))] 
%      x1 = sin(yaw)*sin(roll) - cos(yaw)*sin(pitch)*cos(roll);
%      x2 = cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll);
%      x3 = cos(pitch)*cos(roll);
%      a=pi/180;
%      C_k = [0 a*x3 -a*x2 0 -a*dt*x3 a*dt*x2 1 0 0 0 0 0;-a*x3 0 a*x1 a*dt*x3 0 -a*dt*x1 0 1 0 0 0 0;a*x2 -a*x1 0 -a*dt*x2 a*dt*x1 0 0 0 1 0 0 0;0 a*m -a*m 0 -a*dt*m a*dt*m 0 0 0 -1 0 0;-a*m 0 a*m a*dt*m 0 -a*dt*m 0 0 0 0 -1 0;a*m -a*m 0 -a*dt*m a*dt*m 0 0 0 0 0 0 -1]
%      
%  end
%  
 %   figure(2)
%   plot(timeVector,orientation_acc_readng(:,1))
%   figure(3)
%   plot(timeVector,orientation_acc_readng(:,2))

%%  Kalman filter
t = 0.001;
pitch_hat_kalman    = zeros(1, N);
roll_hat_kalman  = zeros(1, N);
yaw_hat_kalman  = zeros(1, N);
gyro_pitch_hat_kalman    = zeros(1, N);
gyro_roll_hat_kalman  = zeros(1, N);
gyro_yaw_hat_kalman  = zeros(1, N);
acc_pitch_hat_kalman    = zeros(1, N);
acc_roll_hat_kalman  = zeros(1, N);
acc_yaw_hat_kalman  = zeros(1, N);
mag_pitch_hat_kalman    = zeros(1, N);
mag_roll_hat_kalman  = zeros(1, N);
mag_yaw_hat_kalman  = zeros(1, N);
P_kalman = ones(12,12);
Q_w = eye(12)*0.01;
Q_v = eye(6)*0.64;
R = ones(6,6)*10;
state_estimate = [0;;0;0;0;0;0;0;0;0;0;0];
alpha = pi/180;

A = eye(12,12)*0;
for i=2:N 
 %C_k
        wx=gyroReadings(i,1);
        wy=gyroReadings(i,2);
        wz=gyroReadings(i,3);

        bax = accelReadings(i,1);
        bay = accelReadings(i,2);
        baz = accelReadings(i,3);

        bmx = magReadings(i,1);
        bmy = magReadings(i,2);
        bmz = magReadings(i,3);
   

        Ck = [0 alpha*wz -alpha*wy 0 -alpha*t*wz alpha*t*wy 1 0 0 0 0 0;-alpha*wz 0 alpha*wx alpha*t*wz 0 -alpha*t*wx 0 1 0 0 0 0;alpha*wy -alpha*wx 0 -alpha*t*wy alpha*t*wx 0 0 0 1 0 0 0;
            0 alpha*bmz -alpha*bmy 0 -alpha*t*bmz alpha*t*bmy 0 0 0 -1 0 0;
            -alpha*bmz 0 alpha*bmx alpha*t*bmz 0 -alpha*t*bmx 0 0 0 0 -1 0;
            alpha*bmy -alpha*bmx 0 -alpha*t*bmy alpha*t*bmx 0 0 0 0 0 0 -1];
        
      
        
        
  %prediction
    P_kalman = A*P_kalman*A' + Q_w; %(state_estimate*P_kalman)*(state_estimate')
  K_kalman = P_kalman*(Ck')*inv((Ck*P_kalman*Ck'+Q_v));
  state_estimate = state_estimate + K_kalman*([bax-wx;bay-wy;baz-wz;bmx-wx;bmy-wy;bmz-wz]-Ck*state_estimate);
  
  
  %Update
  P_kalman = (eye(12) - K_kalman*Ck)*P_kalman;
  
  
pitch_hat_kalman(i) = state_estimate(1);
roll_hat_kalman(i) = state_estimate(2);
yaw_hat_kalman(i) = state_estimate(3);


end

figure
plot(timeVector,yaw_hat_kalman,...
     timeVector,roll_hat_kalman,...
     timeVector,pitch_hat_kalman,...
     timeVector,Y_added,...
     timeVector,P_added,...
     timeVector,R_added)
 axis([0,duration,-180,180])
 legend('Yaw','Pitch','Roll','Yaw C','Pitch C','Roll C' )
 xlabel('Time (s)')
 ylabel('Rotation (degrees)')
 title('Comparision' )


figure
plot(timeVector,yaw_hat_kalman,...
     timeVector,roll_hat_kalman,...
     timeVector,pitch_hat_kalman)
 axis([0,duration,-180,180])
 legend('Yaw','Pitch','Roll' )
 xlabel('Time (s)')
 ylabel('Rotation (degrees)')
 title('Kalman filter' )
 
        
 
 
 
 
 
 