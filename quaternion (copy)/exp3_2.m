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
length(timeVector)
figure(5)
plot(timeVector,orient_measu1(1:10000,1),...
     timeVector,orient_measu1(1:10000,2),...
     timeVector,orient_measu1(1:10000,3))
 axis([0,duration,-180,180])
 legend('Est_Yaw (Rotation Around Down)','Est_Pitch (Rotation Around East)','Est_Roll (Rotation Around North)')
 xlabel('Time (s)')
 ylabel('Rotation (degrees)')
 title('Orientation_Quaternion')
 
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
 figure(2)
 plot(timeVector,orientation_acc_readng(:,1))
 figure(3)
 plot(timeVector,orientation_acc_readng(:,2))

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
 


figure
plot(timeVector,Y_added,...
     timeVector,P_added,...
     timeVector,R_added)
 axis([0,duration,-180,180])
 legend('Yaw','Pitch','Roll')
 xlabel('Time (s)')
 ylabel('Rotation (degrees)')
 title('complementary')
 
 
 %%
 
 
 