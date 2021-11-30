function dy = DCM(t,y,wx,wy,wz)

ydot1 = y(2)*wz-y(3)*wy;
ydot2 = -y(1)*wz + y(3)*wx;
ydot3 = y(1)*wy - y(2)*wx;
ydot4 = y(5)*wz - y(6)*wy;
ydot5 = -y(4)*wz + y(6)*wx;
ydot6 = y(4)*wy - y(5)*wx;    
ydot7 = y(8)*wz - y(9)*wy;
ydot8 = -y(7)*wz + y(9)*wx;
ydot9 = y(7)*wy - y(8)*wx;    
 
dy = [ydot1;ydot2;ydot3;ydot4;ydot5;ydot6;ydot7;ydot8;ydot9]
end