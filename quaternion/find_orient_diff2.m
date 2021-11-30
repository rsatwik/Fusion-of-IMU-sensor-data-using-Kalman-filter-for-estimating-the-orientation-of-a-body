function dx=find_orient_diff2(t,x,wx,wy,wz)


% xdot1=wx+sin(x(1))*tan(x(2))*wy+cos(x(1))*tan(x(2))*wz;
% xdot2=cos(x(1))*wy-sin(x(1))*wz;
% xdot3=(sin(x(1))/cos(x(2)))*wy+(cos(x(1))/cos(x(2)))*wz;

xdot1=0.5*(wz*x(2)-wy*x(3)+wx*x(4));
xdot2=0.5*(-wz*x(1)+wx*x(3)+wy*x(4));
xdot3=0.5*(wy*x(1)-wx*x(2)+wz*x(4));
xdot4=0.5*(-wx*x(1)-wy*x(2)-wz*x(3));

% dx(1)=xdot1;
% dx(2)=xdot2;
% dx(3)=xdot3;
dx=[xdot1;xdot2;xdot3;xdot4];
end