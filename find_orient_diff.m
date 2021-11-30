function dx=find_orient_diff(t,x,wx,wy,wz)


xdot1=wx+sin(x(1))*tan(x(2))*wy+cos(x(1))*tan(x(2))*wz;
xdot2=cos(x(1))*wy-sin(x(1))*wz;
xdot3=(sin(x(1))/cos(x(2)))*wy+(cos(x(1))/cos(x(2)))*wz;

% dx(1)=xdot1;
% dx(2)=xdot2;
% dx(3)=xdot3;
dx=[xdot1;xdot2;xdot3];
end