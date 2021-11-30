% function [ydot] = myodefun(t,y)
w_x=-0.0041
w_y = 0
w_z = 0.0155

y_dot = @(t,y)[w_x + sin(y(1))*tan(y(2))*w_y+cos(y(1))*tan(y(2))*w_z;
    cos(y(1))*w_y-sin(y(1))*w_z;
    w_y*(sin(y(1))/cos(y(2)))+w_z*(cos(y(1))/cos(y(2)))]


[T,Y] = ode45(y_dot,[0 1000],zeros(3,1));



plot(T,Y(:,1)*(180/pi),'-',T,Y(:,2)*(180/pi),'-.',T,Y(:,3)*(180/pi),'.');