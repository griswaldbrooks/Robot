function x_dot = calc_x_dot(x_prev,x_dot_prev,t)
m = 10;
k = 1;
b = 0;
  
% Control Input
   U = sin(t);
   

%            /  dtheta1
%   x_dot =  |  dtheta2
%            |  ddtheta1
%            \  ddtheta2

ddx = inv(m)*(U - b*x_dot_prev - k*x_prev(1));

x_dot = [x_dot_prev;ddx];
            