%Inverse Kinematics function

function [Th, flag] = inverseKinematics(pos_,param)
T = zeros(1,3);
k = [0,120,-120];
for i = 1:3
    T(i) = jointParam(pos_,k(i),param);
end
if (isnan(T(1)) || isnan(T(2)) || isnan(T(3)))
    flag = 1;
    Th = zeros(1,3);
else
    flag = 0;
    Th = [T(1), T(2), T(3)];
end

end

%calculates joint variable
function [theta] = jointParam(pos_,k,param)
rot_Mat = [cosd(k), -sind(k), 0; sind(k), cosd(k), 0; 0, 0, 1];
%coord_in = [x0,y0,z0];
coord_param = rot_Mat*(pos_)';
e = param(1); %end effector 
f = param(2); %base
rf = param(4); %base arm
re = param(3); %parallelogram
%Coordinates of the end effector
y = coord_param(2) - (e/(2*sqrt(3))); %shift center to edge
z = coord_param(3);
x = coord_param(1);
%projected point on yz plane
y1 = -0.5 * f / sqrt(3);  % - f/2 * tan(30)

a = (x^2 + y^2 + z^2 + rf^2 - re^2 - y1^2)/(2*z);
b = (y1-y)/(z);

%discriminant
d = -(a+b*y1)^2 + rf*(rf*b^2 + rf);
if d < 0
    
    
    theta = nan;
else
    
    yj = (y1 - a*b - sqrt(d))/(b^2 + 1); % choosing outer point
    zj = a + b*yj;
    theta = 180*atan(-zj/(y1 - yj))/pi;
    
    if yj>y1
        theta = theta + 180;
    end
end

end
    
   

    
    


