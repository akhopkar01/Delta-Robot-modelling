function [] = Animation( angles,traj,param )
%ANIMATION Make Plot animation
%simulation step size
N=size(angles,2);
dt=0.1;

for i=1:N 
    tic
    plotRobot(traj(:,i),angles(:,i),param);
    toc
    pause(dt);
end

end

