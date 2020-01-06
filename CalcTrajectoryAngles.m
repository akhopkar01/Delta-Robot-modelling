function [ angles ] = CalcTrajectoryAngles( traj,param )
%do inverse kinematics for trajectory
m=size(traj,2);
angles=zeros(3,m);

for i=1:m
    angles(:,i)=inverseKinematics([traj(1,i),traj(2,i),traj(3,i)],param);
end

end

