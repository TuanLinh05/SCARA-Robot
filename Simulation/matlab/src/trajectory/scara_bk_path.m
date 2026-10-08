function [t,xyz,geometry] = scara_bk_path(p)
% Recreate the supplied BK sketch in its 0..0.5 coordinate system.
% Uniformly scale to 70% of the smaller paper dimension, centre on the paper.
% Three continuous ink strokes: B outline, K stem, K diagonals.
side=0.70*min(2*p.paperHalfSize);
geometry=struct('sourceExtent',[0.5 0.5],'scale',side/0.5, ...
    'offset',[-side/2 -side/2],'sizeMm',side*1000,'strokeCount',3, ...
    'arcRadiusM',side/4);
map=@(points)(points-[0.25 0.25])*geometry.scale;
lift=p.penLift;down=-p.brushCompression;drawSpeed=0.020;travelSpeed=0.030;
xy=map([0 0.5]);t=0;xyz=[xy lift];

% B: top -> bottom along the stem, then two right-facing semicircles.
[t,xyz]=line(t,xyz,[xy down],0.6,p.dt);
[t,xyz]=drawline(t,xyz,map([0 0]),down,drawSpeed,p.dt);
[t,xyz]=arc(t,xyz,map([0 0.125]),geometry.arcRadiusM,down,drawSpeed,p.dt);
[t,xyz]=arc(t,xyz,map([0 0.375]),geometry.arcRadiusM,down,drawSpeed,p.dt);

% K: lift before moving from B to K. The stem is at source X=0.25.
[t,xyz]=move(t,xyz,map([0.25 0.5]),lift,down,travelSpeed,p.dt);
[t,xyz]=drawline(t,xyz,map([0.25 0]),down,drawSpeed,p.dt);

% Lift again before the diagonal stroke: upper right -> centre -> lower right.
[t,xyz]=move(t,xyz,map([0.5 0.5]),lift,down,travelSpeed,p.dt);
[t,xyz]=drawline(t,xyz,map([0.25 0.25]),down,drawSpeed,p.dt);
[t,xyz]=drawline(t,xyz,map([0.5 0]),down,drawSpeed,p.dt);
% scara_reference appends the final lift, as for the other built-in patterns.
end

function [t,xyz]=drawline(t,xyz,xy,z,speed,dt)
duration=max(0.6,norm(xy-xyz(end,1:2))/speed);
[t,xyz]=line(t,xyz,[xy z],duration,dt);
end

function [t,xyz]=move(t,xyz,xy,lift,down,speed,dt)
[t,xyz]=line(t,xyz,[xyz(end,1:2) lift],0.6,dt);
duration=max(0.6,norm(xy-xyz(end,1:2))/speed);
[t,xyz]=line(t,xyz,[xy lift],duration,dt);
[t,xyz]=line(t,xyz,[xy down],0.6,dt);
end

function [t,xyz]=arc(t,xyz,center,radius,z,speed,dt)
n=max(2,ceil(pi*radius/speed/dt));u=(1:n)'/n;
angle=-pi/2+pi*quintic(u);
points=center+radius*[cos(angle) sin(angle)];
t=[t;t(end)+(1:n)'*dt];xyz=[xyz;points z*ones(n,1)];
end

function [t,xyz]=line(t,xyz,target,duration,dt)
n=max(2,ceil(duration/dt));u=(1:n)'/n;
points=xyz(end,:)+(target-xyz(end,:)).*quintic(u);
t=[t;t(end)+(1:n)'*dt];xyz=[xyz;points];
end

function s=quintic(u)
s=u.^3.*(10-15*u+6*u.^2);
end
