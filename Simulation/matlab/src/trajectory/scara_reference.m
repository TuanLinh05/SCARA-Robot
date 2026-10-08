function ref = scara_reference(kind,p)
% Cartesian trajectories with quintic segment timing and brush lift/lower.
% 'circle', 'flower', 'square', 'bk', or a CSV: x_mm,y_mm,pen (0 or 1).
if nargin<1, kind='flower'; end
if nargin<2, p=scara_params(); end
dt=p.dt;
if isfile(kind)
    data=readmatrix(kind);
    if size(data,2)~=3 || size(data,1)<2 || any(~isfinite(data),'all') ...
            || any(~ismember(data(:,3),[0 1]))
        error('scara:csv','CSV must have finite columns x_mm,y_mm,pen, with pen 0 or 1.');
    end
    pts=data(:,1:2)*1e-3; pen=data(:,3)>0;
    % The pen state on row i applies to movement from row i-1 to row i.
    xyz=[pts(1,:) p.penLift]; t=0;
    for k=2:size(pts,1)
        h=p.penLift;
        if pen(k),h=-p.brushCompression;end
        [t,xyz]=segment(t,xyz,[pts(k-1,:) h],0.4,dt);
        duration=max(0.4,norm(pts(k,:)-pts(k-1,:))/0.020);
        [t,xyz]=segment(t,xyz,[pts(k,:) h],duration,dt);
    end
    [t,xyz]=segment(t,xyz,[pts(end,:) p.penLift],0.4,dt);
else
    name=lower(char(kind));
    switch name
        case 'bk'
            [t,xyz,geometry]=scara_bk_path(p);
        case {'circle','flower'}
            drawtime=8;
            td=(0:dt:drawtime)'; s=quintic(td/drawtime);
            a=2*pi*s;
            if strcmp(name,'flower')
                r=0.023+0.011*cos(5*a);
            else
                r=0.034+zeros(size(a));
            end
            xy=[r.*cos(a),r.*sin(a)];
            xyz=[xy(1,:) p.penLift]; t=0;
            [t,xyz]=segment(t,xyz,[xy(1,:) -p.brushCompression],0.6,dt);
            xyz=[xyz;xy(2:end,:) -p.brushCompression*ones(numel(td)-1,1)];
            t=[t;t(end)+td(2:end)];
        case 'square'
            corners=0.032*[1 -1;1 1;-1 1;-1 -1;1 -1];
            xyz=[corners(1,:) p.penLift];t=0;
            [t,xyz]=segment(t,xyz,[corners(1,:) -p.brushCompression],0.6,dt);
            for k=2:size(corners,1)
                [t,xyz]=segment(t,xyz,[corners(k,:) -p.brushCompression],2,dt);
            end
        otherwise
            error('scara:path','Use circle, flower, square, bk, or a CSV filename.');
    end
    [t,xyz]=segment(t,xyz,[xyz(end,1:2) p.penLift],0.6,dt);
end
q=zeros(numel(t),3);
for k=1:numel(t),q(k,:)=scara_ik(xyz(k,:)',p)';end
q(:,1:2)=unwrap(q(:,1:2));
dq=zeros(size(q));ddq=dq;
for j=1:3
    dq(:,j)=gradient(q(:,j),dt); ddq(:,j)=gradient(dq(:,j),dt);
end
ref=struct('t',t,'xyz',xyz,'q',q,'dq',dq,'ddq',ddq, ...
    'pen',xyz(:,3)<0,'kind',char(kind));
ref.signal=[t q dq ddq];
if exist('geometry','var'),ref.geometry=geometry;end
end

function s=quintic(u)
s=u.^3.*(10-15*u+6*u.^2);
end
function [t,xyz]=segment(t,xyz,target,duration,dt)
n=max(2,ceil(duration/dt)); local=(1:n)'/n;
new=xyz(end,:)+(target-xyz(end,:)).*quintic(local);
t=[t;t(end)+(1:n)'*dt];xyz=[xyz;new];
end
