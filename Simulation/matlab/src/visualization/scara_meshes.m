function meshes = scara_meshes(p)
% Original STL arm plates, sidewalls and base; supplemental parts are proxies.
% Bore XY centres inferred from mesh circles. Z stack is an assembly estimate.
% group: 0 static frame; 1 Z carriage; 2 shoulder; 3 elbow; 4 brush tool.
black=[0.22 0.25 0.28]; orange=[0.96 0.47 0.08]; metal=[0.65 0.71 0.75];
meshes=struct('name',{},'source',{},'group',{},'vertices',{},'faces',{},'color',{});
addstl('Arm_Shoulder_Plate_Top',2,[-1 -1 1],[0 49 97],black);
addstl('Arm_Shoulder_Plate_Bottom',2,[-1 -1 1],[0 44.5 70],black);
addstl('Arm_Elbow_Plate_Top',3,[-1 -1 1],[0 43.327927 67],black);
addstl('Arm_Elbow_Plate_Bottom',3,[1 1 1],[0 43.327927 40],black);
addstl('Arm_Base_MotorPlate',1,[-1 -1 1],[36 -3 70],black);
addstl('Arm_Base_MotorPlate',1,[-1 -1 -1],[36 -3 69],black);
% Mirrored walls: inferred placement in the middle of each 98 mm link.
addstl('Arm_SideWall_Left',2,[1 1 1],[14 46 79],orange);
addstl('Arm_SideWall_Right',2,[-1 1 1],[-14 46 79],orange);
addstl('Arm_SideWall_Left',3,[1 1 1],[14 46 49],orange);
addstl('Arm_SideWall_Right',3,[-1 1 1],[-14 46 49],orange);
addstl('Arm_Elbow_RodHub',3,[1 1 1],[0 p.L2*1000 49],orange);
% Frame: rods and aluminium members from BOM; placement is approximate.
for pos=[-38 0;110 0;36 -39]'
    addcylinder('8 mm guide rod',0,[pos'/1000 0.01],0.004,0.30,metal);
end
addcylinder('Tr8 leadscrew proxy',0,[0.036 0 0.01],0.004,0.25,black);
addbox('Aluminium upright',0,[-0.052 -0.035 0],[0.02 0.04 0.30],metal);
addbox('Aluminium upright',0,[0.104 -0.035 0],[0.02 0.04 0.30],metal);
addbox('Frame crossbar bottom',0,[-0.052 -0.035 0],[0.176 0.02 0.02],metal);
addbox('Frame crossbar top',0,[-0.052 -0.035 0.28],[0.176 0.02 0.02],metal);
addbox('Motor 1 proxy',1,[-0.021 0.004 0.019],[0.042 0.042 0.040],black);
addbox('Motor 2 proxy',1,[0.051 0.004 0.019],[0.042 0.042 0.040],black);
addcylinder('Shoulder bearing/shaft proxy',1,[0 0 0.059],0.013,0.049,metal);
addcylinder('Elbow shaft proxy',2,[0 p.L1 0.031],0.004,0.075,metal);
addcylinder('Brush clamp concept',4,[0 0 0.034],0.011,0.015,orange);
addcylinder('Brush handle concept',4,[0 0 0.008],0.0045,0.082,[0.31 0.22 0.12]);
[X,Y,Z]=cylinder([0.001 0.0045],24);Z=Z*0.008;
[F,V]=surf2patch(X,Y,Z,'triangles');addmesh('Brush bristles concept','proxy',4,V,F,[0.08 0.09 0.12]);

    function addstl(stem,group,sgn,translation,color)
        folder=fullfile(p.hardwareRoot,'arm','STL');
        file=dir(fullfile(folder,'**',[stem '_*.stl']));
        assert(numel(file)==1,'scara:mesh','Expected one STL matching %s.',stem);
        source=fullfile(file.folder,file.name);
        [V,F]=scara_stl(source);V=(V.*sgn+translation)*0.001;
        % Link extension is visual only when nominal lengths are edited.
        if group==2,V(:,2)=V(:,2)*p.L1/0.098;end
        if group==3 && ~strcmp(stem,'Arm_Elbow_RodHub'),V(:,2)=V(:,2)*p.L2/0.098;end
        addmesh(stem,source,group,V,F,color);
    end
    function addmesh(name,source,group,V,F,color)
        meshes(end+1)=struct('name',name,'source',source,'group',group, ...
            'vertices',V,'faces',F,'color',color);
    end
    function addcylinder(name,group,pos,radius,height,color)
        [X,Y,Z]=cylinder(radius,24);Z=Z*height;
        [F,V]=surf2patch(X,Y,Z,'triangles');V=V+pos;
        addmesh(name,'proxy',group,V,F,color);
    end
    function addbox(name,group,pos,dim,color)
        V=[0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1].*dim+pos;
        F=[1 2 3;1 3 4;5 7 6;5 8 7;1 5 6;1 6 2;2 6 7;2 7 3;3 7 8;3 8 4;4 8 5;4 5 1];
        addmesh(name,'proxy',group,V,F,color);
    end
end
