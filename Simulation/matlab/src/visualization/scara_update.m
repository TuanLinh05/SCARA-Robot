function scara_update(scene,q)
p=scene.p;[tip,elbow]=scara_fk(q,p);a=q(1);b=a+q(2);
Ra=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
Rb=[cos(b) -sin(b) 0;sin(b) cos(b) 0;0 0 1];
for k=1:numel(scene.meshes)
    m=scene.meshes(k);V=m.vertices;
    switch m.group
        case 0,V=V+[p.base' 0];
        case 1,V=V+[p.base' q(3)];
        case 2,V=V*Ra'+[p.base' q(3)];
        case 3,V=V*Rb'+[elbow' q(3)];
        case 4
            V=V*Rb'+tip';
            if strcmp(m.name,'Brush bristles concept'),V(:,3)=max(0,V(:,3));end
    end
    set(scene.handles(k),'Vertices',V*1000);
end
set(scene.tip,'XData',tip(1)*1000,'YData',tip(2)*1000,'ZData',max(0,tip(3))*1000);
end
