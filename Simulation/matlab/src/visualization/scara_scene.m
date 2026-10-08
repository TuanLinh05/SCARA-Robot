function scene = scara_scene(ax,p)
% Create STL patches once; update their vertices for fast animation.
cla(ax);hold(ax,'on');axis(ax,'equal');grid(ax,'on');
set(ax,'Color',[0.96 0.97 0.98]);view(ax,145,27);
xlabel(ax,'X (mm)');ylabel(ax,'Y (mm)');zlabel(ax,'Z (mm)');
xlim(ax,[-140 160]);ylim(ax,[-180 100]);zlim(ax,[-5 330]);
paper=p.paperHalfSize*1000;
patch(ax,[-paper(1) paper(1) paper(1) -paper(1)], ...
    [-paper(2) -paper(2) paper(2) paper(2)],[0 0 0 0], ...
    [1 1 1],'EdgeColor',[0.6 0.65 0.7]);
text(ax,-paper(1),paper(2),2,'100 x 100 mm paper','FontSize',9);
m=scara_meshes(p);h=gobjects(numel(m),1);
for k=1:numel(m)
    h(k)=patch(ax,'Faces',m(k).faces,'Vertices',m(k).vertices*1000, ...
        'FaceColor',m(k).color,'EdgeColor','none','FaceLighting','gouraud');
end
camlight(ax,'headlight');material(ax,'dull');
scene=struct('ax',ax,'meshes',m,'handles',h,'p',p);
scene.target=plot3(ax,nan,nan,nan,'--','Color',[0.15 0.55 0.80],'LineWidth',1);
scene.ink=plot3(ax,nan,nan,nan,'-','Color',[0.15 0.17 0.2],'LineWidth',1.6);
scene.tip=plot3(ax,nan,nan,nan,'o','MarkerFaceColor',[0.95 0.3 0.1],'MarkerSize',5);
scara_update(scene,scara_ik([0;0;p.penLift],p));
end
