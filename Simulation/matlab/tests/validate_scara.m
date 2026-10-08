function report = validate_scara(includeSimulink)
% Meaningful geometry, mechanics, numerical and Simulink consistency checks.
if nargin<1,includeSimulink=true;end
p=scara_params();rng(41);
for k=1:100
    q=[deg2rad(-90+160*rand);deg2rad(15+145*rand);0.01*rand];
    xyz=scara_fk(q,p);back=scara_ik(xyz,p);
    assert(norm(q-back)<1e-9,'IK/FK round trip failed.');
    [~,~,J]=scara_fk(q,p);h=1e-6;Jnum=zeros(3);
    for j=1:3,d=zeros(3,1);d(j)=h;Jnum(:,j)=(scara_fk(q+d,p)-scara_fk(q-d,p))/(2*h);end
    assert(norm(J-Jnum)<1e-8,'Jacobian check failed.');
    dq=[randn(2,1);0.01*randn];[M,C]=scara_dynamics(q,dq,p);
    assert(min(eig(M))>0,'Mass matrix must be positive definite.');
    [Mp,~]=scara_dynamics(q+h*dq,dq,p);[Mm,~]=scara_dynamics(q-h*dq,dq,p);
    Mdot=(Mp-Mm)/(2*h);
    assert(abs(dq'*C-0.5*dq'*Mdot*dq)<1e-8,'Coriolis energy identity failed.');
end
offsetp=p;offsetp.toolOffset=[0.005;-0.002];
q=[-0.8;1.3;0.007];assert(norm(scara_ik(scara_fk(q,offsetp),offsetp)-q)<1e-9);
expecterror(@()scara_ik([1;1;0],p),'scara:unreachable');
expecterror(@()scara_ik([p.base+[0;p.L1+p.L2];0],p),'scara:singular');
expecterror(@()scara_ik([p.base;0],p),'scara:unreachable');
[~,~,~,Fn]=scara_dynamics([0;1;0.01],zeros(3,1),p);assert(Fn==0);
[~,~,~,Fn]=scara_dynamics([0;1;-0.001],zeros(3,1),p);assert(abs(Fn-0.8)<1e-12);
[a,~]=scara_motor_coordinates([0.6;1.2;0.01],p);
assert(abs(a(2)-1.4)<1e-12,'Motor crosstalk conversion failed.');
u=[10;10;1;zeros(6,1);zeros(9,1)];ctrl=scara_controller(u,p);
assert(all(abs(ctrl(1:3))<=p.actuatorLimits));assert(all(ctrl(4:6)==0));
meshes=scara_meshes(p);
assert(sum([meshes.group]==2)>=4 && sum([meshes.group]==3)>=5);
assert(sum(~strcmp({meshes.source},'proxy'))==11,'Expected eleven instances of original STL.');
report=struct('kinematicsCases',100,'jacobianCases',100,'massMatrixCases',100, ...
    'coriolisEnergyCases',100,'stlInstances',11,'dynamicsParameters','estimates');
names={'circle','flower','square','bk',fullfile(p.simulationRoot,'common','trajectories','example_drawing.csv')};
for k=1:numel(names)
    ref=scara_reference(names{k},p);out=scara_simulate(ref,p);
    assert(out.maxErrorMm<2,'Tracking error unexpectedly large.');
    assert(~out.jointLimitViolation && ~out.zLimitViolation,'Default trajectory exceeds limits.');
    assert(any(out.draw),'No pen contact during drawing.');
    assert(all(out.force(ref.xyz(:,3)>0.005)<1e-8),'Ink must stop when brush is lifted.');
    report.paths(k)=struct('name',names{k},'rmsErrorMm',out.rmsErrorMm, ...
        'maxErrorMm',out.maxErrorMm,'maximumNormalForceN',max(out.force));
    fprintf('%s: RMS %.6f mm, MAX %.6f mm, normal force %.4f N\n', ...
        names{k},out.rmsErrorMm,out.maxErrorMm,max(out.force));
end
ref=scara_reference('flower',p);out=scara_simulate(ref,p);
folder=fullfile(p.outputRoot,'figures');if ~isfolder(folder),mkdir(folder);end
scara_export(out,ref,p);
f=figure('Visible','off','Color','w','Position',[100 100 1100 850]);
if isprop(f,'Theme'),f.Theme='light';end
scene=scara_scene(axes(f),p);
set(scene.target,'XData',ref.xyz(:,1)*1000,'YData',ref.xyz(:,2)*1000, ...
    'ZData',max(0,ref.xyz(:,3))*1000);
ink=out.xyz;ink(~out.draw,:)=nan;ink(:,3)=0.0002;
set(scene.ink,'XData',ink(:,1)*1000,'YData',ink(:,2)*1000,'ZData',ink(:,3)*1000);
scara_update(scene,out.q(round(numel(out.t)*0.65),:)');
title(scene.ax,'X-SCARA from STL | brush drawing | approximate assembly');
exportgraphics(f,fullfile(folder,'scara_3d.png'),'Resolution',140);
zlim(scene.ax,[-3 140]);ylim(scene.ax,[-160 65]);
title(scene.ax,'STL shoulder / elbow + brush concept | detail');
exportgraphics(f,fullfile(folder,'arm_detail.png'),'Resolution',140);close(f);
f=scara_plot(out,p,'off');exportgraphics(f,fullfile(folder,'tracking.png'),'Resolution',140);close(f);
report.simulinkTested=false;
if includeSimulink
    build_scara_simulink(false);
    sl=scara_simulink('flower',p,false);
    native=interp1(out.t,out.q,min(out.t(end),max(out.t(1),sl.t)));
    delta=abs(sl.q-native);report.simulinkMaximumStateDifference=max(delta,[],'all');
    assert(report.simulinkMaximumStateDifference<1e-5,'MATLAB and Simulink disagree.');
    report.simulinkTested=true;
    fprintf('Simulink vs MATLAB maximum state difference: %.3g\n',report.simulinkMaximumStateDifference);
    print('-sscara_brush','-dpng',fullfile(folder,'simulink_model.png'));
    close_system('scara_brush',0);
end
folder=fullfile(p.outputRoot,'validation');if ~isfolder(folder),mkdir(folder);end
fid=fopen(fullfile(folder,'validation.json'),'w','n','UTF-8');
fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
fprintf('PASS: geometry, dynamics, STL rendering and simulation checks.\n');
end

function expecterror(fn,id)
try,fn();catch e,assert(strcmp(e.identifier,id),'Unexpected rejection reason.');return;end
error('Expected rejection: %s',id);
end
