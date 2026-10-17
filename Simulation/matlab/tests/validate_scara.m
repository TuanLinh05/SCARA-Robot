function report = validate_scara()
% Baseline vs independent MATLAB FK, guards, and unmeasured sensitivity cases.
startup_scara();p=scara_params();rng(62);build_scara_simulink(false);
assert(abs(p.stepsPerMeterZ-1600000)<1e-9);
assert(abs(p.coupling*p.stepsPerDegree(2)/p.stepsPerDegree(1)-p.beta)<2e-6);
assert(p.zHomePps==4800);
for k=1:100
    q=[deg2rad(-60+120*rand);deg2rad(10+65*rand);.005+.04*rand];
    x=scara_fk(q,p);qi=scara_ik(x,p);assert(norm(q-qi)<1e-10);
    [~,steps]=scara_motor_coordinates(q,p);back=scara_from_steps(steps',p);
    assert(max(abs(back(1:2)'-q(1:2)))<deg2rad(.04));
end
blocks=find_system('scara_stepper','FollowLinks','on','LookUnderMasks','all','Type','Block');
bad=find_system('scara_stepper','FollowLinks','on','LookUnderMasks','all','BlockType','S-Function');
assert(isempty(bad),'Custom S-Function found.');
assert(isempty(find_system('scara_stepper','MaskType','Stateflow')),'Custom MATLAB Function found.');
patterns={'flower','circle','square','star','heart','bk','linh','thoa'};
runs=struct('pattern',{},'samples',{},'seconds',{},'maxFkDifferenceM',{},'modelRmsMm',{},'modelMaxMm',{});
for k=1:numel(patterns)
    ref=scara_reference(patterns{k},p);out=scara_simulink(ref,p,false);
    index=min(numel(ref.t),round(out.t/p.dt)+1);
    assert(isequal(out.steps,ref.steps(index,:)),'Counts changed during healthy replay.');
    expected=zeros(size(out.xyz));
    for j=1:numel(index),expected(j,:)=scara_fk(ref.q(index(j),:),p)';end
    diff=max(abs(out.xyz-expected),[],'all');
    assert(diff<1e-12 && all(out.allowed),'Standard blocks disagree with kinematics.');
    assert(isequal(out.steps(end,:),ref.steps(end,:)),'Endpoint mismatch.');
    runs(k)=struct('pattern',patterns{k},'samples',numel(index),'seconds',out.t(end), ...
        'maxFkDifferenceM',diff,'modelRmsMm',out.rmsErrorMm,'modelMaxMm',out.maxErrorMm);
    fprintf('%s: %.1fs, FK diff %.3g m, model max %.3f mm\n',patterns{k},out.t(end),diff,out.maxErrorMm);
end
% Watchdog freezes pulses <= one sample after 350 ms without a heartbeat.
ref=scara_reference('star',p);short=ref;idx=ref.t<=4;
fields={'t','steps','xyz','pen','valid','q'};
for k=1:numel(fields),short.(fields{k})=ref.(fields{k})(idx,:);end
fault=p;fault.disconnectTime=1;out=scara_simulink(short,fault,false);
first=find(~out.allowed,1);assert(~isempty(first));
assert(out.t(first)>=1.35 && out.t(first)<=1.35+2*p.dt);
assert(all(out.steps(first:end,:)==out.steps(first,:),'all'));
% Both NC open refuses motion immediately.
fault=p;fault.forceOpenNc(3:4)=true;out=scara_simulink(short,fault,false);
assert(~any(out.allowed));assert(all(out.steps==out.steps(1,:),'all'));
% Single positive switch allows negative retreat but rejects positive motion.
probe=short;probe.t=(0:p.dt:.2)';n=numel(probe.t);probe.q=repmat(p.parkQ',n,1);
probe.steps=zeros(n,3);probe.xyz=zeros(n,3);
fault=p;fault.forceOpenNc(3)=true;
for sign=[-1 1]
    probe.q(:,1)=probe.q(:,1)+sign*probe.t*.1;
    for j=1:n,[~,st]=scara_motor_coordinates(probe.q(j,:),p);probe.steps(j,:)=st';probe.xyz(j,:)=scara_fk(probe.q(j,:),p)';end
    probe.pen=false(n,1);probe.valid=ones(n,1);out=scara_simulink(probe,fault,false);
    if sign<0,assert(all(out.allowed));else,assert(any(~out.allowed));end
    probe.q=repmat(p.parkQ',n,1);
end
% Compare scenario with ideal using exactly the same pulse commands.
ref=scara_reference('star',p);base=scara_simulink(ref,p,false);
scenario=p;scenario.backlash=[deg2rad(.2);deg2rad(.2);0];scenario.lagSeconds=[.01;.01;0];
scenario.scenarioLabel='Illustrative sensitivity: 0.2 deg backlash + 10 ms lag, NOT measured';
out=scara_simulink(ref,scenario,false);
scenarioDelta=max(vecnorm(out.xyz-base.xyz,2,2))*1000;assert(scenarioDelta>.05);
f=scara_plot(out,scenario);close(f);
% STL and GUI smoke tests; no hardware API exists in the app.
f=figure('Visible','off');ax=axes(f);scene=scara_scene(ax,p);
scara_update(scene,p.parkQ);exportgraphics(f,fullfile(p.outputRoot,'figures','assembly.png'),'Resolution',140);close(f);
f=scara_app('off');close(f);
print('-sscara_stepper','-dpng',fullfile(p.outputRoot,'figures','simulink_model.png'));
report=struct('passed',true,'matlab',version,'firmware',p.firmware,'sourceLog',p.sourceLog, ...
    'standardBlocks',numel(blocks),'customFunctionBlocks',0,'fkIkCases',100,'runs',runs, ...
    'watchdogStopSeconds',1.35,'scenarioDeltaMm',scenarioDelta, ...
    'limitations','No physical encoder/force/backlash measurements; home is real telemetry replay, not a contact dynamics model.');
folder=fullfile(p.outputRoot,'validation');if ~isfolder(folder),mkdir(folder);end
fid=fopen(fullfile(folder,'validation.json'),'w');cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));disp('SCARA MATLAB/Simulink validation PASSED');
close_system('scara_stepper',0);
end
