function report = validate_bk(includeSimulink)
% BK-specific geometry, pen-up transfer and closed-loop checks.
if nargin<1,includeSimulink=true;end
p=scara_params();ref=scara_reference('bk',p);out=scara_simulate(ref,p);
% Compare against the supplied source drawing, independently of segment timing.
source=(ref.xyz(:,1:2)-ref.geometry.offset)/ref.geometry.scale;
ink=source(ref.pen,:);h=1e-9;
assert(all(source(:)>=-h & source(:)<=0.5+h));
onStemB=abs(ink(:,1))<h;
lower=abs(vecnorm(ink-[0 0.125],2,2)-0.125)<h & ink(:,1)>=-h & ink(:,2)<=0.25+h;
upper=abs(vecnorm(ink-[0 0.375],2,2)-0.125)<h & ink(:,1)>=-h & ink(:,2)>=0.25-h;
onStemK=abs(ink(:,1)-0.25)<h;
upperK=abs(ink(:,2)-ink(:,1))<h & ink(:,1)>=0.25-h;
lowerK=abs(ink(:,2)+ink(:,1)-0.5)<h & ink(:,1)>=0.25-h;
assert(all(onStemB|lower|upper|onStemK|upperK|lowerK),'BK contains an unintended ink line.');
assert(nnz(lower)>100 && nnz(upper)>100 && nnz(upperK)>100 && nnz(lowerK)>100);
assert(sum(diff([false;ref.pen])==1)==3,'BK must have three ink strokes.');
travel=vecnorm(diff(ref.xyz(:,1:2)),2,2)>1e-12 & ~ref.pen(1:end-1) & ~ref.pen(2:end);
assert(any(travel),'Expected pen-up moves between letters/strokes.');
z=ref.xyz(:,3);indices=find(travel);
assert(all(z(indices)>0.005 & z(indices+1)>0.005),'Pen must be lifted during XY transfers.');
assert(~ref.pen(1) && ~ref.pen(end));
assert(all(abs(ref.xyz(:,1:2))<=p.paperHalfSize'+1e-12,'all'));
assert(~out.jointLimitViolation && ~out.zLimitViolation);
assert(out.maxErrorMm<2 && any(out.draw));
assert(all(out.force(ref.xyz(:,3)>0.005)<1e-8),'Lifted brush should not leave ink.');
report=struct('pattern','BK','sizeMm',ref.geometry.sizeMm,'strokes',3, ...
    'durationSeconds',ref.t(end),'rmsErrorMm',out.rmsErrorMm, ...
    'maxErrorMm',out.maxErrorMm,'maximumNormalForceN',max(out.force),'simulinkTested',false);
if includeSimulink
    sl=scara_simulink('bk',p,false);
    time=min(out.t(end),max(out.t(1),sl.t));delta=sl.q-interp1(out.t,out.q,time);
    report.simulinkMaximumStateDifference=max(abs(delta),[],'all');
    assert(report.simulinkMaximumStateDifference<1e-5,'MATLAB and Simulink BK disagree.');
    report.simulinkTested=true;
    scara_export(sl,ref,p,fullfile(p.outputRoot,'data','simulink','bk'));
    close_system('scara_brush',0);
end
folder=fullfile(p.outputRoot,'validation');if ~isfolder(folder),mkdir(folder);end
fid=fopen(fullfile(folder,'bk_validation.json'),'w','n','UTF-8');assert(fid>=0);
cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
fprintf('PASS BK: 3 ink strokes, lifted transfers, geometry and tracking. RMS %.6f mm.\n',out.rmsErrorMm);
end
