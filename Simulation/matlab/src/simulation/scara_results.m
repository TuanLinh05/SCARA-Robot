function out = scara_results(t,q,dq,ei,ref,p)
% Shared postprocessing for native MATLAB and Simulink results.
t=t(:);n=numel(t);xyz=zeros(n,3);force=zeros(n,1);tau=zeros(n,3);steps=zeros(n,3);
% Fixed-step solvers may round the final timestamp a few ULPs past the stop time.
queryTime=min(ref.t(end),max(ref.t(1),t));
qr=interp1(ref.t,ref.q,queryTime);vr=interp1(ref.t,ref.dq,queryTime);ar=interp1(ref.t,ref.ddq,queryTime);
for k=1:n
    xyz(k,:)=scara_fk(q(k,:)',p)';
    [~,~,~,force(k)]=scara_dynamics(q(k,:)',dq(k,:)',p);
    ctrl=scara_controller([qr(k,:) vr(k,:) ar(k,:) q(k,:) dq(k,:) ei(k,:)]',p);
    tau(k,:)=ctrl(1:3)'; [~,st]=scara_motor_coordinates(q(k,:)',p);steps(k,:)=st';
end
target=interp1(ref.t,ref.xyz,queryTime); err=vecnorm(xyz(:,1:2)-target(:,1:2),2,2);
draw=force>0.01 & abs(xyz(:,1))<=p.paperHalfSize(1) & abs(xyz(:,2))<=p.paperHalfSize(2);
width=(p.lineWidth0+p.lineWidthPerN*force)*1000; width(~draw)=0;
out=struct('t',t,'q',q,'dq',dq,'integral',ei,'xyz',xyz,'target',target, ...
    'force',force,'actuator',tau,'steps',steps,'draw',draw,'xyError',err,'lineWidthMm',width, ...
    'rmsErrorMm',sqrt(mean(err.^2))*1000,'maxErrorMm',max(err)*1000, ...
    'jointLimitViolation',any(q(:,1:2)<p.jointLimits(:,1)' | q(:,1:2)>p.jointLimits(:,2)','all'), ...
    'zLimitViolation',any(q(:,3)<p.zLimits(1) | q(:,3)>p.zLimits(2)));
assert(all(isfinite(q),'all'),'scara:unstable','Non-finite simulation state. Adjust solver or gains.');
assert(all(isfinite([xyz tau target]),'all'),'scara:results','Non-finite simulation outputs.');
end
