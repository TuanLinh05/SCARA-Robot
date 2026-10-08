function validate_webots_interop()
% Produce independent MATLAB oracle cases for Python FK/IK, PID and paths.
p=scara_params();rng(62);cases=struct('q',{},'dq',{},'ddq',{},'xyz',{},'command',{});
for k=1:24
    q=[-1.4+2.3*rand;0.3+2.3*rand;-0.001+0.010*rand];
    dq=[0.2*randn(2,1);0.004*randn];ddq=[0.5*randn(2,1);0.015*randn];
    xyz=scara_fk(q,p);command=scara_controller([q;dq;ddq;q;dq;zeros(3,1)],p);
    cases(k)=struct('q',q,'dq',dq,'ddq',ddq,'xyz',xyz,'command',command(1:3));
end
names={'flower','circle','square',fullfile(p.simulationRoot,'common','trajectories','example_drawing.csv')};
paths=struct('name',{},'duration',{},'sampleCount',{},'t',{},'q',{},'xyz',{});
for k=1:numel(names)
    ref=scara_reference(names{k},p);indices=unique(round(linspace(1,numel(ref.t),25)));
    name=names{k};if k==4,name='csv';end
    paths(k)=struct('name',name,'duration',ref.t(end),'sampleCount',numel(ref.t), ...
        't',ref.t(indices),'q',ref.q(indices,:),'xyz',ref.xyz(indices,:));
end
oracle=struct('source','MATLAB R2026a scara_fk/scara_controller/scara_reference', ...
    'units','SI','cases',cases,'paths',paths);
path=fullfile(p.simulationRoot,'webots','tests','fixtures','matlab_oracle.json');
fid=fopen(path,'w','n','UTF-8');assert(fid>=0);clean=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(oracle,PrettyPrint=true));
fprintf('Saved MATLAB/Python interop oracle: %s\n',path);
end
