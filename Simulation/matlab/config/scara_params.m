function p = scara_params()
% Shared numeric defaults: Simulation/common/config/scara.json (SI units).
% Dynamics, joint limits, brush constants and actuator limits are estimates.
p.root=fileparts(fileparts(mfilename('fullpath')));
p.simulationRoot=fileparts(p.root);
p.hardwareRoot=fullfile(fileparts(p.simulationRoot),'Hardware');
p.outputRoot=fullfile(p.root,'output');
p.modelsRoot=fullfile(p.root,'simulink','models');
paths=p;
p=jsondecode(fileread(fullfile(paths.simulationRoot,'common','config','scara.json')));
vectors={'base','toolOffset','paperHalfSize','rotorInertia','viscous','coulomb', ...
    'stepsPerDegree','Kp','Kd','Ki','actuatorLimits'};
for k=1:numel(vectors),p.(vectors{k})=p.(vectors{k})(:);end
p.zLimits=p.zLimits(:)';
names=fieldnames(paths);
for k=1:numel(names),p.(names{k})=paths.(names{k});end
end
