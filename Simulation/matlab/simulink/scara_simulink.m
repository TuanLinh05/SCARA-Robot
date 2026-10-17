function out = scara_simulink(kind,p,showPlots)
if nargin<1,kind='flower';end
if nargin<2,p=scara_params();end
if nargin<3,showPlots=true;end
path=fullfile(p.modelsRoot,'scara_stepper.slx');
if ~isfile(path),build_scara_simulink(false);end
[p,ref]=scara_setup(kind,p);load_system(path);
r=sim('scara_stepper');
out=scara_results(r.tout,r.q_log,r.xyz_log,r.steps_log,r.allowed_log,ref,p);
scara_export(out,ref,p);
if showPlots,scara_plot(out,p);end
end