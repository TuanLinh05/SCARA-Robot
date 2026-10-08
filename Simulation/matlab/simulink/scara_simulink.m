function out = scara_simulink(kind,p,showPlots)
% Run the saved model and return the same result structure as scara_simulate.
if nargin<1,kind='flower';end
if nargin<2,p=scara_params();end
if nargin<3,showPlots=true;end
path=fullfile(p.modelsRoot,'scara_brush.slx');
if ~isfile(path),build_scara_simulink(false);end
[p,ref]=scara_setup(kind,p);load_system(path);
simout=sim('scara_brush');
out=scara_results(simout.tout,simout.q_log,simout.dq_log,simout.ei_log,ref,p);
scara_export(out,ref,p,fullfile(p.outputRoot,'data','simulink'));
if showPlots,scara_plot(out,p);end
end
