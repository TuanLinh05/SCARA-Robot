function results = compare_scara_controllers()
% Compare plain PID (gravity compensation) with PID + model feedforward.
p=scara_params();ref=scara_reference('flower',p);
plain=p;plain.feedforward=false;
results.pid=scara_simulate(ref,plain);
results.feedforward=scara_simulate(ref,p);
f=figure('Name','SCARA controller comparison','Color','w','Position',[100 100 1000 650]);
if isprop(f,'Theme'),f.Theme='light';end
tiledlayout(f,2,1);
nexttile;plot(ref.t,results.pid.xyError*1000,'LineWidth',1.3);hold on;
plot(ref.t,results.feedforward.xyError*1000,'LineWidth',1.3);grid on;
xlabel('Time (s)');ylabel('XY error (mm)');legend('PID','PID + model feedforward');
title('Same ideal plant; performance on hardware requires measured parameters');
nexttile;plot(ref.t,results.pid.force,'LineWidth',1.3);hold on;
plot(ref.t,results.feedforward.force,'LineWidth',1.3);grid on;
xlabel('Time (s)');ylabel('Normal force (N)');legend('PID','PID + model feedforward');
fprintf('PID XY RMS %.5f mm | PID + feedforward XY RMS %.5f mm\n', ...
    results.pid.rmsErrorMm,results.feedforward.rmsErrorMm);
folder=fullfile(p.outputRoot,'figures');if ~isfolder(folder),mkdir(folder);end
exportgraphics(f,fullfile(folder,'controller_comparison.png'),'Resolution',140);
folder=fullfile(p.outputRoot,'data','matlab');if ~isfolder(folder),mkdir(folder);end
save(fullfile(folder,'controller_comparison.mat'),'results','ref','p');
end
