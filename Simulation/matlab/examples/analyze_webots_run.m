function [T,f] = analyze_webots_run(csvFile)
% Plot actual Webots sensor data; select output/runs/<timestamp>/tracking.csv.
if nargin<1
    [name,folder]=uigetfile('*.csv','Choose Webots tracking.csv');
    if isequal(name,0),T=[];f=[];return;end
    csvFile=fullfile(folder,name);
end
T=readtable(csvFile,'VariableNamingRule','preserve');
required={'time_s','X_ref_m','Y_ref_m','X_tip_m','Y_tip_m','normal_force_N','xy_error_mm','ink_enabled'};
assert(all(ismember(required,T.Properties.VariableNames)),'scara:webots','Select a Webots tracking.csv log.');
f=figure('Name','Webots sensor log','Color','w');
if isprop(f,'Theme'),f.Theme='light';end
tiledlayout(f,2,2);
nexttile;plot(T.X_ref_m*1000,T.Y_ref_m*1000,'--');hold on;
ink=[T.X_tip_m T.Y_tip_m]*1000;ink(~logical(T.ink_enabled),:)=nan;
plot(ink(:,1),ink(:,2));axis equal;grid on;xlabel('X (mm)');ylabel('Y (mm)');legend('Reference','Ink');
nexttile;plot(T.time_s,T.xy_error_mm);grid on;xlabel('Time (s)');ylabel('XY error (mm)');
nexttile;plot(T.time_s,T.normal_force_N);grid on;xlabel('Time (s)');ylabel('Contact force (N)');
nexttile;plot(T.time_s,[T.Z_ref_m T.Z_m T.Z_tip_m]*1000);grid on;
xlabel('Time (s)');ylabel('Z (mm)');legend('Free tip reference','Carriage','Actual tip');
end
