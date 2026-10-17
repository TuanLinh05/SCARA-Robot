function [p,ref] = scara_setup(kind,p)
% Load offline GUI/R9 replay; never connects to hardware.
if nargin<2,p=scara_params();end
if nargin<1,kind='flower';end
if isstruct(kind),ref=kind;else,ref=scara_reference(kind,p);end
assignin('base','scara_p',p);assignin('base','scara_ref',ref);
assignin('base','scara_steps_signal',[ref.t ref.steps]);
assignin('base','scara_xyz_signal',[ref.t ref.xyz]);
assignin('base','scara_pen_signal',[ref.t double(ref.pen)]);
age=zeros(size(ref.t));i=ref.t>=p.disconnectTime;age(i)=ref.t(i)-p.disconnectTime;
assignin('base','scara_link_signal',[ref.t ref.valid age]);
assignin('base','scara_nc_signal',[ref.t repmat(double(p.forceOpenNc(:)'),numel(ref.t),1)]);
assignin('base','scara_q0',ref.q(1,:)');
end