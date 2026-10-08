function [p,ref] = scara_setup(kind,p)
% Initialize the saved Simulink model. Existing supplied parameters persist.
if nargin<2
    if evalin('base','exist(''scara_p'',''var'')')
        p=evalin('base','scara_p');
    else,p=scara_params();end
end
if nargin<1 && evalin('base','exist(''scara_ref'',''var'')')
    ref=evalin('base','scara_ref');
else
    if nargin<1,kind='flower';end
    ref=scara_reference(kind,p);
end
assignin('base','scara_p',p);assignin('base','scara_ref',ref);
assignin('base','scara_ref_signal',ref.signal);
assignin('base','scara_q0',ref.q(1,:)');
numeric=p;names=fieldnames(numeric);
for k=1:numel(names)
    value=numeric.(names{k});
    if ~(isnumeric(value)||islogical(value)),numeric=rmfield(numeric,names{k});end
end
assignin('base','scara_numeric_p',numeric);
end
