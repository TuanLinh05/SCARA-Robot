function result = scara_simulate(ref,p)
% Deterministic RK4 closed-loop PID + coupled dynamics + paper contact.
if nargin<2,p=scara_params();end
n=numel(ref.t); state=zeros(n,9); state(1,1:3)=ref.q(1,:);
for k=1:n-1
    h=ref.t(k+1)-ref.t(k); s=state(k,:)';
    r0=[ref.q(k,:) ref.dq(k,:) ref.ddq(k,:)]';
    r1=[ref.q(k+1,:) ref.dq(k+1,:) ref.ddq(k+1,:)]';
    rm=(r0+r1)/2;
    a=rhs(s,r0,p); b=rhs(s+h*a/2,rm,p);
    c=rhs(s+h*b/2,rm,p); d=rhs(s+h*c,r1,p);
    state(k+1,:)=(s+h*(a+2*b+2*c+d)/6)';
end
result=scara_results(ref.t,state(:,1:3),state(:,4:6),state(:,7:9),ref,p);
end
function ds=rhs(s,r,p)
control=scara_controller([r;s],p);
ddq=scara_accel([s(1:6);control(1:3)],p);
ds=[s(4:6);ddq;control(4:6)];
end
