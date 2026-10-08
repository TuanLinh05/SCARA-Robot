function y = scara_controller(u,p)
% u=[qref(3);dqref(3);ddqref(3);q(3);dq(3);integralError(3)]
% y=[actuator(3);integralDerivative(3)]. Change this file for your algorithm.
qr=u(1:3); dqr=u(4:6); ddqr=u(7:9);
q=u(10:12); dq=u(13:15); ei=u(16:18);
e=qr-q; de=dqr-dq;
feed=[0;0;p.zMass*p.gravity];
if p.feedforward
    [M,C,Fc]=scara_dynamics(qr,dqr,p);
    feed=feed+M*ddqr+C+p.viscous.*dqr+p.coulomb.*tanh(dqr/0.005)-Fc;
end
raw=p.Kp.*e+p.Kd.*de+p.Ki.*ei+feed;
actuator=max(-p.actuatorLimits,min(p.actuatorLimits,raw));
% Conditional integration prevents windup under actuator saturation.
iedot=e;
iedot((raw>p.actuatorLimits & e>0) | (raw<-p.actuatorLimits & e<0))=0;
y=[actuator;iedot];
end
