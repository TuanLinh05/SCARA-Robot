function [M,C,Fcontact,Fn] = scara_dynamics(q,dq,p)
% Rigid planar 2R + independent vertical translation, lumped inertias.
% M*ddq + C + friction + gravity = actuator + paper contact.
% Brush spring model acts on FREE tip height; it is not paper penetration.
q=q(:); dq=dq(:); l=p.L1; c1=l/2; c2=p.L2/2;
I1=p.mass1*l*l/12; I2=p.mass2*p.L2*p.L2/12;
v=p.toolOffset+[0;p.L2]; L=norm(v); delta=atan2(-v(1),v(2));
b=p.mass2*l*c2*cos(q(2))+p.massTool*l*L*cos(q(2)+delta);
A=p.mass1*c1*c1+I1+p.mass2*(l*l+c2*c2)+I2+p.massTool*(l*l+L*L);
D=p.mass2*c2*c2+I2+p.massTool*L*L;
M=[A+2*b+p.rotorInertia(1), D+b, 0; ...
   D+b, D+p.rotorInertia(2), 0; 0 0 p.zMass];
h=p.mass2*l*c2*sin(q(2))+p.massTool*l*L*sin(q(2)+delta);
C=[-h*(2*dq(1)*dq(2)+dq(2)^2);h*dq(1)^2;0];
Fn=0;
if q(3)<=0, Fn=max(0,-p.brushK*q(3)-p.brushC*dq(3)); end
[~,~,J]=scara_fk(q,p);
vxy=J(1:2,:)*dq;
Fxy=-p.brushMu*Fn*vxy/sqrt(dot(vxy,vxy)+0.001^2);
Fcontact=J'*[Fxy;Fn];
end
