clear all;
close all;

pkg load geometry;

# The starting joint space config
q=[0;0];
# the ending joint space config
qf=[pi/2;pi/2];

global a1 = 1;
global a2 = 1;

# the cartisian point of the obstical
obstical=[0;0];

# the force coefficients for the field.
global attXi = 100;
global repXi = 1;
global repDist = 0.5;

function d = dist(x,y)
  d = norm((x-y),2,'cols');
end

function qat = Qat(q,qf)
  global attXi;
  qat = -1*attXi*(q-qf);
end

function qrep = Qrep(q,qo)
  global repXi;
  global repDist;
  r = dist(q,qo);
  if(r > repDist)
    qrep = 0;
    return;
  end
  qrep = repXi*(1./r).*(1./(r.^2)).*((q-qo)./r);
end

function p = fkin(q,as)
  global a1;
  global a2;
  p = as(1)*[cos(q(1));sin(q(1))] + as(2)*[cos(sum(q));sin(sum(q))];
end

# the joint space equivalent of obstical
%Q = rkin(obstical,[a1;a2])(2:3,:)';
Q=[[1;1],[0.813;0.813]];

# The path of the endeffector
%pse=fkin(q,[a1,a2]);
%ps1=fkin(q,[a1,0]);

%axis([-2.1,3.1,-2,3.1],'equal');

#create a line to show the path of the endo
%lnpe = line('xdata',0,'ydata',0,'zdata');
%lnp1 = line('xdata',0,'ydata',0);

%b = 0.03;

# create a rectangle where the goal is
%rectangle("Position",[(fkin(qf,[a1,a2])-[b;b])',b,b]);

# create a rectangle of the obstical
%rectangle("Position",[(obstical-[b;b])',b,b]);

x=linspace(0,pi/2+1,100);
y=linspace(0,pi/2+1,100);

[xs,ys] = meshgrid(x,y);

f = @(x,y) norm(Qat([x;y],qf) + sum(Qrep([x,x;y,y],Q),2));
fs = arrayfun(f,xs,ys);
fs=min(fs,800);

hold on;

xlabel("theta1 (rad)");
ylabel("theta2 (rad)");
zlabel("Force Magnitude");
[sx,sy,sz] = sphere();
%surf(0.1*sx+qs(1,columns(qs)),0.1*sy+qs(2,columns(qs)),25*sz);
mesh(xs,ys,fs);
view(0,30);

# compute joint space trajectory
qs=[q];
qsl = line(q(1),q(2),0,'linewidth',3);
i=1;
Fs=zeros(2,3);
localMin=0.01;
while(i < 3000 && dist(q,qf) > 0.060)
  i++;

  F = Qat(q,qf) + sum(Qrep([q,q],Q),2);
  if(i > 2 && all(norm(repmat(q,1,3)-qs(i-2:i),2,'cols') < localMin))
    F = [0,-1;1,0]*F
  end

  q = q+0.01*(F/norm(F,2));
  qs=[qs,q];
   
  %pse=[pse,fkin(q,[a1,a2])];
  %ps1=[ps1,fkin(q,[a1,0])];
  %set(lnpe,'xdata',pse(1,:),'ydata',pse(2,:));
  %set(lnp1,'xdata',ps1(1,:),'ydata',ps1(2,:));
  set(qsl,'xdata',qs(1,:),'ydata',qs(2,:),'zdata',zeros(1,columns(qs)));
  %sleep(0.01);
end

%surf(0.1*sx+qf(1),0.1*sy+qf(2),25*sz);
%view(-15,30);
