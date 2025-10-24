clear;clc; close all

%cartesian position of point A
xA=1;
yA=0.5;

%cartesian position of point B
xB=-1;
yB=0.5;


%% interpolate the x coordinate

vA=0; %initial velocity
aA=0; %initial acceleration
vB=0; %final velocity
aB=0; %final acceleration

%initial and final times of movement
t0=0;
tf=1;

%Quintic polynomial matrix
A=[1 t0 t0^2 t0^3 t0^4 t0^5;...
   0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;...
   0 0 2 6*t0 12*t0^2 20*t0^3;...
   1 tf tf^2 tf^3 tf^4 tf^5;...
   0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;...
   0 0 2 6*tf 12*tf^2 20*tf^3];
State=[xA;vA;aA;xB;vB;aB];

%solve for c parameters in quintic polynomial
c_vec=inv(A)*State;

%parse the c parameters
c0=c_vec(1);
c1=c_vec(2);
c2=c_vec(3);
c3=c_vec(4);
c4=c_vec(5);
c5=c_vec(6);

%set up time vector
t=linspace(t0,tf,100);

%Cartesian end effector position/vel/accel as a function of time (x)
x_t=c0+c1*t+c2*t.^2+c3*t.^3+c4*t.^4+c5*t.^5;
xd_t=c1+2*c2*t+3*c3*t.^2+4*c4*t.^3+5*c5*t.^4;
xdd_t=2*c2+6*c3.*t+12*c4.*t.^2+20*c5.*t.^3;

%Cartesian end effector position/vel/accel as a function of time (y)
y_t=yA;
yd_t=0;
ydd_t=0;

%plot cartesian trajectory


%% Inverse Kinematics
L1=1; %link 1 length
L2=1; %link 2 length

D=(x_t.^2+y_t.^2-L1^2-L2^2)/(2*L1*L2);

u2=atan2(sqrt(1-D.^2),D); %joint angle (link 1)
u1=atan2(y_t,x_t)-atan2(L2*sin(u2),L1+L2*cos(u2)); %joint angle (link 2)



%% Plot it!

%plots a video of the manipulator performing the pick-and-place task
c = @(x) cos(x);
s = @(x) sin(x);
figure(4)

hFig = figure(4);
% set(hFig, 'Position', [150 100 750 400])
axis([-1.5,1.5,-1,2])
grid on
set(gca,'FontSize',18)
axis('square')
%% prepare a movie

fig = gcf;
axis_handle = fig.CurrentAxes;
set(gcf,'CurrentAxes',axis_handle);
loop_counter = 0;
plotting_skips = 5;

make_movbie1 = 1;
if make_movbie1==1
    vidObj1 = VideoWriter('taskspaceplanning','MPEG-4');
    open(vidObj1);
    set(gcf,'renderer','zbuffer'); %MUST have this for getframe to work well under windows7
    set(gca,'nextplot','replacechildren');
    currFrame = getframe(gcf);
    writeVideo(vidObj1,currFrame);
end

for k=1:length(t)
u1_0=u1(k);
u2_0=u2(k);


x1=L1*c(u1_0);
y1=L1*s(u1_0);
x2=L1*c(u1_0)+L2*c(u1_0+u2_0);
y2=L1*s(u1_0)+L2*s(u1_0+u2_0);
 
strA='A';

 
strB='B';

clf(4,'reset')
plot([0,x1,x2],[0,y1,y2],'LineWidth',5)
hold on
plot([0,x1,x2],[0,y1,y2],'r.','MarkerSize',40)
text(xA-.05,yA+.2,strA,'FontSize',20);
text(xB-.05,yB+.2,strB,'FontSize',20);
plot([xA,xB],[yA,yB],'bo','MarkerSize',10)
axis([-1.5,1.5,-1,2])
set(gca,'FontSize',18)
axis('square')
grid on
pause(.01)

%% save the video
        if make_movbie1==1
            currFrame = getframe(gcf);
            writeVideo(vidObj1,currFrame); %save the frame into the movie            
        end

end

if make_movbie1==1
    close(vidObj1);
end;


 


