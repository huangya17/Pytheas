% 3DOF autonomous boat and simple control logic to follow waypoint 
% Ya Huang, 2017-04-12, School of Engineering, University of Portsmouth, England
% This script calls and runs simulation file *.slx 

%% EOMs and boat parameters adapted from Escario et al (2012):

% xdd = ( F*cos(z) - uxl*xd - uxc*xd*abs(xd) ) / ( m + mx )
% ydd = ( F*sin(z) - uyl*yd - uyc*yd*abs(yd) ) / ( m + my )
% zdd = ( L/2*F*sin(z) - uzl*L*zd - uzc*zd*abs(zd) ) / ( I + Iz )

% xyz - coordinate for local body-fixed frame
% XYZ - coordinate for global inertial frame fixed to the ground
% x - m, body-fixed longitudinal 'surge' displacement 
% xd - m/s velcoity 
% xdd - m/s2 acceleration
% y - m, body-fixed transverse 'sway' displacement 
% yd - m/s velcoity 
% ydd - m/s2 acceleration
% z - rad, heading 'yaw' angle relative to X-axis of inerital frame
% zd - rad/s ang velcoity 
% zdd - rad/s2 ang acceleration
% b - rad, [-pi/4 pi/4], steering angle of thruster relative to heading (x-axis) 
% F - N, [0 20], propulsion force of the thruster 

close all; clc; clear;

% V = 1 ; % m/s, surge speed demand
R = 20 ; % m, waypoint radius for trial
ad = 0 ; % deg
Xw = R*cosd(ad) 
Yw = R*sind(ad) 

% Xw = -20 ; % current waypoint
% Yw = 0 ; % current waypoint
bmax = pi*(0.47) ; % rad, maximum degree of thrust angle beta 'b', full range +/-45 deg 
Fmax = 20 ; % N, maximum thrust force 'F' by all propellers 
% Xwps = [2 20 50 100] ; % m, waypoint demands in earth X- 
% Ywps = [40 20 40 100] ; % m, waypoint demands in earth Y- 
wp_tol = 2.5 ; % m, waypoint tolerance 
wp_sh = 10 ; % m, short distance to waypoint to enble reduced thrust F

% Disturbance force 
B = 30 ; % N, disurbance force bias
A = 5 ; % N, disurbance force amplitude
fd = 0.2 ; % Hz, disturbance force frequency
wd = fd*2*pi ; % rad/s 
kai = pi*3/4 ; % rad, disturbance force angle relative to global frame +X
td = 0 ; % s, delay to start disturbance force - allow boat to build up speed

% Boat parameters
L = 1 ; % m, boat length
Lt = 0.4 ; % m, longitudinal distance between mass centre and thruster force point of action 
W = 0.55 ; % m, boat width 
H = 0.20 ; % m, boat height
Dh = 0.15 ; % m, hull duct diameter for one side of cataraman
D = 0.05 ; % m, mean submerged depth at 7 kg mass
m = 7 ; % kg, boat mass
I = m*(W^2+L^2)/12 ; % kgm2, mass moment of inertia about mass centre G in yaw z-axis 
mxx = 0.05*m ; % kg, added mass in forward x-axis of boat - Muske et al 2008
myy = 1000*pi*D^2*L/2 ; % kg, added mass in lateral y-axis of boat  - Muske et al 2008
Izz = (0.1*m*W^2+1000*pi*D^2*L^3)/2/12 ; % kgm2, added mass moment of inertia in yaw z-axis of boat - Muske et al 2008
% d1 = 2.44 ; % forward surge hydrodynamic damping coefficient in SI units - Muske et al 2008
% d2 = 13.0 ; % lateral sway hydrodynamic damping coefficient in SI units - Muske et al 2008
% d3 = 0.0564 ; % yaw hydrodynamic damping coefficient in SI units - Muske et al 2008
% a1 = 1.51 ; % expoential for forward surge hydrodynamic damping - Muske et al 2008
% a2 = 1.75 ; % expoential for lateral sway hydrodynamic damping - Muske et al 2008
% a3 = 1.59 ; % expoential for yaw hydrodynamic damping - Muske et al 2008
d1 = 14 ;
d2  = 80 ;
d3 = 5 ;
a1 = 1 ;
a2 = 1 ;
a3 = 1 ;
% uzl = 10 ; % Nms/rad, linear coefficient of damping to turn
% uzq = 100 ; % Nms2/rad2, quadratic coefficient of damping to turn
% uxl = 1.75 ; % Ns/m, linear coefficient of forward hydrodynamic damping
% uxq = 2.5 ; % Ns2/m2, quadratic coefficient of forward hydrodynamic damping
% uyl = 25 ; % Ns/m, linear coefficient of lateral hydrodynamic damping
% uyq = 250 ; % Ns2/m2, quadratic coefficient of lateral hydrodynamic damping

% Simulink configuration
fixedstep = 0.01 ; % s, simulation time step
fs = 1/fixedstep ; % Hz, sampling frequency 
time = 150  ; % s, simulation time 

open_system('boatx006_3');
options = simset('Solver','ode4','FixedStep',fixedstep);
 
%     set_param('boatx003/Heading','After','3/4*pi');
%     set_param('boatx003/Heading','Before','0');
%     set_param('boatx003/Heading','Time','5');
%     set_param('hsld_sdof_bm/kv','kv',num2str(kv));
%     set_param('hsld_sdof_bm/Subsystem','c',num2str(c));
%     set_param('sdof_bm_general1/Integrator1','InitialCondition','0');
      
[ t , uu , vv ] = sim ( 'boatx006_3' , time - fixedstep , options ) ;

% Plot parameters
X_lo = floor(min(X.data)-0.1)*1.5 ; % m, scale for plot
X_hi = ceil(max(X.data)+0.1)*1.5 ;
Y_lo = floor(min(Y.data)-0.1)*1.5 ; % m, scale for plot
Y_hi = ceil(max(Y.data)+0.1)*1.5 ;

if (X_hi-X_lo)<(Y_hi-Y_lo)
    X_lo = X_lo ;
    X_hi = X_lo + (Y_hi-Y_lo) ;
elseif (X_hi-X_lo)>(Y_hi-Y_lo)
    Y_lo = Y_lo ;
    Y_hi = Y_lo + (X_hi-X_lo) ;     
end
ut = 15;
% ut = t(end);

% aa = [t X.data Y.data xd.data] ;
% aai = find((abs(aa(:,2)-Xw)<=wp_tol) & (abs(aa(:,3)-Yw)<=wp_tol));
% aat = aa(aai(1),1);
% vvi = find(aa(:,4)>=max(aa(:,4))*0.99);
% vvmax = max(aa(:,4))
% vvt = aa(vvi(1),1)
% vvd = sqrt(aa(vvi(1),2)^2 + aa(vvi(1),3)^2)
% figure(10);
% plot(t,X.data,'r-',t,Y.data,'b--'); grid; hold on; legend('X','Y');
% line([aat aat],[0 Xw]);
% xlabel('t');ylabel('X and Y ( m )');

figure(20);
plot(t,dst.data); xlabel('Time ( s )'); ylabel('dst ( m )'); grid on;

figure(30);
subplot(2,2,1)
plot(t,z.data*180/pi,'k-',t,z_dm.data*180/pi,'k-.',t,dhd.data*180/pi,'k--',t,b.data*180/pi,'k:');hold on; box on; grid on;
xlim([0 ut]);ylabel('Angle ( deg )');xlabel('Time ( s )');grid on;box on;
% legend('Current heading z (deg)','Demand heading hd (deg)','Heading difference dhd = hd - z (deg)',...
%     ['Thrust angle b, ' 'disturbance angle k = ' num2str(k.data(5)*180/pi) ' deg']);
title('( a )','interpreter','none');

subplot(2,2,2)
plot(X.data,Y.data,'k-'); xlabel('X (m)'); ylabel('Y (m)'); grid on; box on; hold on;
plot(0,0,'bo',Xw,Yw,'r+'); hold on;
r = wp_tol ; x = Xw ; y = Yw ; theta = linspace(0,2*pi) ; 
plot(r*cos(theta)+x , r*sin(theta)+y , 'r--') ; hold on;
axis([X_lo X_hi Y_lo Y_hi]);
title('( b )','interpreter','none');
% text(Xw,Yw+wp_tol,['( ' num2str(Xw) ' , ' num2str(Yw) ' )']);
% plot(X.data,Y.data,'b-');axis([X_lo X_hi Y_lo Y_hi]);
% ylabel('Y ( m )','interpreter','none');xlabel('X ( m )','interpreter','none');grid on;box on;
% title('Earth inertial frame');

subplot(2,2,3)
plot(t,xd.data,'k-',t,yd.data,'k--',t,zd.data,'k:');
xlim([0 ut]);axis([0 ut -3 3]);xlabel('Time ( s )');ylabel('Speed ( m/s or rad/s )');grid on;box on;
% legend('Speed x: Surge','Speed y: Sway','Speed z: Yaw');
title('( c )','interpreter','none');

subplot(2,2,4)
% plot(t,xd.data,'b-',t,yd.data,'r-',t,zd.data,'b:',t,F.data,'k-',t,Fd.data,'k:');
plot(t,F.data,'k-',t,Fd.data,'k--');
xlim([0 ut]);axis([0 ut 0 50]);xlabel('Time ( s )');ylabel('Force ( N )');grid on;box on;
% legend('Force F','Disturbance force Fd','Location','best');
title('( d )','interpreter','none');


% figure();
% plot(X.data,Y.data); xlabel('X (m)'); ylabel('Y (m)'); grid on; box on; hold on;
% plot(0,0,'bo',Xw,Yw,'r+'); hold on;
% r = wp_tol ; x = Xw ; y = Yw ; theta = linspace(0,2*pi) ; 
% plot(r*cos(theta)+x , r*sin(theta)+y , 'r--') ; hold on;
% text(Xw,Yw+wp_tol,['( ' num2str(Xw) ' , ' num2str(Yw) ' )']);
% axis([X_lo X_hi Y_lo Y_hi]);
 
% text(Xw-1,Yw+1,['( ' num2str(Xw) ' , ' num2str(Yw) ' )']);
% text(0-1,0+1,['( ' num2str(0) ' , ' num2str(0) ' )']);
% xlim([-8 2]); ylim([-4 6]);
% set ( gca , 'xtick' , [-8 -6 -4 -2 0 2] , 'ytick' , [-4 -2 0 2 4 6] ) ;

% figure();
% q=quiver(X.data(1:50:end),Y.data(1:50:end),Xd.data(1:50:end),Yd.data(1:50:end),'k-'); hold on;grid on;box on;
% plot(0,0,'bo',Xw,Yw,'r+'); hold on;
% plot(X.data,Y.data); xlabel('X (m)'); ylabel('Y (m)'); grid on; box on; hold on;

% quiver(X.data,Y.data,gradient(X.data),gradient(Y.data),'r-'); 
