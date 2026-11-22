   clear all;close all;clc

   pkg load control
   s = tf('s');

   %  TREN DE ENGRANAJES

   % Modelo: 406769
   % Relación de transformación; 100:1

   J_t = 0.31 * 0.0001/1000; % Kg/m^2
   B_t = 0; %Según datasheet
   n = 1/71;

   %  MOTOR
   % Modelo:  339152
   % V nominal: 12

   R = 1.53;     % Ohm
   L = 0.186e-3; % H
   Ke = 1/(460 * 2*pi/60); % rad/(seg*V)
   Kt = 20.8e-3; % Nm/A
   J_m = 14.7 * 0.0001/1000; % Kg/m^2
   B_m = 0.7325e-6; %según datasheet

   B_l = 0.075;
   J_l = 0.0535;

   Je = J_m + J_t + J_l*n^2;
   Be = B_m + B_t + B_l*n^2;

   G1 = 1/(R + s*L);
   G2 = 1/(Be + s*Je);

   %Kpot = 0.159; % V/rad realimentación
   Kpot = 24/(20*pi);

   % SISTEMA A LAZO CERRADO SIN COMPENSADOR

   F1 = feedback(G1*Kt*G2,Ke);
   F2 = F1*n/s; %Lazo abierto
   FdT = minreal(feedback(F2,Kpot))

   t = 0:0.1:20;

   u_step = 12*ones(size(t));

   u_ramp = 12*t;

   figure;lsim(FdT*Kpot,u_step,t)
   figure;lsim(FdT*Kpot,u_ramp,t)

   %figure;pzmap(FdT)
   p = pole(FdT)
   z = zero(FdT)

   VF = dcgain(FdT*12)*Kpot

   L = minreal(F2 * Kpot);

   Kp = dcgain(L);

   ess_step = 1/(1+Kp)

   Kv = dcgain(minreal(s*L));

   fprintf('Kv = %f\n',Kv);

   ess_ramp = 1/Kv;

   fprintf('Error estacionario ante rampa = %f\n',ess_ramp);

   %stepinfo(FdT)

   % DISEÑO DEL COMPENSADOR

   Ki = -1/(p(3))

   1/Ki

   PI = (s + 1/Ki)/s

   p = pole(minreal(FdT*PI))
   z = zero(minreal(FdT*PI))

   %rlocusx(minreal(FdT*PI))

   %K = 22.62; % n = 1/100
   K = 9.38; % n = 1/71

   FdTC = minreal(feedback(minreal(F2*PI*K),Kpot))

   t = 0:0.1:20;

   u_step = 12*ones(size(t));

   u_ramp = 12*t;

   figure;lsim(FdTC*Kpot,u_step,t)
   figure;lsim(FdTC*Kpot,u_ramp,t)

   L = minreal(PI * F2 * Kpot);

   Kp = dcgain(L);

   ess_step = 1/(1+Kp)

   Kv = dcgain(minreal(s*L));

   fprintf('Kv = %f\n',Kv);

   ess_ramp = 1/Kv;

   fprintf('Error estacionario ante rampa = %f\n',ess_ramp);

