   clear all;close all;clc

   pkg load control
   s = tf('s');

   % TREN DE ENGRANAJES REDCUTOR

   % Modelo: 406769
   % Relación de transformación; 100:1
   TLmax=4.5; %N*m
   J_t = 0.31 * 0.0001/1000; % Kg/m^2
   B_t = 0; %Según datasheet
   n = 1/71;


   %  MOTOR
   % Modelo:  339152
   Wnom=9690*2*pi/60; %velocidad Nominal en Rad/s
   R = 1.53;     % Ohm
   L_ind= 0.186e-3; % H
   Ke = 1/(460 * 2*pi/60); % rad/(seg*V)
   Kt = 20.8e-3; % Nm/A
   J_m = 14.7 * 0.0001/1000; % Kg/m^2
   B_m = 0.7325e-6; %según datasheet


   %CARGA
   %Brazo Robotico
   TL=TLmax*0.5;      %Torque de Carga en N*m como el 50% de la carga maxima del tren reductor
   L_brazo = 0.35;    %Longitud del Brazo en metros
   g = 9.81;          % gravedad m/s^2

   mL=TL/(g*(L_brazo/2));     %Masa Equivalente del Brazo en Kg (con centro de graverdad en L_brazo/2)
   J_l =(1/3)*mL*(L_brazo)^2; %Inercia de la Carga en el eje secundario en Kg/m^2

   WL=Wnom*n;      %Velocidad Nominal en el Eje Secundaro en Rad/s
   B_l = TL/WL;    %Friccion de la carga en el eje secundario en N*m*s/rad



   Je = J_m + J_t + J_l*n^2;  %Inercia equivalente
   Be = B_m + B_t + B_l*n^2;  %Friccion equivalente


   %SENSOR DE POSICION
   %Potenciometro:
   Vref=24; %Tension de Referencia del Potenciometro
   Kpot = Vref/(20*pi); % V/rad realimentación(Ganancia del Potenciometro)


   %SETs POINTs
   V_step=12; %V
   V_ramp=12; %V

   %Torque de Perturbacion Externa
   TL_per=TL;
   TL_eq = TL_per * n; %Torque Reflejado al Eje del Motor

   %SISTEMA A LAZO CERRADO SIN COMPENSADOR
   %Algebra de Bloques :
   G1 = 1/(R + s*L_ind);
   G2 = 1/(Be + s*Je);
   F1 = feedback(G1*Kt*G2,Ke);
   F2 = F1*n/s;
   L = minreal(F2 * Kpot); %FDT de Lazo Abierto
   FdT = minreal(feedback(F2,Kpot));% Theta(s)/Vin(s)

   %Mason para la FDT de la Pertubacion:
   L1=-Ke*Kt*G1*G2;
   L2=-G2*n*(1/s)*Kpot*G1*Kt;
   P1=-G2*n/s;
   Delta=1-(L1+L2); %Ecuacion Caracterisitca de Todo el Sistema no Compensado
   FdT_per=minreal(P1/Delta); % Theta(s)/Text(s)



   disp('==========================================================================')
   disp('===============FUNCION DE TRANSFERNCIA DE LAZO CERRADO====================')
   disp('==========================================================================')
   FdT%FDT de Lazo Cerrado
   disp("Polos de Lazo Cerrado:");
   disp(pole(FdT));
   disp("Ceros de Lazo Cerrado:");
   disp(zero(FdT));
   if(isstable(FdT))
     disp("El sistema a lazo cerrado es estable.");
   else
     disp("El sistema a lazo cerrado es inestable.");

   endif

   %Mapa de Polos de Lazo cerrado
   figure(1)
   pzmap(FdT,'r');
   sgrid on;
   title("Polos de Lazo Cerrado del Sistema no compensado");

   %Lugar de Raices del Sistema
   figure(2)
   rlocus(L); grid on; title("LGR del sistema no compensado")



   %Simulaciones de Respuestas Temporales
   t = 0:0.01:20;
   %Entradas de Tension
   u_step = V_step*ones(size(t));
   u_ramp = V_ramp*t;
   %Entrada de Perturbacion
   u_per = TL_eq*ones(size(t));

   %Referencias Ideales de Posicion Angular en Radianes
   ref_step = u_step / Kpot;
   ref_ramp = u_ramp / Kpot;

   %Respuestas Del Sistema
   [y_step,t]=lsim(FdT,u_step,t);
   [y_ramp,t]=lsim(FdT,u_ramp,t);
   [y_per,t]=lsim(FdT_per,u_per,t);


   %Grafico de Respuestas sin Pertubacion
   figure(3);
   subplot(2,1,1);
   plot(t,y_step,'r', 'LineWidth', 1.5); hold on;
   plot(t, ref_step, '--k', 'LineWidth', 1.2); hold off;
   grid on; title("Respuesta al Escalon no compensada")
   xlabel('Tiempo[s]'); ylabel('Angulo[rad]');
   legend('Respuesta y(t)', 'Referencia r(t)', 'Location', 'southeast');

   subplot(2,1,2);
   plot(t,y_ramp,'r', 'LineWidth', 1.5); hold on;
   plot(t, ref_ramp, '--k', 'LineWidth', 1.2); hold off;
   grid on; title("Respuesta a la Rampa no compensada")
   xlabel('Tiempo[s]'); ylabel('Angulo[rad]');
   legend('Respuesta y(t)', 'Referencia r(t)', 'Location', 'southeast');

   %Grafico de Respuestas con el Efecto de la Pertubacion
   figure(4);
   subplot(2,1,1);
   plot(t,y_step+y_per,'r','LineWidth', 1.5); hold on;
   plot(t, ref_step, '--k', 'LineWidth', 1.2); hold off;
   grid on; title("Respuesta al Escalon no compensada con Perturbacion Constante")
   xlabel('Tiempo[s]'); ylabel('Angulo[rad]');
   legend('Respuesta y(t)', 'Referencia r(t)', 'Location', 'southeast');

   subplot(2,1,2);
   plot(t,y_ramp+y_per,'r', 'LineWidth', 1.5); hold on;
   plot(t, ref_ramp, '--k', 'LineWidth', 1.2); hold off;
   grid on; title("Respuesta a la Rampa no compensada con Perturbacion Constante")
   xlabel('Tiempo[s]'); ylabel('Angulo[rad]');
   legend('Respuesta y(t)', 'Referencia r(t)', 'Location', 'southeast');



   disp('==========================================================================')
   disp('===============ESPECIFICACIONES DEL SISTEMA NO COMPENSADO=================')
   disp('==========================================================================')


   disp('Errores en regimen permanente:')
   disp('------------------------------')
   disp('Para Entrada Escalon:')
   Kp = dcgain(L)
   ess_step = 1/(1+Kp)
   disp('Para Entrada de Rampa:')
   Kv = dcgain(minreal(s*L))
   ess_ramp = 1/Kv
   disp('------------------------------')
   disp('Sobrepasamiento Mp(%):')
   disp('------------------------------')
   y_final = y_step(end);
   y_max = max(y_step);
   Mp=100*(y_max-y_final)/y_final

   disp('--------------------------------')
   disp('Tiempo de Establecimiento ts(s):')
   disp('--------------------------------')
   banda_error = abs(y_step - y_final);
   indices_fuera = find(banda_error > (0.02 * y_final));  %calcula la ventana del 2% del valor final y busca todas las posiciones en el vector donde el error es mayor al 2%
   if isempty(indices_fuera)
     ts = t(1);
   else % ultimo tiempo en que la señal estuvo fuera de la banda del 2%
     ts = t(indices_fuera(end))
   endif

   disp('-------------------------------------')
   disp('Rechazo a las Pertubaciones Externas:')
   disp('-------------------------------------')
   TL_final=dcgain(TL_eq*FdT_per)
   if(TL_final==0)
    disp('El Sistema Rechaza las Perturbaciones Externas Constantes');
   else
    disp('El Sistema NO Rechaza las Perturbaciones Externas Constantes');

 endif




   %stepinfo(FdT)

   % DISEÑO DEL COMPENSADOR

   %Ki = -1/p(3)

   %1/Ki

   %PI = (s + 1/Ki)/s

   %p = pole(minreal(FdT*PI))
  % z = zero(minreal(FdT*PI))

   %rlocusx(minreal(FdT*PI))

   %K = 22.62; % n = 1/100
  % K = 9.38; % n = 1/71

  % FdTC = minreal(feedback(minreal(F2*PI*K),Kpot))

   %t = 0:0.1:20;


  % figure;lsim(FdTC*Kpot,u_step,t)
  % figure;lsim(FdTC*Kpot,u_ramp,t)

  % L = minreal(PI * F2 * Kpot);

   %Kp = dcgain(L);

   %ess_step = 1/(1+Kp)

  % Kv = dcgain(minreal(s*L));

   %fprintf('Kv = %f\n',Kv);

  % ess_ramp = 1/Kv;

  % fprintf('Error estacionario ante rampa = %f\n',ess_ramp);

