% Simulazione temperatura Pinza tramite script

grid on
set(0,'defaultfigurecolor',[1 1 1])
close all; clear; clc;
tic;

% Inizio e fine dati di interesse [secondi]
start   = 30;             %[s]
finish  = 480;            %[s]

% -------------Dati impianto front    
dP           =      0.024;                                             % Diametro pistoncini [m]
dM           =      0.174;                                             % Diametro medio disco [m]
nP           =      4;                                                 % Numero pistoncini
caliperMass  =      0.167;                                             % Massa disco [kg]
caliperArea  =      30723e-6;                                          % Area disco [m2]
% Cp           =      interp1(linspace(0,800,9),[420 450 480 500 520 540 540 540 540],linspace(0,800,800)); % Calore specifico disco [J/(kg*K)]
Cp           =      interp1(linspace(0,400,9),[850 900 930 950 975 1000 1000 1000 1000],linspace(0,800,800)); % Calore specifico pinza [J/(kg*K)]
% -------------------------------

% load('COMPLETE_2024_11_08-10_34-Martelli-CERVESINA-RUN1_all_from_CSV.mat');       % duct
load('COMPLETE_2024_11_08-11_00-Martelli-CERVESINA-RUN2_all_from_CSV.mat');       % no duct

% Trova gli indici dei valori di start e finish in Timestamp
Timestamp = Data.pBrakeF(:,1);
[~, idxStart] = min(abs(Timestamp - start));
[~, idxFinish] = min(abs(Timestamp - finish));

% Variabile Temperatura
tBrakeCaliper = movmean(Data.tBrakeFR(:,2),20);
pBrakeF = Data.pBrakeF(:,2);
nMotorFL = (Data.nMotorFL(:,2))./11.5 * 2*pi/60;

% Coefficienti termici prima iterazione
ass          =      0.4;                                               % Percentuale potenza termica assorbita dal disco
irr          =      0.7;                                               % Coefficiente irraggiamento
maxDelay     =     1000;


% Dati storia di carico
T0           =      7;                                                % Temperatura ambiente

time100Hz = (1:1:length(tBrakeCaliper))';
fine = time100Hz(end);

% Taglia inizio
BPS              =      [time100Hz pBrakeF];
wWheel           =      [time100Hz nMotorFL];
TempC            =      [time100Hz tBrakeCaliper];
Timestamp        =      [time100Hz Timestamp];

    for b=1:fine
        if BPS(b,2) < 1
            BPS(b,2) = 0;
        end
    end

% mu front
      mu = interp1([0 100 200 300 400 500 600 700 1000],[0.2 0.65 0.7 0.75 0.85 0.75 0.6 0.5 0.5],linspace(0,1000,1000));

% mu rear
%     mu = interp1([0 50 100 200 300 400],[0.1 0.35 0.55 0.6 0.55 0.45],linspace(0,400,400)); 


% Plot variabili acquisite

figure;

% subplot Braking Pressure
subplot(3, 1, 1);
plot(Timestamp(:,2), BPS(:,2), 'b', 'LineWidth', 2);
xlabel('Time');
ylabel('Braking Pressure (bar)');
grid on;

% subplot Wheel Angular Velocity
subplot(3, 1, 2);
plot(Timestamp(:,2), wWheel(:,2), 'g', 'LineWidth', 2);
xlabel('Time');
ylabel('Wheel Angular Velocity (rad/s)');
grid on;

% subplot Brake Temperature
subplot(3, 1, 3);
plot(Timestamp(idxStart:idxFinish,2), TempC(idxStart:idxFinish,2), 'r', 'LineWidth', 2);
xlabel('Time');
ylabel('Brake Temperature (°C)');
grid on;

sgtitle('Plot variabili acquisite');

% Impostazioni ottimizzatore

T(maxDelay-1) = tBrakeCaliper(maxDelay-1);

f = @(x)optFUN(x,mu,BPS,nP,dM,dP,wWheel,caliperArea,caliperMass,Cp,T,T0,TempC, fine, Timestamp,maxDelay);     % ottimizzatore con tutti gli input costanti e x da ottimizzare


% surrogate
options = optimoptions('surrogateopt','PlotFcn','surrogateoptplot');
options.MaxFunctionEvaluations = 500;

[x, err] = surrogateopt(f,[0.1 0.1 0.05 0],[50 1 0.15 maxDelay],[],[],[],[],[], options);   % Funzione che da in output x e err

[err,Torque,P_tot,P_mech,P_conv,P_irr,T] = optFUN(x,mu,BPS,nP,dM,dP,wWheel,caliperArea,caliperMass,Cp,T,T0,TempC, fine, Timestamp,maxDelay);    % Ricalcolo con valori ottimizzati

figure;
width = 600;    % Larghezza in pixel
height = 600;   % Altezza in pixel
set(gcf, 'Position', [100, 100, width, height]);
plot(TempC(1:fine,1), TempC(1:fine,2), TempC(1:fine,1), T(:));
grid on;
legend('Measured temperature','Simulated temperature');
xlabel('time');
ylabel('Disk temperature')

% Salvare la figura
% figureHandle = gcf;
% saveas(figureHandle, 'report_temp.png');
% disp('Report salvato con successo.');

% save('H.mat',"H");
% save('omega.mat',"omega");

figure;
H = x(1)+x(2)*(linspace(0,150,10));
plot(linspace(0,150,10),H);
title('Convettivo');
xlabel('[Rad/s]')

toc;




% Calcolo temperatura secondo modello analitico
function [err,Torque,P_tot,P_mech,P_conv,P_irr,T] = optFUN(x,mu,BPS,nP,dM,dP,wWheel,caliperArea,caliperMass,Cp,T,T0,TempC, fine,Timestamp,maxDelay)

H_t          = x(1) + wWheel(:,2)*x(2);

ass = x(3);              % Percentuale potenza assorbita da pastiglie
delay = x(4);
irr = 0.05;             % Coeff irraggiamento
Boltzmann = 5.67e-8;    % [W/m^2 K^4]

Torque(maxDelay-1) = 0;

for t=(maxDelay):fine
    Torque(t)   = mu(floor(T(t-1)))*(BPS(t,2)/10)*(BPS(t,2)/10); % Coppia frenante
    P_mech(t)   = ass*Torque(t)*wWheel(t,2);                                % Potenza meccanica * ass
    P_irr(t)    = caliperArea*Boltzmann*irr*((T(t-1)+273)^4-(T0+273)^4);    % Potenza dissipata per irragiamento
    P_conv(t)   = H_t(t)*(T(t-1)-T0)*caliperArea;   % Potenza dissipata per convettivo

    P_tot(t)    = P_mech(t-delay)-P_conv(t)-P_irr(t);                             % Potenza netta in entrata  
    deltaT(t)   = (P_tot(t).*(Timestamp(t,2)-Timestamp(t-1,2)))./(Cp(floor(T(t-1)))*caliperMass);   % Incremento di temperatura
    
    T(t)        = T(t-1)+deltaT(t);
end
err = rms(T(2:fine)'-TempC(2:fine,2));

if isnan(err)
    err=1000;
end
end