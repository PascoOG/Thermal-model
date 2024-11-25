% Simulazione temperatura Pinza tramite script

grid on
set(0,'defaultfigurecolor',[1 1 1])
close all; clear; clc;
tic;

% Inizio e fine dati di interesse [secondi]
start   = 150;             %[s]
finish  = 600;             %[s]

% -------------Dati impianto front    
dP           =      0.024;                                             % Diametro pistoncini [m]
dM           =      0.174;                                             % Diametro medio disco [m]
nP           =      4;                                                 % Numero pistoncini
caliperMass  =      0.167;                                             % Massa disco [kg]
caliperArea  =      30723e-6;                                          % Area disco [m2]
% Cp           =      interp1(linspace(0,800,9),[420 450 480 500 520 540 540 540 540],linspace(0,800,800)); % Calore specifico disco [J/(kg*K)]
Cp           =      interp1(linspace(0,800,9),[900 900 900 900 900 900 900 900 900],linspace(0,800,800)); % Calore specifico pinza [J/(kg*K)]
omega        =      linspace(0, 150, 5);
c_pad        = 500;     % [j/(kg*k)]
padMass      = 0.05;    % [kg]
% -------------------------------

load('COMPLETE_2024_11_08-10_34-Martelli-CERVESINA-RUN1_all_from_CSV.mat');

Timestamp = Data.pBrakeF(:,1);

% Trova gli indici dei valori di start e finish in Timestamp
[~, idxStart] = min(abs(Timestamp - start));
[~, idxFinish] = min(abs(Timestamp - finish));

% Variabile Temperatura
tBrakeCaliper = movmean(Data.tBrakeFR(:,2),20);

pBrakeF = Data.pBrakeF(:,2);
nMotorFL = (Data.nMotorFL(:,2))./11.5 * 2*pi/60;

% Coefficienti termici prima iterazione
H            =      linspace(10, 200, 5);                              % Coefficiente convettivo
ass          =      0.4;                                               % Percentuale potenza termica assorbita dal disco
irr          =      0.05;                                              % Coefficiente irraggiamento
c_pad        =      c_pad*padMass;                                     % [J/k] calore specifico
k_t          =      2;                                                 % [w/k] coefficiente di trasferimento termicotra pads e caliper
ass          =      0.1;
%% Dati storia di carico
T0           =      7;                                                 % Temperatura ambiente

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


%% Plot variabili acquisite

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

%% Impostazioni ottimizzatore

T(1) = tBrakeCaliper(1);

f = @(x)optFUN(x,mu,BPS,nP,dM,dP,wWheel,caliperArea,caliperMass,Cp,T,T0,TempC, omega, fine, Timestamp,c_pad,k_t);     % ottimizzatore con tutti gli input costanti e x da ottimizzare

% surrogate
options = optimoptions('surrogateopt','PlotFcn','surrogateoptplot');
options.MaxFunctionEvaluations = 200;

[x, err] = surrogateopt(f,[5 10 10 10 10 1 1 0.1],[100 100 100 100 100 5 100 0.3],[],[],[],[],[], options);   % Funzione che da in output x e err

[err,Torque,P_tot,P_mech,P_conv,P_irr,T,H_counter,T_pad,P_transfer] = optFUN(x,mu,BPS,nP,dM,dP,wWheel,caliperArea,caliperMass,Cp,T,T0,TempC, omega, fine, Timestamp,c_pad,k_t);    % Ricalcolo con valori ottimizzati

H         = [x(1) x(1)+x(2) x(1)+x(2)+x(3) x(1)+x(2)+x(3)+x(4) x(1)+x(2)+x(3)+x(4)+x(5)];
H_counter = H_counter';
c_pad = x(6);
k_t = x(7);
ass = x(8);

figure;
width = 600;    % Larghezza in pixel
height = 600;   % Altezza in pixel
set(gcf, 'Position', [100, 100, width, height]);
plot(TempC(1:fine,1), TempC(1:fine,2), TempC(1:fine,1), T(:)); hold on;
plot(TempC(1:fine,1), T_pad(:), 'b--', 'LineWidth', 1);
grid on;
legend('Measured temperature','Simulated temperature','Pad Temperature');
xlabel('time');
ylabel('Temperature [°C]')

fprintf('H: ');
disp(H);
fprintf('H_counter: ');
disp(H_counter);

figure;
width = 600;    % Larghezza in pixel
height = 600;   % Altezza in pixel
set(gcf, 'Position', [100, 100, width, height]);
plot(TempC(1:fine,1), P_transfer(:),TempC(1:fine,1), P_mech(:),TempC(1:fine,1), P_conv(:),TempC(1:fine,1), P_irr(:)); hold on;
grid on;
legend('P Transfer','P Mech','P Conv','P irr');
xlabel('time');
ylabel('[w]');

% Save results to Excel
% xlswrite('report.xlsx', {Data}, 'Sheet1', 'B1');
% xlswrite("report.xlsx", omega, 'Sheet1', 'B2');
% xlswrite("report.xlsx", H, 'Sheet1', 'B3');
% xlswrite("report.xlsx", H_counter, 'Sheet1', 'B4');
% xlswrite("report.xlsx", fine, 'Sheet1', 'B25');
% xlswrite("report.xlsx", err, 'Sheet1', 'B24');

% Salvare la figura
figureHandle = gcf;

% Salva la figura come immagine PNG
saveas(figureHandle, 'report_temp.png');
disp('Report salvato con successo.');

save('H.mat',"H");
save('omega.mat',"omega");

toc;

%% Calcolo temperatura secondo modello analitico
function [err,Torque,P_tot,P_mech,P_conv,P_irr,T,H_counter,T_pad,P_transfer] = optFUN(x,mu,BPS,nP,dM,dP,wWheel,caliperArea,caliperMass,Cp,T,T0,TempC, omega, fine,Timestamp,c_pad,k_t)

H           = [x(1) x(1)+x(2) x(1)+x(2)+x(3) x(1)+x(2)+x(3)+x(4) x(1)+x(2)+x(3)+x(4)+x(5)];
c_pad       = x(6);
k_t         = x(7);
ass         = x(8);

% ass = 0.1;              % Percentuale potenza assorbita da pastiglie
irr = 0.05;             % Coeff irraggiamento
Boltzmann = 5.67e-8;    % [W/m^2 K^4]

Torque(1) = 0;
T_pad(1) = T(1);
H_counter = zeros(length(H),1);

for t=2:fine
    if wWheel(t,2) > 150
        wWheel(t,2) = 150;
        H_counter(5) = H_counter(5) + 1;
    elseif wWheel(t,2) > 131.25 && wWheel(t,2) <= 150
        H_counter(5) = H_counter(5) + 1;
    elseif wWheel(t,2) > 93.75 && wWheel(t,2) <= 131.25
        H_counter(4) = H_counter(4) + 1;
    elseif wWheel(t,2) > 56.25 && wWheel(t,2) <= 93.75
        H_counter(3) = H_counter(3) + 1;
    elseif wWheel(t,2) > 18.75 && wWheel(t,2) <= 56.25
        H_counter(2) = H_counter(2) + 1;
    elseif wWheel(t,2) > 0 && wWheel(t,2) <= 18.75
        H_counter(1) = H_counter(1) + 1;
    elseif wWheel(t,2) <= 0
        H_counter(1) = H_counter(1) + 1;
        wWheel(t,2) = 0;
    end
    
    P_transfer(t) = k_t * (T_pad(t-1) - T(t-1));                            % Calore trasferito da pastiglie a pinza
    P_transfer = P_transfer.*(P_transfer>0);

    Torque(t)   = mu(floor(T(t-1)))*(BPS(t,2)/10)*nP*dM/2*pi*(dP*1000)^2/4; % Coppia frenante
    P_mech(t)   = ass*Torque(t)*wWheel(t,2);                                % Potenza meccanica * ass

    deltaTPad(t) = (P_mech(t).*(Timestamp(t,2)-Timestamp(t-1,2)) - P_transfer(t)) ./ c_pad;     % Equazione per la massa intermedia

    P_irr(t)    = caliperArea*Boltzmann*irr*((T(t-1)+273)^4-(T0+273)^4);    % Potenza dissipata per irragiamento
    P_conv(t)   = (interp1(omega,H,wWheel(t,2))*(T(t-1)-T0))*caliperArea;   % Potenza dissipata per convettivo

    
    % P_tot(t)    = P_mech(t)-P_conv(t)-P_irr(t);                             % Potenza netta in entrata
    P_tot(t)    = P_transfer(t)-P_conv(t)-P_irr(t);                             % Potenza netta in entrata
      
    deltaT(t)   = (P_tot(t).*(Timestamp(t,2)-Timestamp(t-1,2)))./(Cp(floor(T(t-1)))*caliperMass);   % Incremento di temperatura
    
    T(t)        = T(t-1) + deltaT(t);             % temperatura pinza
    T_pad(t)    = T_pad(t-1) + deltaTPad(t);

end
err = rms(T(2:fine)'-TempC(2:fine,2));

if isnan(err)
    err=1000;
end
end