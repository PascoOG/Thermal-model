clear; clc;close all;

load('COMPLETE_2024_06_27-14_55-Pagani-Cervesina-RUN2_all_from_CSV');   % Vecchia FEM
% load('COMPLETE_2024_11_08-10_34-Martelli-CERVESINA-RUN1_all_from_CSV.mat');
% load('COMPLETE_2024_06_27-14_55-Pagani-Cervesina-RUN2_all_from_CSV');

front.pBrake = Data.pBrakeF(:,2);
rear.pBrake = Data.pBrakeR(:,2);
vx = Data.Vx_DATRAW(:,2);
time = Data.Vx_DATRAW(:,1);
heatFlux.FEMold = load("AbaqusOld.txt").*0.37;

% Plot pressioni frenanti
figure(1);
subplot (2,1,1);
plot (time, front.pBrake,'DisplayName','pBrake_{front}'); hold on;
title('pBrake_{front}');
xlabel('Time [s]');
ylabel('Pressure [Bar]');
ylim([-1 45]);
grid on;
subplot (2,1,2);
plot (time, rear.pBrake,'DisplayName','pBrake_{rear}');
title('pBrake_{rear}');
xlabel('Time [s]');
ylabel('Pressure [Bar]');
ylim([-1 45]);
grid on;

wheel_radius = 0.198;
Pad_Surface = 1142.93222335*1e-6;

% Data front
front.muPad = 0.8;
front.disk_radius = 0.170/2;
front.Sup_pistons = 4 * pi * 12^2 * 1e-6;                                           % [m^2]

% Data rear
rear.muPad = 0.6;
rear.disk_radius = 0.165/2;
rear.Sup_pistons = 2 * pi * 11^2 * 1e-6;                                            % [m^2]

Angular_Velocity = vx ./ wheel_radius;                                              % [rad/s]

% Heat surface flux
% Forze front
front.brakingForce = front.pBrake * 1e5 * front.Sup_pistons * front.muPad;          % [N]
front.torque = front.brakingForce * front.disk_radius;                              % [Nm]
front.brakingPower = front.torque .* Angular_Velocity;                              % [w]
front.brakingPower_abaqus = 1e3 * front.brakingPower(2362 : end);
time_abaqus = time(1:length(front.brakingPower_abaqus));

% Forze rear
rear.brakingForce = rear.pBrake * 1e5 * rear.Sup_pistons * rear.muPad;              % [N]
rear.torque = rear.brakingForce * rear.disk_radius;                                 % [Nm]
rear.brakingPower = rear.torque .* Angular_Velocity;                                % [w]
rear.brakingPower_abaqus = 1e3 * rear.brakingPower(2362 : end);

% Plot POWER
figure(3);
subplot (2,1,1);
plot (time, front.brakingPower,'DisplayName','brakingPower_{front}'); hold on;
title('Torque_{front}');
xlabel('Time [s]');
ylabel('[2]');
% ylim([0 450]);
grid on;
subplot (2,1,2);
plot (time, rear.brakingPower,'DisplayName','brakingPower_{rear}')
title('brakingPower_{rear}');
xlabel('Time [s]');
ylabel('[w]');
% ylim([0 450]);
grid on;

% Plot coppie frenanti
figure(2);
subplot (2,1,1);
plot (time, front.pBrake,'DisplayName','pBrake_{front}'); hold on;
title('pBrake_{front}');
xlabel('Time [s]');
ylabel('Pressure [Bar]');
ylim([-1 45]);
grid on;
subplot (2,1,2);
plot (time, rear.pBrake,'DisplayName','pBrake_{rear}');
title('pBrake_{rear}');
xlabel('Time [s]');
ylabel('Pressure [Bar]');
ylim([-1 45]);
grid on;


% Output FEM

time_FEM = [];
Temp_termotape = [];

% Temp Disk
front.tempRight_acq = Data.tBrakeFR(:,2);
front.tempLeft_acq = Data.tBrakeFL(:,2);

rear.tempRight_acq = Data.tBrakeRR(:,2);
rear.tempLeft_acq = Data.tBrakeRL(:,2);

timeTemp = Data.tBrakeFL(:,1);

figure(4);
title('Temp Disks')
plot(timeTemp, front.tempRight_acq,'DisplayName','Front_{Right}'); hold on;
plot(timeTemp, front.tempLeft_acq,'DisplayName','Front_{Left}');
plot(timeTemp, rear.tempRight_acq,'DisplayName','Rear_{Riglt}');
plot(timeTemp, rear.tempLeft_acq,'DisplayName','Rear_{Left}');
ylabel('[°C]');
legend show;

% Corrente pacco
acc.power = Data.vTSACC(:,2) .* Data.iAccOutput(:,2);


% Calcolo regen
front.powerLeft = Data.cMotorFL(:,2).*(Data.nMotorFL(:,2)*pi/30);
front.powerRight = Data.cMotorFR(:,2).*(Data.nMotorFR(:,2)*pi/30);
rear.powerLeft = Data.cMotorRL(:,2).*(Data.nMotorRL(:,2)*pi/30);
rear.powerRight = Data.cMotorRR(:,2).*(Data.nMotorRR(:,2)*pi/30);

powerMotorTot = front.powerLeft+front.powerRight+rear.powerLeft+rear.powerRight;

front.regenLeft = front.powerLeft.*(front.powerLeft<0);
front.regenRight = front.powerRight.*(front.powerRight<0);
rear.regenLeft = rear.powerLeft.*(rear.powerLeft<0);
rear.regenRight = rear.powerRight.*(rear.powerRight<0);


% --------- Calcolo nuova regen
front.regenLeftNew = front.regenLeft * 24/15;
front.regenLeftDelta = front.regenLeft*(24/15-1);

% Nuova regen Abaqus
front.brakingPower_abaqus_New = 1e3 * (front.brakingPower(2362 : end)+front.regenLeftDelta(2362 : end));
front.brakingPower_abaqus_New = front.brakingPower_abaqus_New .* (front.brakingPower_abaqus_New>0);

heatFlux.new = 1e-3 * (front.brakingPower_abaqus_New./Pad_Surface); % mW/mm^2
heatFlux.old = 1e-3 * (front.brakingPower_abaqus./Pad_Surface); % mW/mm^2

figure;
plot(timeTemp(2363 : end), heatFlux.FEMold,'DisplayName','Old riferimento','Color','g'); hold on;
plot(timeTemp(2362 : end), heatFlux.old,'DisplayName','heat flux abaqus','Color','b');
plot(timeTemp(2362 : end), heatFlux.new,'DisplayName','heat flux abaqus new','Color','r');

ylabel('[mW/mm^2]');

legend show;

% Plot Regen
figure;
subplot(2,1,1)
plot(timeTemp, front.regenLeft,'DisplayName','Regen_{Left}');hold on;
plot(timeTemp, front.regenRight,'DisplayName','Regen_{Right}');
legend show;
ylabel('Power [W]');
subplot(2,1,2)
plot(timeTemp, acc.power,'DisplayName','Potenza Pacco'); hold on;
plot(timeTemp, powerMotorTot,'DisplayName','Potenza Motori');
ylabel('Power [W]');
legend show;

% Grafico a bande cumulative
powerMatrix = [front.powerLeft, front.powerRight, rear.powerLeft, rear.powerRight];
figure;
area(timeTemp, powerMatrix, 'LineStyle', 'none');

xlabel('Time');
ylabel('Power [W]');
title('Power Distribution');
legend({'Front Left', 'Front Right', 'Rear Left', 'Rear Right'}, 'Location', 'Best');
grid on;

% Vx adimensionale
amp_vx = vx(2362:end) / max(vx);

% Calcolo matrice per Abaqus
outputAbaqus(:,1) = timeTemp(2362 : end)-timeTemp(2362);
outputAbaqus(:,2) = heatFlux.new;
stepTime = outputAbaqus(end,1) - outputAbaqus(1,1);
disp('Steptime:');
disp(stepTime);
