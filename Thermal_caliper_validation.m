clear; clc; close all;

% load('COMPLETE_2024_06_27-14_55-Pagani-Cervesina-RUN2_all_from_CSV');     % Vecchia FEM
% load('COMPLETE_2024_11_08-10_34-Martelli-CERVESINA-RUN1_all_from_CSV.mat'); % Run DUCT
load('COMPLETE_2024_11_08-11_00-Martelli-CERVESINA-RUN2_all_from_CSV.mat');     %Run no duct

% Tempo acquisizione interessante da buttare ad abaqus
test.start = 30;
test.end = 480;

front.pBrake = Data.pBrakeF(:,2);
rear.pBrake = Data.pBrakeR(:,2);
vx = Data.Vx_DATRAW(:,2);
time = Data.Vx_DATRAW(:,1);
% heatFlux.FEMold = load("AbaqusOld.txt").*0.37;


[~, test.start] = min(abs(time - test.start));
[~, test.end] = min(abs(time - test.end));

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

wheel_radius = 0.198;                                                               % [m]
Pad_Surface = 2*1142.9 * 1e-6;                                                      % Area di entrambe le Pad

% Data front
front.muPad = 0.8;
front.disk_radius = 0.170/2;
front.Sup_pistons = 4 * pi * 12^2 * 1e-6;                                           % [m^2]

% Data rear
rear.muPad = 0.6;
rear.disk_radius = 0.165/2;                                                         % [m]
rear.Sup_pistons = 2 * pi * 11^2 * 1e-6;                                            % [m^2]

Angular_Velocity = vx ./ wheel_radius;                                              % [rad/s]

% Forze front
front.brakingForce = front.pBrake * 1e5 * front.Sup_pistons * front.muPad;          % [N]
front.torque = front.brakingForce * front.disk_radius;                              % [Nm]
front.brakingPower = front.torque .* Angular_Velocity;                              % [w]
front.brakingPower_abaqus = 1e3 * front.brakingPower;

% Forze rear
rear.brakingForce = rear.pBrake * 1e5 * rear.Sup_pistons * rear.muPad;              % [N]
rear.torque = rear.brakingForce * rear.disk_radius;                                 % [Nm]
rear.brakingPower = rear.torque .* Angular_Velocity;                                % [w]
rear.brakingPower_abaqus = 1e3 * rear.brakingPower;

% Plot POWER
figure(2);
subplot (2,1,1);
plot (time, front.brakingPower,'DisplayName','brakingPower_{front}'); hold on;
title('Power_{front}');
xlabel('Time [s]');
ylabel('[w]');
grid on;
subplot (2,1,2);
plot (time, rear.brakingPower,'DisplayName','brakingPower_{rear}')
title('Power_{rear}');
xlabel('Time [s]');
ylabel('[w]');
grid on;

% Plot coppie frenanti
figure(3);
subplot (2,1,1);
plot (time, front.torque,'DisplayName','Torque_{front}'); hold on;
title('Torque_{front}');
xlabel('Time [s]');
ylabel('Torque [Nm]');
ylim([-1 450]);
grid on;
subplot (2,1,2);
plot (time, rear.torque,'DisplayName','Torque_{rear}');
title('Torque_{rear}');
xlabel('Time [s]');
ylabel('Torque [Nm]');
ylim([-1 450]);
grid on;


% Temp Disk
front.tempCaliper = Data.tBrakeFR(:,2);
front.tempLeft_acq = Data.tBrakeFL(:,2);

rear.tempRight_acq = Data.tBrakeRR(:,2);
rear.tempLeft_acq = Data.tBrakeRL(:,2);

timeTemp = Data.tBrakeFL(:,1);

figure(4);
title('Temp Disks')
plot(timeTemp, front.tempCaliper,'DisplayName','Caliper_{FL}'); hold on;
plot(timeTemp, front.tempLeft_acq,'DisplayName','Front_{Left}');
ylabel('[°C]');
legend show;
grid on;

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

% Nuova regen Abaqus
front.brakingPower_abaqus = 1e3 * (front.brakingPower);
front.brakingPower_abaqus = front.brakingPower_abaqus .* (front.brakingPower_abaqus>0);

% Heat surface flux
heatFlux.new = 1e-3 * (front.brakingPower_abaqus./Pad_Surface);             % [mW/mm^2]

% Plot heat flux
figure;
plot(timeTemp(test.start:test.end), heatFlux.new(test.start:test.end),'DisplayName','heat flux abaqus','Color','m');
ylabel('[mW/mm^2]');
legend show;

% Plot Regen
% figure;
% subplot(2,1,1)
% plot(timeTemp, front.regenLeft,'DisplayName','Regen_{Left}');hold on;
% plot(timeTemp, front.regenRight,'DisplayName','Regen_{Right}');
% legend show;
% ylabel('Power [W]');
% subplot(2,1,2)
% plot(timeTemp, acc.power,'DisplayName','Potenza Pacco'); hold on;
% plot(timeTemp, powerMotorTot,'DisplayName','Potenza Motori');
% ylabel('Power [W]');
% legend show;

% Grafico a bande cumulative
% powerMatrix = [front.powerLeft, front.powerRight, rear.powerLeft, rear.powerRight];
% figure(7);
% area(timeTemp, powerMatrix, 'LineStyle', 'none');
% xlabel('Time');
% ylabel('Power [W]');
% title('Power Distribution');
% legend({'Front Left', 'Front Right', 'Rear Left', 'Rear Right'}, 'Location', 'Best');
% grid on;

% Vx adimensionale
load("H.mat");
load('omega.mat');
figure(20);
plot(omega*wheel_radius*3.6,H);
xlabel('[km/h]');
ylabel('[H]');
grid on;

convettivo = interp1(omega, H, Angular_Velocity, 'linear', 'extrap');
convettivoNormalizzato = convettivo./max(convettivo);
conv_abaqus(:,1) = timeTemp(test.start:test.end)-timeTemp(test.start);
conv_abaqus(:,2) = convettivoNormalizzato(test.start:test.end);

figure(8)
title('Covettivo normalizzato')
yyaxis left;
plot(timeTemp(test.start:test.end), convettivoNormalizzato(test.start:test.end),'DisplayName','convettivo'); hold on;
ylim([0 1]);
yyaxis right;
plot(timeTemp(test.start:test.end), vx(test.start:test.end)*3.6,'DisplayName','Vx [km/h]');
legend show;


% Calcolo matrice per Abaqus
outputAbaqus(:,1) = timeTemp(test.start:test.end)-timeTemp(test.start);
outputAbaqus(:,2) = heatFlux.new(test.start:test.end);
stepTime = outputAbaqus(end,1) - outputAbaqus(1,1);
disp('Steptime:');
disp(stepTime);


% Confronto FEM - test
load("temperature.mat");    % diverse curve di temperatura pinza da n-fem
incrementAbaqus = 2;
timeAbaqus = 1:1:size(temperature,1);
timeAbaqus = timeAbaqus*incrementAbaqus + timeTemp(test.start);

figure(10)
title('Temp')
plot(timeTemp(test.start:test.end), front.tempCaliper(test.start:test.end),'DisplayName','Caliper_{FL}','LineWidth',2); hold on;
legend show,
ylabel('[°C]');
xlabel('Time');
grid on;

for j=1:size(temperature,2)
    plot(timeAbaqus, temperature(:,j),"LineWidth",1.5, 'DisplayName', sprintf('JOB_{%.0f}', j));
end

yyaxis right;
plot(timeTemp(test.start:test.end), heatFlux.new(test.start:test.end),timeTemp(test.start:test.end), vx(test.start:test.end).*10e5);


% Calcolo errore di ogni JOB
errors = zeros(1, size(temperature, 2));
for j = 1:size(temperature, 2)
    % Interpolazione dei dati di Abaqus ai tempi del caliper
    interpTemperature = interp1(timeAbaqus, temperature(:, j), timeTemp(test.start:test.end), 'linear', 'extrap');
    
    % Calcolo dell'errore RMS
    errors(j) = rms(interpTemperature - front.tempCaliper(test.start:test.end));
    
end

[minError, minIndex] = min(errors);  % Trova il valore minimo e l'indice
fprintf('Errore RMS per JOB_{%.0f}: %.0f\n', minIndex, minError);
