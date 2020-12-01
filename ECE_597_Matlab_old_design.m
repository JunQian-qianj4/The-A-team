%% ECE 597 - MATLAB Script - peak torque and power
%%
%  - - - - - - - - - - - - - - - - - - - - - - - - 
%  PURPOSE:   use MABLAB process and compare data
%  - - - - - - - - - - - - - - - - - - - - - - - -
%   | DATE |   | VER |   | AUTHOR |     | ACTION |
%  2020-11-20    1.0      Jun Qian       Finished
%  - - - - - - - - - - - - - - - - - - - - - - - -
clear all; close all; clc;
%% Vehicle Mechanical
curbMass = 178;         % kg
riderMass = 70;         % kg
meq = curbMass + riderMass;
Ngb = 3.285;            % gearbox final reduction ratio
ngb = 0.97;
rw = (17/2) * 0.0254;   % wheel redious (m)
% EPA parameters
A = 14.8;
B = 0;
C = 0.0271*12.96;             % rolling resistance
Af = 0.67;

%% Electric Machine Selection: HPVES AC 12 72 V
t_rated = 23.3;  % continuous rated torque
p_rated = 18400;
%% define basic parameters
wr = p_rated/t_rated;   % rated rotor angular speed
wmr = wr / Ngb;         % rated motor angular speed
vmr = wmr*rw;           % rated vehicle speed
vts = 180/3.6;          % setup the top speed(m/s)
t(1) = 0;
dt = 0.1;
vm(1) = 0;
i = 1;
distance(1) = 0;
distance2(1) = 0;
tq(1) = 0;
p(1) = 0;
p2(1) = 0;
p3 = 0;
EPA(1) = 0;
V2(1) = 0;
V3(1) = 0;
travel_distance = 1/4; % mile
travel_distance_meter = round(travel_distance * 1609.34) + 1;
% constant torque mode
while distance < travel_distance_meter
    if vm(i) < vmr
        vm(i+1) = vm(i)+dt*(Ngb*ngb*t_rated - rw*(A+B*vm(i)+C*vm(i)^2))/rw/meq;
        % since the vm unit is m/0.1s, so to calculate the distance, the unit should times 1/10.
        % calculate power
        FA = meq * (vm(i+1)-vm(i))/dt;
        FV = A + B*vm(i) + C*vm(i)^2;
        EPA(i+1) = FV;
        V2(i+1) = vm(i+1);
        FC = meq * 9.81 * sin(0);
        FM = FA + FV + FC;
        p(i+1) = FM * vm(i);
        tq(i+1) = t_rated;
    % constant power mode
    else
        vm(i+1) = vm(i) + dt*(ngb*p_rated/vm(i)*rw - rw*(A+B*vm(i)+C*vm(i)*vm(i)))/rw/meq;
        p(i+1) = p_rated;
        tq(i+1) = p(i+1)*rw/vm(i)/Ngb;
    end
    t(i+1) = t(i) + dt;
    distance = trapz(t(1:length(vm)), vm);
    distance2(i+1) = trapz(t(1:length(vm)), vm);
    p2(i+1) = trapz(t(1:length(p)), p);
    p3 = trapz(t(1:length(p)), p);
    i = i + 1;
end
EPA2 = zeros(length(vm));    
EPA2 = A + B.*vm + C.*vm.^2;
%% plot power vs distance
figure();
plot(distance2, p,'LineWidth', 1.5);
xlabel('distance(km)');
ylabel('power');
title('power vs distance');

%% plot torque vs distance
figure();
plot(distance2, tq,'LineWidth', 1.5);
xlabel('distance(km)');
ylabel('torque');
title('torque vs distance');
%% plot speed vs distance
figure();
plot(distance2, vm*3.6,'LineWidth', 1.5);
xlabel('distance(km)');
ylabel('speed(km/h)');
title('speed vs. distance');
%% plot Velocity VS EPA forces at constant torque mode
figure();
plot(V2*3.6, EPA, 'LineWidth', 1.5);
ylabel('EPA forces at constant Torque');
xlabel('Speed (km/h) at constant Torque');
title('speed vs EPA forces at constant Torque')
%% plot  Velocity VS EPA forces at constant power mode
figure();
plot(vm(length(EPA)+1:length(vm))*3.6, EPA2(length(EPA)+1:length(EPA2)), 'LineWidth', 1.5);
xlabel('Speed at constant Power (km/h)');
ylabel('EPA forces at constant pOWER');
title('speed vs EPA forces at constant Power');
%% find the shortest time travels 1/4 mile
time_max = length(t);
fprintf('The fast time to travel 1/4 mile is: %.1f s\n',time_max/10);
fprintf('The total energy consumption is: %.1f \n',p3);

