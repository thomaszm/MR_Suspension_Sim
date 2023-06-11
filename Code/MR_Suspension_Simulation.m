clear all;
clc;

%Factor Variables
s = input('Car Speed, 7.333, 20.1317, 58.66, 88 ft/s: ');
l = input('Road Wavelength, 10, 20, 88 ft: ');

%Skyhook control
skyhook = logical(input(['Use Skyhook Control Algorithm?' ...
    ' 0 (no) or 1 (yes): ']));
if skyhook == true
    skyhook_mod = logical(input(['Modify Skyhook Control Algorithm?' ...
        ' 0 (no) or 1 (yes): ' ]));
else
    skyhook_mod = false;
end

%Constant Input Variables
K = 4000;       %lb/ft (stiffness of car suspension)
W = 3220;       %lb (weight of car)
g = 32.2;       %ft/sec^2 (acceleration due to gravity)
C = 100;        %lb*sec/ft (damping coefficient of car suspension)
Cmr = 10;       %lb*sec/ft (damping coefficient of MR damper suspension)
y0 = 0.1;       %ft (amplitude of road input)
Fyieldmx = 483; %lb (max force in MR damper)

%Integration Constant
B = 1/4;        %Beta
L = 1/2;        %Lambda
dt = 0.0005;    %dsec (time step)

%Calculated Variables
M = W / g;                      %lb/ft/sec^2 (mass of car)
we = 2 * pi * s / l;            %Hz excitation frequency
wn = (K / M) ^ (1/2);           %Hz natural frequency
wratio = we / wn;               %Frequency Ratio

%iInitialize integration variables to zero
x0 = 0.;    %old displacement
dx0 = 0.;   %old velocity
ddx0 = 0.;  %old acceleration

x1 = 0.;    %new displacement
dx1 = 0.;   %new velocity
ddx1 = 0.;  %new acceleration

%The number of integration time points
ntimepts = 10000;

%Time Loop, integration loop
for itime = 1:ntimepts

    %Road position (ft) and derivative
    y = y0 * sin(we * itime * dt);
    dy = we * y0 * cos(we * itime * dt);
    ddy = y0 * -1 * we^2 * sin(we * itime * dt);

    %integration force balance loop
    for iter = 1:10
        
        %Displacement & Velocity Newmark Beta Equations
        x1 = x0 + (dx0 * dt) + (((0.5 - B) * ddx0) + (B * ddx1)) * dt^2;
        dx1 = dx0 + (((1 - L) * ddx0) + (L * ddx1)) * dt;

        %Compute Fyield with MR Damper
        if skyhook == true
            %Modified Skyhook Algorithm
            if skyhook_mod == true
                 if dx1 > 0
                     if dx1 > dy
                         if ddx1 > ddy
                             Fyield = Fyieldmx;
                         else
                             Fyield = 0;
                         end
                     else
                         Fyield = 0;
                     end
                 else
                     if dx1 > dy
                         Fyield = 0;
                     else
                         if ddx1 > ddy
                            Fyield = 0;
                         else
                            Fyield = Fyieldmx;
                         end
                     end
                 end

            %Original Skyhook Algorithm
            else 
                if dx1 > 0 && dx1 > dy
                    Fyield = Fyieldmx;
    
                elseif dx1 > 0 && dx1 <= dy
                    Fyield = 0;
    
                elseif dx1 < 0 && dx1 > dy
                    Fyield = 0;
                    
                else
                    Fyield = Fyieldmx;
    
                end
            
            end

            %Calculate acceleration
            Fs = K * (x1 - y);                      %Spring Force
            Fd = C * (dx1 - dy);                    %Damper Force
            Fa = (Fyield * sign(dx1 - dy)) + (Cmr * (dx1 - dy)); %MR Force
            ddx1 = (-1 / M) * (Fs + Fd + Fa);       %Acceleration
        
        else
            %standard suspension
            Fs = K * (x1 - y);
            Fd = C * (dx1 - dy);
            ddx1 = (-1 / M) * (Fs + Fd);

        end 

    end 

    %Set old values equal to new values
    x0 = x1;
    dx0 = dx1;
    ddx0 = ddx1;
    
    %store variables in arrays
    x(itime) = x1;                      %ft
    v(itime) = dx1;                     %ft/s
    a(itime) = ddx1;                    %ft/sec^2
    time(itime) = itime * dt;           %sec
    spring_force(itime) = Fs;           %lb
    damper_force(itime) = Fd;           %lb
    if skyhook == true
        mr_force(itime) = Fa;           %lb
    
    else
        mr_force(itime) = 0;

    end

end



%Calculate max values in second half of array
xmax = max(x(0.5 * length(x):length(x)), [], 'ComparisonMethod', 'abs');
vmax = max(v(0.5 * length(v):length(v)), [], 'ComparisonMethod', 'abs');
amax = max(a(0.5 * length(a):length(a)), [], 'ComparisonMethod', 'abs');
spring_max = max(spring_force(0.5 * ...
    length(spring_force):length(spring_force)), ...
    [], 'ComparisonMethod', 'abs');
damper_max = max(damper_force(0.5 * ...
    length(damper_force):length(damper_force)), ...
    [], 'ComparisonMethod', 'abs');
mr_max = max(mr_force(0.5 * ...
    length(mr_force):length(mr_force)), ...
    [], 'ComparisonMethod', 'abs');


%Calculate velocity ratio
vratio = vmax / s;

%Print Variables
fprintf('Car Weight (W) = %i lb\n', W)
fprintf('Acceleration Due to Gravity (g) = %i ft/s^2\n', g)
fprintf('Car Mass (M) = %i lb*sec/ft\n', M)
fprintf('Suspension Stiffness (K) = %i lb/ft\n', K)
fprintf('Car Suspension Damping Coefficient (C) = %i lb*sec/ft\n', C)
fprintf('MR Suspension Damping Coefficient (Cmr) = %i lb*sec/ft\n', Cmr)
fprintf('Max Force in MR Damper (Fyieldmx) = %i lb\n', Fyieldmx)
fprintf('Road Amplitude (y0) = %i ft\n', y0)
fprintf('Integration Constant Beta (B) = %i\n', B)
fprintf('Integration Constant Lambda (L) = %i\n', L)
fprintf('Time Step (dt) = %i sec\n', dt)
fprintf('Number of Time Steps (ntimepts) = %i\n', ntimepts)

fprintf('Road Wavelength (l) = %i ft\n', l)
fprintf('Car Speed (s) = %i ft/sec\n', s)
fprintf('Excitation Frequency (we) = %i Hz\n', we)
fprintf('Natural Frequency (wn) = %i Hz\n', wn)
fprintf('Frequency Ratio (wratio) = %i\n', wratio)
fprintf('Car Max Displacement (xmax) = %i ft\n', xmax)
fprintf('Car Max Velocity (vmax) = %i ft/sec\n', vmax)
fprintf('Velocity Ratio (vratio) = %i\n', vratio)
fprintf('Car Max Acceleration (amax) = %i ft/sec^2\n', amax)
fprintf('Max Spring Force (spring_max) = %i lb\n', spring_max)
fprintf('Max Damper Force (damper_max) = %i lb\n', damper_max)
fprintf('Max MR Damper Force (mr_max) = %i lb\n', mr_max)


figure(1)
sgtitle({ ...
    ['Car Velocity s = ' num2str(s) ' ft/s', '      Skyhook Control = ' mat2str(skyhook)]
    ['Road Wavelength l = ' num2str(l) ' ft', '       Skyhook Modified = ' mat2str(skyhook_mod)]}, 'FontSize', 8);

subplot(3,2,1)
plot(time,x)
title('Displacement')
xlabel('Time (s)')
ylabel('Displacement (ft)')

subplot(3,2,3)
plot(time, v)
title('Velocity')
xlabel('Time (s)')
ylabel('Velocity (ft/s)')

subplot(3,2,5)
plot(time, a)
title('Acceleration')
xlabel('Time (s)')
ylabel('Acceleration (ft/s^2)')

subplot(3,2,2)
plot(time, spring_force)
title('Spring Force')
xlabel('Time (s)')
ylabel('Force (lb)')

subplot(3,2,4)
plot(time, damper_force)
title('Damper Force')
xlabel('Time (s)')
ylabel('Force (lb)')

subplot(3,2,6)
plot(time, mr_force)
title('MR Damper Force')
xlabel('Time (s)')
ylabel('Force (lb)')


