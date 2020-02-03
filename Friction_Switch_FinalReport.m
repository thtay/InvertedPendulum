clear;clc;
load sroots; %Load the roots to be used in spole calculations
% Initializing the constants that will be needed throughout the simulink
% model. The consants that are "hardware constants" are depend on the
% hardware the is being used for the physical system. These values should
% be double checked prior to running the program when integrated with the
% hardware system. The values used in this simulation is based on
% available resources from University of Rhode Island.
g=9.81; %Acceleration due gravity constant
A = 22.8; %Square of natural frequency of pendulum,will change for 
          %different rod. (hardware constants)
%A= 41.5;
n=495; % Transduction ratio to the lead screw (hardware constants)
C=25; % Motor Control Constants (hardware constants)
Cprime=28; % Motor Control Constants (hardware constants)
D=2350; % Motor Control Constants (hardware constants)
alpha=0.018; %Co-efficient of friction (User input)original=0.04; This 
             %value must be estimated for each different rods used.

%The constants below are used in the "Energy and Motor Control" simulink
%model.
KM=((Cprime/2)^2)/D; %Motor Control Constants
C1= (2/Cprime)^2; % Motor Control Constants
C2= 4/Cprime; % Motor Control Constants
Ts=1; %Settling Time for Linear Regulator 

% Calculations for determing the gain value (k) to be used for the linear
% regulator that will drive the pendulum system to the inverted position.
Amat = [0 1 0 0; A 0 0 ((A*C)/(n*g));0 0 0 1; 0 0 0 -C];
Bmat = [ 0; -((A*D)/(n*g)); 0 ;D];
plantpoles = eig(Amat);
%spoles = [s3/Ts plantpoles(4)];
spoles = [s2/Ts plantpoles(2) plantpoles(4)]; %Calculated based on the 
                                              %plantpoles of the system.
k=place(Amat,Bmat,spoles); %The linear regulator gain value


% Calulations for the region of attraction model. This will determine
% when the system switch from the non-linear controller (initial
% swing-up) to the linear regulator to drive the system to the inverted
% position. Recall that our system is a fourth order system, but upon
% investigation it was found that the fourth order system produce a
% region of attration that is too small, and thus not useful. To work
% around this issue we take the second order version of our system and
% calculate its region of attraction
a_2 = [0 1; A 0];
b_2 = [0;A/g];
Ts_2 =1; %Settling Time for Linear Regulator 
spoles_2 = s2/Ts_2;
k_2 = place(a_2,b_2,spoles_2);
P = lyap((a_2-b_2*k_2)',eye(2));
r = find_r(P,k_2); %Finding the largest value of r for for which Vdot is 
                   %negative definite in a circle of radius r.
cc= r^2;
ce = (min(eig(P)))*cc; %This is the minimum eigenvalues of the P matrix 
                       %multiplied by the square of the radius r.

%Plots for the r, where v_dot is negative definite and the regin of
%attraction is estimated to be the ellipse. The position and velocity of
%the simulink runs are plotted in overlay to show how the system respond 
%as it enter the region of attraction.
plot(x1_sim,x2_sim);
hold on
plot_ellipse(eye(2),cc);
hold on
plot_ellipse(P,ce)



 
 
