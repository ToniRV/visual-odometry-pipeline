clear all % clear all variables in the workspace
close all % close all figures
clc       % clear the command window

addpath('8point/');
addpath('triangulation/');
addpath('plot/');

rng(42);

N = 10;         % Number of 3-D points
P = randn(4,N);  % Homogeneous coordinates of 3-D points

%% Test linear triangulation
P(3, :) = P(3, :) * 5 + 10;
P(4, :) = 1;

M1 =   [500 0 320 0
        0 500 240 0
        0 0 1 0];

M2 =   [500 0 320 -100
        0 500 240 0
        0 0 1 0];
				
p1 = M1 * P;     % Image (i.e., projected) points
p2 = M2 * P;

P_est = linearTriangulation(p1,p2,M1,M2);

fprintf('P_est-P=\n');
(P_est-P)