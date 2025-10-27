clc; clear; close all;

% Columns: [theta d a alpha]
% d and a in meters, angles in radians

dh = [...
    0       -0.450   0.150   pi/2;     % Joint 1
   -pi/2     0        0.590   pi;       % Joint 2
    pi/2     0        0.130  -pi/2;     % Joint 3
    0       -0.6471   0      -pi/2;     % Joint 4
    0        0        0       pi/2;     % Joint 5
    pi      -0.095    0       pi];      % Joint 6

L(1) = Link('d', dh(1,2), 'a', dh(1,3), 'alpha', dh(1,4), 'offset', 0);
L(2) = Link('d', dh(2,2), 'a', dh(2,3), 'alpha', dh(2,4), 'offset', dh(2,1));
L(3) = Link('d', dh(3,2), 'a', dh(3,3), 'alpha', dh(3,4), 'offset', dh(3,1));
L(4) = Link('d', dh(4,2), 'a', dh(4,3), 'alpha', dh(4,4), 'offset', dh(4,1));
L(5) = Link('d', dh(5,2), 'a', dh(5,3), 'alpha', dh(5,4), 'offset', dh(5,1));
L(6) = Link('d', dh(6,2), 'a', dh(6,3), 'alpha', dh(6,4), 'offset', dh(6,1));

COMAU_SmartSix = SerialLink(L, 'name', 'COMAU Smart Six');

% Rotate the whole robot around X0 by pi radians (flip Z direction)
T0fix = trotz(0) * troty(0) * trotx(pi);  % rotation of base
COMAU_SmartSix.base = T0fix;

% Plot robot in q configuration ---
q = [pi 0 0 0 0 0];
COMAU_SmartSix.plot(q, 'workspace', [-1 1 -1 1 -0.5 3], 'scale', 0.5);
title('COMAU Smart Six at q = [0 0 0 0 0 0]');
