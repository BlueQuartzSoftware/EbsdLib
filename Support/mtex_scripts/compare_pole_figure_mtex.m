% compare_pole_figure_mtex.m
%
% Compares the pole figure produced by EbsdLib's ODFTest against MTEX.
% Expects a CSV of Bunge Euler angles (phi1, Phi, phi2) in degrees.
%
% Usage: edit csvPath below to point at the file written by ODFTest, then run.
%
% The ODFTest writes the CSV to:
%   <EbsdLib build dir>/Testing/Temporary/ODFTest_Eulers_deg.csv
% e.g. /Users/mjackson/Workspace7/DREAM3D-Build/ebsdlib-Release/Testing/Temporary/ODFTest_Eulers_deg.csv

csvPath = '/Users/mjackson/Workspace7/DREAM3D-Build/ebsdlib-Release/Testing/Temporary/ODFTest_Eulers_deg.csv';

% Load Euler angles
T = readtable(csvPath);
eulers_deg = [T.phi1, T.Phi, T.phi2];
fprintf('Loaded %d Euler triples from %s\n', size(eulers_deg,1), csvPath);
fprintf('First 3 rows (deg):\n');
disp(eulers_deg(1:min(3,end), :));

% Crystal symmetry: hexagonal high (6/mmm).
% Use X||a (real-space a-axis along X) to match EbsdLib's hexagonal
% direction convention in HexagonalOps.cpp. EbsdLib's (10-10) pole uses
% direction (sqrt(3)/2, 1/2, 0) and (2-1-10) uses (1, 0, 0), which is the
% X||a convention. MTEX's default is X||a* (reciprocal), which swaps the
% labels of the two prismatic pole figures — that is why EbsdLib's (10-10)
% looks like MTEX's (2-1-10) and vice versa. Switching MTEX to X||a here
% aligns the labels.
cs = crystalSymmetry('6/mmm', [1 1 1.6], 'X||a', 'Z||c*');
% Specimen symmetry: triclinic (no sample symmetry)
ss = specimenSymmetry('1');

% Build orientations (Bunge convention is MTEX default)
ori = orientation.byEuler(eulers_deg(:,1)*degree, eulers_deg(:,2)*degree, eulers_deg(:,3)*degree, cs, ss);

% MTEX default plotting: X east (right), Y north (up), Z outOfPlane.
% These match EbsdLib's pole figure convention after this fix.
setMTEXpref('xAxisDirection', 'east');
setMTEXpref('zAxisDirection', 'outOfPlane');

% Miller indices for the pole figures EbsdLib shows: (0001), (10-10), (2-1-10)
h = [ ...
    Miller(0, 0, 0, 1, cs), ...
    Miller(1, 0,-1, 0, cs), ...
    Miller(2,-1,-1, 0, cs)];

% Plot pole figures as scatter of individual orientations (like EbsdLib discrete PF)
figure('Name', 'MTEX pole figures for EbsdLib ODFTest Euler sample');
plotPDF(ori, h, 'MarkerSize', 3, 'upper', 'projection', 'eangle', 'complete');
title('MTEX: (0001), (10-10), (2-1-10) for EbsdLib ODFTest sample');

% Print a short diagnostic
fprintf('\n');
fprintf('Expected for Bunge (180, 90, 0):\n');
fprintf('  c-axis [0001] in sample frame = +Y, so (0001) cluster at 12:00 (top).\n');
fprintf('  Centrosymmetric 6/mmm: antipodal [000-1] at 6:00 (bottom) also appears.\n');
fprintf('  The (0001) pole figure should show clusters at 12:00 and 6:00.\n');
