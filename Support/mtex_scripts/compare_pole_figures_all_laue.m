% compare_pole_figures_all_laue.m
%
% Companion to EbsdLib's PoleFigureLaueComparisonTest (Source/Test/PoleFigureLaueComparisonTest.cpp).
%
% For every Laue class listed in manifest.txt, reads eulers.csv, reconstructs
% the orientations in MTEX with the matching crystal symmetry, and saves
% a pole figure PNG named mtex.png next to EbsdLib's ebsdlib.png — so the two
% images can be compared side-by-side per Laue class.
%
% Usage:
%   1. Run the EbsdLib test first:
%        cd .../DREAM3D-Build/ebsdlib-Release && \
%          Bin/EbsdLibUnitTest "ebsdlib::PoleFigureLaueComparisonTest::GenerateAllLaueClasses"
%   2. Edit `baseDir` below to point at the PoleFigureComparison directory
%   3. Run this script in MATLAB (MTEX must be on the path; run `startup_mtex` first)

baseDir = '/Users/mjackson/Workspace7/DREAM3D-Build/ebsdlib-Release/Testing/Temporary/PoleFigureComparison';

if ~exist(baseDir, 'dir')
    error('baseDir does not exist: %s\nRun the PoleFigureLaueComparisonTest first.', baseDir);
end

% MTEX pole-figure plotting conventions (match EbsdLib after the axis fixes)
setMTEXpref('xAxisDirection', 'east');
setMTEXpref('zAxisDirection', 'outOfPlane');

% Map EbsdLib rotation point group → MTEX crystalSymmetry Laue class string
% plus the 3 Miller-Bravais / Miller indices to plot. EbsdLib now uses
% X||a* for hexagonal/trigonal crystals (MTEX default).
laueMap = containers.Map();
% cubic
laueMap('432') = struct('cs', crystalSymmetry('m-3m'), ...
    'h', {{[0 0 1], [0 1 1], [1 1 1]}});
laueMap('23')  = struct('cs', crystalSymmetry('m-3'), ...
    'h', {{[0 0 1], [0 1 1], [1 1 1]}});
% hexagonal
laueMap('622') = struct('cs', crystalSymmetry('6/mmm', [1 1 1.6]), ...
    'h', {{[0 0 0 1], [1 0 -1 0], [2 -1 -1 0]}});
laueMap('6')   = struct('cs', crystalSymmetry('6/m', [1 1 1.6]), ...
    'h', {{[0 0 0 1], [1 0 -1 0], [1 1 -2 0]}});
% tetragonal
laueMap('422') = struct('cs', crystalSymmetry('4/mmm'), ...
    'h', {{[0 0 1], [1 0 0], [1 1 0]}});
laueMap('4')   = struct('cs', crystalSymmetry('4/m'), ...
    'h', {{[0 0 1], [1 0 0], [1 1 0]}});
% trigonal
laueMap('32')  = struct('cs', crystalSymmetry('-3m', [1 1 1.6]), ...
    'h', {{[0 0 0 1], [0 -1 1 0], [1 -1 0 0]}});
laueMap('3')   = struct('cs', crystalSymmetry('-3', [1 1 1.6]), ...
    'h', {{[0 0 0 1], [-1 -1 2 0], [2 -1 -1 0]}});
% orthorhombic
laueMap('222') = struct('cs', crystalSymmetry('mmm'), ...
    'h', {{[0 0 1], [1 0 0], [0 1 0]}});
% monoclinic
laueMap('2')   = struct('cs', crystalSymmetry('2/m'), ...
    'h', {{[0 0 1], [1 0 0], [0 1 0]}});
% triclinic
laueMap('1')   = struct('cs', crystalSymmetry('-1'), ...
    'h', {{[0 0 1], [1 0 0], [0 1 0]}});

% Discover all Laue class subdirectories
entries = dir(baseDir);
for e = 1:numel(entries)
    if ~entries(e).isdir, continue; end
    name = entries(e).name;
    if name(1) == '.', continue; end
    if ~isKey(laueMap, name)
        fprintf('skipping %s (not in laueMap)\n', name);
        continue;
    end

    classDir = fullfile(baseDir, name);
    csvPath = fullfile(classDir, 'pole_figure_input_eulers.csv');
    if ~exist(csvPath, 'file')
        fprintf('skipping %s (no pole_figure_input_eulers.csv)\n', name);
        continue;
    end

    info = laueMap(name);
    cs = info.cs;
    ss = specimenSymmetry('1');

    T = readtable(csvPath);
    eulers_deg = [T.phi1, T.Phi, T.phi2];
    ori = orientation.byEuler(eulers_deg(:,1)*degree, eulers_deg(:,2)*degree, eulers_deg(:,3)*degree, cs, ss);

    % Build Miller objects for the 3 pole figures
    hArr = cell(1, numel(info.h));
    for k = 1:numel(info.h)
        idx = info.h{k};
        if numel(idx) == 4
            hArr{k} = Miller(idx(1), idx(2), idx(3), idx(4), cs);
        else
            hArr{k} = Miller(idx(1), idx(2), idx(3), cs);
        end
    end
    h = [hArr{:}];

    f = figure('Visible', 'off', 'Position', [100 100 700 250]);
    plotPDF(ori, h, 'MarkerSize', 3, 'upper', 'projection', 'eangle', 'complete');
    ttl = sprintf('MTEX %s — Euler %.1f, %.1f, %.1f (deg)', name, eulers_deg(1,1), eulers_deg(1,2), eulers_deg(1,3));
    sgtitle(ttl);

    outPath = fullfile(classDir, 'mtex_pole_figure.png');
    outDir = fileparts(outPath);
    if ~isfolder(outDir)
        mkdir(outDir);
    end
    exportgraphics(f, outPath, 'Resolution', 72);
    close(f);
    fprintf('wrote %s\n', outPath);
end

fprintf('\nDone. For each Laue class directory there should now be:\n');
fprintf('  <class>/eulers.csv      input Euler samples\n');
fprintf('  <class>/ebsdlib.png     EbsdLib-rendered composite\n');
fprintf('  <class>/mtex.png        MTEX-rendered composite\n');
fprintf('Compare the two image files per class to verify convention agreement.\n');
