% compare_ipf_legends_all_laue.m
%
% Companion to EbsdLib's IPFLegendTest::MTEXCompare_AllLaueClasses.
%
% For every Laue class in IPFComparison/, writes two MTEX legends so the
% pairs can be compared apples-to-apples against the EbsdLib outputs:
%   ebsdlib_ipf_legend_tsl.png  vs  mtex_ipf_legend_tsl.png
%   ebsdlib_ipf_legend_nh.png   vs  mtex_ipf_legend_hsv.png
% The TSL pair uses MTEX's ipfTSLKey; the NH pair uses MTEX's ipfHSVKey,
% which is the Nolze-Hielscher-style HSV scheme that EbsdLib's
% NolzeHielscherColorKey is modeled on.
%
% Usage:
%   1. Build and run the EbsdLib unit test first:
%        cd .../DREAM3D-Build/ebsdlib-Release && \
%          Bin/EbsdLibUnitTest "ebsdlib::IPFLegendTest::MTEXCompare_AllLaueClasses"
%   2. Edit `baseDir` below to point at the IPFComparison directory
%   3. Run this script in MATLAB (MTEX must be on the path: startup_mtex)


baseDir = '/Users/mjackson/Workspace7/DREAM3D-Build/ebsdlib-Release/Testing/Temporary/IPFComparison';

if ~exist(baseDir, 'dir')
    error('baseDir does not exist: %s\nRun the IPFLegendTest MTEXCompare first.', baseDir);
end

setMTEXpref('xAxisDirection', 'east');
setMTEXpref('zAxisDirection', 'outOfPlane');

% Map EbsdLib rotation point group -> MTEX crystalSymmetry.
% Hexagonal/trigonal use X||a to match EbsdLib's X||a* after the 30-degree
% reciprocal-basis reinterpretation. (MTEX's default is X||a*; using X||a
% here rotates MTEX 30 degrees to match EbsdLib.)
laueMap = containers.Map();
laueMap('432') = crystalSymmetry('m-3m');
laueMap('23')  = crystalSymmetry('m-3');
laueMap('622') = crystalSymmetry('6/mmm', [1 1 1.6], 'X||a', 'Z||c*');
laueMap('6')   = crystalSymmetry('6/m',   [1 1 1.6], 'X||a', 'Z||c*');
laueMap('422') = crystalSymmetry('4/mmm');
laueMap('4')   = crystalSymmetry('4/m');
laueMap('32')  = crystalSymmetry('-3m',   [1 1 1.6], 'X||a', 'Z||c*');
laueMap('3')   = crystalSymmetry('-3',    [1 1 1.6], 'X||a', 'Z||c*');
laueMap('222') = crystalSymmetry('mmm');
laueMap('2')   = crystalSymmetry('2/m');
laueMap('1')   = crystalSymmetry('-1');

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
    cs = laueMap(name);

    % --- TSL pair (compare ebsdlib_ipf_legend_tsl.png vs mtex_ipf_legend_tsl.png) ---
    try
        keyTSL = ipfTSLKey(cs);
    catch
        warning('ipfTSLKey not available for %s; falling back to ipfHKLKey', name);
        keyTSL = ipfHKLKey(cs);
    end
    fT = figure('Visible', 'off', 'Position', [100 100 600 600]);
    plot(keyTSL);
    title(sprintf('MTEX ipfTSLKey %s', name));
    outTSL = fullfile(classDir, 'tsl_gridded_mtex_ipf_legend.png');
    outDir = fileparts(outTSL);
    if ~isfolder(outDir)
        mkdir(outDir);
    end
    %exportgraphics(outTSL, outTSL, 'Resolution', 72);
    saveas(fT, outTSL);
    close(fT);
    fprintf('wrote %s\n', outTSL);

    % --- NH/HSV pair (compare ebsdlib_ipf_legend_nh.png vs mtex_ipf_legend_hsv.png) ---
    keyHSV = ipfHSVKey(cs);
    fH = figure('Visible', 'off', 'Position', [100 100 600 600]);
    plot(keyHSV);
    title(sprintf('MTEX ipfHSVKey %s', name));
    outHSV = fullfile(classDir, 'nh_gridded_mtex_ipf_legend.png');
    %exportgraphics(outHSV, outHSV, 'Resolution', 72);
    saveas(fH, outHSV);
    close(fH);
    fprintf('wrote %s\n', outHSV);
end

fprintf('\nDone. For each Laue class directory there should now be:\n');
fprintf('  <class>/ebsdlib_ipf_legend_tsl.png   EbsdLib TSL legend\n');
fprintf('  <class>/ebsdlib_ipf_legend_nh.png    EbsdLib Nolze-Hielscher legend\n');
fprintf('  <class>/mtex_ipf_legend_tsl.png       MTEX ipfTSLKey legend\n');
fprintf('  <class>/mtex_ipf_legend_hsv.png       MTEX ipfHSVKey legend\n');
fprintf('Compare each pair (TSL <-> TSL, NH <-> HSV) side-by-side.\n');
