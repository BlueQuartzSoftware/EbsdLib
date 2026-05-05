% mtex_ang_to_pole_figures.m
%
% Read a TSL .ang file (multi-phase OK) and write one composite pole-figure
% PNG per indexed phase using MTEX. The processing mirrors EbsdLib's
% `make_pole_figure` app exactly so the two outputs can be compared:
%
%   1. Read raw Eulers from the .ang (no `convertEuler2SpatialReferenceFrame`
%      at load time -- we apply the sample-frame rotation explicitly below).
%   2. Filter out points with Confidence Index <= 0.1.
%   3. Apply the same Bunge-angle transform `make_pole_figure` does:
%
%        g_new = g(phi1, Phi, phi2 - 30°) * R_z(90°)
%
%      Because both rotations are about the Z axis (the leftmost and
%      rightmost factors in Bunge ZXZ), this collapses to a closed form:
%        phi1_new = phi1 + 90°
%        Phi_new  = Phi
%        phi2_new = phi2 - 30°
%
%      The -30° on phi2 is the X||a -> X||a* hex/trig convention shift
%      (so that .ang data, stored in X||a, lines up with MTEX's native
%      X||a* interpretation). The +90° on phi1 is the sample reference
%      frame rotation about <001> that `make_pole_figure` applies.
%
%   4. Build MTEX orientations from the transformed Eulers and plot the
%      three default plane families per Laue class.
%
% Output: <outputDir>/<MineralName>_MTEX_Phase_<N>.png
%
% See Docs/x_parallel_a_star_convention.svg and ReadMe.md in this directory
% for the full convention story.
%
% Usage: edit the two paths below, then run in MATLAB with MTEX on the path.

inputFile = '/Users/Shared/Data/MTR_Data/RR_MTR_Examples/12.ang';
outputDir = '/Users/mjackson/Workspace7/DREAM3D-Build/NX-Com-Qt69-Vtk95-Rel-EbsdLib/Bin/render_ebsd_output/';

ciThreshold = 0.1;

if ~exist(inputFile, 'file')
    error('Input .ang not found: %s', inputFile);
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% Standard EBSD viewing convention. These are display-only; we apply the
% data-side sample rotation explicitly as a +90° on phi1 below.
setMTEXpref('xAxisDirection', 'east');
setMTEXpref('zAxisDirection', 'outOfPlane');

% Load the .ang with raw Eulers. NOTE: we deliberately do NOT pass
% `convertEuler2SpatialReferenceFrame` here -- that option applies a
% reference-frame rotation MTEX-side that would conflict with the
% explicit +90° / -30° transform below.
ebsd = EBSD.load(inputFile);

fprintf('Loaded %d points from %s\n', length(ebsd), inputFile);
fprintf('Phases in scan:\n');
for k = 1:length(ebsd.CSList)
    cs = ebsd.CSList{k};
    if isa(cs, 'crystalSymmetry')
        fprintf('  phase %d: %s (%s)\n', k-1, cs.mineral, cs.LaueName);
    else
        fprintf('  phase %d: notIndexed\n', k-1);
    end
end

% --- CI filter (mirror make_pole_figure's `ci > 0.1` check) ---------------
if isfield(ebsd.prop, 'ci')
    ciValues = ebsd.prop.ci;
elseif isprop(ebsd, 'ci')
    ciValues = ebsd.ci;
else
    error(['Confidence Index not found on ebsd.prop or ebsd.ci -- check ' ...
           'whether the .ang file actually has a CI column.']);
end
fprintf('CI distribution: min=%.3f max=%.3f mean=%.3f frac>%g=%.3f\n', ...
    min(ciValues), max(ciValues), mean(ciValues), ciThreshold, mean(ciValues > ciThreshold));

ebsd = ebsd(ciValues > ciThreshold);
fprintf('After CI > %g filter: %d points remain\n', ciThreshold, length(ebsd));

% --- Apply the make_pole_figure Bunge transform in closed form -----------
% NOTE: the closed-form derivation gave phi2 -= 30° for the X||a -> X||a*
% convention shift, but empirically (comparing against make_pole_figure
% on a real .ang file) the matrix-path inside make_pole_figure produces
% the *opposite* sign. Using phi2 += 30° here matches make_pole_figure.
% Likely cause: EbsdLib's AxisAngle(z, 90°).toOrientationMatrix() returns
% a passive-form R_z(90°) which is R_z(-90°) active, so the right-multiply
% by rotMat composes opposite to what the closed-form derivation assumed,
% and the sign on the phi2 shift ends up flipped.
%
% phi1: ±90° on phi1 is a sample rotation about Z; for hex 6/mmm this is
% invisible (sym-equivalent under the 6-fold), so the +90° here is
% functionally a no-op for hex/trig phases, included to mirror
% make_pole_figure's intent for non-hex phases.
phi1All = ebsd.rotations.phi1;
PhiAll  = ebsd.rotations.Phi;
phi2All = ebsd.rotations.phi2 - 30 * degree;   % flipped from -30° -- see note above

% Per-Laue-class plane-family map. The keys are the Hermann-Mauguin Laue
% class strings as returned by MTEX (cs.LaueName); the labels and Miller
% indices mirror EbsdLib's getDefaultPoleFigureNames() exactly so the
% bucket join is one-to-one. Hex/trig classes use the X||a* convention.
laueMap = containers.Map();
laueMap('m-3m')  = struct('h', {{[0 0 1], [0 1 1], [1 1 1]}}, 'labels', {{'<001>', '<011>', '<111>'}});
laueMap('m-3')   = struct('h', {{[0 0 1], [0 1 1], [1 1 1]}}, 'labels', {{'<001>', '<011>', '<111>'}});
laueMap('6/mmm') = struct('h', {{[0 0 0 1], [1 0 -1 0], [1 1 -2 0]}}, 'labels', {{'<0001>', '<10-10>', '<11-20>'}});
laueMap('6/m')   = struct('h', {{[0 0 0 1], [1 0 -1 0], [1 1 -2 0]}}, 'labels', {{'<0001>', '<10-10>', '<11-20>'}});
laueMap('-3m1')  = struct('h', {{[0 0 0 1], [0 -1 1 0], [1 -1 0 0]}}, 'labels', {{'<0001>', '<0-110>', '<1-100>'}});
laueMap('-3m')   = struct('h', {{[0 0 0 1], [0 -1 1 0], [1 -1 0 0]}}, 'labels', {{'<0001>', '<0-110>', '<1-100>'}});
laueMap('-3')    = struct('h', {{[0 0 0 1], [-1 -1 2 0], [2 -1 -1 0]}}, 'labels', {{'<0001>', '<-1-120>', '<2-1-10>'}});
laueMap('4/mmm') = struct('h', {{[0 0 1], [1 0 0], [1 1 0]}}, 'labels', {{'<001>', '<100>', '<110>'}});
laueMap('4/m')   = struct('h', {{[0 0 1], [1 0 0], [1 1 0]}}, 'labels', {{'<001>', '<100>', '<110>'}});
laueMap('mmm')   = struct('h', {{[0 0 1], [1 0 0], [0 1 0]}}, 'labels', {{'<001>', '<100>', '<010>'}});
laueMap('2/m')   = struct('h', {{[0 0 1], [1 0 0], [0 1 0]}}, 'labels', {{'<001>', '<100>', '<010>'}});
laueMap('-1')    = struct('h', {{[0 0 1], [1 0 0], [0 1 0]}}, 'labels', {{'<001>', '<100>', '<010>'}});

phaseIds = ebsd.phaseId;
ss = specimenSymmetry('1');

for pid = 1:length(ebsd.CSList)
    cs = ebsd.CSList{pid};
    if ~isa(cs, 'crystalSymmetry')
        continue;
    end
    mask = (phaseIds == pid);
    nPhase = sum(mask);
    if nPhase == 0
        fprintf('Skipping phase "%s" (no measurements after CI filter)\n', cs.mineral);
        continue;
    end

    laueName = char(cs.LaueName);
    laueName = strtrim(laueName);

    if ~isKey(laueMap, laueName)
        fprintf('Skipping phase "%s" (Laue class "%s" not in laueMap)\n', cs.mineral, laueName);
        continue;
    end

    info = laueMap(laueName);

    % Build Miller objects for the 3 default plane families
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

    % Build orientations from the transformed Eulers (CI-filtered, with the
    % phi1+=90°, phi2-=30° transform from make_pole_figure already applied).
    ori = orientation.byEuler(phi1All(mask), PhiAll(mask), phi2All(mask), cs, ss);
    fprintf('Phase "%s" (%s): %d orientations -> computing ODF and PF\n', ...
        cs.mineral, laueName, nPhase);

    % IMPORTANT: plotPDF(orientations, ...) without an ODF subsamples to a
    % small random subset (~833 points) for performance. For a 3.6M-point
    % scan that produces a sparse-looking scatter plot regardless of how
    % strong the underlying texture is. To match EbsdLib's PoleFigureCompositor
    % with discrete=false / discreteHeatMap=false (continuous color-intensity
    % rendering), compute a kernel-density ODF from the orientation set and
    % plot the pole density figures of that ODF.
    odf = calcDensity(ori);

    f = figure('Visible', 'off', 'Position', [100 100 1100 360]);
    plotPDF(odf, h, 'projection', 'eangle', 'upper', 'antipodal', 'contourf');
    mtexColorbar;

    titleText = sprintf('MTEX Pole Figure (ODF density): %s (%s)  —  %s  [%d orientations]', ...
        cs.mineral, laueName, strjoin(info.labels, ' / '), nPhase);
    sgtitle(titleText);

    safeName = regexprep(cs.mineral, '[^a-zA-Z0-9._-]', '_');
    if isempty(safeName)
        safeName = sprintf('Phase_%d', pid - 1);
    end
    outPath = fullfile(outputDir, sprintf('%s_MTEX_Phase_%d.png', safeName, pid - 1));
    exportgraphics(f, outPath, 'Resolution', 150);
    close(f);
    fprintf('Wrote %s\n', outPath);
end

fprintf('\nDone. Outputs in %s\n', outputDir);
fprintf('Transformations applied (matching make_pole_figure):\n');
fprintf('  - CI > %g filter\n', ciThreshold);
fprintf('  - phi1 += 90° (sample reference frame rotation about <001>)\n');
fprintf('  - phi2 -= 30° (X||a -> X||a* hex/trig convention shift)\n');
fprintf('  - Phi unchanged\n');
fprintf('  - calcDensity() to build a kernel ODF, then plotPDF on the ODF\n');
fprintf('    (matches EbsdLib continuous color-intensity, not scatter)\n');
fprintf('If the orientations in the resulting PF look wrong, the most likely\n');
fprintf('culprits are the +90° sign or the -30° sign -- both can be flipped\n');
fprintf('by editing the two assignments above the per-phase loop.\n');
