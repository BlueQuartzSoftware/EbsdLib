% mtex_ang_to_ipf.m
%
% Read a TSL .ang file (multi-phase OK), build the IPF-Z color map using
% MTEX's ipfTSLKey for each phase, and write a TIFF with one pixel per
% measurement so that the result can be compared 1:1 against:
%   - EDAX OIM Analysis output (the .ang vendor)
%   - EbsdLib `make_ipf` output
%
% Usage: edit the angPath / outPath below, then run this script in MATLAB
% with MTEX on the path (`startup_mtex` first).

angPath = '/Users/Shared/Data/Edax_IPF_Test/AllLaueClasses_RandO.ang';
outPath = '/tmp/AllLaueClasses_mtex.png';

if ~exist(angPath, 'file')
    error('Input .ang not found: %s', angPath);
end

% Match TSL/EDAX viewing convention: X east, Y north, Z out of page.
setMTEXpref('xAxisDirection', 'east');
setMTEXpref('zAxisDirection', 'outOfPlane');

% Load the EBSD scan. MTEX auto-detects phases and crystal symmetries from
% the .ang header.
ebsd = EBSD.load(angPath, 'convertEuler2SpatialReferenceFrame','setting 2');

fprintf('Phases in scan:\n');
for k = 1:length(ebsd.CSList)
    cs = ebsd.CSList{k};
    if isa(cs, 'crystalSymmetry')
        fprintf('  phase %d: %s (%s)\n', k-1, cs.mineral, cs.LaueName);
    else
        fprintf('  phase %d: notIndexed\n', k-1);
    end
end

% Convert to a gridded EBSD object so we can index by (row, col) and ask
% for grid dimensions directly. (MTEX 6.x changed prop.x / prop.y away.)
ebsd = gridify(ebsd);
sz = size(ebsd);
nY = sz(1);
nX = sz(2);
fprintf('Scan grid: %d cols (X) x %d rows (Y)\n', nX, nY);

% IPF-Z reference direction
refDir = vector3d(0, 0, 1);

% Build per-pixel RGB by querying each phase's ipfTSLKey. Iterate over the
% phaseId vector (1-based index into ebsd.CSList) and color each phase's
% pixels with its own crystalSymmetry. Notindexed pixels (CSList entry is
% the string 'notIndexed') are left black.
nPixels = numel(ebsd);
rgb = zeros(nPixels, 3);

phaseIds = ebsd.phaseId;
rot = ebsd.rotations;   % plain rotation array, no symmetry attached

for pid = 1:length(ebsd.CSList)
    cs = ebsd.CSList{pid};
    if ~isa(cs, 'crystalSymmetry')
        continue;
    end
    mask = (phaseIds == pid);
    if ~any(mask)
        continue;
    end
    % Build single-phase orientations by attaching this phase's cs to the
    % corresponding rotations. orientation2color requires single-phase input.
    oriPhase = orientation(rot(mask), cs);
    key = ipfTSLKey(cs);
    key.inversePoleFigureDirection = refDir;
    rgb(mask, :) = key.orientation2color(oriPhase);
end

% Reshape to image. Gridded EBSD stores pixels in row-major scan order; the
% reshape with [nY, nX] mirrors how the .ang lines were ordered (row by row,
% column varying fastest within each row).
img = reshape(rgb, nY, nX, 3);

% Convert to uint8 and write as TIFF.
imgU8 = uint8(round(img * 255));
imwrite(imgU8, outPath);
fprintf('Wrote %s\n', outPath);
fprintf('\nFor side-by-side compare:\n');
fprintf('  EDAX reference:  /Users/Shared/Data/Edax_IPF_Test/crystallography_output/ipfs.tif\n');
fprintf('  EbsdLib output:  /tmp/AllLaueClasses_ebsdlib.png (run make_ipf first)\n');
fprintf('  MTEX output:     %s\n', outPath);
