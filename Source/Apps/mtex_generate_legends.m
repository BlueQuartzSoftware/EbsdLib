%% mtex_generate_legends.m
%
% Generates IPF triangle legend images using MTEX for comparison with
% EbsdLib's output. Produces both the traditional TSL color key and the
% Nolze-Hielscher (HSV) color key for cubic m-3m, hexagonal 6/mmm, and
% orthorhombic mmm crystal symmetries.
%
% Usage:
%   1. Open MATLAB
%   2. Ensure MTEX is installed (see below for instructions)
%   3. Run this script:  mtex_generate_legends
%
% Output files are saved as TIFF images in the specified output directory.

%% Check for MTEX availability
if ~exist('crystalSymmetry', 'file')
  fprintf('\n');
  fprintf('=============================================================\n');
  fprintf('  MTEX is not available on the MATLAB path.\n');
  fprintf('=============================================================\n');
  fprintf('\n');
  fprintf('To install MTEX:\n');
  fprintf('\n');
  fprintf('  1. Download MTEX from: https://mtex-toolbox.github.io/download\n');
  fprintf('  2. Extract the archive to a permanent location, e.g.:\n');
  fprintf('       /Users/mjackson/MATLAB/mtex-6.0.0\n');
  fprintf('  3. In MATLAB, navigate to the extracted folder and run:\n');
  fprintf('       startup_mtex\n');
  fprintf('  4. (Optional) To load MTEX automatically, add the startup\n');
  fprintf('     command to your MATLAB startup.m file:\n');
  fprintf('       edit(fullfile(userpath, ''startup.m''))\n');
  fprintf('     Then add the line:\n');
  fprintf('       run(''/path/to/mtex/startup_mtex.m'')\n');
  fprintf('\n');
  fprintf('After installing MTEX, re-run this script.\n');
  fprintf('\n');
  return;
end

%% Configuration
outputDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Data', 'IPF_Legend', 'MTEX_Reference');

% Create output directory if it does not exist
if ~exist(outputDir, 'dir')
  mkdir(outputDir);
  fprintf('Created output directory: %s\n', outputDir);
end

% Image resolution for saved figures (dots per inch)
imageDPI = 300;

% Inverse pole figure projection direction (sample Z axis)
ipfDirection = vector3d.Z;

%% Define the crystal symmetries to process
symmetries = struct( ...
  'name',      {'Cubic',           'Hexagonal',       'Orthorhombic'    }, ...
  'hm',        {'m-3m',            '6/mmm',           'mmm'             }, ...
  'prefix',    {'cubic',           'hexagonal',       'orthorhombic'    }  ...
);

%% Generate IPF legends for each symmetry
for idx = 1:length(symmetries)
  symName   = symmetries(idx).name;
  symHM     = symmetries(idx).hm;
  symPrefix = symmetries(idx).prefix;

  fprintf('\n--- %s (%s) ---\n', symName, symHM);

  % Create crystal symmetry object
  cs = crystalSymmetry(symHM);

  %% TSL-style IPF color key
  fprintf('  Generating TSL color key...\n');
  ipfKeyTSL = ipfColorKey(cs);
  ipfKeyTSL.inversePoleFigureDirection = ipfDirection;

  fig1 = figure('Visible', 'off');
  plot(ipfKeyTSL);
  title(sprintf('%s (%s) - TSL Color Key', symName, symHM));

  tslFile = fullfile(outputDir, sprintf('%s_TSL_Z.png', symPrefix));
  exportgraphics(fig1, tslFile, 'Resolution', imageDPI);
  fprintf('  Saved: %s\n', tslFile);
  close(fig1);

  %% Nolze-Hielscher (HSV) IPF color key
  fprintf('  Generating Nolze-Hielscher (HSV) color key...\n');
  ipfKeyHSV = ipfHSVKey(cs);
  ipfKeyHSV.inversePoleFigureDirection = ipfDirection;

  fig2 = figure('Visible', 'off');
  plot(ipfKeyHSV);
  title(sprintf('%s (%s) - Nolze-Hielscher HSV Color Key', symName, symHM));

  hsvFile = fullfile(outputDir, sprintf('%s_NH_HSV_Z.png', symPrefix));
  exportgraphics(fig2, hsvFile, 'Resolution', imageDPI);
  fprintf('  Saved: %s\n', hsvFile);
  close(fig2);

  %% Also generate X and Y direction legends for cubic (the most common case)
  if strcmp(symHM, 'm-3m')
    % X direction - TSL
    fprintf('  Generating TSL color key (X direction)...\n');
    ipfKeyTSL.inversePoleFigureDirection = vector3d.X;
    fig3 = figure('Visible', 'off');
    plot(ipfKeyTSL);
    title(sprintf('%s (%s) - TSL Color Key [X]', symName, symHM));
    tslFileX = fullfile(outputDir, sprintf('%s_TSL_X.png', symPrefix));
    exportgraphics(fig3, tslFileX, 'Resolution', imageDPI);
    fprintf('  Saved: %s\n', tslFileX);
    close(fig3);

    % Y direction - TSL
    fprintf('  Generating TSL color key (Y direction)...\n');
    ipfKeyTSL.inversePoleFigureDirection = vector3d.Y;
    fig4 = figure('Visible', 'off');
    plot(ipfKeyTSL);
    title(sprintf('%s (%s) - TSL Color Key [Y]', symName, symHM));
    tslFileY = fullfile(outputDir, sprintf('%s_TSL_Y.png', symPrefix));
    exportgraphics(fig4, tslFileY, 'Resolution', imageDPI);
    fprintf('  Saved: %s\n', tslFileY);
    close(fig4);

    % X direction - NH HSV
    fprintf('  Generating Nolze-Hielscher (HSV) color key (X direction)...\n');
    ipfKeyHSV.inversePoleFigureDirection = vector3d.X;
    fig5 = figure('Visible', 'off');
    plot(ipfKeyHSV);
    title(sprintf('%s (%s) - NH HSV Color Key [X]', symName, symHM));
    hsvFileX = fullfile(outputDir, sprintf('%s_NH_HSV_X.png', symPrefix));
    exportgraphics(fig5, hsvFileX, 'Resolution', imageDPI);
    fprintf('  Saved: %s\n', hsvFileX);
    close(fig5);

    % Y direction - NH HSV
    fprintf('  Generating Nolze-Hielscher (HSV) color key (Y direction)...\n');
    ipfKeyHSV.inversePoleFigureDirection = vector3d.Y;
    fig6 = figure('Visible', 'off');
    plot(ipfKeyHSV);
    title(sprintf('%s (%s) - NH HSV Color Key [Y]', symName, symHM));
    hsvFileY = fullfile(outputDir, sprintf('%s_NH_HSV_Y.png', symPrefix));
    exportgraphics(fig6, hsvFileY, 'Resolution', imageDPI);
    fprintf('  Saved: %s\n', hsvFileY);
    close(fig6);
  end
end

%% Summary
fprintf('\n=============================================================\n');
fprintf('  All IPF legend images have been saved to:\n');
fprintf('    %s\n', outputDir);
fprintf('\n');
fprintf('  Files generated:\n');
listing = dir(fullfile(outputDir, '*.png'));
for k = 1:length(listing)
  fprintf('    %s\n', listing(k).name);
end
fprintf('\n');
fprintf('  Compare these reference images against the output of\n');
fprintf('  EbsdLib''s generate_ipf_legends application.\n');
fprintf('=============================================================\n');
