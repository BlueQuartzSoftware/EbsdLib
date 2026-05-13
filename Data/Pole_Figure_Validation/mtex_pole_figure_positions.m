% mtex_pole_figure_positions.m
%
% Companion to PoleFigurePositionTest.cpp — produces the same CSV schema
% from MTEX so the EbsdLib output can be checked against ground truth.
%
% For each ideal canonical orientation x each unique Laue class x each
% default plane family, this script:
%   1. Builds the orientation in MTEX with the matching crystal symmetry
%   2. Builds the corresponding Miller plane normal
%   3. Computes the symmetry orbit (sample-frame vector3d)
%   4. Stereographic-projects each direction onto the unit disk using
%      the same antipodal-fold rule as EbsdLib's
%      ComputeStereographicProjection (z<0 -> flip, then x/(1+z), y/(1+z))
%   5. Emits one CSV row per pole
%
% CSV schema (matches PoleFigurePositionTest.cpp exactly):
%   orient_id, orient_name, rotation_point_group, symmetry_name,
%   plane_family, x, y
%
% Comparison is then a per-bucket nearest-neighbor match between
% mtex_pole_figure_positions.csv and ebsdlib_pole_figure_positions.csv.
%
% Usage:
%   1. Run this script in MATLAB (MTEX must be on the path; run
%      `startup_mtex` first if needed). The companion shell wrapper
%      run_mtex_pole_figure_positions.sh handles MTEX startup
%      automatically.
%   2. The CSV is written into the same directory as this script
%      (Data/Pole_Figure_Validation/). It is the committed golden against
%      which PoleFigurePositionTest.cpp compares the EbsdLib output.

scriptDir = fileparts(mfilename('fullpath'));
csvPath = fullfile(scriptDir, 'mtex_pole_figure_positions.csv');

% -----------------------------------------------------------------------------
% Reference Bunge tuples in degrees -- mirror the C++ test exactly.
% -----------------------------------------------------------------------------
canonical = { ...
    'Cube',    0.0,   0.0,   0.0  ; ...
    'Goss',    0.0,  45.0,   0.0  ; ...
    'Brass',  35.0,  45.0,   0.0  ; ...
    'Copper', 90.0,  35.0,  45.0  ; ...
    'S',      59.0,  37.0,  63.0  ; ...
    'S1',     55.0,  30.0,  65.0  ; ...
    'S2',     45.0,  35.0,  65.0  ; ...
    'R',      55.0,  75.0,  25.0  ; ...
    'RC_rd1',  0.0,  20.0,   0.0  ; ...
    'RC_rd2',  0.0,  35.0,   0.0  ; ...
    'RC_nd1', 20.0,   0.0,   0.0  ; ...
    'RC_nd2', 35.0,   0.0,   0.0  ; ...
};

% -----------------------------------------------------------------------------
% Per-Laue-class mapping. Borrowed from compare_pole_figures_all_laue.m and
% kept in lockstep with EbsdLib's getDefaultPoleFigureNames() output. The
% label strings MUST match the C++ side exactly so the CSV bucket join
% works.
%
% rpg : EbsdLib rotation point group string (also the join key)
% symName : informational label, mirrors LaueOps::getSymmetryName()
% cs : MTEX crystalSymmetry instance
% h : 1x3 cell array of Miller index tuples (3- or 4-element)
% labels : 1x3 cell array of pole figure label strings (must match
%          EbsdLib output verbatim, including angle brackets)
% -----------------------------------------------------------------------------
laue = struct([]);

laue(end+1).rpg = '432';
laue(end).symName = 'Cubic_High m-3m';
laue(end).cs = crystalSymmetry('m-3m');
laue(end).h = {[0 0 1], [0 1 1], [1 1 1]};
laue(end).labels = {'<001>', '<011>', '<111>'};

laue(end+1).rpg = '23';
laue(end).symName = 'Cubic_Low m-3';
laue(end).cs = crystalSymmetry('m-3');
laue(end).h = {[0 0 1], [0 1 1], [1 1 1]};
laue(end).labels = {'<001>', '<011>', '<111>'};

laue(end+1).rpg = '622';
laue(end).symName = 'Hexagonal_High 6/mmm';
laue(end).cs = crystalSymmetry('6/mmm', [1 1 1.6], 'X||a*');
laue(end).h = {[0 0 0 1], [1 0 -1 0], [1 1 -2 0]};
laue(end).labels = {'<0001>', '<10-10>', '<11-20>'};

laue(end+1).rpg = '6';
laue(end).symName = 'Hexagonal_Low 6/m';
laue(end).cs = crystalSymmetry('6/m', [1 1 1.6], 'X||a*');
laue(end).h = {[0 0 0 1], [1 0 -1 0], [1 1 -2 0]};
laue(end).labels = {'<0001>', '<10-10>', '<11-20>'};

laue(end+1).rpg = '32';
laue(end).symName = 'Trigonal_High -3m';
laue(end).cs = crystalSymmetry('-3m', [1 1 1.6], 'X||a*');
laue(end).h = {[0 0 0 1], [0 -1 1 0], [1 -1 0 0]};
laue(end).labels = {'<0001>', '<0-110>', '<1-100>'};

laue(end+1).rpg = '3';
laue(end).symName = 'Trigonal_Low -3';
laue(end).cs = crystalSymmetry('-3', [1 1 1.6], 'X||a*');
laue(end).h = {[0 0 0 1], [-1 -1 2 0], [2 -1 -1 0]};
laue(end).labels = {'<0001>', '<-1-120>', '<2-1-10>'};

laue(end+1).rpg = '422';
laue(end).symName = 'Tetragonal_High 4/mmm';
laue(end).cs = crystalSymmetry('4/mmm');
laue(end).h = {[0 0 1], [1 0 0], [1 1 0]};
laue(end).labels = {'<001>', '<100>', '<110>'};

laue(end+1).rpg = '4';
laue(end).symName = 'Tetragonal_Low 4/m';
laue(end).cs = crystalSymmetry('4/m');
laue(end).h = {[0 0 1], [1 0 0], [1 1 0]};
laue(end).labels = {'<001>', '<100>', '<110>'};

laue(end+1).rpg = '222';
laue(end).symName = 'OrthoRhombic mmm';
laue(end).cs = crystalSymmetry('mmm');
laue(end).h = {[0 0 1], [1 0 0], [0 1 0]};
laue(end).labels = {'<001>', '<100>', '<010>'};

laue(end+1).rpg = '2';
laue(end).symName = 'Monoclinic 2/m';
laue(end).cs = crystalSymmetry('2/m');
laue(end).h = {[0 0 1], [1 0 0], [0 1 0]};
laue(end).labels = {'<001>', '<100>', '<010>'};

laue(end+1).rpg = '1';
laue(end).symName = 'Triclinic -1';
laue(end).cs = crystalSymmetry('-1');
laue(end).h = {[0 0 1], [1 0 0], [0 1 0]};
laue(end).labels = {'<001>', '<100>', '<010>'};

% -----------------------------------------------------------------------------
% Open CSV and write header
% -----------------------------------------------------------------------------
fid = fopen(csvPath, 'w');
if fid < 0
    error('Could not open output CSV: %s', csvPath);
end
fprintf(fid, 'orient_id,orient_name,rotation_point_group,symmetry_name,plane_family,x,y\n');

ss = specimenSymmetry('1');

% Iterate orientations x Laue classes x plane families
for oi = 1:size(canonical, 1)
    name = canonical{oi, 1};
    phi1 = canonical{oi, 2} * degree;
    Phi  = canonical{oi, 3} * degree;
    phi2 = canonical{oi, 4} * degree;
    orientId = oi - 1;  % 0-based to match the C++ test

    for li = 1:numel(laue)
        info = laue(li);
        cs = info.cs;
        ori = orientation.byEuler(phi1, Phi, phi2, cs, ss);

        for fi = 1:3
            idx = info.h{fi};
            if numel(idx) == 4
                m = Miller(idx(1), idx(2), idx(3), idx(4), cs);
            else
                m = Miller(idx(1), idx(2), idx(3), cs);
            end

            % Symmetry orbit in crystal frame, then map to sample frame.
            % MTEX's symmetrise() returns |cs| entries -- one per symmetry
            % operation, including stabilizer ops that fix the pole. EbsdLib
            % returns the unique orbit (size = |cs| / |stabilizer|), so we
            % dedupe below.
            %
            % MTEX's Miller cartesian is in *lattice units* -- e.g. Miller([1 1 1])
            % has length sqrt(3), and Miller([0 0 0 1]) for hex with c=1.6 has
            % length 1.6. EbsdLib explicitly normalizes its hardcoded direction
            % vectors before projection, so we must normalize here too.
            mSym = symmetrise(m, cs);
            vSample = ori * mSym;        % vector3d array, length = |cs|
            xs = vSample.x; ys = vSample.y; zs = vSample.z;
            mag = sqrt(xs.^2 + ys.^2 + zs.^2);
            xs = xs ./ mag; ys = ys ./ mag; zs = zs ./ mag;

            % Dedupe in 3D after normalization (round to 1e-8 to absorb FP noise).
            xyzKey = round([xs(:), ys(:), zs(:)] * 1e8) / 1e8;
            [~, ia] = unique(xyzKey, 'rows', 'stable');
            xs = xs(ia); ys = ys(ia); zs = zs(ia);

            % Project all directions in this bucket, then emit in sorted
            % (px, py) order so the CSV is byte-stable across MTEX runs.
            % MTEX's symmetrise() emission order isn't guaranteed stable when
            % multiple symmetry-equivalent directions hash-collide (we hit
            % this in the 622/<11-20> bucket -- same 12 points emitted, but
            % the two halves of an antipodal pair swap order between runs).
            % The PoleFigurePositionTest comparator is order-independent so
            % the math is unaffected, but sorting here means `git diff` on a
            % regenerated golden is a clean signal of "the goldens moved"
            % rather than "MTEX shuffled the rows".
            bucketRows = zeros(numel(xs), 2);
            for k = 1:numel(xs)
                x = xs(k); y = ys(k); z = zs(k);
                if z < 0.0
                    x = -x; y = -y; z = -z;
                end
                bucketRows(k, 1) = x / (1.0 + z);
                bucketRows(k, 2) = y / (1.0 + z);
            end
            bucketRows = sortrows(round(bucketRows * 1e8) / 1e8);

            for k = 1:size(bucketRows, 1)
                fprintf(fid, '%d,%s,%s,%s,%s,%.8f,%.8f\n', ...
                    orientId, name, info.rpg, info.symName, info.labels{fi}, ...
                    bucketRows(k, 1), bucketRows(k, 2));
            end
        end
    end
end

fclose(fid);
fprintf('Wrote %s\n', csvPath);
fprintf('This is the committed golden -- PoleFigurePositionTest.cpp loads it\n');
fprintf('and runs the comparison automatically. See ReadMe.md in this directory.\n');
