classdef LiteVNA < handle
% LiteVNA - MATLAB interface for LiteVNA 62/64 Vector Network Analyzer.
%
% Creates a LiteVNA object to communicate with LiteVNA hardware over a serial connection. 
% This class allows configuration of sweep parameters (frequency range, number of points,
% averaging) and enables single or dual-channel S-parameter measurements (S11, S21). Also 
% provides calibration utilities.
%
% Example:
%   % Open device and print info
%   vna = LiteVNA('/dev/ttyACM0', 115200); % On Windows use 'COM<num>'
%   info = vna.getDeviceInfo() % Display info as specified in the manual
%   
%   % Configure a sweep: 5.70-6.30 GHz, 401 pts, avg 10
%   f_start = 5.70e9; f_end = 6.30e9; pts = 401; avg = 10;
%   step = (f_end - f_start)/(pts - 1);
%   vna.configureSweep(f_start, step, pts, avg);
%
%   % Calibrate device using available SOLT standards
%   vna.performCalibration(f_start, f_end, pts, avg);
%   vna.saveCalibration("calibration-file");
%   vna.loadCalibration("test-calibration", true); % Forceful load of another file
%
%   % Measure both channels (use calibration if available/valid)
%   [~, hasCal] = vna.checkCalibration();
%   if hasCal
%       [S11, S21] = vna.calMeasureS11S21();
%   else
%       [S11, S21] = vna.rawMeasureS11S21();
%   end
%
% See also:
%   - Documentation for the LiteVNA 62/64: https://www.zeenko.tech/litevna, https://groups.io/g/liteVNA/
%   - Python example for NanoVNA: https://gist.github.com/nanovna

    properties (Constant, Access=private)
        % Command identifiers
        CMD_WRITE1              = hex2dec('20'); % Write 1 byte (8-bit) to a register
        CMD_WRITE2              = hex2dec('21'); % Write 2 bytes (16-bit) to a register
        CMD_WRITE4              = hex2dec('22'); % Write 4 bytes (32-bit) to a register
        CMD_WRITE8              = hex2dec('23'); % Write 8 bytes (64-bit) to a register
        CMD_READ1               = hex2dec('10'); % Read 1 byte (8-bit) from a register
        CMD_READ2               = hex2dec('11'); % Read 2 bytes (16-bit) from a register
        CMD_READ4               = hex2dec('12'); % Read 4 bytes (32-bit) from a register
        CMD_READFIFO            = hex2dec('18'); % Read measurement data from FIFO

        % Register addresses
        ADDR_SWEEP_START        = hex2dec('00'); % Sweep start frequency register
        ADDR_SWEEP_STEP         = hex2dec('10'); % Sweep step frequency register
        ADDR_SWEEP_POINTS       = hex2dec('20'); % Number of frequency points in sweep
        ADDR_VALS_PER_FREQ      = hex2dec('22'); % Number of averaged samples per point
        ADDR_AVERAGE            = hex2dec('40'); % Averaging control register
        
        ADDR_LOW_FREQ_POWER     = hex2dec('41'); % MS5351 power (low-med-high): 0x01..0x03 (default 0x01)
        ADDR_HIGH_FREQ_POWER    = hex2dec('42'); % MAX2871 power (low-med-high): 0x01..0x03 (default 0x03)

        ADDR_CHANNEL_SELECT     = hex2dec('44'); % Select measurement channel (S11, S21, or both)
        ADDR_VALUES_FIFO        = hex2dec('30'); % FIFO buffer for measurement results
        
        ADDR_UNIX_TIME          = hex2dec('58'); % Unix 4-byte timestamp (uint32)
        ADDR_DEVICE_VARIANT     = hex2dec('F0'); % Always 0x02 for LiteVNA
        ADDR_PROTOCOL_VERSION   = hex2dec('F1'); % Always 0x01 (this wire protocol)
        ADDR_HW_REVISION        = hex2dec('F2'); % Hardware revision
        ADDR_FW_MAJOR           = hex2dec('F3'); % Firmware major
        ADDR_FW_MINOR           = hex2dec('F4'); % Firmware minor

        ADDR_RAW_SAMPLES_MODE   = hex2dec('26'); % Write 0x01 to switch to raw-samples (leaves this protocol)

        % Channel select values
        CHANNEL_BOTH            = hex2dec('00'); % S11 and S21 (slightly slower)
        CHANNEL_S11             = hex2dec('01'); % S11 only
        CHANNEL_S21             = hex2dec('02'); % S21 only
    end

    properties (Access=private)
        % LiteVNA measurement properties
        serial                  = []; % Serialport object
        sweepStartHz            = -1; % Starting frequency of sweep [Hz]
        sweepStepHz             = -1; % Frequency step between each point [Hz]
        sweepPoints             = -1; % Total number of frequency points in the sweep
        avgCount                = -1; % Number of samples to average per frequency

        % Calibration properties
        cal = struct( ...
            'mode',               "none", ... % "none" | "sol" | "tr" | "er"
            'f',                  [], ...
            'e00',                [], ...
            'e11',                [], ...
            'e01e10',             [], ...
            'e10e32',             [], ...
            'created',            datetime.empty, ...
            'note',               "" ...
        );
        calCache = struct( ...
            'f',                  [], ...
            'e00',                [], ...
            'e11',                [], ...
            'e01e10',             [], ...
            'e10e32',             [] ...
        );
    end

    methods (Access=public)
        function obj = LiteVNA(port, baud)
        % LiteVNA - Class constructor
            obj.serial = serialport(port, baud, 'Timeout', 15);
            flush(obj.serial);
        end

        function configureSweep(obj, sweepStart, sweepStep, sweepPoints, avgCount)
        % configureSweep - Set up sweep parameters for the LiteVNA measurement.
        %
        % These parameters are written to the device registers and stored
        % internally for later use in measurement functions.
        %
        % Parameters:
        %   sweepStart   - Starting frequency of sweep [Hz]
        %   sweepStep    - Frequency increment per point [Hz]
        %   sweepPoints  - Total number of frequency points in the sweep
        %   avgCount     - Number of samples to average per frequency

            obj.assertValidSweep(sweepStart, sweepStep, sweepPoints, avgCount);

            obj.sweepStartHz = sweepStart;
            obj.sweepStepHz = sweepStep;
            obj.sweepPoints = sweepPoints;
            obj.avgCount = avgCount;

            % Write frequency and sweep settings to the VNA registers
            obj.writeSweep(sweepStart, sweepStep, sweepPoints, avgCount);

            % Clear cached calibration
            obj.clearCalCache();
            flush(obj.serial);
        end

        function flag = isSweepConfigured(obj)
        % isSweepConfigured - Returns true if sweep parameters are configured.
        %
        % Returns:
        %   flag - Logical true if all sweep parameters are valid, false otherwise
    
            flag = obj.sweepStartHz > 0 && obj.sweepStepHz > 0 && obj.sweepPoints > 0 && obj.avgCount > 0;
        end

        function freqArray = getFrequencies(obj)
        % getFrequencies - Returns frequency vector used in the last sweep.
        %
        % Returns:
        %   freqArray - Nx1 vector with frequencies used in each measurements (N = sweepPoints)

            obj.assertSweepConfigured();

            freqArray = obj.sweepStartHz + (0:1:obj.sweepPoints-1).' * obj.sweepStepHz;
        end

        function setLowFreqPower(obj, level)
        % setLowFreqPower - Set MS5351 (low-frequency) source power (0x01..0x03).

            if ~isscalar(level) || level < 1 || level > 3
                error('LiteVNA:InvalidLFPower', 'Low-frequency power must be 1, 2 or 3.');
            end

            obj.writeReg(obj.CMD_WRITE1, obj.ADDR_LOW_FREQ_POWER, level, 1);
        end
        
        function level = getLowFreqPower(obj)
        % getLowFreqPower - Read MS5351 power setting (returns 1..3).

            level = uint8(obj.readReg(obj.CMD_READ1, obj.ADDR_LOW_FREQ_POWER, 1));
            level = sprintf('0x%02X', level);
        end
        
        function setHighFreqPower(obj, level)
        % setHighFreqPower - Set MAX2871 (high-frequency) source power (0x01..0x03).

            if ~isscalar(level) || level < 1 || level > 3
                error('LiteVNA:InvalidHFPower', 'High-frequency power must be 1, 2 or 3.');
            end

            obj.writeReg(obj.CMD_WRITE1, obj.ADDR_HIGH_FREQ_POWER, level, 1);
        end
        
        function level = getHighFreqPower(obj)
        % getHighFreqPower - Read MAX2871 power setting (returns 1..3).

            level = uint8(obj.readReg(obj.CMD_READ1, obj.ADDR_HIGH_FREQ_POWER, 1));
            level = sprintf('0x%02X', level);
        end

        function S11 = rawMeasureS11(obj)
        % rawMeasureS11 - Retrieves the complex reflection coefficient (S11) data.
        %
        % Enables single-channel mode on the LiteVNA for faster scanning time.
        %
        % Returns:
        %   S11 - Nx1 complex vector of S11 measurements (N = sweepPoints)

            [S11, ~] = obj.readS11S21FIFO(obj.CHANNEL_S11);
        end

        function S21 = rawMeasureS21(obj)
        % rawMeasureS21 - Retrieves the complex forward transmission (S11) data.
        %
        % Enables single-channel mode on the LiteVNA for faster scanning time.
        %
        % Returns:
        %   S21 - Nx1 complex vector of S21 measurements (N = sweepPoints)

            [~, S21] = obj.readS11S21FIFO(obj.CHANNEL_S21);
        end

        function [S11, S21] = rawMeasureS11S21(obj)
        % rawMeasureS11S21 - Simultaneously retrieves both S11 and S21 measurements.
        %
        % Enables dual-channel mode on the LiteVNA to capture both reflection
        % and transmission coefficients in a single sweep.
        %
        % Returns:
        %   S11 - Nx1 complex vector of reflection coefficients (N = sweepPoints)
        %   S21 - Nx1 complex vector of transmission coefficients

            [S11, S21] = obj.readS11S21FIFO(obj.CHANNEL_BOTH);
        end

        function performCalibration(obj, sweepStart, sweepEnd, sweepPoints, avgCount)
        % performCalibration - Guides the user through calibration.
        %
        % Incomplete 2-port vector network analyzer calibration methods implemented,
        % such as, 'sol' = 1-port calibration at port 1, 'tr' = 1-port calibration at 
        % port 1 and normalization at port 2, and 'er' = forward parameters calibrated
        %
        % Notes:
        %   - To save the calibration file, one must call 'saveCalibration' method.
        %   - Simple calibration methods from https://ieeexplore.ieee.org/document/6868593.
        
            % Apply calibration sweep
            sweepStep = (sweepEnd - sweepStart) / (sweepPoints - 1);
            obj.assertValidSweep(sweepStart, sweepStep, sweepPoints, avgCount);

            sweepStartHz_bck = obj.sweepStartHz;
            sweepStepHz_bck = obj.sweepStepHz;
            sweepPoints_bck = obj.sweepPoints;
            avgCount_bck = obj.avgCount;

            obj.configureSweep(sweepStart, sweepStep, sweepPoints, avgCount);

            % Ask before overwriting
            [currentMode, hasCal] = obj.checkCalibration();
            if hasCal
                fprintf('A calibration is already set (mode: %s).\n', currentMode);
                s = lower(strtrim(input('Overwrite existing calibration? [y/N]: ', 's')));
                if isempty(s) || ~any(strcmp(s, {'y','yes'}))
                    disp('Calibration aborted (kept existing calibration).');
                    return;
                end
            end

            % Choose method: sol | tr | er
            disp('Select calibration method:');
            disp('  sol : One-port (S11 cal)');
            disp('  tr  : One-port + Normalization (S11 cal & S21 norm)');
            disp('  er  : Two-port Enhanced Response (S11 cal & S21 cal)');
            method = '';
            while ~any(strcmp(method, {'sol','tr','er'}))
                method = lower(strtrim(input('Enter method (sol/tr/er): ', 's')));
            end

            % Calibration measurements
            disp('Starting...');

            disp('Connect SHORT in Port-1 and press any key to continue...'); pause;
            t1 = tic;
            S11_short = obj.rawMeasureS11();
            fprintf('Took %.3f seconds.\n', toc(t1));
            
            disp('Connect OPEN in Port-1 and press any key to continue...'); pause;
            t2 = tic;
            S11_open = obj.rawMeasureS11();
            fprintf('Took %.3f seconds.\n', toc(t2));
        
            disp('Connect LOAD in Port-1 and press any key to continue...'); pause;
            t3 = tic;
            S11_load = obj.rawMeasureS11();
            fprintf('Took %.3f seconds.\n', toc(t3));

            if ~strcmp(method,'sol')
                disp('Connect THRU between Port-1 and Port2 and press any key to continue...'); pause;
                t4 = tic;
                S21_thru = obj.rawMeasureS21();
                fprintf('Took %.3f seconds.\n', toc(t4));
            end
        
            % Assign values to struct
            obj.cal.mode = string(method);
            obj.cal.created = datetime('now');
            obj.cal.note = strtrim(input('Enter note: ', 's'));
            obj.cal.f = linspace(sweepStart, sweepEnd, sweepPoints);

            obj.cal.e00 = S11_load(:);
            obj.cal.e11 = (S11_open + S11_short - 2 .* S11_load) ./ (S11_open - S11_short);
            obj.cal.e01e10 = (-2 .* (S11_open - S11_load) .* (S11_short - S11_load)) ./ (S11_open - S11_short);
            if strcmp(method,'sol')
                obj.cal.e10e32 = [];
            else
                obj.cal.e10e32 = S21_thru(:);
            end

            disp('Calibration complete.');
            obj.clearCalCache();
            if obj.isSweepConfigured()
                % restore to sweep config. previously in use
                obj.configureSweep(sweepStartHz_bck, sweepStepHz_bck, sweepPoints_bck, avgCount_bck);
            end
        end

        function [mode, flag] = checkCalibration(obj)
        % checkCalibration - Sanity check that a calibration is set.
        %
        % Modes: "none" | "sol" | "tr" | "er"
        %   "sol" requires: e00, e11, e01e10
        %   "tr"/"er" require the above and e10e32
        
            mode = string(obj.cal.mode);
            if mode == "none"
                warning('LiteVNA:Uncalibrated','Calibration has not been performed.');
                flag = false;
                return;
            end
        
            f = obj.cal.f;
            n = numel(f);
            if n == 0
                warning('LiteVNA:Uncalibrated','Calibration has no frequency grid.');
                flag = false;
                return;
            end
              
            % Must have SOL terms
            if ~sameLen(obj.cal.e00, n) || ~sameLen(obj.cal.e11, n) || ~sameLen(obj.cal.e01e10, n)
                warning('LiteVNA:Uncalibrated','SOL terms missing or size mismatch (need e00, e11, e01e10).');
                flag = false;
                return;
            end
        
            % TR/ER require the thru term e10e32
            if mode ~= "sol"
                if ~sameLen(obj.cal.e10e32, n)
                    warning('LiteVNA:Uncalibrated','Transmission term e10e32 missing or size mismatch.');
                    flag = false;
                    return;
                end
            end
            
            % If it passes all checks
            flag = true;

            function flag = sameLen(v, n)
                flag = ~isempty(v) && isvector(v) && numel(v) == n;
            end
        end

        function saveCalibration(obj, filename)
        % saveCalibration - Export current calibration to a .mat
        %
        % Usage: obj.saveCalibration('mycal.mat')

            [~, hasCal] = obj.checkCalibration();
            if ~hasCal
                error('LiteVNA:CalibrationSave', 'Calibration has not been set.');
            end

            cal = obj.cal;
            [p,f,e] = fileparts(filename);
            if isempty(e)
                filename = fullfile(p, [f '.mat']); 
            end
            save(filename, 'cal');
            fprintf('Saved calibration to %s (mode: %s)\n', filename, string(obj.cal.mode));
        end

        function loadCalibration(obj, filename, force)
        % loadCalibration - Import calibration from a .mat
        %
        % Usage: 
        %   obj.loadCalibration('mycal.mat')
        %   obj.loadCalibration('mycal.mat', true) % force overwrite (no prompt)

            if nargin < 3, force = false; end
    
            % if something already set, ask before overwrite (unless forced)
            [curMode, hasCal] = obj.checkCalibration();
            if hasCal && ~force
                fprintf('A calibration is already set (mode: %s).\n', curMode);
                s = lower(strtrim(input('Overwrite existing calibration? [y/N]: ', 's')));
                if isempty(s) || ~any(strcmp(s, {'y','yes'}))
                    disp('Load aborted (kept existing calibration).');
                    return;
                end
            end
    
            S = load(filename, 'cal');
            if ~isfield(S, 'cal')
                error('LiteVNA:CalibrationLoad', 'File does not contain a variable named "cal".');
            end
            newcal = S.cal;
    
            % stage, validate, then commit (restore on failure)
            oldcal = obj.cal;
            obj.cal = newcal;
            [loadedMode, ok] = obj.checkCalibration();
            if ~ok
                obj.cal = oldcal; % restore previous calibration
                error('LiteVNA:CalLoad', 'Loaded file does not contain a valid calibration.');
            end
    
            fprintf('Loaded calibration from %s (mode: %s)\n', filename, loadedMode);
        end

        function S11 = calMeasureS11(obj)
        % calMeasureS11 - Measure and apply selected calibration to S11.
        %
        % Returns:
        %   S11 - Nx1 complex vector of calibrated reflection coefficients (N = sweepPoints)
        
            [~, hasCal] = obj.checkCalibration();
            if ~hasCal
                error('LiteVNA:CalibrationMissing', 'Calibration has not been set.');
            end
        
            rawS11 = obj.rawMeasureS11();
            f_meas = obj.getFrequencies();
            f_cal  = obj.cal.f(:);

            if obj.sameGrid(f_meas, obj.calCache.f) && (~isempty(obj.calCache.e00) && ...
                    ~isempty(obj.calCache.e11) && ~isempty(obj.calCache.e01e10))
                e00 = obj.calCache.e00;
                e11 = obj.calCache.e11;
                e01e10 = obj.calCache.e01e10;
            else
                if obj.sameGrid(f_meas, f_cal)
                    e00 = obj.cal.e00(:);
                    e11 = obj.cal.e11(:);
                    e01e10 = obj.cal.e01e10(:);
                else
                    % interpolate onto current sweep grid
                    e00 = obj.interpolateC(f_cal, obj.cal.e00, f_meas);
                    e11 = obj.interpolateC(f_cal, obj.cal.e11, f_meas);
                    e01e10 = obj.interpolateC(f_cal, obj.cal.e01e10, f_meas);
                end
                % update cache
                obj.calCache = struct('f',f_meas,'e00',e00,'e11',e11,'e01e10',e01e10,'e10e32',[]);
            end
            
            numerator = rawS11 - e00;
            delta_e = e00 .* e11 - e01e10;
            denominator = rawS11 .* e11 - delta_e;
            denominator(abs(denominator) < 1e-12) = 1e-12;

            S11 = numerator ./ denominator;
        end
        
        function S21 = calMeasureS21(obj)
        % calMeasureS21 - Measure and apply selected calibration to S21.
        %
        % Returns:
        %   S21 - Nx1 complex vector of calibrated transmission coefficients (N = sweepPoints)
        
            [curCal, hasCal] = obj.checkCalibration();
            if ~hasCal
                error('LiteVNA:CalibrationMissing', 'Calibration has not been set.');
            end
            if strcmpi(curCal,'sol')
                warning('LiteVNA:S21Uncalibrated', ...
                    'Current calibration mode is "sol". S21 will be uncalibrated (raw).');
                S21 = obj.rawMeasureS21();
                return;
            end
        
            rawS21 = obj.rawMeasureS21();
            f_meas = obj.getFrequencies();
            f_cal  = obj.cal.f(:);

            if obj.sameGrid(f_meas, obj.calCache.f) && ~isempty(obj.calCache.e10e32)
                e10e32 = obj.calCache.e10e32;
                if strcmpi(curCal,'er') && (~isempty(obj.calCache.e00) && ~isempty(obj.calCache.e11) && ~isempty(obj.calCache.e01e10))
                    e00 = obj.calCache.e00;
                    e11 = obj.calCache.e11;
                    e01e10 = obj.calCache.e01e10;
                end
            else
                if obj.sameGrid(f_meas, f_cal)
                    e10e32 = obj.cal.e10e32(:);
                    if strcmpi(curCal,'er')
                        e00 = obj.cal.e00(:);
                        e11 = obj.cal.e11(:);
                        e01e10 = obj.cal.e01e10(:);
                    end
                else
                    e10e32 = obj.interpolateC(f_cal, obj.cal.e10e32, f_meas);
                    if strcmpi(curCal,'er')
                        e00 = obj.interpolateC(f_cal, obj.cal.e00, f_meas);
                        e11 = obj.interpolateC(f_cal, obj.cal.e11, f_meas);
                        e01e10 = obj.interpolateC(f_cal, obj.cal.e01e10, f_meas);
                    end
                end
                % update cache
                if strcmpi(curCal,'er')
                    obj.calCache = struct('f',f_meas,'e00',e00,'e11',e11,'e01e10',e01e10,'e10e32',e10e32);
                else
                    obj.calCache.e10e32 = e10e32;
                end
            end
        
            e10e32(abs(e10e32) < 1e-12) = 1e-12;
            S21 = rawS21 ./ e10e32; % normalization only

            if strcmpi(curCal,'er')
                rawS11 = obj.rawMeasureS11();
                
                numerator = S21 .* e01e10;
                delta_e = e00 .* e11 - e01e10;
                denominator = e11 .* rawS11 - delta_e;
                S21 = numerator ./ denominator;
            end
        end

        function [S11, S21] = calMeasureS11S21(obj)
        % calMeasureS11S21 - Measure and apply selected calibration to both S11 and S21.
        %
        % Returns:
        %   S11 - Nx1 complex vector of calibrated reflection coefficients (N = sweepPoints)
        %   S21 - Nx1 complex vector of calibrated transmission coefficients
        
            [curCal, hasCal] = obj.checkCalibration();
            if ~hasCal
                error('LiteVNA:CalibrationMissing', 'Calibration has not been set.');
            end
        
            [rawS11, rawS21] = obj.rawMeasureS11S21();
            f_meas = obj.getFrequencies();
            f_cal = obj.cal.f(:);
        
            if obj.sameGrid(f_meas, obj.calCache.f) && (~isempty(obj.calCache.e00) && ...
                    ~isempty(obj.calCache.e11) && ~isempty(obj.calCache.e01e10) && ~isempty(obj.calCache.e10e32))
                e00 = obj.calCache.e00;
                e11 = obj.calCache.e11;
                e01e10 = obj.calCache.e01e10;
                e10e32 = [];
                if ~strcmpi(curCal,'sol')
                    e10e32 = obj.calCache.e10e32;
                end
            else
                if obj.sameGrid(f_meas, f_cal)
                    e00 = obj.cal.e00(:);
                    e11 = obj.cal.e11(:);
                    e01e10 = obj.cal.e01e10(:);
                    e10e32 = [];
                    if ~strcmpi(curCal,'sol')
                        e10e32 = obj.cal.e10e32(:);
                    end
                else
                    % interpolate onto current sweep grid
                    e00 = obj.interpolateC(f_cal, obj.cal.e00, f_meas);
                    e11 = obj.interpolateC(f_cal, obj.cal.e11, f_meas);
                    e01e10 = obj.interpolateC(f_cal, obj.cal.e01e10, f_meas);
                    if ~strcmpi(curCal,'sol')
                        e10e32 = obj.interpolateC(f_cal, obj.cal.e10e32, f_meas);
                    end
                end
                % update cache
                obj.calCache = struct('f',f_meas,'e00',e00,'e11',e11,'e01e10',e01e10,'e10e32',e10e32);
            end
            
            numerator = rawS11 - e00;
            delta_e = e00 .* e11 - e01e10;
            denominator = rawS11 .* e11 - delta_e;
            denominator(abs(denominator) < 1e-12) = 1e-12;

            S11 = numerator ./ denominator;

            if ~strcmpi(curCal,'sol')
                e10e32(abs(e10e32) < 1e-12) = 1e-12;
                S21 = rawS21 ./ e10e32; % normalization only

                if strcmpi(curCal,'er')                  
                    numerator = S21 .* e01e10;
                    delta_e = e00 .* e11 - e01e10;
                    denominator = e11 .* rawS11 - delta_e;
                    S21 = numerator ./ denominator;
                end
            else
                warning('LiteVNA:S21Uncalibrated', ...
                    'Current calibration mode is "sol". S21 will be uncalibrated (raw).');
                S21 = rawS21;
            end
        end

        function setUnixTime(obj, time)
        % setUnixTime - Set device RTC with Unix time (seconds since 1970-01-01 UTC).
        % Accepts numeric or datetime.

            if isa(time,'datetime')
                time = posixtime(time); 
            end

            if ~isscalar(time) || time < 0 || time > 2^32-1
                error('LiteVNA:InvalidTime','Unix time must be uint32 range.');
            end

            obj.writeReg(obj.ADDR_UNIX_TIME, time, 4);
        end

        function info = getDeviceInfo(obj)
        % getDeviceInfo - Return device identity and firmware info.
        
            dv = uint8(obj.readReg(obj.CMD_READ1, obj.ADDR_DEVICE_VARIANT, 1));
            pv = uint8(obj.readReg(obj.CMD_READ1, obj.ADDR_PROTOCOL_VERSION, 1));
            hw = uint8(obj.readReg(obj.CMD_READ1, obj.ADDR_HW_REVISION, 1));
            maj = uint8(obj.readReg(obj.CMD_READ1, obj.ADDR_FW_MAJOR, 1));
            min = uint8(obj.readReg(obj.CMD_READ1, obj.ADDR_FW_MINOR, 1));
        
            info.deviceVariant = sprintf('0x%02X', dv);
            info.protocolVersion = sprintf('0x%02X', pv);
            info.hardwareRevision = sprintf('0x%02X', hw);
            info.firmwareMajor = sprintf('0x%02X', maj);
            info.firmwareMinor = sprintf('0x%02X', min);
        end
    end

    methods (Access=private)
        function [S11, S21] = readS11S21FIFO(obj, channel)
        % readS11S21FIFO - Internal method to read raw S11/S21 data from FIFO.
        %
        % Configures the VNA to the specified channel mode (S11, S21, or both),
        % clears the FIFO buffer, and reads the binary sweep data from the device.
        %
        % Parameters:
        %   channel - Integer value selecting measurement channel:
        %             0x00: both S11 and S21 (slightly slower)
        %             0x01: S11 only
        %             0x02: S21 only
        %
        % Returns:
        %   S11 - Nx1 complex vector of S11 raw data (valid only if channel is 0 or 1)
        %   S21 - Nx1 complex vector of S21 raw data (valid only if channel is 0 or 2)

            obj.assertSweepConfigured();

            obj.writeReg(obj.CMD_WRITE1, obj.ADDR_CHANNEL_SELECT, channel, 1); % enable channel
            obj.writeReg(obj.CMD_WRITE1, obj.ADDR_VALUES_FIFO, 0, 1); % clear FIFO
            pause(0.005); % 5 [ms]

            data = obj.readFIFO();

            S11 = complex(zeros(obj.sweepPoints, 1));
            S21 = complex(zeros(obj.sweepPoints, 1));

            for k = 1:(obj.sweepPoints * obj.avgCount)
                idx = (k - 1) * 32;
                fwd0 = typecast(uint8(data(idx+1:idx+8)), 'int32'); % channel 0 outgoing wave
                rev0 = typecast(uint8(data(idx+9:idx+16)), 'int32'); % channel 0 incoming wave
                rev1 = typecast(uint8(data(idx+17:idx+24)), 'int32'); % channel 1 incoming wave
                freq_idx = typecast(uint8(data(idx+25:idx+26)), 'int16') + 1;

                fwd0_c = complex(double(fwd0(1)), double(fwd0(2)));
                rev0_c = complex(double(rev0(1)), double(rev0(2)));
                rev1_c = complex(double(rev1(1)), double(rev1(2)));

                S11(freq_idx) = S11(freq_idx) + (rev0_c / max(fwd0_c, 1e-12));
                S21(freq_idx) = S21(freq_idx) + (rev1_c / max(fwd0_c, 1e-12));
            end

            S11 = S11 ./ obj.avgCount;
            S21 = S21 ./ obj.avgCount;
        end

        function data = readFIFO(obj)
        % readFIFO - Read the complete sweep measurement data from the FIFO.
        %
        % Sends a FIFO read command for the current sweep configuration and
        % retrieves raw complex measurement data for all sweep points.
        %
        % Returns:
        %   data - 1x(32*N) uint8 array with raw data (N=sweepPoints)

            totalBytes = 32 * obj.sweepPoints * obj.avgCount;        
            data = zeros(1, totalBytes, 'uint8');
        
            left = obj.sweepPoints * obj.avgCount; ptr = 1; 
            while left > 0
                toRead = min(255, left);
                nBytes = 32 * toRead;
        
                write(obj.serial, uint8([obj.CMD_READFIFO, obj.ADDR_VALUES_FIFO, uint8(toRead)]), 'uint8');
                pause( ...
                    max(0.05, 0.001 * toRead) ...
                );
        
                chunk = read(obj.serial, nBytes, 'uint8');
                if numel(chunk) ~= nBytes
                    error('LiteVNA:IO', 'FIFO read short: got %d of %d bytes.', numel(chunk), nBytes);
                end
        
                data(ptr:ptr+nBytes-1) = chunk;
                ptr = ptr + nBytes;
                left = left - toRead;
            end
        end

        function writeSweep(obj, sweepStart, sweepStep, sweepPoints, avgCount)
        % writeSweep - Write sweep configuration to device registers.
        %
        % Parameters:
        %   sweepStart   - Start frequency [Hz]
        %   sweepStep    - Frequency step per point [Hz]
        %   sweepPoints  - Number of frequency points
        %   avgCount     - Averaging count per point

            obj.writeReg(obj.CMD_WRITE8, obj.ADDR_SWEEP_START, sweepStart, 8);
            pause(0.005); % 5 [ms]
            obj.writeReg(obj.CMD_WRITE8, obj.ADDR_SWEEP_STEP, sweepStep, 8);
            pause(0.005);
            obj.writeReg(obj.CMD_WRITE2, obj.ADDR_SWEEP_POINTS, sweepPoints, 2);
            pause(0.005);
            obj.writeReg(obj.CMD_WRITE2, obj.ADDR_VALS_PER_FREQ, avgCount, 2);
            pause(0.005);
            obj.writeReg(obj.CMD_WRITE1, obj.ADDR_AVERAGE, avgCount, 1);
            pause(0.005);
        end

        function writeReg(obj, cmd, addr, val, nbytes)
        % writeReg - Write an N-byte value to a LiteVNA register.
        %
        % Converts the value to the appropriate number of bytes and sends it with the
        % specified command to the register address on the device.
        %
        % Parameters:
        %   cmd    - Command identifier (e.g. CMD_WRITE1, CMD_WRITE2, ...)
        %   addr   - Register address to write to
        %   val    - Unsigned integer value to send
        %   nbytes - Number of bytes (1, 2, 4, or 8)
        
            switch nbytes
                case 1
                    bytes = uint8(val); % already 1 byte
                case 2
                    bytes = typecast(uint16(val), 'uint8');
                case 4
                    bytes = typecast(uint32(val), 'uint8');
                case 8
                    bytes = typecast(uint64(val), 'uint8');
                otherwise
                    error('Unsupported register size: %d (must be 1, 2, 4, or 8)', nbytes);
            end
        
            write(obj.serial, uint8([cmd, addr, bytes]), 'uint8');
        end

        function data = readReg(obj, cmd, addr, nbytes)
        % readReg - Read raw bytes from a LiteVNA register.
        %
        % Sends the specified read command and register address to the device,
        % then reads back the requested number of bytes from the serial port.
        %
        % Parameters:
        %   cmd    - Command identifier (e.g. CMD_READ1, CMD_READ2, CMD_READ4)
        %   addr   - Register address to read from
        %   nbytes - Number of bytes to read (1, 2 or 4 depending on the register)
        %
        % Returns:
        %   data   - 1 x length uint8 array containing the raw register data

            flush(obj.serial, "input");
            write(obj.serial, uint8([cmd, addr]), 'uint8');
            data = read(obj.serial, nbytes, 'uint8');
        end

        function assertSweepConfigured(obj)
        % assertSweepConfigured - Validates that sweep parameters have been configured.
        %
        % This method checks whether the sweep configuration (start frequency,
        % frequency step, number of points, and averaging count) has been set
        % to valid values. If they are uninitialized, an error is raised.

            if obj.sweepStartHz < 0 || obj.sweepStepHz < 0 || obj.sweepPoints < 0 || obj.avgCount < 0
                error('LiteVNA:SweepNotConfigured', 'Sweep must be configured before performing measurements.');
            end
        end
    
        function clearCalCache(obj)
            obj.calCache = struct('f',[],'e00',[],'e11',[],'e01e10',[],'e10e32',[]);
        end
    end

    methods (Access=private, Static)
        function assertValidSweep(sweepStartHz, sweepStepHz, sweepPoints, avgCount)
        % assertValidSweep - Validate sweep configuration against LiteVNA limits.
        %
        % Checks if all sweep parameters are within the hardware's supported ranges,
        % including start frequency, step size, number of points, and averaging count.
        % Throws an error if any of the constraints are violated.
        %
        % Parameters:
        %   sweepStartHz - Start frequency in Hz (must be between 50 kHz and 6.3 GHz)
        %   sweepStepHz  - Step size between points (must be > 0)
        %   sweepPoints  - Total number of points (1 to 65535)
        %   avgCount     - Averaging count (1 to 80, based on FIFO sample limits)

            if sweepStartHz < 50.0e3 || sweepStartHz > 6.3e9
                error('LiteVNA:InvalidSweep', 'Start frequency must be within 50 kHz - 6.3 GHz range.');
            end

            if sweepStepHz < 0
                error('LiteVNA:InvalidSweep', 'Step frequency must be positive.');
            end

            if sweepPoints < 1 || sweepPoints > 65535
                error('LiteVNA:InvalidSweep', 'Sweep points must be between 1 and 65535.');
            end

            if (sweepStartHz + sweepStepHz * (sweepPoints - 1)) > 6.3e9
                error('LiteVNA:InvalidSweep', 'End frequency %.3f GHz exceeds 6.3 GHz limit.', ...
                    (sweepStartHz + sweepStepHz * (sweepPoints - 1)) * 1e-9);
            end

            if avgCount < 1 || avgCount > 80
                error('LiteVNA:InvalidSweep', 'Average count must be between 1 and 80.');
            end
        end

        function flag = sameGrid(f1, f2)
        % sameGrid - Grid equality with a tiny relative tolerance

            if isempty(f1) || isempty(f2) || numel(f1) ~= numel(f2) 
                flag = false; 
                return; 
            end

            f1 = f1(:); f2 = f2(:);
            scale = max(1, max(abs(f1)));
            flag = max(abs(f1 - f2)) <= 1e-9 * scale;
        end

        function zt = interpolateC(fcal, zcal, ftarget)
        % interpolateC - Linear interpolation of complex valued vector

            % Clamp then interpolate
            fcal = fcal(:); 
            zcal = zcal(:); 
            ftarget = ftarget(:);
            fmin = fcal(1); fmax = fcal(end);
            fclamp = min(max(ftarget, fmin), fmax);
        
            % Interpolate real/imag
            zr = interp1(fcal, real(zcal), fclamp, 'linear');
            zi = interp1(fcal, imag(zcal), fclamp, 'linear');
            zt = complex(zr, zi);
        end
    end
end
