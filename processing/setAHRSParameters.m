function [p,optim] = setAHRSParameters(sensor,algorithm,AHRS,Di,info,files,opts)
ii = sensor;
opts.sensor = sensor;
sensorname = info.sensorNames{ii};
alignmentinfo = info.alignment;
calibration_info = info.calibration;
algo = algorithm;
optsIn = opts;

p = [];
optim = struct.empty;
switch opts.parameterSelectionMethod
    case 'SubjectOptimized'
        % Performs an optimization on the supplied dataset 
        % Setup file output directories and file names
        sdir = [files.optimizedParametersDir 's' num2str(opts.subjects(opts.subject)) filesep];
        cdir = [sdir opts.conditions{opts.condition} filesep];
        sensdir = [cdir sensorname filesep];
        
        % NOTE: this only keeps track of intermediate files for sampling
        % frequency rounded to the nearest Hz. The loading will not
        % distinguish at any lower resolution with this current code.
        optimFname = [sensdir algo '_' sprintf('%.0f',round(1/AHRS.SamplePeriod)) '.mat'];

        if ~optsIn.reoptimizeParameters && exist(optimFname,'file')
            % Load the optimal parameters
            load(optimFname);
            return
        end

        % Optimize parameter values
        
        [ahrs,cost,q,p,optim] = optimizeAHRSparameters(AHRS,Di,opts);

        % Save optimal parameter values
        if ~exist(sdir,'dir')
            mkdir(sdir);
        end
        if ~exist(cdir,'dir')
            mkdir(cdir);
        end
        if ~exist(sensdir,'dir')
            mkdir(sensdir);
        end
        save(optimFname,'ahrs','p','optim','opts','calibration_info','alignmentinfo');

    case 'SampleRateOptimized'

        dd = load('F:\research\painData\validation2021\estimated\nominalParameters.mat');

        adex = strcmp(dd.algos,algo);

        fs = dd.Fsm{adex};
        pp = dd.PP{adex};

        if resampleFreq > max(fs)
            p = pp(end,:);
        elseif resampleFreq < min(Fs)
            p = pp(1,:);
        else
            p = interp1(fs,pp,resampleFreq);
        end
    case 'NominalWolly'

        % Use a nominal value for the parameter for a given sampling
        % frequency. Based on an optimization procedure performed by Wolly.
        load Fusion_Coef_Fs_all
        condiDex = strcmp({Coefsave.condition},opts.conditions{opts.condition});
        algoDex = strcmp({Coefsave.algorithm},algo);

        if any(algoDex)
            % Interpolate optimal parameters to sampling frequency
            Coeff= Coefsave(condiDex & algoDex);
            coeffVal = [Coeff.mean.OptimA Coeff.mean.OptimB];
            Fs = Coeff.mean.Fs;

            if resampleFreq > max(Fs)
                P = coeffVal(end,:);
            elseif resampleFreq < min(Fs)
                P = coeffVal(1,:);
            else
                P = interp1(Fs,coeffVal,resampleFreq);
            end
            p = P(1:length(opts.(algo).parameters.Names));
        end

end