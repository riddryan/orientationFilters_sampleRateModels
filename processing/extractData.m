function O = extractData(opts,files)
if nargin < 1
    opts = defaultDataOpts();
end
if nargin < 2
    files = setFileNames();
end
for ss = 1:length(opts.subjects)
    sstr = ['Subject' num2str(opts.subjects(ss))];
    O.(sstr) = processSubject(ss,opts.conditions,files,opts);
end