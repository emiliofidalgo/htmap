function l = get_files(dirName, ext)
    % This function lists all files included in dirName and filter them
    % according to the ext extension.
    
    dirData = dir(strcat(dirName, '*loops*.txt'));      %# Get the data for the current directory
    dirIndex = [dirData.isdir];  %# Find the index for directories
    fileList = {dirData(~dirIndex).name}';  %'# Get a list of the files
    if ~isempty(fileList)
        fileList = cellfun(@(x) fullfile(dirName,x),...  %# Prepend path to files
            fileList,'UniformOutput',false);
    end
    
    % Filtering files.
    l = [];
    for i=1:length(fileList)
        [path, name, xt] = fileparts(char(fileList(i)));
        if strcmp(xt, ext)
           l = [l; fileList(i)]; 
        end
    end    
end