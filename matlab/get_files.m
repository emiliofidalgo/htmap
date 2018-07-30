%
% This file is part of htmap.
%
% Copyright (C) 2018 Emilio Garcia-Fidalgo <emilio.garcia@uib.es> (University of the Balearic Islands)
%
% htmap is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% htmap is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with htmap. If not, see <http://www.gnu.org/licenses/>.

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