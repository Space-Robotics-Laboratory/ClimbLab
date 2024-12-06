% Change directory and add all folders and files to path
function initializeDirectory()

% Change current folder if it is not "ClimbLab" directory
current_dir = convertCharsToStrings(pwd);
ClimbLab_dir = convertCharsToStrings(erase(mfilename('fullpath'), "\src\" + mfilename));
if ~strcmp(current_dir, ClimbLab_dir)
  cd(ClimbLab_dir);
end

% Make "ClimbLab/dat" folder for data save if it does not exist
if ~isfolder("dat")
  mkdir(ClimbLab_dir + "/dat");
end

addpath(genpath(ClimbLab_dir));

end
% EOF