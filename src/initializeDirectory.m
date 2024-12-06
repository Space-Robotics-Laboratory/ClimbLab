% Change directory and add all folders and files to path
function initializeDirectory()

  % Change current folder if it is not "ClimbLab" directory
  kCurrentPath = convertCharsToStrings(pwd);
  kClimbLabPath = convertCharsToStrings( ...
      erase(mfilename('fullpath'), filesep + "src" + filesep + mfilename));
  if (kCurrentPath ~= kClimbLabPath)
    cd(kClimbLabPath);
  end

  % Make "ClimbLab/dat" folder for data save if it does not exist
  if (~isfolder("dat"))
    mkdir(kClimbLabPath + "/dat");
  end

  addpath(genpath(kClimbLabPath));

end
% EOF
