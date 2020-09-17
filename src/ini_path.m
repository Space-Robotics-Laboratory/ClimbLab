%%%%%% Initialize
%%%%%% ini_path
%%%%%% 
%%%%%% Check and initialize folders
%%%%%% 
%%%%%% Created 2020-04-20
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-20
%
%
% Check and initialize if path was defined correctly
%
%     Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         -

function ini_path(~)


if ~exist('dat', 'dir')
    disp('Wrong directory. Please change your current folder at MATLAB to `climbing_simulator` folder.');
    disp('In this folder you should be able to see `README.md` file, `src/` and `lib/` folders.');
else
	addpath(genpath('lib'));
	addpath(genpath('src'));
	addpath(genpath('config'));
end