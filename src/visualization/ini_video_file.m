%%%%%% Initialize
%%%%%% ini_video_file
%%%%%% 
%%%%%% Create video file
%%%%%% 
%%%%%% Created 2020-04-10
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-17
%
%
% Create file to save simulation video and define video parameters, such as quality and frame rate
%
%     Function variables:
%
%     OUTPUT
%         video           : File ID
%
%     INPUT
%         ani_settings    : Animation settings (class)
%         run_cod         : Program identification code (string)
%         run_id          : Run identification (string)
%         run_date        : Run identification date (string)
 

function video   = ini_video_file(ani_settings, run_cod, run_id, run_date)

%%% Definition of files for video %%%
if strcmp(ani_settings.save,'on')
    % Create directory
    dir_name = ['dat/' run_cod '/' run_id];
    mkdir(dir_name);
    % Create file
    video = VideoWriter([dir_name '/' run_date 'video.avi']);
else
    % Create file
    video = VideoWriter('dat/last_video.avi');
end

video.Quality = 100;
video.FrameRate = ani_settings.frame_rate; 
open(video);

end