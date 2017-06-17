%%% Descriptions: Them subfolder vao MATLAB workspace
%%% File: setup_system_path.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function setup_system_path

path0       = mfilename('fullpath');
[path0,~,~] = fileparts(path0);
forbidden   = { [path0,filesep,'bib'],...
    [path0,filesep,'MTIS',filesep,'ToMatlabBinWin32'] };
parse__path_robotics(path0,forbidden);

end

function parse__path_robotics(path,forbidden)
addpath(path);
list = dir(path);
for n=1:length(list)
    if(list(n).isdir)
        if( (list(n).name(1)~='.') && ((list(n).name(1)~='@')) )
            nestedpath = [path,filesep,list(n).name];
            if(~any(strcmp(nestedpath,forbidden)))
                parse__path_robotics(nestedpath,forbidden);
            end
        end
    end
end
end
