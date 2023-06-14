%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% MIT License
% 
% Copyright (c) 2021 David Wuthier (dwuthier@gmail.com)
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialization
%close all
%clear
clc

% Trajectory generation
knots = [0 15];
waypoints = cell(1,2);
waypoints{1} = [1 ; 1 ; 1];
waypoints{2} = [4 ; 5 ; 1];

%waypoints{1} = [1 ; 1 ; 1];
%waypoints{2} = [5 ; 6 ; 1];

% Fix this...
box_size = 0.25;
run("traj_corridors.m")
save('corridors.mat','-struct','corridors_scaled') % <- note -struct option
%corridors_loaded = load('corridors.mat')
order = 12;
%corridors.times = [1 2 3 4 5];
%corridors.x_lower = [-1 3 8.75 8.5 8];
%corridors.x_upper = [1 7 10.75 9.5 10];
%corridors.y_lower = [-1 0 -0.5 3 8];
%corridors.y_upper = [1 1 1.5 6 10];
%corridors.z_lower = [0 0 0 0 0];
%corridors.z_upper = [2 2 2 2 2];
% ...until here
make_plots = true;

poly_traj = uas_minimum_snap(knots, order, waypoints, corridors, make_plots, map, route);
poly_traj.polyCoeffSet
