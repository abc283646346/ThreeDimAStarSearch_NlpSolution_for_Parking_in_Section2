clear all
close all
clc
warning off

% Basic parameter setting
InitParams;
% Specify starting and goal configs
global BV_
BV_.x0 = -15;
BV_.y0 = -5;
BV_.theta0 = 0;
BV_.v0 = 0;
BV_.a0 = 0;
BV_.phy0 = 0;
BV_.w0 = 0;
BV_.xtf = 4;
BV_.ytf = -3;
BV_.thetatf = rand * pi;
BV_.vtf = 0;
BV_.atf = 0;
BV_.phytf = 0;
BV_.wtf = 0;

global num_static_obs num_dynamic_obs
num_static_obs = 2;
num_dynamic_obs = 2;
Nfe = 101;
global norm_tf
norm_tf = 40;
global obstacle_vertexes_
obstacle_vertexes_ = GenerateRandomStaticObstacles(num_static_obs);
global moving_obstacle_vertexes_
moving_obstacle_vertexes_ = GenerateRandomStaticObstacles(num_dynamic_obs);
global obstacle_frame_x_ obstacle_frame_y_
[obstacle_frame_x_, obstacle_frame_y_] = GenerateWholeObstacles(Nfe, obstacle_vertexes_, moving_obstacle_vertexes_);
global costmap_
costmap_ = CreateDilatedCostmap(Nfe);
WriteFilesForNLP(Nfe, num_static_obs, num_dynamic_obs);

global coarse_x coarse_y
[x, y, theta] = PlanXYTAStarPath(Nfe);
[coarse_x, coarse_y, coarse_theta, ~] = ResampleConfig(x, y, theta, theta, Nfe);
WriteInitialGuessForNLP(coarse_x, coarse_y, coarse_theta);
!ampl r1.run
IllustrateStaticResult();
IllustrateThreeDimSpaceAndSearchSolution(Nfe);
% IllustrateDynamicResult();