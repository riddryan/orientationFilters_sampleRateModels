function [x,y,z,Fs,pointsLabel,unlabelled,time]=Import_c3d_data_osim_dorsaVi(filename)

% W van den Hoorn, \
% a wrapper to maintain x,y,z extraction from c3d files using Opensim
% toolbox
% Import opensim library
import org.opensim.modeling.*
 
% Get the path to a C3D file
% [filename, path] = uigetfile('*.c3d');
% c3dpath = fullfile(path,filename);
 
%Construct an opensimC3D object with input c3d path
% Constructor takes full path to c3d file and an integer for forceplate
% representation (1 = COP).
c3dpath = fullfile(filename);
c3d = osimC3D(c3dpath,1);

% Extract marker data;
[markerStruct, forceStruct] = c3d.getAsStructs();

time    = markerStruct.time;

start   = c3d.getStartTime();

fin     = c3d.getEndTime();

Fs      = c3d.getRate_marker();

L       = length(time);%round((fin-start)*Fs+1);

% The labels / and order desired
pointsLabel={
    'cluster1_cluster11'
    'cluster1_cluster12'
    'cluster1_cluster13'
    'cluster1_cluster14'
    
    'cluster2_cluster21'
    'cluster2_cluster22'
    'cluster2_cluster23'
    'cluster2_cluster24'
    
    'cluster3_cluster31'
    'cluster3_cluster32'
    'cluster3_cluster33'
    'cluster3_cluster34'
    
    'board_board1'
    'board_board2'
    'board_board3'
    'board_board4'
    'board_board5'
    };
% Number of markers
nTrajectories = length(pointsLabel);

x=nan(L,nTrajectories);
y=nan(L,nTrajectories);
z=nan(L,nTrajectories);
unlabelled={};
counter=1;
for ii = 1:nTrajectories
    
    if ~isfield(markerStruct,pointsLabel{ii}) % label is not present
        %         organised_marker_data(:,i,:)=NaN(size(markers,1),1,3);
        unlabelled{counter,1}=pointsLabel{ii};
        counter=counter+1;
               
    else
        
        x(:,ii)=markerStruct.(pointsLabel{ii})(:,1);
        y(:,ii)=markerStruct.(pointsLabel{ii})(:,2);
        z(:,ii)=markerStruct.(pointsLabel{ii})(:,3);
        
        %     extractedPoints(:, 1+(i-1)*3:i*3) = points.(pointsLabel{i});
    end
end

