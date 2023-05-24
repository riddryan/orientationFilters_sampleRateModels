function [sensordata, vicondata,index_start_vicon,index_end_vicon] = correctDelay(sensordata, vicondata, sensornames, delay)

% if delay < 0 remove from vicon at the start
% if delay > 0 remove from dorsaVi at the start
L = size(vicondata.x,1);

index_start_vicon = [];
index_end_vicon = [];
if delay<0
    index_start_vicon = 1:abs(delay);
    index_end_vicon = [];
    
    index_start_dorsavi = [];
    index_end_dorsavi = L-abs(delay)+1:L;
elseif delay>0
    index_start_vicon = [];
    index_end_vicon = L-abs(delay)+1:L;
    
    index_start_dorsavi = 1:abs(delay);
    index_end_dorsavi = [];
end

if delay~=0
    
    % remove from Vicon
    vicondata.x(index_start_vicon,:) = [];
    vicondata.x(index_end_vicon,:) = [];
    vicondata.y(index_start_vicon,:) = [];
    vicondata.y(index_end_vicon,:) = [];
    vicondata.z(index_start_vicon,:) = [];
    vicondata.z(index_end_vicon,:) = [];
    
    vicondata.time(index_start_vicon) = [];
    vicondata.time(index_end_vicon) = [];
    
    % remove from darsaVi
    for ii = 1:length(sensornames)
        
        sensordata.(sensornames{ii}).sensordata(index_start_dorsavi,:) = [];
        sensordata.(sensornames{ii}).sensordata(index_end_dorsavi,:) = [];
        
        sensordata.(sensornames{ii}).attitude(index_start_dorsavi,:) = [];
        sensordata.(sensornames{ii}).attitude(index_end_dorsavi,:) = [];
    end
end
    
    