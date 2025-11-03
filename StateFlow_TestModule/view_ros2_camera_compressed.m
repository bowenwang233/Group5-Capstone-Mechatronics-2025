function view_ros2_camera_compressed(topic, reliability)
% view_ros2_camera_compressed("/depth_cam/rgb/image_raw/compressed","besteffort")
if nargin < 1, topic = "/depth_cam/rgb/image_raw/compressed"; end
if nargin < 2, reliability = "besteffort"; end

setenv("ROS_DOMAIN_ID","0");
setenv("RMW_IMPLEMENTATION","rmw_fastrtps_cpp");

n = ros2node("/matlab_camviewer_cb");

fig = figure('Name',"ROS2 Camera (compressed): "+topic,'NumberTitle','off');
ax  = axes('Parent',fig);
hIm = imshow(uint8(zeros(480,640,3)),'Parent',ax);
title(ax, topic);

% Use callback so frames update as they arrive (no receive loop stall)
sub = ros2subscriber(n, topic, "sensor_msgs/CompressedImage", ...
    Reliability=reliability, History="keeplast", Depth=50, DataFormat="struct", ...
    Callback=@(~,msg) cb(msg));

waitfor(fig);  % keep alive until window is closed

    function cb(msg)
        if ~ishandle(hIm), return; end
        try
            img = rosReadImage(msg);
            set(hIm,'CData',img);
            drawnow limitrate nocallbacks
        catch ME
            warning("Decode error: %s", ME.message);
        end
    end
end