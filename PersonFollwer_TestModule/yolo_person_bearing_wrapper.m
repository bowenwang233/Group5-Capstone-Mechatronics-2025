function [personDetected, angle_rad, bbox, label_u8] = yolo_person_bearing_wrapper(img)
% Returns Simulink-friendly fixed types/sizes (no char):
%   personDetected : uint8
%   angle_rad      : double
%   bbox           : 1x4 double
%   label_u8       : 1x32 uint8 (ASCII, padded with spaces)

persistent det HFOV
if isempty(det)
    det  = yolov8ObjectDetector('yolov8s');   % requires CV Toolbox
    HFOV = deg2rad(58.4);
end

% defaults
personDetected = uint8(0);
angle_rad      = NaN;
bbox           = zeros(1,4);
label_u8       = uint8(32*ones(1,32));  % 32 = ASCII space

% run detection
[bboxes, scores, labels] = detect(det, img, 'Threshold', 0.8);
if isempty(labels), return; end

isPerson = (string(labels) == "person");
if ~any(isPerson), return; end

pb = bboxes(isPerson, :);
ps = scores(isPerson);
[~, k] = max(ps);
bb = pb(k, :);          % [x y w h]
sc = ps(k);

% thetaCam (same math as your script)
[~, W, ~] = size(img);
cx = W/2;
u_c = double(bb(1)) + double(bb(3))/2;
xnorm = (u_c - cx) / (W/2);
thetaCam = xnorm * (HFOV/2);

% fill outputs
personDetected = uint8(1);
angle_rad      = thetaCam;
bbox           = double(bb);

% build ASCII label "person: 0.92" as uint8[1x32]
txt = sprintf('person: %.2f', sc);
label_u8 = pad_ascii(txt, 32);

end

function out = pad_ascii(s, L)
% Return uint8(1xL) padded/truncated with spaces (ASCII 32)
u = uint8(s);
if numel(u) >= L
    out = u(1:L);
else
    out = uint8([u, repmat(uint8(32), 1, L-numel(u))]);
end
end
