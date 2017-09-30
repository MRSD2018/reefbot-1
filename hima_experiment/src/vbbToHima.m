function vbbToHima(inputFile, outputFile)
% Converts a video bounding box annotation in vbb format (the Caltech
% format) into the format used by the HIMA dataset.
%
% The new format is a single space seaprated variable file with the following fields on each
% line:
% 
% 1: time stamp
% 2: frame number
% 3: person ID
% 4: annotation type (0=only partly visible, 1=completely visible, 2 unknown)
% 5-8: corners of the bounding box in left frame
% 9-12: corners of the bounding box in right frame
% 13-14: center of the bounding box in the left frame
% 15-16: center of the bounding box in the right frame
% 17-19: rough 3d coordinates of the annotation, especially 0-type annotations contain errros

vbbData = vbb('vbbLoad', inputFile);

% We don't know the frame rate, so assume 30fps
frameRate = 30.0;
startTime = now;

outStream = fopen(outputFile, 'w+t');
for i = 1:vbbData.nFrame
    for j = 1:length(vbbData.objLists{i})
        curLabel = vbbData.objLists{i}(j);
        fprintf(outStream, '%f %i %i %i %0.2f %0.2f %0.2f %0.2f -1 -1 -1 -1 %0.2f %0.2f -1 -1 -1 -1 -1\n', ...
            startTime + i/frameRate, ...
            i-1, ...
            curLabel.id, ...
            ~curLabel.occl, ...
            curLabel.pos(1), curLabel.pos(2), ...
            curLabel.pos(1) + curLabel.pos(3), ...
            curLabel.pos(2) + curLabel.pos(4), ...    
            (curLabel.pos(1) + curLabel.pos(3)/2.0), ...
            (curLabel.pos(2) + curLabel.pos(4)/2.0));
    end
end

fclose(outStream);

end
