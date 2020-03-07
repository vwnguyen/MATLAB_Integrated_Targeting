
%#codegen

%hwobj = jetson;
%w = webcam(hwobj,1);
%d = imageDisplay(hwobj);

%persistent yolo2Object;
%persistent w;

%if isempty(yolo2Object)
    yolo2Object = coder.loadDeepLearningNetwork('resnet101_10_preprocessed.mat');
    w = webcam;
%end

% grab an image and annotate it
for i=1:1000000
    img = snapshot(w);
    img = imresize(img,[224 224]);
    [bboxes,scores,labels] = yolo2Object.detect(img,'Threshold',0.6);
    [~,idx] = max(scores);

    % Annotate detections in the image.
    if ~isempty(bboxes)
        outImg = insertObjectAnnotation(img,'Rectangle',bboxes(idx,:),labels{idx});
    else
        outImg = img;
    end
    imshow(outImg);
end
