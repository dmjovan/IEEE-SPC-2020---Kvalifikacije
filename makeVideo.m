function makeVideo(filename,normality_flag)

bag = rosbag(filename); % ne mogu DataExtract, jer sam tamo namestio da mi vraca samo celije, pa cu ovako
time = bag.EndTime - bag.StartTime;

foto_bag_sel = select(bag,'Topic','/pylon_camera_node/image_raw');
foto_bag_mes = readMessages(foto_bag_sel);  

images = cell(length(foto_bag_mes),1);

for i = 1: length(images)
    images{i} = readImage(foto_bag_mes{i});
    images{i} = imrotate(images{i},180);
end

if normality_flag == 1
    writerObj = VideoWriter(['normal-',filename(length(filename)-22:length(filename)-4),'.avi']);
elseif normality_flag == 0
    writerObj = VideoWriter(['abnormal-',filename(length(filename)-22:length(filename)-4),'.avi']);
end

writerObj.FrameRate = length(images)/time; 

open(writerObj);

for i=1:length(images)
    frame = im2frame(images{i});
    writeVideo(writerObj, frame);
end
close(writerObj);
end
