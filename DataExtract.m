function [y] = DataExtract(filename,topic)
    bag = rosbag(filename);
    bSel = select(bag,'Topic',topic);
    y = readMessages(bSel); % nece da mi se pretvori u struct, ali radi i ovako kao objekat
end
