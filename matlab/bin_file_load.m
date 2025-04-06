phyMode = 'LE1M';

fileID = fopen([phyMode '.bin'], 'r');
data = fread(fileID, 'double');
fclose(fileID);

disp(['从 ' phyMode '.bin 读取的数据：']);
disp(data);