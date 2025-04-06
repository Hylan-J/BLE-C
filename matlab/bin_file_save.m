clear
clc
close all

%% Configuration parameter
phyMode = 'LE1M';               % Physical layer mode
sps = 16;                       % Samples per symbol
channelIndex = 37;              % Channel index (0~39)
accessAddress = '8E89BED0';     % Access address (32 bit)

%% Configure BLE physical layer parameters
blePHY = BLEPHY(phyMode, sps, channelIndex, accessAddress);
blePHY.config_advertising('Advertising non connectable indication', '1234567890AB', "Public");

%% Save data to .bin file
save_data = [];
for i=1:length(blePHY.RefSeq)
    temp = blePHY.RefSeq(i,:);
    save_data = [real(temp); imag(temp); save_data];
end

fileID = fopen([phyMode, '.bin'], 'w');
fwrite(fileID, save_data, 'double');
fclose(fileID);

disp(['数据已成功保存为 ' phyMode '.bin']);