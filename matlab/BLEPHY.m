classdef BLEPHY < handle & matlab.mixin.Copyable
    %BLEPHY BLE physical layer implementation
    properties
        %%%%%%%%%%%%%%%%%%%%%%%%
        % Main experimental parameters used
        %%%%%%%%%%%%%%%%%%%%%%%%
        Mode                    % PHY Mode: {LE1M, LE2M, LE500K, LE125K}
        SamplesPerSymbol        % Samples per symbol
        SymbolRate              % Symbol rate
        SampleRate              % Sample rate (Symbol rate × Samples per symbol)
        ChannelIndex            % BLE channel index

        PDUType                 % Type of PDU, element in
                                % {'Advertising indication', 'Advertising direct indication',
                                % 'Advertising non connectable indication', 'Scan request',
                                % 'Scan response', 'Connection indication',
                                % ' Advertising scannable indication'}

        AccessAddress           % Access address (default '8E89BED6' for advertise)
        AdvertiserAddress       % MAC address of advertiser
        AdvertiserAddressType   % Type of MAC address, element in {'Public', 'Random'}

        Bt                      % Bandwidth-symbol time product
        Span                    % Number of symbols
        H                       % FIR filter coefficients
        S                       % Number of repeating patterns(LE500K: 2, LE125K: 8)

        %%%%%%%%%%%%%%%%%%%%%%%%
        % Length of different parts
        %%%%%%%%%%%%%%%%%%%%%%%%
        PreambleLen             % Length of preamble (determined by PHY mode)
        AccessAddLen            % Length of access address (fixed)
        CodingIndLen            % Length of coding indicator (fixed, only for coded modes (LE125K and LE500K))
        TermSeqLen              % Length of termination fields (fixed, only for coded modes (LE500K and LE125K))
        CRCLen                  % Length of CRC check (fixed)
        HeaderLen               % Length of header (fixed, only for the packet with empty payload)
        MaximumPayloadLen       % Length of maximum payload (fixed)

        %%%%%%%%%%%%%%%%%%%%%%%%
        % Length of different types of packet (packet contains bits)
        %%%%%%%%%%%%%%%%%%%%%%%%
        PacketLen               % Length of packet
        EmptyPacketLen          % Length of empty packet

        %%%%%%%%%%%%%%%%%%%%%%%%
        % Length of different types of frame (frame contains waveforms)
        %%%%%%%%%%%%%%%%%%%%%%%%
        FrameLen                % Length of frame
        MinimumPacketLen        % Length of minimum frame

        %%%%%%%%%%%%%%%%%%%%%%%%
        % Bits of different parts
        %%%%%%%%%%%%%%%%%%%%%%%%
        PreambleBits            % Bits of preamble
        AccessAddressBits       % Bits of access address
        RefSeqBits              % Bits of reference sequence

        %%%%%%%%%%%%%%%%%%%%%%%%
        % Waveform of different parts
        %%%%%%%%%%%%%%%%%%%%%%%%
        Sig                     % Total waveform
        Preamble                % Waveform of preamble
        RefSeq                  % Waveform of reference sequence
        
        %%%%%%%%%%%%%%%%%%%%%%%%
        % Object for demodulation
        %%%%%%%%%%%%%%%%%%%%%%%%
        PrbDet                  % Preamble detector
    end

    methods

        function self = BLEPHY(phyMode, sps, channelIndex, accessAddress)
            %%%%%%%%%%%%%%%%%%%%%%%%
            % Check input parameters
            %%%%%%%%%%%%%%%%%%%%%%%%
            if nargin == 0
                phyMode = 'LE1M';
                sps = 8;
                channelIndex = 37;
                accessAddress = '8E89BED6';
            elseif nargin == 1
                sps = 8;
                channelIndex = 37;
                accessAddress = '8E89BED6';
            elseif nargin == 2
                channelIndex = 37;
                accessAddress = '8E89BED6';
            elseif nargin == 3
                accessAddress = '8E89BED6';
            end

            %%%%%%%%%%%%%%%%%%%%%%%%
            % Assign input parameters
            %%%%%%%%%%%%%%%%%%%%%%%%
            self.Mode = phyMode;
            self.SamplesPerSymbol = sps;
            self.ChannelIndex = channelIndex;

            self.AccessAddress = accessAddress;
            self.AccessAddLen = 32;
            self.AccessAddressBits = int2bit(hex2dec(self.AccessAddress), self.AccessAddLen, false);

            %%%%%%%%%%%%%%%%%%%%%%%%
            % Determine preamble, symbol rate and length of some parts
            %%%%%%%%%%%%%%%%%%%%%%%%
            self.CRCLen = 24;
            self.HeaderLen = 16;
            self.MaximumPayloadLen = 255 * 8;

            self.SymbolRate = 1e6;

            if strcmp(self.Mode, 'LE1M') || strcmp(self.Mode, 'LE2M')

                if strcmp(self.Mode, 'LE1M')
                    self.PreambleLen = 8;
                else
                    self.PreambleLen = 16;
                    self.SymbolRate = 2e6;
                end

                self.PacketLen = self.PreambleLen + self.AccessAddLen + self.MaximumPayloadLen + self.CRCLen;
                self.EmptyPacketLen = self.PreambleLen + self.AccessAddLen + self.HeaderLen + self.CRCLen;
            else
                self.PreambleLen = 80;
                self.CodingIndLen = 2;
                self.TermSeqLen = 3;
                self.S = 8;
                fecBlock1Len = (self.AccessAddLen + self.CodingIndLen + self.TermSeqLen) * self.S;

                if strcmp(self.Mode, 'LE500K')
                    self.S = 2;
                end

                self.PacketLen = self.PreambleLen + fecBlock1Len + (self.MaximumPayloadLen + self.CRCLen + self.TermSeqLen) * self.S;
                self.EmptyPacketLen = self.PreambleLen + fecBlock1Len + (self.HeaderLen + self.CRCLen + self.TermSeqLen) * self.S;
            end

            self.SampleRate = self.SymbolRate * self.SamplesPerSymbol;
            self.FrameLen = self.SamplesPerSymbol * self.PacketLen;
            self.MinimumPacketLen = self.SamplesPerSymbol * self.EmptyPacketLen;

            %%%%%%%%%%%%%%%%%%%%%%%%
            % Matched filter coefficients
            %%%%%%%%%%%%%%%%%%%%%%%%
            self.Bt = 0.5;
            self.Span = 1;
            self.H = gaussdesign(self.Bt, self.Span, self.SamplesPerSymbol);

            %%%%%%%%%%%%%%%%%%%%%%%%
            % Generate reference sequences waveform for synchronization
            %%%%%%%%%%%%%%%%%%%%%%%%

            % generate preamble bits
            self.PreambleBits = ble.internal.preambleGenerator(self.Mode, self.AccessAddressBits);

            % generate reference bits
            if any(strcmp(self.Mode, {'LE1M', 'LE2M'}))
                self.RefSeqBits = [self.PreambleBits; self.AccessAddressBits];
            else
                trellis = poly2trellis(4, [17 13]);
                fecAA = convenc(self.AccessAddressBits, trellis);
                pattern = [1 1 0 0].';
                patternLen = length(pattern);
                repBlock = reshape(repmat(fecAA.', patternLen, 1), 1, []);
                repPattern = reshape(repmat(pattern, 1, length(fecAA)), 1, []);
                codedAA = ~xor(repBlock, repPattern).';
                self.RefSeqBits = [self.PreambleBits; codedAA];
            end

            % Modulate the reference bits into reference sequence waveform
            self.RefSeq = ble.internal.gmskmod(self.RefSeqBits, self.SamplesPerSymbol);
            
            % Generate PreambleDetector objects
            self.PrbDet = comm.PreambleDetector(self.RefSeq, 'Detections', 'First');
        end

        function config_advertising(self, PDUType, advertiserAddress, advertiserAddressType)
            if nargin == 0
                PDUType = 'Advertising non connectable indication';
                advertiserAddress = '1234567890AB';
                advertiserAddressType = "Public";
            elseif nargin == 1
                advertiserAddress = '1234567890AB';
                advertiserAddressType = "Public";
            elseif nargin == 2
                advertiserAddressType = "Public";
            end

            self.PDUType = PDUType;
            self.AdvertiserAddress = advertiserAddress;
            self.AdvertiserAddressType = advertiserAddressType;
        end
        
        function secretBlock = encodeSecretBlock(self, secretBits)
            %encodeSecretBlock  Encode secret messages into BLE standard AD block
            %
            %   [SECRETBLOCK] = ENCODESECRETBLOCK(SECRETBITS)
            %
            %   Input Arguments:
            %       secretBits - Secret messages in the form of bits, N×1
            %
            %   Output Arguments:
            %       secretBlock - Secret messages in the form of BLE's AD struct
        
            secretHexList = self.bit2hex(secretBits);
            % Register a service UUID "EEFD", take the secret message as its data
            secretBlock = [dec2hex(length(secretHexList) + 3, 2); ['16'; 'FD'; 'EE']; secretHexList];
        end

        function secretBits = decodeSecretBlock(self, secretBlock)
            %decodeSecretBlock  Decode secret AD blocks into bits
            %
            %   [SECRETBITS] = DECODESECRETBLOCK(SECRETBLOCK)
            %
            %   Input Arguments:
            %       secretBlock - Secret messages in the form of BLE's AD struct
            %       
            %   Output Arguments:
            %       secretBits - Secret messages in the form of bits, N×1

            secretBits = [];
            for i = 1: length(secretBlock)
                secretBits = [secretBits; str2num(self.hex2bit(secretBlock(i, :))')];
            end
        end

        function sig = modulate(self, secretBits)
            %modulate  Modulate the secret message combined with the information (Flags, Local name) according to the AD structure standard of BLE
            %
            %   [SIG] = MODULATE(SECRETBITS)
            %
            %   Input Arguments:
            %       secretBits - Secret messages in the form of bits, N×1
            %       
            %   Output Arguments:
            %       sig - Signal for BLE advertising containing secret messages
            %
            
            %%%%%%%%%%%%%%%%%%%%%%%%
            % Generate AD struct by general information
            %%%%%%%%%%%%%%%%%%%%%%%%
            cfgGAP = bleGAPDataBlockConfig;
            cfgGAP.AdvertisingDataTypes = {'Flags'; 'Local name'};
            cfgGAP.LEDiscoverability = 'General';
            cfgGAP.BREDR = false;
            cfgGAP.LE = 'None';
            cfgGAP.LocalName = 'Steganography';
            dataBlock1 = bleGAPDataBlock(cfgGAP);

            %%%%%%%%%%%%%%%%%%%%%%%%
            % Generate AD struct by secret information
            %%%%%%%%%%%%%%%%%%%%%%%%
            dataBlock2 = self.encodeSecretBlock(secretBits);

            payload = [dataBlock1; dataBlock2];

            %%%%%%%%%%%%%%%%%%%%%%%%
            % Configure an advertising channel PDU
            %%%%%%%%%%%%%%%%%%%%%%%%
            cfgLLAdv = bleLLAdvertisingChannelPDUConfig;
            cfgLLAdv.PDUType = self.PDUType;
            cfgLLAdv.AdvertiserAddress = self.AdvertiserAddress;
            cfgLLAdv.AdvertiserAddressType = self.AdvertiserAddressType;
            cfgLLAdv.AdvertisingData = payload;

            messageBits = bleLLAdvertisingChannelPDU(cfgLLAdv);

            sig = bleWaveformGenerator(messageBits, ...
                'Mode', self.Mode, ...
                'SamplesPerSymbol', self.SamplesPerSymbol, ...
                'ChannelIndex', self.ChannelIndex, ...
                'AccessAddress', self.AccessAddressBits);
        end

        function [bleSig, prbSig, secretBits, snr, pktDetected, crcChecked] = demodulate(self, sig, secretOriLen)
            %demodulate  Demodulate signals and extract information
            %
            %   [BLESIG, PRBSIG, SECRETBITS, SNR, PKTDETECTED, CRCCHECKED] = MODULATE(SIG, SECRETORILEN)
            %
            %   Input Arguments:
            %       sig - Signal received by the receiver
            %       secretOriLen - Original secret message length
            %       
            %   Output Arguments:
            %       bleSig - BLE signal after synchronisation in the signal
            %       prbSig - Preamble part of the BLE signal
            %       secretBits - Secret messages extracted from demodulated content
            %       snr - SNR of the BLE signal
            %       pktDetected - flag indicate the BLE packet is detected
            %       crcChecked - flag indicate the CRC check is detected

            %%%%%%%%%%%%%%%%%%%%%%%%
            % Initialize output
            %%%%%%%%%%%%%%%%%%%%%%%%
            bleSig = [];
            prbSig = [];
            secretBits = [];
            pktDetected = false;
            crcChecked = false;

            %%%%%%%%%%%%%%%%%%%%%%%%
            % Parameters for SNR
            %%%%%%%%%%%%%%%%%%%%%%%%
            snr = NaN;
            signalPower = NaN;
            noisePower = NaN;
            noiseLen = 1000;

            %%%%%%%%%%%%%%%%%%%%%%%%
            % Parameters of dewhitens
            %%%%%%%%%%%%%%%%%%%%%%%%
            dewhitenStateLen = 6;
            chanIdxBin = int2bit(self.ChannelIndex, dewhitenStateLen)';
            initState = [1 chanIdxBin];
            dewhiten = bluetoothWhiten(InitialConditions = initState');

            %%%%%%%%%%%%%%%%%%%%%%%%
            % Parameters for synchronization
            %%%%%%%%%%%%%%%%%%%%%%%%

            % Detect the preamble index
            [~, dtMt] = self.PrbDet(sig);
            release(self.PrbDet)
            self.PrbDet.Threshold = max(dtMt);
            prbIdx = self.PrbDet(sig);
            release(self.PrbDet);
            
            refSeqLen = length(self.RefSeq); % Reference sequence length
            syncHeadIdx = 1 + prbIdx - refSeqLen; % Target signal start index
            syncTailIdx = NaN; % Target signal end index

            if syncHeadIdx >= 1
                bleSig = sig(syncHeadIdx:end); % Synchronization frames always start with preamble

                if length(bleSig) >= self.MinimumPacketLen
                    % Intercept the tail of the synchronization frame,
                    % make the length of the synchronization frame is an integer multiple of the number of symbol samples
                    bleSig = bleSig(1:end - mod(length(bleSig), self.SamplesPerSymbol));

                    demodBits = ble.internal.gfskdemod(bleSig, self.Mode, self.SamplesPerSymbol, 0.5, 1, 1);

                    if any(strcmp(self.Mode, {'LE1M', 'LE2M'}))
                        preambleBits = demodBits(1:self.PreambleLen);
                    else
                        preambleBits = demodBits(1:self.PreambleLen) < 0;
                    end

                    % Demodulated bits without pramble
                    leftBits = demodBits(1 + self.PreambleLen:end);

                    %%%%%%%%%%%%%%%%%%%%%%%%
                    % Check whether the preamble is the preamble of target packet (target BLE mode)
                    %%%%%%%%%%%%%%%%%%%%%%%%
                    if sum(xor(preambleBits, ~self.PreambleBits)) > length(self.PreambleBits) * 3/4

                        % For 'LE1M', 'LE2M' mode (uncoded mode)
                        if any(strcmp(self.Mode, {'LE1M', 'LE2M'}))
                            accessAddress = int8(leftBits(1:self.AccessAddLen)); % Get access address (32-bit)
                            decodeData = int8(leftBits(1 + self.AccessAddLen:end)); % Get PDU (16-2056 bits) and CRC (24-bit)

                        % For 'LE500K', 'LE125K' mode (coded mode)
                        else

                            if strcmp(self.Mode, 'LE500K') && (rem(length(leftBits), 2) ~= 0) % For 'LE500K' mode, if the length of left bit is not a multiple of 2
                                padLen = 2 - rem(length(leftBits), 2);
                            elseif strcmp(self.Mode, 'LE125K') && (rem(length(leftBits), 8) ~= 0) % For 'LE125K' mode, if the length of left bit is not a multiple of 8
                                padLen = 8 - rem(length(leftBits), 8);
                            else
                                padLen = 0;
                            end

                            leftBits = [leftBits; zeros(padLen, 1)];
                            [decodeData, accessAddress] = ble.internal.decode(-leftBits, self.Mode);
                        end

                        %%%%%%%%%%%%%%%%%%%%%%%%
                        % Check whether the access address is the target access address
                        %%%%%%%%%%%%%%%%%%%%%%%%
                        if isequal(accessAddress, self.AccessAddressBits)
                            % dewhiten data
                            dewhitenedBits = dewhiten(decodeData);
                            reset(dewhiten)

                            PDULenField = double(dewhitenedBits(9:16)); % Second byte of PDU header
                            PDULenInBytes = bit2int(PDULenField, 8, false);
                            PDULenInBits = PDULenInBytes * 8;

                            if length(dewhitenedBits) >= PDULenInBits + self.CRCLen + self.HeaderLen
                                pduBitsWithCRC = dewhitenedBits(1:self.HeaderLen + PDULenInBits + self.CRCLen);
                                [status, cfgLLAdv] = bleLLAdvertisingChannelPDUDecode(pduBitsWithCRC);

                                if strcmp(status, 'Success') && ...
                                        strcmp(cfgLLAdv.AdvertiserAddress, self.AdvertiserAddress) && ...
                                        strcmp(cfgLLAdv.PDUType, self.PDUType)

                                    %%%%%%%%%%%%%%%%%%%%%%%%
                                    % Synchronise the signal
                                    %%%%%%%%%%%%%%%%%%%%%%%%

                                    % Calculate the number of signal's sampling point
                                    if strcmp(self.Mode, 'LE1M') || strcmp(self.Mode, 'LE2M')
                                        signalSamplesLen = (self.PreambleLen + self.AccessAddLen + (self.HeaderLen + PDULenInBits) + self.CRCLen) * self.SamplesPerSymbol;
                                    else
                                        fecBlock1Len = (self.AccessAddLen + self.CodingIndLen + self.TermSeqLen) * self.S;
                                        fecBlock2Len = (self.HeaderLen + PDULenInBits + self.CRCLen + self.TermSeqLen) * self.S;
                                        signalSamplesLen = (self.PreambleLen + fecBlock1Len + fecBlock2Len) * self.SamplesPerSymbol;
                                    end

                                    % Decide the tail index
                                    syncTailIdx = min(signalSamplesLen, length(bleSig));

                                    % Get the synchronized signal
                                    bleSig = bleSig(1:syncTailIdx);

                                    % Get the preamble of signal
                                    prbSig = bleSig(1:self.SamplesPerSymbol * self.PreambleLen);
                                    
                                    % Extract secret AD block and decode it
                                    secretPadLen = ceil(secretOriLen / 8) * 8 - secretOriLen;
                                    secretBlock = cfgLLAdv.AdvertisingData(end - ((secretOriLen + secretPadLen) / 8) + 1:end, :);
                                    secretBits = self.decodeSecretBlock(secretBlock);

                                    %%%%%%%%%%%%%%%%%%%%%%%%
                                    % Estimate the signal's SNR
                                    %%%%%%%%%%%%%%%%%%%%%%%%

                                    % Calculate signal power
                                    signalPower = mean(abs(bleSig) .^ 2);

                                    % Get the noise samples
                                    if syncHeadIdx > 0.9 * noiseLen
                                        noise = sig(1:0.9 * noiseLen);
                                    elseif syncHeadIdx + syncTailIdx + 1 + 0.9 * noiseLen < length(sig)
                                        noise = sig(syncHeadIdx + syncTailIdx + 1:syncHeadIdx + syncTailIdx + 1 + 0.9 * noiseLen);
                                    end

                                    % if syncHeadIdx > 2
                                    %     noise = sig(1: syncHeadIdx-1);
                                    % elseif syncHeadIdx+syncTailIdx+1 < length(sig)
                                    %     noise = sig(syncHeadIdx+syncTailIdx+1: end);
                                    % end

                                    % Calculate noise power
                                    noisePower = mean(abs(noise) .^ 2);

                                    % Estimate SNR
                                    snr = 10 * log10(signalPower / noisePower);

                                    crcChecked = true;

                                end

                            end

                        end

                        pktDetected = true;
                    end

                end

            end

        end

    end

    methods (Static)
        function dataHex = bit2hex(dataBit)
            dataDec = bit2int(dataBit, 8);
            dataHex = dec2hex(dataDec, 2);
        end

        function dataBit = hex2bit(dataHex)
            dataDec = hex2dec(dataHex);
            dataBit = dec2bin(dataDec, 8);
        end

    end

end
