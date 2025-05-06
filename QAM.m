% PCS QAM Project
% RUID: 208001821

% Parameters
T = 2;              
A = 1;                
fc = 5;               
Ts = 0.05;           
Fs = 1/Ts;            
samples_per_symbol = T / Ts;
N = 100000;           
rng(208001821); 

%Generate bitstream
bb = randi([0 1], 1, N);        
disp('First 10 bits:');
disp(bb(1:10));

% Group bits into 4-bit symbols for 16-QAM
bb = bb(1:floor(length(bb)/4)*4);
symbols = reshape(bb, 4, []).';

% Map to 16-QAM (Gray Coding)
M = 16;
qamTable = [-3 -1 +3 +1];
I = qamTable(bi2de(symbols(:,1:2),'left-msb') + 1);
Q = qamTable(bi2de(symbols(:,3:4),'left-msb') + 1);

%Add AWGN at various SNRs
SNR_dBs = [0, 3, 7];
vi_list = 10.^(-SNR_dBs/10); 

% Define pulse shapes
p_square = ones(1, samples_per_symbol) * A;
p_sinc = A * sinc((0:Ts:T-Ts) - T/2);

pulse_shapes = {p_square, p_sinc};
pulse_names = {'Square', 'Sinc'};

% Processing for both pulse shapes
for p_idx = 1:2
    pulse = pulse_shapes{p_idx};
    pulse_energy = sum(pulse.^2);
    
    for snr_idx = 1:length(SNR_dBs)
        snr_db = SNR_dBs(snr_idx);
        noise_var = vi_list(snr_idx);

        % Pulse shaping
        I_upsampled = upsample(I, samples_per_symbol);
        Q_upsampled = upsample(Q, samples_per_symbol);
        I_shaped = conv(I_upsampled, pulse, 'same');
        Q_shaped = conv(Q_upsampled, pulse, 'same');

        % Carrier modulation
        t = (0:length(I_shaped)-1) * Ts;
        tx_signal = I_shaped .* cos(2*pi*fc*t) - Q_shaped .* sin(2*pi*fc*t);

        % Add AWGN
        noise = sqrt(noise_var) * randn(size(tx_signal));
        rx_signal = tx_signal + noise;

        % Down-conversion
        I_rx = rx_signal .* cos(2*pi*fc*t);
        Q_rx = -rx_signal .* sin(2*pi*fc*t);

        % Matched filter
        I_filtered = conv(I_rx, fliplr(pulse), 'same');
        Q_filtered = conv(Q_rx, fliplr(pulse), 'same');

        % Sample
        sample_indices = samples_per_symbol:samples_per_symbol:length(I_filtered);
        I_samples = I_filtered(sample_indices);
        Q_samples = Q_filtered(sample_indices);

        % Detection (thresholding)
        pam_levels = [-3 -1 1 3];
        I_detected = pam_levels(knnsearch(pam_levels.', I_samples.'));
        Q_detected = pam_levels(knnsearch(pam_levels.', Q_samples.'));

        % Decode bits
        I_bits = de2bi((I_detected+3)/2, 2, 'left-msb');
        Q_bits = de2bi((Q_detected+3)/2, 2, 'left-msb');
        rx_bits = reshape([I_bits Q_bits].', 1, []);

        % Compute BER
        rx_bits = rx_bits(1:length(bb));
        ber(p_idx, snr_idx) = sum(bb ~= rx_bits) / length(bb);

        % Eye diagram for snr_idx == 1
        if snr_idx == 1
            bits50 = bb(1:200);
            symbols50 = reshape(bits50, 4, []).';
            I_50 = qamTable(bi2de(symbols50(:,1:2), 'left-msb') + 1);
            I_up50 = upsample(I_50, samples_per_symbol);

            % Zero-padding
            pad_len = length(pulse) - 1;
            I_up50_padded = [zeros(1, pad_len), I_up50, zeros(1, pad_len)];
            I_shaped50_full = conv(I_up50_padded, pulse, 'full');
            center_start = pad_len + 1;
            center_end = center_start + length(I_up50) - 1;
            I_shaped50 = I_shaped50_full(center_start:center_end);

            % Eye diagram
            eyediagram(I_shaped50, 2 * samples_per_symbol);
            title(sprintf('Eye Diagram (%s pulse, SNR = ∞ dB)', pulse_names{p_idx}));
            drawnow;
            exportgraphics(gcf, sprintf('eye_%s_snrInf.png', lower(pulse_names{p_idx})));
            close(gcf);
        end

        % Constellation diagram
        if snr_db < 10
            fig = figure;
            scatter(I_samples(1:min(250,end)), Q_samples(1:min(250,end)), 'filled');
            title(sprintf('Constellation (%s pulse, SNR = %d dB)', pulse_names{p_idx}, snr_db));
            xlabel('I'); ylabel('Q'); axis equal; grid on;
            drawnow; pause(0.1);
            saveas(fig, sprintf('constellation_%s_snr%d.jpg', lower(pulse_names{p_idx}), snr_db));
            close(fig);
        end
    end
end

%Constellation Plot for SNR = ∞ dB (no noise)
for p_idx = 1:2
    pulse = pulse_shapes{p_idx};
    pulse_energy = sum(pulse.^2);

    % Pulse shaping
    I_upsampled = upsample(I, samples_per_symbol);
    Q_upsampled = upsample(Q, samples_per_symbol);
    I_shaped = conv(I_upsampled, pulse, 'same');
    Q_shaped = conv(Q_upsampled, pulse, 'same');

    % Carrier modulation
    t = (0:length(I_shaped)-1) * Ts;
    tx_signal = I_shaped .* cos(2*pi*fc*t) - Q_shaped .* sin(2*pi*fc*t);

    % No noise
    rx_signal = tx_signal;

    % Down-conversion
    I_rx = rx_signal .* cos(2*pi*fc*t);
    Q_rx = -rx_signal .* sin(2*pi*fc*t);

    % Matched filter
    I_filtered = conv(I_rx, fliplr(pulse), 'same');
    Q_filtered = conv(Q_rx, fliplr(pulse), 'same');

    % Sample
    sample_indices = samples_per_symbol:samples_per_symbol:length(I_filtered);
    I_samples = I_filtered(sample_indices);
    Q_samples = Q_filtered(sample_indices);

    % Plot constellation
    fig = figure;
    scatter(I_samples(1:min(250,end)), Q_samples(1:min(250,end)), 'filled');
    title(sprintf('Constellation (%s pulse, SNR = ∞ dB)', pulse_names{p_idx}));
    xlabel('I'); ylabel('Q'); axis equal; grid on;
    drawnow; pause(0.1);
    saveas(fig, sprintf('constellation_%s_snrInf.jpg', lower(pulse_names{p_idx})));
    close(fig);
end

%BER Plot
fig = figure;
semilogy(SNR_dBs, ber(1,:), '-o', 'DisplayName', 'Square Pulse');
hold on;
semilogy(SNR_dBs, ber(2,:), '-s', 'DisplayName', 'Sinc Pulse');
legend;
xlabel('SNR (dB)');
ylabel('Bit Error Rate (BER)');
title('BER vs SNR');
grid on;
drawnow; pause(0.1);
saveas(fig, 'ber_vs_snr.jpg');
close(fig);

%Bit Rate Calculation
bitrate = 4 / T;  % 4 bits per symbol, 1 symbol per T seconds
fprintf('Bit Rate of the system: %.2f bits/sec\n', bitrate);
