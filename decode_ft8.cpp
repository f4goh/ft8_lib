#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>

#include "ft8/unpack.h"
#include "ft8/ldpc.h"
#include "ft8/decode.h"
#include "ft8/constants.h"
#include "ft8/encode.h"

#include "common/wave.h"
#include "common/debug.h"
#include "fft/kiss_fftr.h"

#define LOG_LEVEL   LOG_INFO

const int kMax_candidates = 100;
const int kLDPC_iterations = 20;

const int kMax_decoded_messages = 50;
const int kMax_message_length = 20;


void usage() {
    fprintf(stderr, "Decode a 15-second WAV file.\n");
}


float hann_i(int i, int N) {
    float x = sinf((float)M_PI * i / (N - 1));
    return x*x;
}


float hamming_i(int i, int N) {
    const float a0 = (float)25 / 46;
    const float a1 = 1 - a0;

    float x1 = cosf(2 * (float)M_PI * i / (N - 1));
    return a0 - a1*x1;
}


float blackman_i(int i, int N) {
    const float alpha = 0.16f; // or 2860/18608
    const float a0 = (1 - alpha) / 2;
    const float a1 = 1.0f / 2;
    const float a2 = alpha / 2;

    float x1 = cosf(2 * (float)M_PI * i / (N - 1));
    //float x2 = cosf(4 * (float)M_PI * i / (N - 1));
    float x2 = 2*x1*x1 - 1; // Use double angle formula

    return a0 - a1*x1 + a2*x2;
}


// Compute FFT magnitudes (log power) for each timeslot in the signal
void extract_power(const float signal[], int num_blocks, int num_bins, uint8_t power[]) {
    const int block_size = 2 * num_bins;      // Average over 2 bins per FSK tone
    const int nfft = 2 * block_size;          // We take FFT of two blocks, advancing by one
    const float fft_norm = 2.0f / nfft;

    float   window[nfft];
    for (int i = 0; i < nfft; ++i) {
        window[i] = blackman_i(i, nfft);
    }

    size_t  fft_work_size;
    kiss_fftr_alloc(nfft, 0, 0, &fft_work_size);

    LOG(LOG_INFO, "N_FFT = %d\n", nfft);
    LOG(LOG_INFO, "FFT work area = %lu\n", fft_work_size);

    void        *fft_work = malloc(fft_work_size);
    kiss_fftr_cfg fft_cfg = kiss_fftr_alloc(nfft, 0, fft_work, &fft_work_size);

    int offset = 0;
    float max_mag = -100.0f;
    for (int i = 0; i < num_blocks; ++i) {
        // Loop over two possible time offsets (0 and block_size/2)
        for (int time_sub = 0; time_sub <= block_size/2; time_sub += block_size/2) {
            kiss_fft_scalar timedata[nfft];
            kiss_fft_cpx    freqdata[nfft/2 + 1];
            float           mag_db[nfft/2 + 1];

            // Extract windowed signal block
            for (int j = 0; j < nfft; ++j) {
                timedata[j] = window[j] * signal[(i * block_size) + (j + time_sub)];
            }

            kiss_fftr(fft_cfg, timedata, freqdata);

            // Compute log magnitude in decibels
            for (int j = 0; j < nfft/2 + 1; ++j) {
                float mag2 = (freqdata[j].i * freqdata[j].i + freqdata[j].r * freqdata[j].r);
                mag_db[j] = 10.0f * log10f(1E-10f + mag2 * fft_norm * fft_norm);
            }

            // Loop over two possible frequency bin offsets (for averaging)
            for (int freq_sub = 0; freq_sub < 2; ++freq_sub) {                
                for (int j = 0; j < num_bins; ++j) {
                    float db1 = mag_db[j * 2 + freq_sub];
                    float db2 = mag_db[j * 2 + freq_sub + 1];
                    float db = (db1 + db2) / 2;

                    // Scale decibels to unsigned 8-bit range and clamp the value
                    int scaled = (int)(2 * (db + 120));
                    power[offset] = (scaled < 0) ? 0 : ((scaled > 255) ? 255 : scaled);
                    ++offset;

                    if (db > max_mag) max_mag = db;
                }
            }
        }
    }

    LOG(LOG_INFO, "Max magnitude: %.1f dB\n", max_mag);
    free(fft_work);
}


void normalize_signal(float *signal, int num_samples) {
    float max_amp = 1E-5f;
    for (int i = 0; i < num_samples; ++i) {
        float amp = fabsf(signal[i]);
        if (amp > max_amp) {
            max_amp = amp;
        }
    }
    for (int i = 0; i < num_samples; ++i) {
        signal[i] /= max_amp;
    }    
}


void print_tones(const uint8_t *code_map, const float *log174) {
    for (int k = 0; k < ft8::N; k += 3) {
        uint8_t max = 0;
        if (log174[k + 0] > 0) max |= 4;
        if (log174[k + 1] > 0) max |= 2;
        if (log174[k + 2] > 0) max |= 1;
        LOG(LOG_DEBUG, "%d", code_map[max]);
    }
    LOG(LOG_DEBUG, "\n");
}


int main(int argc, char **argv) {
    // Expect one command-line argument
    if (argc < 2) {
        usage();
        return -1;
    }

    const char *wav_path = argv[1];

    int sample_rate = 12000;
    int num_samples = 15 * sample_rate;
    float signal[num_samples];

    int rc = load_wav(signal, num_samples, sample_rate, wav_path);
    if (rc < 0) {
        return -1;
    }
    normalize_signal(signal, num_samples);

    const float fsk_dev = 6.25f;    // tone deviation in Hz and symbol rate

    // Compute DSP parameters that depend on the sample rate
    const int num_bins = (int)(sample_rate / (2 * fsk_dev));
    const int block_size = 2 * num_bins;
    const int num_blocks = (num_samples - (block_size/2) - block_size) / block_size;

    LOG(LOG_INFO, "%d blocks, %d bins\n", num_blocks, num_bins);

    // Compute FFT over the whole signal and store it
    uint8_t power[num_blocks * 4 * num_bins];
    extract_power(signal, num_blocks, num_bins, power);

    // Find top candidates by Costas sync score and localize them in time and frequency
    ft8::Candidate candidate_list[kMax_candidates];
    int num_candidates = ft8::find_sync(power, num_blocks, num_bins, ft8::kCostas_map, kMax_candidates, candidate_list);

    // TODO: sort the candidates by strongest sync first?

    // Go over candidates and attempt to decode messages
    char    decoded[kMax_decoded_messages][kMax_message_length];
    int     num_decoded = 0;
    for (int idx = 0; idx < num_candidates; ++idx) {
        ft8::Candidate &cand = candidate_list[idx];
        float freq_hz  = (cand.freq_offset + cand.freq_sub / 2.0f) * fsk_dev;
        float time_sec = (cand.time_offset + cand.time_sub / 2.0f) / fsk_dev;

        float   log174[ft8::N];
        ft8::extract_likelihood(power, num_bins, cand, ft8::kGray_map, log174);

        // bp_decode() produces better decodes, uses way less memory
        uint8_t plain[ft8::N];
        int     n_errors = 0;
        ft8::bp_decode(log174, kLDPC_iterations, plain, &n_errors);
        //ldpc_decode(log174, kLDPC_iterations, plain, &n_errors);

        if (n_errors > 0) {
            LOG(LOG_DEBUG, "ldpc_decode() = %d (%.0f Hz)\n", n_errors, freq_hz);
            continue;
        }
        
        // Extract payload + CRC (first ft8::K bits)
        uint8_t a91[ft8::K_BYTES];
        ft8::pack_bits(plain, ft8::K, a91);

        // Extract CRC and check it
        uint16_t chksum = ((a91[9] & 0x07) << 11) | (a91[10] << 3) | (a91[11] >> 5);
        a91[9] &= 0xF8;
        a91[10] = 0;
        a91[11] = 0;
        uint16_t chksum2 = ft8::crc(a91, 96 - 14);
        if (chksum != chksum2) {
            LOG(LOG_DEBUG, "Checksum: message = %04x, CRC = %04x\n", chksum, chksum2);
            continue;
        }

        char message[kMax_message_length];
        ft8::unpack77(a91, message);

        // Check for duplicate messages (TODO: use hashing)
        bool found = false;
        for (int i = 0; i < num_decoded; ++i) {
            if (0 == strcmp(decoded[i], message)) {
                found = true;
                break;
            }
        }

        if (!found && num_decoded < kMax_decoded_messages) {
            strcpy(decoded[num_decoded], message);
            ++num_decoded;

            // Fake WSJT-X-like output for now
            int snr = 0;    // TODO: compute SNR
            printf("000000 %3d %4.1f %4d ~  %s\n", cand.score, time_sec, (int)(freq_hz + 0.5f), message);
        }
    }
    LOG(LOG_INFO, "Decoded %d messages\n", num_decoded);

    return 0;
}