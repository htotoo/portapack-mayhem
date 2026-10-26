#ifndef __PROC_WIDEBAND_CATCH_H__
#define __PROC_WIDEBAND_CATCH_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "rssi_thread.hpp"
#include "dsp_decimate.hpp"
#include "dsp_demodulate.hpp"
#include "dsp_fft.hpp"
#include "audio_output.hpp"
#include "message.hpp"
#include "utility.hpp"

#include <complex>
#include <array>
#include <cmath>

class WidebandCatchProcessor : public BasebandProcessor {
   public:
    WidebandCatchProcessor();

    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const message) override;

   private:
    static constexpr size_t SAMPLE_RATE = 1536000;  // 1.536 MSPS (64 * 24 kHz)
    static constexpr size_t AUDIO_RATE = 24000;
    static constexpr size_t DECIMATION_FACTOR = SAMPLE_RATE / AUDIO_RATE;  // 64
    static constexpr size_t FFT_SIZE = 256;                                // log2(256) = 8 levels, 6 kHz / bin

    enum class State {
        SCANNING,
        LOCKED
    };

    State state{State::SCANNING};

    uint8_t mod_type{0};  // 0: NFM, 1: AM
    uint16_t threshold{35};
    uint8_t min_bins{1};                    // Min wide (1 bin = 6 kHz)
    uint8_t max_bins{4};                    // Max wide (4 bin = 24 kHz)
    uint32_t hold_samples_max{12000};       // 500ms @ 24kHz
    std::array<uint8_t, 32> ignore_mask{};  // 256-bit bin blacklist

    int32_t locked_freq_offset{0};
    uint32_t phase_acc{0};
    uint32_t phase_inc{0};
    uint32_t hold_counter{0};
    uint32_t status_update_counter{0};

    std::array<complex16_t, 256> nco_lut{};

    std::array<complex16_t, FFT_SIZE> fft_input_tmp{};
    std::array<std::complex<float>, FFT_SIZE> channel_spectrum{};
    std::array<float, FFT_SIZE> fft_mag{};

    int32_t cic_i_integ1{0}, cic_i_integ2{0};
    int32_t cic_q_integ1{0}, cic_q_integ2{0};
    int32_t cic_i_comb1{0}, cic_i_comb2{0};
    int32_t cic_q_comb1{0}, cic_q_comb2{0};
    size_t decim_counter{0};

    std::array<complex16_t, 32> baseband_buf{};
    std::array<int16_t, 32> audio_buf{};

    // am dc offset
    int32_t am_dc_est{0};

    dsp::demodulate::FM fm_demod{};
    AudioOutput audio_output{};

    void init_nco_lut();
    void set_nco_frequency(int32_t freq_hz);
    void process_scanning(const buffer_c8_t& buffer);
    void process_locked(const buffer_c8_t& buffer);
    void send_status(bool locked, int32_t offset, uint16_t level, uint8_t width);

    BasebandThread baseband_thread{
        SAMPLE_RATE, this, baseband::Direction::Receive,
        true};
    RSSIThread rssi_thread{};
};

#endif /* __PROC_WIDEBAND_CATCH_H__ */