/*
 * Copyright (C) 2024 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2024 Vladimir Sadovnikov <sadko4u@gmail.com>
 *
 * This file is part of lsp-plugins-chorus
 * Created on: 23 мар 2024 г.
 *
 * lsp-plugins-chorus is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * lsp-plugins-chorus is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with lsp-plugins-chorus. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef PRIVATE_PLUGINS_CHORUS_H_
#define PRIVATE_PLUGINS_CHORUS_H_

#include <lsp-plug.in/dsp-units/ctl/Bypass.h>
#include <lsp-plug.in/dsp-units/ctl/Toggle.h>
#include <lsp-plug.in/dsp-units/filters/Equalizer.h>
#include <lsp-plug.in/dsp-units/misc/lfo.h>
#include <lsp-plug.in/dsp-units/util/Delay.h>
#include <lsp-plug.in/dsp-units/util/Oversampler.h>
#include <lsp-plug.in/dsp-units/util/RingBuffer.h>
#include <lsp-plug.in/plug-fw/core/IDBuffer.h>
#include <lsp-plug.in/plug-fw/plug.h>
#include <private/meta/chorus.h>

namespace lsp
{
    namespace plugins
    {
        /**
         * Base class for the latency compensation delay
         */
        class chorus: public plug::Module
        {
            protected:
                typedef float (*mix_func_t)(float o_value, float n_value, float k);

                typedef struct voice_t
                {
                    uint32_t                nPhase;             // Phase shift relative to global LFO
                    float                   nOvlDelay;          // Overlapping delay shift in samples
                    float                   nOvlDepth;          // Overlapping depth
                    float                   fNormShift;         // Normalized shift
                    float                   fNormScale;         // Normalized scale
                    float                   fOutPhase;          // Output phase value
                    float                   fOutShift;          // Output shift value
                    uint32_t                nOutDelay;          // Output delay

                    plug::IPort            *pPhase;             // Output phase
                    plug::IPort            *pShift;             // Output delay shift
                    plug::IPort            *pDelay;             // Actual delay
                    plug::IPort            *pLfoId;             // Actual LFO used (0=none, 1=first, 2=second)
                } voice_t;

                typedef struct lfo_t
                {
                    uint32_t                nType;              // LFO type
                    uint32_t                nPeriod;            // LFO period (full, first half, second half)
                    float                   fOverlap;           // LFO overlapping
                    float                   fDelay;             // Delay
                    uint32_t                nOldDelay;          // Old delay
                    uint32_t                nDelay;             // Actual delay
                    uint32_t                nOldInitPhase;      // Old init phase
                    uint32_t                nInitPhase;         // Initial phase
                    float                   fIVoicePhase;       // Inter-voice phase
                    float                   fIChanPhase;        // Inter-channel phase
                    float                   fArg[2];            // LFO arguments

                    uint32_t                nVoices;            // Number of active voices
                    dspu::lfo::function_t   pFunc;              // LFO function
                    float                  *vLfoMesh;           // LFO mesh amplitude data
                    voice_t                *vVoices;            // Pointer to first voice stored in array

                    bool                    bSyncMesh;          // Need to synchronize mesh with UI

                    plug::IPort            *pType;              // LFO type
                    plug::IPort            *pPeriod;            // LFO period
                    plug::IPort            *pOverlap;           // Overlap
                    plug::IPort            *pDelay;             // Delay
                    plug::IPort            *pInitPhase;         // Initial phase
                    plug::IPort            *pIVoicePhase;       // Inter-voice phase
                    plug::IPort            *pIChannelPhase;     // Inter-channel phase
                    plug::IPort            *pMesh;              // Mesh data
                } lfo_t;

                typedef struct channel_t
                {
                    // DSP processing modules
                    dspu::Bypass            sBypass;            // Bypass
                    dspu::Delay             sDelay;             // Delay for dry signal
                    dspu::RingBuffer        sRing;              // Ring buffer for flanger effect processing
                    dspu::RingBuffer        sFeedback;          // Feedback delay buffer
                    dspu::Oversampler       sOversampler;       // Oversampler
                    dspu::Equalizer         sEq;                // Equalizer for processed signal

                    // Parameters
                    float                   *vIn;               // Input buffer
                    float                   *vOut;              // Output buffer
                    float                   *vBuffer;           // Processed signal

                    // Data ports ports
                    plug::IPort            *pIn;                // Input port
                    plug::IPort            *pOut;               // Output port
                    plug::IPort            *pInLevel;           // Input level meter
                    plug::IPort            *pOutLevel;          // Output level meter
                } channel_t;

            protected:
                uint32_t                nChannels;          // Number of channels
                uint32_t                nLfo;               // Number of LFO used (1 or 2)

                dspu::Toggle            sReset;             // Reset toggle
                channel_t              *vChannels;          // Delay channels
                voice_t                *vVoices;            // Voices
                lfo_t                   vLfo[2];            // Low-frequency oscillators
                float                  *vBuffer;            // Temporary buffer for processing
                float                  *vLfoPhase;          // LFO phase

                uint32_t                nRealSampleRate;    // Real sample rate after oversampling
                uint32_t                nPhase;             // Current base LFO phase
                uint32_t                nOldPhaseStep;      // Old phase increment
                uint32_t                nPhaseStep;         // Phase increment
                uint32_t                nVoices;            // Number of voices
                uint32_t                nCrossfade;         // Cross-fade threshold
                float                   fCrossfade;         // Cross-fade coefficient
                mix_func_t              pCrossfadeFunc;     // Cross-fade function
                float                   fDepth;             // Depth
                uint32_t                nOldDepth;          // Old Depth
                uint32_t                nDepth;             // Old Depth
                float                   fRate;              // Rate
                float                   fOldInGain;         // Old input gain
                float                   fInGain;            // Input gain
                float                   fOldDryGain;        // Old dry gain
                float                   fDryGain;           // Dry gain
                float                   fOldWetGain;        // Old wet gain
                float                   fWetGain;           // Wet gain
                float                   fOldFeedGain;       // Old feedback gain
                float                   fFeedGain;          // Feed-back gain
                size_t                  nOldFeedDelay;      // Old feedback delay
                size_t                  nFeedDelay;         // Feed-back delay
                bool                    bMS;                // Mid/Side mode
                bool                    bMono;              // Mono mode

                plug::IPort            *pBypass;            // Bypass switch
                plug::IPort            *pMono;              // Mono compatibility test
                plug::IPort            *pMS;                // Mid/Side switch
                plug::IPort            *pInvPhase;          // Phase inverse
                plug::IPort            *pOversampling;      // Oversampling
                plug::IPort            *pHpfMode;           // High-pass filter mode
                plug::IPort            *pHpfFreq;           // High-pass filter frequency
                plug::IPort            *pLpfMode;           // Low-pass filter mode
                plug::IPort            *pLpfFreq;           // Low-pass filter frequency

                plug::IPort            *pRate;              // Rate
                plug::IPort            *pFraction;          // Time fraction
                plug::IPort            *pTempo;             // Tempo
                plug::IPort            *pTempoSync;         // Tempo sync
                plug::IPort            *pTimeMode;          // Time computing method
                plug::IPort            *pReset;             // Reset phase to initial value

                plug::IPort            *pVoices;            // Number of voices
                plug::IPort            *pDepth;             // Depth
                plug::IPort            *pCrossfade;         // Crossfade length
                plug::IPort            *pCrossfadeType;     // Crossfade type
                plug::IPort            *pLfo2Enable;        // Enable second LFO

                plug::IPort            *pFeedOn;            // Enable feedback
                plug::IPort            *pFeedGain;          // Feedback gain
                plug::IPort            *pFeedDelay;         // Feedback delay
                plug::IPort            *pFeedPhase;         // Feedback phase

                plug::IPort            *pInGain;            // Input gain
                plug::IPort            *pDryGain;           // Dry gain
                plug::IPort            *pWetGain;           // Wet gain
                plug::IPort            *pDryWet;            // Dry/wet balance
                plug::IPort            *pOutGain;           // Output gain

                uint8_t                *pData;              // Allocated data

            protected:
                static dspu::lfo::function_t    all_lfo_functions[];
                static dspu::over_mode_t        all_oversampling_modes[];

            protected:
                static inline uint32_t  phase_to_int(float phase);
                static inline float     lerp(float o_value, float n_value, float k);
                static inline float     qlerp(float o_value, float n_value, float k);
                static inline int32_t   ilerp(int32_t o_value, int32_t n_value, float k);

            protected:
                void                    do_destroy();

            public:
                explicit chorus(const meta::plugin_t *meta);
                chorus (const chorus &) = delete;
                chorus (chorus &&) = delete;
                virtual ~chorus() override;

                chorus & operator = (const chorus &) = delete;
                chorus & operator = (chorus &&) = delete;

                virtual void        init(plug::IWrapper *wrapper, plug::IPort **ports) override;
                virtual void        destroy() override;

            public:
                virtual void        update_sample_rate(long sr) override;
                virtual void        update_settings() override;
                virtual bool        set_position(const plug::position_t *pos) override;
                virtual void        process(size_t samples) override;
                virtual void        ui_activated() override;
                virtual bool        inline_display(plug::ICanvas *cv, size_t width, size_t height) override;
                virtual void        dump(dspu::IStateDumper *v) const override;
        };

    } /* namespace plugins */
} /* namespace lsp */


#endif /* PRIVATE_PLUGINS_CHORUS_H_ */

