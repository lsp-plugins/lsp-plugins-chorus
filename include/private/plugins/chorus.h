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
                typedef struct channel_t
                {
                    // DSP processing modules
                    dspu::Bypass            sBypass;            // Bypass
                    dspu::Delay             sDelay;             // Delay for dry signal
                    dspu::RingBuffer        sRing;              // Ring buffer for flanger effect processing
                    dspu::RingBuffer        sFeedback;          // Feedback delay buffer
                    dspu::Oversampler       sOversampler;       // Oversampler

                    // Parameters
                    uint32_t                nOldPhaseShift;     // Old phase shift
                    uint32_t                nPhaseShift;        // Phase shift
                    uint32_t                nLfoType[2];        // Type of LFO (x, y)
                    dspu::lfo::function_t   pXLfoFunc[2];       // LFO function (x, y)

                    float                   *vIn;               // Input buffer
                    float                   *vOut;              // Output buffer
                    float                   *vBuffer;           // Processed signal
                    float                   *vLfoMesh;          // LFO mesh amplitude data

                    // Input ports
                    plug::IPort             *pIn;               // Input port
                    plug::IPort             *pOut;              // Output port

                    // Output ports
                    plug::IPort             *pPhase;            // Current phase
                    plug::IPort             *pLfoType;          // Oscillator type
                    plug::IPort             *pLfoPeriod;        // Oscillator period
                    plug::IPort             *pLfoShift;         // LFO shift
                    plug::IPort             *pLfoMesh;          // LFO mesh
                    plug::IPort             *pInLevel;          // Input signal level
                    plug::IPort             *pOutLevel;         // Output signal level
                } channel_t;

            protected:
                dspu::Toggle        sReset;             // Reset toggle

                size_t              nChannels;          // Number of channels
                channel_t          *vChannels;          // Delay channels

                uint8_t            *pData;              // Allocated data

            protected:
                void                do_destroy();

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

