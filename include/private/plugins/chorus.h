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

#include <lsp-plug.in/dsp-units/util/Delay.h>
#include <lsp-plug.in/dsp-units/ctl/Bypass.h>
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
                enum mode_t
                {
                    CD_MONO,
                    CD_STEREO,
                    CD_X2_STEREO
                };

                typedef struct channel_t
                {
                    // DSP processing modules
                    dspu::Delay         sLine;              // Delay line
                    dspu::Bypass        sBypass;            // Bypass

                    // Parameters
                    ssize_t             nDelay;             // Actual delay of the signal
                    float               fDryGain;           // Dry gain (unprocessed signal)
                    float               fWetGain;           // Wet gain (processed signal)

                    // Input ports
                    plug::IPort        *pIn;                // Input port
                    plug::IPort        *pOut;               // Output port
                    plug::IPort        *pDelay;             // Delay (in samples)
                    plug::IPort        *pDry;               // Dry control
                    plug::IPort        *pWet;               // Wet control

                    // Output ports
                    plug::IPort        *pOutDelay;          // Output delay time
                    plug::IPort        *pInLevel;           // Input signal level
                    plug::IPort        *pOutLevel;          // Output signal level
                } channel_t;

            protected:
                size_t              nChannels;          // Number of channels
                channel_t          *vChannels;          // Delay channels
                float              *vBuffer;            // Temporary buffer for audio processing

                plug::IPort        *pBypass;            // Bypass
                plug::IPort        *pGainOut;           // Output gain

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
                virtual void        process(size_t samples) override;
                virtual void        dump(dspu::IStateDumper *v) const override;
        };

    } /* namespace plugins */
} /* namespace lsp */


#endif /* PRIVATE_PLUGINS_CHORUS_H_ */

