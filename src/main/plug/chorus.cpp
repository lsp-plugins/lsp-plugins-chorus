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

#include <lsp-plug.in/common/alloc.h>
#include <lsp-plug.in/common/debug.h>
#include <lsp-plug.in/dsp/dsp.h>
#include <lsp-plug.in/dsp-units/units.h>
#include <lsp-plug.in/plug-fw/meta/func.h>
#include <lsp-plug.in/shared/debug.h>

#include <private/plugins/chorus.h>

namespace lsp
{
    namespace plugins
    {
        static constexpr size_t     BUFFER_SIZE             = 0x600;
//        static constexpr uint32_t   PHASE_MAX               = 0x80000000;
//        static constexpr uint32_t   PHASE_MASK              = PHASE_MAX - 1;
//        static constexpr float      PHASE_COEFF             = 1.0f / float(PHASE_MAX);


        //---------------------------------------------------------------------
        // Plugin factory
        static const meta::plugin_t *plugins[] =
        {
            &meta::chorus_mono,
            &meta::chorus_stereo
        };

        static plug::Module *plugin_factory(const meta::plugin_t *meta)
        {
            return new chorus(meta);
        }

        static plug::Factory factory(plugin_factory, plugins, 2);

        //---------------------------------------------------------------------
        // Implementation
        chorus::chorus(const meta::plugin_t *meta):
            Module(meta)
        {
            // Compute the number of audio channels by the number of inputs
            nChannels       = 0;
            for (const meta::port_t *p = meta->ports; p->id != NULL; ++p)
                if (meta::is_audio_in_port(p))
                    ++nChannels;

            // Initialize other parameters
            vChannels       = NULL;


            pData           = NULL;
        }

        chorus::~chorus()
        {
            do_destroy();
        }

        void chorus::init(plug::IWrapper *wrapper, plug::IPort **ports)
        {
            // Call parent class for initialization
            Module::init(wrapper, ports);

            // Estimate the number of bytes to allocate
            size_t szof_channels    = align_size(sizeof(channel_t) * nChannels, OPTIMAL_ALIGN);
            size_t buf_sz           = BUFFER_SIZE * sizeof(float);
            size_t alloc            = szof_channels + buf_sz;

            // Allocate memory-aligned data
            uint8_t *ptr            = alloc_aligned<uint8_t>(pData, alloc, OPTIMAL_ALIGN);
            if (ptr == NULL)
                return;

            // Initialize pointers to channels and temporary buffer
            vChannels               = advance_ptr_bytes<channel_t>(ptr, szof_channels);

            // TODO
        }

        void chorus::destroy()
        {
            Module::destroy();
            do_destroy();
        }

        void chorus::do_destroy()
        {
            // Destroy channels
            if (vChannels != NULL)
            {
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c    = &vChannels[i];
                    c->sBypass.destroy();
                    // TODO
                }
                vChannels   = NULL;
            }

            // Free previously allocated data chunk
            if (pData != NULL)
            {
                free_aligned(pData);
                pData       = NULL;
            }
        }

        void chorus::update_sample_rate(long sr)
        {
            plug::Module::update_sample_rate(sr);
        }

        void chorus::update_settings()
        {
        }

        bool chorus::set_position(const plug::position_t *pos)
        {
            return pos->beatsPerMinute != pWrapper->position()->beatsPerMinute;
        }

        void chorus::process(size_t samples)
        {
        }

        void chorus::ui_activated()
        {
            // TODO
        }

        bool chorus::inline_display(plug::ICanvas *cv, size_t width, size_t height)
        {
            // TODO
            return false;
        }

        void chorus::dump(dspu::IStateDumper *v) const
        {
            plug::Module::dump(v);

            // TODO
        }

    } /* namespace plugins */
} /* namespace lsp */


