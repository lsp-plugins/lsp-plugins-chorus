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
        static constexpr uint32_t   PHASE_MAX               = 0x80000000;
        static constexpr uint32_t   PHASE_MASK              = PHASE_MAX - 1;
        static constexpr float      PHASE_COEFF             = 1.0f / float(PHASE_MAX);

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

        dspu::lfo::function_t chorus::all_lfo_functions[] =
        {
            dspu::lfo::triangular,
            dspu::lfo::sine,
            dspu::lfo::step_sine,
            dspu::lfo::cubic,
            dspu::lfo::step_cubic,
            dspu::lfo::parabolic,
            dspu::lfo::rev_parabolic,
            dspu::lfo::logarithmic,
            dspu::lfo::rev_logarithmic,
            dspu::lfo::sqrt,
            dspu::lfo::rev_sqrt,
            dspu::lfo::circular,
            dspu::lfo::rev_circular
        };

        dspu::over_mode_t chorus::all_oversampling_modes[] =
        {
            dspu::over_mode_t::OM_NONE,
            dspu::over_mode_t::OM_LANCZOS_2X16BIT,
            dspu::over_mode_t::OM_LANCZOS_2X24BIT,
            dspu::over_mode_t::OM_LANCZOS_3X16BIT,
            dspu::over_mode_t::OM_LANCZOS_3X24BIT,
            dspu::over_mode_t::OM_LANCZOS_4X16BIT,
            dspu::over_mode_t::OM_LANCZOS_4X24BIT,
            dspu::over_mode_t::OM_LANCZOS_6X16BIT,
            dspu::over_mode_t::OM_LANCZOS_6X24BIT,
            dspu::over_mode_t::OM_LANCZOS_8X16BIT,
            dspu::over_mode_t::OM_LANCZOS_8X24BIT
        };

        chorus::chorus(const meta::plugin_t *meta):
            Module(meta)
        {
            // Compute the number of audio channels by the number of inputs
            nChannels           = 0;
            nLfo                = 0;
            for (const meta::port_t *p = meta->ports; p->id != NULL; ++p)
                if (meta::is_audio_in_port(p))
                    ++nChannels;

            // Cleanup data
            vChannels           = NULL;
            vVoices             = NULL;

            for (size_t i=0; i<2; ++i)
            {
                lfo_t *lfo          = &vLfo[i];

                lfo->nType          = -1;
                lfo->nPeriod        = -1;
                lfo->fOverlap       = 0.0f;
                lfo->fDelay         = 0.0f;
                lfo->nOldDelay      = 0;
                lfo->nDelay         = 0;
                lfo->nOldInitPhase  = 0;
                lfo->nInitPhase     = 0.0f;
                lfo->fIVoicePhase   = 0.0f;
                lfo->fIChanPhase    = 0;

                lfo->nVoices        = 0;
                lfo->bSyncMesh      = 0;
                lfo->pFunc          = NULL;
                lfo->vLfoMesh       = NULL;
                lfo->vVoices        = NULL;

                lfo->pType          = NULL;
                lfo->pPeriod        = NULL;
                lfo->pOverlap       = NULL;
                lfo->pDelay         = NULL;
                lfo->pInitPhase     = NULL;
                lfo->pIVoicePhase   = NULL;
                lfo->pIChannelPhase = NULL;
                lfo->pMesh          = NULL;
            }

            vBuffer             = NULL;
            vLfoPhase           = NULL;

            nRealSampleRate     = 0;
            nPhase              = 0;
            nOldPhaseStep       = 0;
            nPhaseStep          = 0;
            nVoices             = 0;
            nCrossfade          = 0;
            fCrossfade          = PHASE_COEFF;
            pCrossfadeFunc      = qlerp;
            fDepth              = 0.0f;
            nOldDepth           = 0;
            nDepth              = 0;
            fRate               = 0.0f;
            fOldInGain          = GAIN_AMP_0_DB;
            fInGain             = GAIN_AMP_0_DB;
            fOldDryGain         = GAIN_AMP_M_6_DB;
            fDryGain            = GAIN_AMP_M_6_DB;
            fOldWetGain         = GAIN_AMP_M_6_DB;
            fWetGain            = GAIN_AMP_M_6_DB;
            fOldFeedGain        = 0.0f;
            fFeedGain           = 0.0f;
            nOldFeedDelay       = 0;
            nFeedDelay          = 0;

            bMS                 = false;
            bMono               = false;

            // Cleanup pointers to ports
            pBypass             = NULL;
            pMono               = NULL;
            pMS                 = NULL;
            pInvPhase           = NULL;
            pOversampling       = NULL;

            pRate               = NULL;
            pFraction           = NULL;
            pTempo              = NULL;
            pTempoSync          = NULL;
            pTimeMode           = NULL;
            pReset              = NULL;

            pVoices             = NULL;
            pDepth              = NULL;
            pCrossfade          = NULL;
            pCrossfadeType      = NULL;
            pLfo2Enable         = NULL;

            pFeedOn             = NULL;
            pFeedGain           = NULL;
            pFeedDelay          = NULL;
            pFeedPhase          = NULL;

            pInGain             = NULL;
            pDryGain            = NULL;
            pWetGain            = NULL;
            pDryWet             = NULL;
            pOutGain            = NULL;

            // Initialize other parameters
            pData               = NULL;
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
            size_t max_voices       = nChannels * meta::chorus::VOICES_MAX;
            size_t szof_channels    = align_size(sizeof(channel_t) * nChannels, OPTIMAL_ALIGN);
            size_t szof_voices      = align_size(sizeof(voice_t) * max_voices, OPTIMAL_ALIGN);
            size_t buf_sz           = BUFFER_SIZE * sizeof(float);
            size_t mesh_buf_sz      = align_size(meta::chorus::LFO_MESH_SIZE * sizeof(float), OPTIMAL_ALIGN);
            size_t to_alloc         =
                szof_channels +         // vChannels
                szof_voices +           // vVoices
                buf_sz +                // vBuffer
                mesh_buf_sz +           // vLfoPhase
                2 * mesh_buf_sz +       // lfo_t::vBuffer
                nChannels * buf_sz;     // channel_t::vBuffer

            // Allocate memory-aligned data
            uint8_t *ptr            = alloc_aligned<uint8_t>(pData, to_alloc, OPTIMAL_ALIGN);
            if (ptr == NULL)
                return;
            lsp_guard_assert(uint8_t *save   = ptr);

            // Initialize pointers to channels and temporary buffer
            vChannels               = advance_ptr_bytes<channel_t>(ptr, szof_channels);
            vVoices                 = advance_ptr_bytes<voice_t>(ptr, szof_voices);
            vBuffer                 = advance_ptr_bytes<float>(ptr, buf_sz);
            vLfoPhase               = advance_ptr_bytes<float>(ptr, mesh_buf_sz);

            // Initialize channels
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                c->sBypass.construct();
                c->sDelay.construct();
                c->sRing.construct();
                c->sFeedback.construct();
                c->sOversampler.construct();
                c->sOversampler.init();

                c->vIn                  = NULL;
                c->vOut                 = NULL;
                c->vBuffer              = advance_ptr_bytes<float>(ptr, buf_sz);

                c->pIn                  = NULL;
                c->pOut                 = NULL;
                c->pInLevel             = NULL;
                c->pOutLevel            = NULL;
            }

            // Initialize LFO
            for (size_t i=0; i<2; ++i)
            {
                lfo_t *lfo              = &vLfo[i];
                lfo->vLfoMesh           = advance_ptr_bytes<float>(ptr, mesh_buf_sz);
            }

            // Initialize voices
            for (size_t i=0; i< max_voices; ++i)
            {
                voice_t *v              = &vVoices[i];

                v->nPhase               = 0;
                v->fNormShift           = 0;
                v->fNormScale           = 0;

                v->pPhase               = NULL;
                v->pShift               = NULL;
                v->pDelay               = NULL;
                v->pLfoId               = NULL;
            }
            lsp_assert(ptr <= &save[to_alloc]);

            // Bind ports
            size_t port_id      = 0;

            // Bind I/O ports
            lsp_trace("Binding I/O ports");
            for (size_t i=0; i<nChannels; ++i)
                BIND_PORT(vChannels[i].pIn);
            for (size_t i=0; i<nChannels; ++i)
                BIND_PORT(vChannels[i].pOut);

            // Bind bypass
            lsp_trace("Binding bypass ports");
            BIND_PORT(pBypass);

            // Operating modes
            lsp_trace("Binding operating modes");
            if (nChannels > 1)
            {
                BIND_PORT(pMono);              // Mono compatibility test
                BIND_PORT(pMS);                // Mid/Side switch
            }
            BIND_PORT(pInvPhase);
            BIND_PORT(pOversampling);

            // Tempo/rate controls
            lsp_trace("Binding tempo/rate controls");
            BIND_PORT(pRate);
            BIND_PORT(pFraction);
            SKIP_PORT("Denominator");   // Skip denominator
            BIND_PORT(pTempo);
            BIND_PORT(pTempoSync);
            BIND_PORT(pTimeMode);
            BIND_PORT(pReset);

            // LFO settings
            lsp_trace("Binding LFO settings");
            BIND_PORT(pVoices);
            BIND_PORT(pDepth);
            BIND_PORT(pCrossfade);
            BIND_PORT(pCrossfadeType);
            BIND_PORT(pLfo2Enable);
            for (size_t i=0; i<2; ++i)
            {
                lfo_t *lfo          = &vLfo[i];

                BIND_PORT(lfo->pType);
                BIND_PORT(lfo->pPeriod);
                BIND_PORT(lfo->pOverlap);
                BIND_PORT(lfo->pDelay);
                BIND_PORT(lfo->pInitPhase);
                BIND_PORT(lfo->pIVoicePhase);
                if (nChannels > 1)
                    BIND_PORT(lfo->pIChannelPhase);
                BIND_PORT(lfo->pMesh);
            }

            // Feedback settings
            lsp_trace("Binding feedback settings");
            BIND_PORT(pFeedOn);
            BIND_PORT(pFeedGain);
            BIND_PORT(pFeedDelay);
            BIND_PORT(pFeedPhase);

            // Loudness control settings
            lsp_trace("Binding loudness control settings");
            BIND_PORT(pInGain);
            BIND_PORT(pDryGain);
            BIND_PORT(pWetGain);
            BIND_PORT(pDryWet);
            BIND_PORT(pOutGain);

            // Bind voice meters
            lsp_trace("Binding voice meters");
            for (size_t i=0; i<max_voices; ++i)
            {
                voice_t *v          = &vVoices[i];

                BIND_PORT(v->pPhase);
                BIND_PORT(v->pShift);
                BIND_PORT(v->pDelay);
                BIND_PORT(v->pLfoId);
            }

            // Bind signal meters
            lsp_trace("Binding signal meters");
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = &vChannels[i];

                BIND_PORT(c->pInLevel);
                BIND_PORT(c->pOutLevel);
            }

            // Fill LFO phase data
            float phase_k           = 360.0f / (meta::chorus::LFO_MESH_SIZE - 1);
            for (size_t i=0; i<meta::chorus::LFO_MESH_SIZE; ++i)
                vLfoPhase[i]            = i * phase_k;
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
                    c->sDelay.destroy();
                    c->sRing.destroy();
                    c->sFeedback.destroy();
                    c->sOversampler.destroy();
                }
                vChannels   = NULL;
            }

            vVoices     = NULL;

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

            // Update sample rate for the bypass processors
            size_t max_delay = dspu::millis_to_samples(sr, meta::chorus::LFO_DELAY_MAX + meta::chorus::DEPTH_MAX);
            size_t max_feedback = dspu::millis_to_samples(sr, meta::chorus::LFO_DELAY_MAX + meta::chorus::FEEDBACK_DELAY_MAX);

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = &vChannels[i];
                c->sBypass.init(sr);
                c->sDelay.init(BUFFER_SIZE*2);
                c->sRing.init(max_delay * meta::chorus::OVERSAMPLING_MAX + BUFFER_SIZE*2);
                c->sFeedback.init(max_feedback * meta::chorus::OVERSAMPLING_MAX + BUFFER_SIZE*2);
                c->sOversampler.set_sample_rate(sr);
            }
        }

        inline uint32_t chorus::phase_to_int(float phase)
        {
            if (phase >= 360.0f)
                phase      -= 360.0f;
            return float(PHASE_MAX) * (phase / 360.0f);
        }

        inline float chorus::lerp(float o_value, float n_value, float k)
        {
            return o_value + (n_value - o_value) * k;
        }

        inline float chorus::qlerp(float o_value, float n_value, float k)
        {
            return o_value * sqrtf(1.0f - k) + n_value * sqrtf(k);
        }

        inline int32_t chorus::ilerp(int32_t o_value, int32_t n_value, float k)
        {
            return o_value + (n_value - o_value) * k;
        }

        void chorus::update_settings()
        {
            // Update oversampling settings
            dspu::over_mode_t omode = all_oversampling_modes[size_t(pOversampling->value())];
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                if (c->sOversampler.mode() != omode)
                {
                    c->sOversampler.set_mode(omode);
                    c->sOversampler.set_filtering(false);
                    c->sOversampler.update_settings();

                    c->sDelay.set_delay(c->sOversampler.latency());
                    c->sDelay.clear();
                    c->sRing.clear();
                    c->sFeedback.clear();
                }
            }
            size_t oversampling     = vChannels[0].sOversampler.get_oversampling();
            size_t latency          = vChannels[0].sOversampler.latency();
            size_t srate            = fSampleRate * oversampling;
            bool srate_changed      = nRealSampleRate != srate;
            nRealSampleRate         = srate;

            // Update state of the 'reset' trigger
            sReset.submit(pReset->value());

            // Pre-compute several attributes
            bool update_voices      = false;
            const float in_gain     = pInGain->value();
            const float out_gain    = pOutGain->value();
            const bool bypass       = pBypass->value() >= 0.5f;
            bool fb_on              = pFeedOn->value() >= 0.5f;
            float feed_gain         = (fb_on) ? pFeedGain->value() : 0.0f;
            const bool mid_side     = (pMS != NULL) ? pMS->value() >= 0.5f : false;
            float crossfade         = pCrossfade->value() * 0.01f;

            // Compute LFO rate
            float rate              = pRate->value();
            if (pTimeMode->value() >= 1.0f)
            {
                // Use tempo instead of rate
                float tempo             = (pTempoSync->value() >= 0.5f) ? pWrapper->position()->beatsPerMinute : pTempo->value();
                rate                    =
                    lsp_limit(
                        dspu::time_signature_to_frequency(pFraction->value(), tempo),
                        meta::chorus::RATE_MIN,
                        meta::chorus::RATE_MAX);
            }
            rate                   /= nRealSampleRate;
            if (fRate != rate)
                update_voices           = true;

            // Update common parameters
            const float dry_gain    = pDryGain->value();
            const float wet_gain    = (pInvPhase->value() < 0.5f) ? pWetGain->value() : -pWetGain->value();
            const float drywet      = pDryWet->value() * 0.01f;

            nOldPhaseStep           = nPhaseStep;
            nPhaseStep              = float(PHASE_MAX) * rate;
            fOldInGain              = fInGain;
            fOldDryGain             = fDryGain;
            fOldWetGain             = fWetGain;
            fInGain                 = in_gain;
            fDryGain                = (dry_gain * drywet + 1.0f - drywet) * out_gain;
            fWetGain                = wet_gain * drywet * out_gain;
            nOldFeedDelay           = nFeedDelay;
            nFeedDelay              = dspu::millis_to_samples(srate, pFeedDelay->value());
            fOldFeedGain            = fFeedGain;
            fFeedGain               = (pFeedPhase->value() >= 0.5f) ? -feed_gain : feed_gain;
            nCrossfade              = float(PHASE_MAX) * crossfade * 2;
            fCrossfade              = PHASE_COEFF * (1.0f - crossfade);
            pCrossfadeFunc          = (int(pCrossfadeType->value()) == 0) ? lerp : qlerp;

            // LFO setup
            const size_t n_lfo      = (pLfo2Enable->value() >= 0.5f) ? 2 : 1;
            const size_t voices     = pVoices->value() + 1;
            const float depth       = pDepth->value();
            if ((depth != fDepth) || (srate_changed))
            {
                update_voices           = true;
                fDepth                  = depth;
                nOldDepth               = nDepth;
                nDepth                  = dspu::millis_to_samples(nRealSampleRate, depth);
            }

            // Re-allocate voices if number of LFOs has changed
            if ((n_lfo != nLfo) || (voices != nVoices))
            {
                if (n_lfo == 2)
                {
                    lfo_t *lfo1             = &vLfo[0];
                    lfo_t *lfo2             = &vLfo[1];
                    lfo1->vVoices           = &vVoices[0];
                    lfo2->vVoices           = &vVoices[(nChannels * meta::chorus::VOICES_MAX) / 2];
                    lfo2->nVoices           = voices/2;
                    lfo1->nVoices           = voices - lfo2->nVoices;
                    lfo1->bSyncMesh         = true;
                    lfo2->bSyncMesh         = true;
                }
                else
                {
                    lfo_t *lfo              = &vLfo[0];
                    lfo->vVoices            = vVoices;
                    lfo->nVoices            = voices;
                    lfo->bSyncMesh          = true;
                }

                nLfo                    = n_lfo;
                nVoices                 = voices;
                update_voices           = true;
            }

            // Check that LFO parameters have changed
            for (size_t i=0; i<n_lfo; ++i)
            {
                lfo_t *lfo              = &vLfo[i];
                const float iv_phase    = lfo->pIVoicePhase->value();
                const float ichan_phase = (lfo->pIChannelPhase != NULL) ? lfo->pIChannelPhase->value() : 0.0f;
                const float overlap     = lfo->pOverlap->value() * 0.01f;
                const float delay       = lfo->pDelay->value();

                if (lfo->fOverlap != overlap)
                {
                    lfo->fOverlap           = overlap;
                    update_voices           = true;
                    lfo->bSyncMesh          = true;
                }

                if ((lfo->fIVoicePhase != iv_phase) ||
                    (lfo->fIChanPhase != ichan_phase) ||
                    (lfo->fDelay != delay) ||
                    (srate_changed))
                {
                    lfo->fIVoicePhase       = iv_phase;
                    lfo->fIChanPhase        = ichan_phase;
                    lfo->nOldDelay          = lfo->nDelay;
                    lfo->nDelay             = dspu::millis_to_samples(nRealSampleRate, delay);
                    lfo->fDelay             = delay;
                    update_voices           = true;
                }
            }

            // Update voices if required
            if (update_voices)
            {
                for (size_t i=0; i<nLfo; ++i)
                {
                    lfo_t *lfo              = &vLfo[i];
                    const float p_step      = lfo->fIVoicePhase / float(lfo->nVoices);
                    const float ovl_width   = lerp(1.0f / lfo->nVoices, 1.0f, lfo->fOverlap);
                    const float ovl_step    = (lfo->nVoices > 1) ? (1.0f - ovl_width) / (lfo->nVoices - 1) : 0.0f;

                    for (size_t j=0; j<lfo->nVoices; ++j)
                    {
                        const float v_shift     = j * ovl_step;

                        for (size_t k=0; k<nChannels; ++k)
                        {
                            voice_t *v              = &lfo->vVoices[j*nChannels + k];

                            v->nPhase               = phase_to_int(lfo->fIChanPhase*k + p_step*j);
                            v->fNormShift           = v_shift;
                            v->fNormScale           = ovl_width;
                        }
                    }
                }
            }

            // Update LFO settings
            for (size_t i=0; i<2; ++i)
            {
                lfo_t *lfo              = &vLfo[i];

                // Update LFO preferences
                size_t lfo_type         = size_t(lfo->pType->value());
                size_t lfo_period       = size_t(lfo->pPeriod->value());
                if (i > 0)
                {
                    if (lfo_type == 0)
                    {
                        lfo_type                = vLfo[0].nType;
                        lfo_period              = vLfo[0].nPeriod;
                    }
                    else
                        --lfo_type;
                }

                // The form of the LFO has changed?
                if ((lfo_type != lfo->nType) || (lfo_period != lfo->nPeriod))
                {
                    lfo->nType              = lfo_type;
                    lfo->nPeriod            = lfo_period;
                    lfo->pFunc              = all_lfo_functions[lfo_type];
                    lfo->bSyncMesh          = true;

                    // Select the function coefficients
                    switch (lfo_period)
                    {
                        case meta::chorus::OSC_FIRST:
                            lfo->fArg[0]        = 0.5f;
                            lfo->fArg[1]        = 0.0f;
                            break;
                        case meta::chorus::OSC_LAST:
                            lfo->fArg[0]        = 0.5f;
                            lfo->fArg[1]        = 0.5f;
                            break;
                        case meta::chorus::OSC_FULL:
                        default:
                            lfo->fArg[0]        = 1.0f;
                            lfo->fArg[1]        = 0.0f;
                            break;
                    }

                    // Update LFO image
                    float k                 = lfo->fArg[0] / (meta::chorus::LFO_MESH_SIZE - 1);
                    for (size_t j=0; j<meta::chorus::LFO_MESH_SIZE; ++j)
                        lfo->vLfoMesh[j]        = lfo->pFunc(j * k + lfo->fArg[1]);
                }

                // Store the parameters for each processor
                lfo->nInitPhase         = phase_to_int(lfo->pInitPhase->value());
            }

            // Update channels
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = &vChannels[i];

                // For Mid/Side switch change, clear the buffers
                if (mid_side != bMS)
                {
                    c->sRing.clear();
                    c->sFeedback.clear();
                }

                // Update bypass
                c->sBypass.set_bypass(bypass);
            }

            bMS                     = mid_side;
            bMono                   = (pMono != NULL) ? pMono->value() >= 0.5f : false;

            // Update latency
            set_latency(latency);
        }

        bool chorus::set_position(const plug::position_t *pos)
        {
            return pos->beatsPerMinute != pWrapper->position()->beatsPerMinute;
        }

        void chorus::process(size_t samples)
        {
            // Reset phase if phase request is pending
            if (sReset.pending())
            {
                nPhase                  = 0;
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c            = &vChannels[i];
                    c->sRing.clear();
                    c->sFeedback.clear();
                }
                sReset.commit();
            }

            // Perform the routing
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];
                c->vIn                  = c->pIn->buffer<float>();
                c->vOut                 = c->pOut->buffer<float>();

                // Measure the input level
                c->pInLevel->set_value(dsp::abs_max(c->vIn, samples) * fInGain);
            }

            size_t oversampling     = vChannels[0].sOversampler.get_oversampling();
            size_t max_buf_samples  = BUFFER_SIZE / oversampling;

            for (size_t offset=0; offset<samples; )
            {
                uint32_t to_do          = lsp_min(samples - offset, max_buf_samples);
                uint32_t phase          = nPhase;

                // Convert to Mid/Side if needed
                if ((bMS) && (nChannels > 1))
                {
                    dsp::lr_to_ms(
                        vChannels[0].vBuffer,
                        vChannels[1].vBuffer,
                        vChannels[0].vIn,
                        vChannels[1].vIn,
                        to_do
                    );

                    dsp::lramp2(vChannels[0].vBuffer, vChannels[0].vBuffer, fOldInGain, fInGain, to_do);
                    dsp::lramp2(vChannels[1].vBuffer, vChannels[1].vBuffer, fOldInGain, fInGain, to_do);
                }
                else
                {
                    dsp::lramp2(vChannels[0].vBuffer, vChannels[0].vIn, fOldInGain, fInGain, to_do);
                    if (nChannels > 1)
                        dsp::lramp2(vChannels[1].vBuffer, vChannels[1].vIn, fOldInGain, fInGain, to_do);
                }

                // Do audio processing
                for (size_t nc=0; nc<nChannels; ++nc)
                {
                    channel_t *c            = &vChannels[nc];
                    phase                   = nPhase;

                    // Apply oversampling and delay stored into temporary buffer
                    uint32_t up_to_do       = to_do * oversampling;
                    float k_up_to_do        = 1.0f / float(up_to_do);
                    c->sOversampler.upsample(vBuffer, c->vBuffer, to_do);

                    // Process each sample
                    for (size_t i=0; i<up_to_do; ++i)
                    {
                        const float c_sample    = vBuffer[i];
                        const float s           = i * k_up_to_do;
                        float p_sample          = 0.0f;

                        c->sRing.append(c_sample);

                        // Apply changes from each LFO
                        for (size_t j=0; j<nLfo; ++j)
                        {
                            lfo_t *lfo              = &vLfo[j];

                            const float lfo_delay   = ilerp(lfo->nOldDelay, lfo->nDelay, s);
                            const float lfo_depth   = ilerp(nOldDepth, nDepth, s);

                            // Process each voice that matches the channel for current LFO
                            for (size_t k=0; k<lfo->nVoices; ++k)
                            {
                                voice_t *v              = &lfo->vVoices[k*nChannels + nc];
                                uint32_t i_phase        = (phase + ilerp(lfo->nOldInitPhase + v->nPhase, lfo->nInitPhase + v->nPhase, s)) & PHASE_MASK;
                                float o_phase           = i_phase * fCrossfade;
                                float c_phase           = o_phase * lfo->fArg[0] + lfo->fArg[1];
                                float c_func            = v->fNormScale * lfo->pFunc(c_phase) + v->fNormShift;
                                size_t c_shift          = lfo_delay + lfo_depth * c_func;
                                float c_dsample         = c->sRing.get(c_shift);

                                v->fOutPhase            = o_phase;
                                v->fOutShift            = c_func;
                                v->nOutDelay            = c_shift;

                                // Perform cross-fade if required
                                if (i_phase < nCrossfade)
                                {
                                    float mix               = float(i_phase) / float(nCrossfade);
                                    i_phase                 = i_phase + PHASE_MAX;
                                    c_phase                 = i_phase * fCrossfade * lfo->fArg[0] + lfo->fArg[1];
                                    c_func                  = v->fNormScale * lfo->pFunc(c_phase) + v->fNormShift;
                                    c_shift                 = lfo_delay + lfo_depth * c_func;
                                    c_dsample               = pCrossfadeFunc(c->sRing.get(c_shift), c_dsample, mix);
                                }

                                // Compute the sample
                                p_sample               += c_dsample;
                            }
                        }

                        // Process feedback
                        ssize_t c_feed_delay    = ilerp(vLfo[0].nOldDelay, vLfo[0].nDelay, s);
                        if (nLfo > 1)
                            c_feed_delay            = lsp_min(c_feed_delay, ilerp(vLfo[1].nOldDelay, vLfo[1].nDelay, s));
                        size_t c_fbshift        = c_feed_delay + ilerp(nOldFeedDelay, nFeedDelay, s) - 1;
                        float fb_sample         = c->sFeedback.get(c_fbshift);
                        p_sample               += fb_sample * lerp(fOldFeedGain, fFeedGain, s);

                        c->sFeedback.append(p_sample);

                        // Update buffer sample
                        vBuffer[i]              = p_sample;

                        // Update the phase
                        phase                   = (phase + ilerp(nOldPhaseStep, nPhaseStep, s)) & PHASE_MASK;
                    }

                    // Perform downsampling back into channel's buffer
                    c->sOversampler.downsample(c->vBuffer, vBuffer, to_do);
                }

                // Update LFO parameters
                for (size_t j=0; j<nLfo; ++j)
                {
                    lfo_t *lfo              = &vLfo[j];
                    lfo->nOldDelay          = lfo->nDelay;
                    lfo->nOldInitPhase      = lfo->nInitPhase;
                }

                // Convert back to left-right if needed
                if ((bMS) && (nChannels > 1))
                {
                    dsp::ms_to_lr(
                        vChannels[0].vBuffer,
                        vChannels[1].vBuffer,
                        vChannels[0].vBuffer,
                        vChannels[1].vBuffer,
                        to_do
                    );
                }

                // Apply Dry/Wet and measure output level
                for (size_t nc=0; nc<nChannels; ++nc)
                {
                    channel_t *c            = &vChannels[nc];

                    // Apply latency compensation
                    c->sDelay.process(vBuffer, c->vIn, to_do);

                    // Mix dry/wet
                    dsp::lramp1(c->vBuffer, fOldWetGain, fWetGain, to_do);
                    dsp::lramp_add2(c->vBuffer, vBuffer, fOldDryGain, fDryGain, to_do);
                    c->pOutLevel->set_value(dsp::abs_max(c->vBuffer, to_do));
                }

                // Apply mono compatibility switch
                if ((nChannels > 1) && (bMono))
                {
                    dsp::lr_to_mid(vChannels[0].vBuffer, vChannels[0].vBuffer, vChannels[1].vBuffer, to_do);
                    dsp::copy(vChannels[1].vBuffer, vChannels[0].vBuffer, to_do);
                }

                // Apply bypass and update buffer pointers
                for (size_t nc=0; nc<nChannels; ++nc)
                {
                    channel_t *c            = &vChannels[nc];

                    // Apply bypass
                    c->sBypass.process(c->vOut, c->vIn, c->vBuffer, to_do);

                    // Move pointers
                    c->vIn                 += to_do;
                    c->vOut                += to_do;
                }

                // Commit values
                nPhase              = phase;
                nOldPhaseStep       = nPhaseStep;
                nOldDepth           = nDepth;
                fOldFeedGain        = fFeedGain;
                nOldFeedDelay       = nFeedDelay;
                fOldInGain          = fInGain;
                fOldDryGain         = fDryGain;
                fOldWetGain         = fWetGain;

                offset             += to_do;
            }

            // Output information about phases for each voice
            const size_t max_v      = (nLfo > 1) ? (nChannels * meta::chorus::VOICES_MAX)/2 : nChannels * meta::chorus::VOICES_MAX;

            // Apply changes from each LFO
            for (size_t i=0; i<nLfo; ++i)
            {
                lfo_t *lfo              = &vLfo[i];
                voice_t *v              = &lfo->vVoices[0];
                const voice_t *end      = &lfo->vVoices[max_v];

                // Process each voice that matches the channel for current LFO
                for (size_t j=0, n=lfo->nVoices*nChannels; j<n; ++j, ++v)
                {
                    v->pPhase->set_value(v->fOutPhase * 360.0f);
                    v->pShift->set_value(v->fOutShift);
                    v->pDelay->set_value(dspu::samples_to_millis(nRealSampleRate, v->nOutDelay));
                    v->pLfoId->set_value(i + 1);

//                    lsp_trace("lfo %d voice %d = {sc=%f, sh=%f, p=%f, s=%f, d=%f, id=%f}",
//                        int(i), int(j),
//                        v->fNormScale,
//                        v->fNormShift,
//                        v->pPhase->value(),
//                        v->pShift->value(),
//                        v->pDelay->value(),
//                        v->pLfoId->value());
                }

                // Clear other meters
                for ( ; v < end; ++v)
                {
                    v->pPhase->set_value(0.0f);
                    v->pShift->set_value(0.0f);
                    v->pDelay->set_value(0.0f);
                    v->pLfoId->set_value(0.0f);
                }
            }

            // Output information about phase for each channel
            for (size_t i=0; i<2; ++i)
            {
                lfo_t *lfo              = &vLfo[i];
                if (!lfo->bSyncMesh)
                    continue;

                // Need to synchronize LFO mesh?
                plug::mesh_t *mesh      = (lfo->pMesh != NULL) ? lfo->pMesh->buffer<plug::mesh_t>() : NULL;
                if ((mesh != NULL) && (mesh->isEmpty()))
                {
                    if (i < nLfo)
                    {
                        dsp::copy(mesh->pvData[0], vLfoPhase, meta::chorus::LFO_MESH_SIZE);

                        for (size_t j=0; j<lfo->nVoices; ++j)
                        {
                            const voice_t *v    = &lfo->vVoices[j*nChannels];
                            dsp::mul_k3(mesh->pvData[j+1], lfo->vLfoMesh, v->fNormScale, meta::chorus::LFO_MESH_SIZE);
                            dsp::add_k2(mesh->pvData[j+1], v->fNormShift, meta::chorus::LFO_MESH_SIZE);
                        }

                        mesh->data(lfo->nVoices + 1, meta::chorus::LFO_MESH_SIZE);
                    }
                    else
                        mesh->data(0, 0);

                    lfo->bSyncMesh      = false;
                }
            }

            // Request the inline display for redraw
            if (pWrapper != NULL)
                pWrapper->query_display_draw();
        }

        void chorus::ui_activated()
        {
            for (size_t i=0; i<2; ++i)
            {
                lfo_t *lfo              = &vLfo[i];
                lfo->bSyncMesh          = true;
            }
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


