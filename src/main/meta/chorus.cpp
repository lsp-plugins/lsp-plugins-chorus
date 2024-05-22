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

#include <lsp-plug.in/plug-fw/meta/ports.h>
#include <lsp-plug.in/shared/meta/developers.h>
#include <private/meta/chorus.h>

#define LSP_PLUGINS_CHORUS_VERSION_MAJOR       1
#define LSP_PLUGINS_CHORUS_VERSION_MINOR       0
#define LSP_PLUGINS_CHORUS_VERSION_MICRO       2

#define LSP_PLUGINS_CHORUS_VERSION  \
    LSP_MODULE_VERSION( \
        LSP_PLUGINS_CHORUS_VERSION_MAJOR, \
        LSP_PLUGINS_CHORUS_VERSION_MINOR, \
        LSP_PLUGINS_CHORUS_VERSION_MICRO  \
    )

namespace lsp
{
    namespace meta
    {
        #define LFO_LIST \
            { "Triangular",             "chorus.osc.triangular"             }, \
            { "Sine",                   "chorus.osc.sine"                   }, \
            { "Stepped Sine",           "chorus.osc.stepped_sine"           }, \
            { "Cubic",                  "chorus.osc.cubic"                  }, \
            { "Stepped Cubic",          "chorus.osc.stepped_cubic"          }, \
            { "Parabolic",              "chorus.osc.parabolic"              }, \
            { "Reverse Parabolic",      "chorus.osc.reverse_parabolic"      }, \
            { "Logarithmic",            "chorus.osc.logarithmic"            }, \
            { "Reverse Logarithmic",    "chorus.osc.reverse_logarithmic"    }, \
            { "Square Root",            "chorus.osc.square_root"            }, \
            { "Reverse Square Root",    "chorus.osc.reverse_square_root"    }, \
            { "Circular",               "chorus.osc.circular"               }, \
            { "Reverse Circular",       "chorus.osc.reverse_circular"       }

        static const port_item_t osc1_functions[] =
        {
            LFO_LIST,
            { NULL, NULL }
        };

        static const port_item_t osc2_functions[] =
        {
            { "Same",                   "chorus.osc.same"                   }, \
            LFO_LIST,
            { NULL, NULL }
        };

        static const port_item_t osc_periods[] =
        {
            { "Full",                   "chorus.period.full"                }, \
            { "First",                  "chorus.period.first"               }, \
            { "Last",                   "chorus.period.last"                }, \
            { NULL, NULL }
        };

        static const port_item_t crossfade_type[] =
        {
            { "Linear",                 "fade.linear"      },
            { "Const Power",            "fade.const_power" },
            { NULL, NULL }
        };

        static const port_item_t rate_type[] =
        {
            { "Rate",                   "chorus.rate.rate"                  },
            { "Tempo",                  "chorus.rate.tempo"                 },
            { "Static",                 "chorus.rate.static"                },
            { NULL, NULL }
        };

        static const port_item_t oversampling_mode[] =
        {
            { "None",                   "oversampler.none"                  },
            { "2x/16bit",               "oversampler.normal.2x16bit"        },
            { "2x/24bit",               "oversampler.normal.2x24bit"        },
            { "3x/16bit",               "oversampler.normal.3x16bit"        },
            { "3x/24bit",               "oversampler.normal.3x24bit"        },
            { "4x/16bit",               "oversampler.normal.4x16bit"        },
            { "4x/24bit",               "oversampler.normal.4x24bit"        },
            { "6x/16bit",               "oversampler.normal.6x16bit"        },
            { "6x/24bit",               "oversampler.normal.6x24bit"        },
            { "8x/16bit",               "oversampler.normal.8x16bit"        },
            { "8x/24bit",               "oversampler.normal.8x24bit"        },
            { NULL,                     NULL}
        };

        static const port_item_t voices_list[] =
        {
            { "2",                      NULL                                },
            { "3",                      NULL                                },
            { "4",                      NULL                                },
            { "5",                      NULL                                },
            { "6",                      NULL                                },
            { "7",                      NULL                                },
            { "8",                      NULL                                },
            { "9",                      NULL                                },
            { "10",                     NULL                                },
            { "11",                     NULL                                },
            { "12",                     NULL                                },
            { "13",                     NULL                                },
            { "14",                     NULL                                },
            { "15",                     NULL                                },
            { "16",                     NULL                                },
            { NULL,                     NULL}
        };

        static const port_item_t filter_slopes[] =
        {
            { "off",        "eq.slope.off"      },
            { "12 dB/oct",  "eq.slope.12dbo"    },
            { "24 dB/oct",  "eq.slope.24dbo"    },
            { "36 dB/oct",  "eq.slope.36dbo"    },
            { NULL, NULL }
        };

        //-------------------------------------------------------------------------
        // Plugin metadata

        #define CHORUS_LFO_MONO(id, label, max_voices, osc_functions, dfl_function, phase, delay) \
            COMBO("lt" id, "LFO type" label, dfl_function, osc_functions), \
            COMBO("lp" id, "LFO period" label, 0, osc_periods), \
            CONTROL("lo" id, "LFO overlap" label, U_PERCENT, chorus::OVERLAP), \
            CONTROL_DFL("ld" id, "LFO delay" label, U_MSEC, chorus::LFO_DELAY, delay), \
            CYC_CONTROL_DFL("lip" id, "Initial phase" label, U_DEG, chorus::PHASE, phase), \
            CYC_CONTROL("lvp" id, "Inter-voice phase range" label, U_DEG, chorus::VOICE_PHASE), \
            MESH("lgr" id, "LFO graph" label, (max_voices) + 1, chorus::LFO_MESH_SIZE)

        #define CHORUS_LFO_STEREO(id, label, max_voices, osc_functions, dfl_function, phase, delay) \
            COMBO("lt" id, "LFO type" label, dfl_function, osc_functions), \
            COMBO("lp" id, "LFO period" label, 0, osc_periods), \
            CONTROL("lo" id, "LFO overlap" label, U_PERCENT, chorus::OVERLAP), \
            CONTROL_DFL("ld" id, "LFO delay" label, U_MSEC, chorus::LFO_DELAY, delay), \
            CYC_CONTROL_DFL("lip" id, "Initial phase" label, U_DEG, chorus::PHASE, phase), \
            CYC_CONTROL("lvp" id, "Inter-voice phase range" label, U_DEG, chorus::VOICE_PHASE), \
            CYC_CONTROL("lcp" id, "Inter-channel phase" label, U_DEG, chorus::CHANNEL_PHASE), \
            MESH("lgr" id, "LFO graph" label, (max_voices) + 1, chorus::LFO_MESH_SIZE)

        #define VOICE_METER_MONO(id, label) \
            METER("vmp" id, "Voice meter" label " phase", U_DEG, chorus::PHASE), \
            METER("vms" id, "Voice meter" label " shift", U_NONE, chorus::SHIFT), \
            METER("vmd" id, "Voice meter" label " delay", U_MSEC, chorus::MTR_VOICE_DELAY), \
            METER_MINMAX("vml" id, "Voice meter " label " LFO", U_NONE, 0.0f, 2.0f)

        #define VOICE_METER_STEREO(id, label) \
            VOICE_METER_MONO(id "l", label " left"), \
            VOICE_METER_MONO(id "r", label " right")

        static const port_t chorus_mono_ports[] =
        {
            // Input and output audio ports
            PORTS_MONO_PLUGIN,

            // Bypass
            BYPASS,

            // Operating modes
            SWITCH("sphase", "Signal phase switch", 0.0f),
            COMBO("ovs", "Oversampling", 0, oversampling_mode),
            COMBO("hpm", "High-pass filter mode", 0, filter_slopes),
            LOG_CONTROL("hpf", "High-pass filter frequency", U_HZ, chorus::HPF),
            COMBO("lpm", "Low-pass filter mode", 0, filter_slopes),
            LOG_CONTROL("lpf", "Low-pass filter frequency", U_HZ, chorus::LPF),

            // Tempo/rate controls
            LOG_CONTROL("rate", "Rate", U_HZ, chorus::RATE),
            CONTROL("frac", "Time fraction", U_BAR, chorus::FRACTION),
            CONTROL("denom", "Time fraction denominator", U_BAR, chorus::DENOMINATOR),
            CONTROL("tempo", "Tempo", U_BPM, chorus::TEMPO),
            SWITCH("sync", "Tempo sync", 0.0f),
            COMBO("time", "Time computing method", 0, rate_type),
            TRIGGER("reset", "Reset phase to initial value"),

            // LFO settings
            COMBO("voices", "Number of voices", 2, voices_list),
            CONTROL("depth", "Depth", U_MSEC, chorus::DEPTH),
            CONTROL("xfade", "Crossfade", U_PERCENT, chorus::CROSSFADE),
            COMBO("xtype", "Crossfade Type", 1, crossfade_type),
            SWITCH("lfo2", "Enable second LFO", 0.0f),
            CHORUS_LFO_MONO("_1", " 1", chorus::VOICES_MAX, osc1_functions, 1, 0.0f, 5.0f),
            CHORUS_LFO_MONO("_2", " 2", chorus::VOICES_MAX/2, osc2_functions, 0, 180.0f, 15.0f),

            // Feedback chain
            SWITCH("fb_on", "Feedback on", 0),
            CONTROL("fgain", "Feedback gain", U_GAIN_AMP, chorus::FEEDBACK_GAIN),
            CONTROL("fdelay", "Feedback delay", U_MSEC, chorus::FEEDBACK_DELAY),
            SWITCH("fphase", "Feedback phase switch", 0.0f),

            // Loudness control
            IN_GAIN,
            DRY_GAIN(GAIN_AMP_0_DB),
            WET_GAIN(GAIN_AMP_M_6_DB),
            DRYWET(100.0f),
            OUT_GAIN,

            // Voice meters
            VOICE_METER_MONO("_1", " 1"),
            VOICE_METER_MONO("_2", " 2"),
            VOICE_METER_MONO("_3", " 3"),
            VOICE_METER_MONO("_4", " 4"),
            VOICE_METER_MONO("_5", " 5"),
            VOICE_METER_MONO("_6", " 6"),
            VOICE_METER_MONO("_7", " 7"),
            VOICE_METER_MONO("_8", " 8"),
            VOICE_METER_MONO("_9", " 9"),
            VOICE_METER_MONO("_10", " 10"),
            VOICE_METER_MONO("_11", " 11"),
            VOICE_METER_MONO("_12", " 12"),
            VOICE_METER_MONO("_13", " 13"),
            VOICE_METER_MONO("_14", " 14"),
            VOICE_METER_MONO("_15", " 15"),
            VOICE_METER_MONO("_16", " 16"),

            // Gain meters
            METER_GAIN("min", "Input gain", GAIN_AMP_P_48_DB),
            METER_GAIN("mout", "Output gain", GAIN_AMP_P_48_DB),

            PORTS_END
        };

        static const port_t chorus_stereo_ports[] =
        {
            // Input and output audio ports
            PORTS_STEREO_PLUGIN,

            // Bypass
            BYPASS,

            // Operating modes
            SWITCH("mono", "Test for mono compatibility", 0),
            SWITCH("ms", "Mid/Side mode switch", 0.0f),
            SWITCH("sphase", "Signal phase switch", 0.0f),
            COMBO("ovs", "Oversampling", 0, oversampling_mode),
            COMBO("hpm", "High-pass filter mode", 0, filter_slopes),
            LOG_CONTROL("hpf", "High-pass filter frequency", U_HZ, chorus::HPF),
            COMBO("lpm", "Low-pass filter mode", 0, filter_slopes),
            LOG_CONTROL("lpf", "Low-pass filter frequency", U_HZ, chorus::LPF),

            // Tempo/rate controls
            LOG_CONTROL("rate", "Rate", U_HZ, chorus::RATE),
            CONTROL("frac", "Time fraction", U_BAR, chorus::FRACTION),
            CONTROL("denom", "Time fraction denominator", U_BAR, chorus::DENOMINATOR),
            CONTROL("tempo", "Tempo", U_BPM, chorus::TEMPO),
            SWITCH("sync", "Tempo sync", 0.0f),
            COMBO("time", "Time computing method", 0, rate_type),
            TRIGGER("reset", "Reset phase to initial"),

            // LFO settings
            COMBO("voices", "Number of voices", 2, voices_list),
            CONTROL("depth", "Depth", U_MSEC, chorus::DEPTH),
            CONTROL("xfade", "Crossfade", U_PERCENT, chorus::CROSSFADE),
            COMBO("xtype", "Crossfade Type", 1, crossfade_type),
            SWITCH("lfo2", "Enable second LFO", 0.0f),
            CHORUS_LFO_STEREO("_1", " 1", chorus::VOICES_MAX, osc1_functions, 1, 0.0f, 5.0f),
            CHORUS_LFO_STEREO("_2", " 2", chorus::VOICES_MAX/2, osc2_functions, 0, 180.0f, 15.0f),

            // Feedback chain
            SWITCH("fb_on", "Feedback on", 0),
            CONTROL("fgain", "Feedback gain", U_GAIN_AMP, chorus::FEEDBACK_GAIN),
            CONTROL("fdelay", "Feedback delay", U_MSEC, chorus::FEEDBACK_DELAY),
            SWITCH("fphase", "Feedback phase switch", 0.0f),

            // Loudness control
            IN_GAIN,
            DRY_GAIN(GAIN_AMP_0_DB),
            WET_GAIN(GAIN_AMP_M_6_DB),
            DRYWET(100.0f),
            OUT_GAIN,

            // Voice meters
            VOICE_METER_STEREO("_1", " 1"),
            VOICE_METER_STEREO("_2", " 2"),
            VOICE_METER_STEREO("_3", " 3"),
            VOICE_METER_STEREO("_4", " 4"),
            VOICE_METER_STEREO("_5", " 5"),
            VOICE_METER_STEREO("_6", " 6"),
            VOICE_METER_STEREO("_7", " 7"),
            VOICE_METER_STEREO("_8", " 8"),
            VOICE_METER_STEREO("_9", " 9"),
            VOICE_METER_STEREO("_10", " 10"),
            VOICE_METER_STEREO("_11", " 11"),
            VOICE_METER_STEREO("_12", " 12"),
            VOICE_METER_STEREO("_13", " 13"),
            VOICE_METER_STEREO("_14", " 14"),
            VOICE_METER_STEREO("_15", " 15"),
            VOICE_METER_STEREO("_16", " 16"),

            // Gain meters
            METER_GAIN("min_l", "Input gain left",  GAIN_AMP_P_48_DB),
            METER_GAIN("mout_l", "Output gain left",  GAIN_AMP_P_48_DB),
            METER_GAIN("min_r", "Input gain right",  GAIN_AMP_P_48_DB),
            METER_GAIN("mout_r", "Output gain right", GAIN_AMP_P_48_DB),

            PORTS_END
        };

        static const int plugin_classes[]       = { C_CHORUS, -1 };
        static const int clap_features_mono[]   = { CF_AUDIO_EFFECT, CF_CHORUS, CF_MONO, -1 };
        static const int clap_features_stereo[] = { CF_AUDIO_EFFECT, CF_CHORUS, CF_STEREO, -1 };

        const meta::bundle_t chorus_bundle =
        {
            "chorus",
            "Plugin Template",
            B_EFFECTS,
            "SSQaXDN9yXI",
            "This plugin allows to simulate the chorus effect"
        };

        const plugin_t chorus_mono =
        {
            "Chorus Mono",
            "Chorus Mono",
            "Chorus Mono",
            "CH1M",
            &developers::v_sadovnikov,
            "chorus_mono",
            LSP_LV2_URI("chorus_mono"),
            LSP_LV2UI_URI("chorus_mono"),
            "ch1m",
            LSP_VST3_UID("ch1m    ch1m"),
            LSP_VST3UI_UID("ch1m    ch1m"),
            LSP_LADSPA_CHORUS_BASE + 0,
            LSP_LADSPA_URI("chorus_mono"),
            LSP_CLAP_URI("chorus_mono"),
            LSP_PLUGINS_CHORUS_VERSION,
            plugin_classes,
            clap_features_mono,
            E_DUMP_STATE | E_INLINE_DISPLAY,
            chorus_mono_ports,
            "effects/chorus.xml",
            "effects/chorus",
            mono_plugin_port_groups,
            &chorus_bundle
        };

        const plugin_t chorus_stereo =
        {
            "Chorus Stereo",
            "Chorus Stereo",
            "Chorus Stereo",
            "CH1S",
            &developers::v_sadovnikov,
            "chorus_stereo",
            LSP_LV2_URI("chorus_stereo"),
            LSP_LV2UI_URI("chorus_stereo"),
            "ch1s",
            LSP_VST3_UID("ch1s    ch1s"),
            LSP_VST3UI_UID("ch1s    ch1s"),
            LSP_LADSPA_CHORUS_BASE + 1,
            LSP_LADSPA_URI("chorus_stereo"),
            LSP_CLAP_URI("chorus_stereo"),
            LSP_PLUGINS_CHORUS_VERSION,
            plugin_classes,
            clap_features_stereo,
            E_DUMP_STATE | E_INLINE_DISPLAY,
            chorus_stereo_ports,
            "effects/chorus.xml",
            "effects/chorus",
            stereo_plugin_port_groups,
            &chorus_bundle
        };
    } /* namespace meta */
} /* namespace lsp */



