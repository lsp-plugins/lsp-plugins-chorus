<?php
plugin_header();
$m      =   ($PAGE == 'chorus_mono') ? 'm' : 's';
?>

<p>
	This plugin allows to simulate the chorus effect (<?= ($m == 'm') ? 'mono' : 'stereo' ?> version).
</p>
<p>
	The effect can be reached by applying multiple short and varying delays to the the signal and mixing it with
	the original signal. Additionally, more deep effect can be reached by passing the processed output to the
	input of the plugin. 
</p>
<p>
	One or tow of available Low Frequency Oscillators (LFO) are responsible for periodical changing of delays and
	also can be adjusted to match some musical tempo. There are many different kinds of oscillation forms are available
	which makes the chorus a good toy to play, experiment with sound and enhance the stereo image of the signal.
</p>

<p><b>Common controls:</b></p>
<ul>
	<li><b>Rate/Tempo/Static</b> - the combo box that allows to select how to set the oscillation rate: using rate knob, tempo settings or fixed delays (no oscillations).</li>
	<li><b>Rate knob</b> - the knob that allows to control the oscillation rate.</li>
	<li><b>Tempo</b> - the knob that allows to control the tempo which can be used for computing oscillation rate.</li>
    <li><b>Sync</b> - enables tempo synchronization with host or DAW.</li>
    <li><b>Tap</b> - the button that allows to estimate a tempo by performing series of clicks on it.</li>
	<li><b>Crossfade</b> - the part of the period to perform the crossfade.</li>
    <li><b>Crossfade type</b> - the type of crossfade.</li>
    <ul>
    	<li><b>Linear</b> - linear crossfade</li>
    	<li><b>Constant power</b> - constant power crossfade</li>
    </ul>
    <li><b>Voices</b> - the overall number of voices<?php if ($m == 's') { ?> for each audio channel in the stereo pair<?php } ?>.
    <li><b>Depth</b> - the difference betweeen maximum delay and minimum delay for LFOs.
	<li><b>Oversampling</b> - oversampling mode.</li>
	<ul>
		<li><b>None</b> - no oversampling applied.</li>
		<li><b>2x/16bit</b> - 2x oversampling with 16-bit precision.</li>
		<li><b>2x/24bit</b> - 2x oversampling with 24-bit precision.</li>
		<li><b>3x/16bit</b> - 3x oversampling with 16-bit precision.</li>
		<li><b>3x/24bit</b> - 3x oversampling with 24-bit precision.</li>
		<li><b>4x/16bit</b> - 4x oversampling with 16-bit precision.</li>
		<li><b>4x/24bit</b> - 4x oversampling with 24-bit precision.</li>
		<li><b>6x/16bit</b> - 6x oversampling with 16-bit precision.</li>
		<li><b>6x/24bit</b> - 6x oversampling with 24-bit precision.</li>
		<li><b>8x/16bit</b> - 8x oversampling with 16-bit precision.</li>
		<li><b>8x/24bit</b> - 8x oversampling with 24-bit precision.</li>
	</ul>
	<li><b>Restart</b> - resets the current phase of the LFO to the initial value.</li>
	<li><b>Separate</b> - enables additional second LFO which allows to obtain more deep chorus effect.</li>
</ul>

<p><b>'LFO 1' and 'LFO 2' sections:</b></p>
<ul>
	<li>
		<b>Type</b> - allows to configure the type of the oscillation form.
		The oscillator can work in three modes: full period, the first half of the period (raising) and the second half of the period (falling).
		When using half period mode, then additional crossfade is performed to make the smooth transform of the sound.
	</li>
	<li><b>Overlap</b> - the overlap amount between oscillator voices.
	<li><b>Delay</b> - the minimum delay introduced by the oscillator. 
	<li><b>Phase</b> - the initial phase of the oscillator.
	<li><b>Phase range</b> - the phase range used to evenly assign the phase for each voice.
	<?php if ($m == 's') { ?>
	<li><b>Difference</b> - the phase difference between voices of the left and right channels.
	<?php } ?>.
</ul>


<p><b>'Controls' section:</b></p>
<ul>
	<?php if ($m == 's') { ?>
	<li><b>Mid/Side</b> - switches the effect to work on Mid/Side components of the signal instead of Left and Right.</li>
	<li><b>Mono</b> - allows to test the output of the plugin for mono compatibility.</li>
	<?php } ?>
	<li><b>Phase</b> - the button that allows to enable phase inversion of the processed signal which is added to original one.</li>
	<li><b>Feedback</b> - the button that switches on the feedback chain.</li>
	<li><b>Feedback</b> - the knob that controls the amount of signal being fed back.</li>
    <li><b>Feedback Phase</b> - the button that allows to invert the polarity of the feedback signal.</li>
    <li><b>Feedback Delay</b> - the additional delay which can be applied to the feedback signal.</li>
    <li><b>HPF</b> - enables additional High-Pass Filter which filters the process signal.</li>
    <li><b>HPF frequency</b> - the knob that allows to control the cut-off frequency of the High-Pass Filter.</li>
    <li><b>LPF</b> - enables additional Low-Pass Filter which filters the process signal.</li>
    <li><b>LPF frequency</b> - the knob that allows to control the cut-off frequency of the Low-Pass Filter.</li>
</ul>

<p><b>'Signal' section:</b></p>
<ul>
	<li><b>Input</b> - the amount of input signal.</li>
	<li><b>Dry</b> - the amount of dry (unprocessed) signal.</li>
	<li><b>Wet</b> - the amount of wet (processed) signal.</li>
	<li><b>Dry/Wet</b> - the balance between dry (unprocessed) signal and mixed signal formed by Dry/Wet knobs.</li>
	<li><b>Output</b> - the output volume of the plugin.</li>
</ul>
