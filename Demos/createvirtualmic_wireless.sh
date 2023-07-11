#!/bin/bash
pactl load-module module-null-sink media.class=Audio/Sink sink_name=my-combined-sink channel_map=front-left,front-right,front-center
pactl load-module module-null-sink media.class=Audio/Source/Virtual sink_name=my-virtualmic channel_map=front-left,front-right,front-center
pw-metadata -n settings 0 clock.force-rate 16000
pw-link alsa_input.usb-0c76_Sandberg_126-20-00.mono-fallback:capture_MONO my-combined-sink:playback_FL
pw-link alsa_input.usb-0c76_Sandberg_126-20-00.2.mono-fallback:capture_MONO my-combined-sink:playback_FC
pw-link alsa_input.pci-0000_00_1b.0.analog-stereo:capture_FL my-combined-sink:playback_FR
pw-link my-combined-sink:monitor_FL my-virtualmic:input_FL
pw-link my-combined-sink:monitor_FR my-virtualmic:input_FR
pw-link my-combined-sink:monitor_FC my-virtualmic:input_FC
