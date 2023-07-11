#!/bin/bash
pactl load-module module-null-sink media.class=Audio/Sink sink_name=my-combined-sink channel_map=front-left,front-right
pactl load-module module-null-sink media.class=Audio/Source/Virtual sink_name=my-virtualmic channel_map=front-left,front-right
pw-link alsa_input.usb-0c76_Sandberg_126-20-00.mono-fallback:capture_MONO my-combined-sink:playback_FL
pw-link alsa_input.usb-0c76_Sandberg_126-20-00.2.mono-fallback:capture_MONO my-combined-sink:playback_FR
pw-link my-combined-sink:monitor_FL my-virtualmic:input_FL
pw-link my-combined-sink:monitor_FR my-virtualmic:input_FR
