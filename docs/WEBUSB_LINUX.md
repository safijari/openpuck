# WebUSB on Linux

By default, not every user is allowed to access raw USB devices like the OpenPuck. This can result in the configuration webpage showing "disconnected" after selecting the correct device. 

You can check the error log by going to `chrome://device-log/`. (or `chromium://device-log`, `edge://device-log`, ... depending on your browser.)

If it shows "Failed to open /dev/bus/usb/xxx/xxx: Operation not permitted", you need to follow these instructions to give the current user raw access to Valve hardware (vendor ID 28de).



## Instructions

As root, create the file `/etc/udev/rules.d/50-openpuck.rules` with the following content: 

```
SUBSYSTEM=="usb", ATTR{idVendor}=="28de", MODE="0664", GROUP="plugdev"
```

This ensures that all members of the `plugdev` group can access all Valve hardware directly. 

Then, run `sudo usermod -a -G plugdev $USER` to make sure your user is a member of that group, and then run `sudo udevadm control --reload-rules` (or reboot your machine) to enable the new rule. 

Only if you have chromium installed through Snap, you may also need to run `snap connect chromium:raw-usb` to give it direct USB access.

Then just unplug and re-plug the OpenPuck and your browser should be able to connect to the OpenPuck while it's in Steam Controller mode.
