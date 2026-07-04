# Puck cloning

The Steam Controller has two different pairing slots that each contain an 8-byte key and the serial number of the paired puck. The Steam Puck has four pairing slots that each contain an 8-byte key and the serial number of the paired controller. 

However, the serial numbers both on the Puck and on the Controller seem to only be used to display them to the user. The only thing that handles the pairing is the 8-byte key. 

Usually, when pairing, Steam generates that key randomly, then writes it to the Puck and to the Controller. 

With the script `scpair.py`, it is possible to read these pairing records both from the controller and from a Puck (Steam Puck or OpenPuck). Then, with the key known, you can pair one single controller slot to multiple Pucks, so they seamlessly switch to whichever Puck they can reach. 

`scpair.py list` lists all pairings on the attached pucks, including the controller serial and the 8-byte pairing key. 

Use `scpair.py write-controller --slot <0|1> --serial <puck-serial> --key <pairing-key>` to write the key to a controller.

Use `scpair.py write-puck --slot <0|1|2|3> --serial <controller-serial> --key <pairing-key>` to write the key to a Puck. 

For example, if you want to pair one controller to multiple different pucks, then just pair the controller to one Puck normally, then run `scpair.py list`. This will dump the pairing record from the controller pair. Note the controller serial and the key. 

Then plug in another puck and run `scpair.py write-puck --serial <controller-serial> --key <pairing-key>`. Repeat that process with as many pucks as you like. 

Now your controller will always choose any of the available pucks to connect to.
