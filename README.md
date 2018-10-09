## Suzuki SV650 ECU Decoder

This is the Arduino code (for a [PJRC Teensy 2.0](http://www.pjrc.com/store/teensy.html)) 
and necessary CAD/schematic files for creating a PCB to creating your own 
2nd Generation SV650 ECU Decoder to replace the error lights & codes normally 
displayed by the stock dash unit.

This supports the following:

 * Oil pressure warning light
 * Neutral light
 * EFI warning light
 * Low fuel light *
 * Display the water temp from the the stock water temp sensor
 * Ability to decode EFI error codes from the SV650 ECU
 * TPS adjustment indicator **

[*] Note: Untested.

[**] Note: Normally the 4 digit LED display shows the water temp. 
To show the TPS indicator or ECU error codes, you’ll need to use a 
“Suzuki diagnostic tool” (aka a paperclip) to short two wires per the shop manual.


More information about this project is available 
[on my blog](http://synfin.net/sv650ecu "Suzuki SV650 ECU Decoder") and 
[on the wiki](https://github.com/synfinatic/sv650ecu/wiki).
