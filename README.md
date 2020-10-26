# LoRaWAN Application Framework Sample

This is an example LoRaWAN Application Framework.  There are three keys required for
over the air activiation:
* Device EUI
* Application EUI
* Application Key

To join a LoRaWAN network, modify the following defines and keys listed in the file `se-identity.h`:

* Assign the Application EUI to `LORAWAN_JOIN_EUI` (line 79)
* Assign the Application Key to the application root key `APP_KEY` (line 105)
* Assign the Application Key to the network root key `NWK_KEY` (line 113)

Finally, adjust values in the file `Comissioning.h` to reflect the network that the device will be
joining to.

# Application Setup

All the application specific settings are located in the file application.mk.  The
instructions are listed inside the file.  There are basically four steps:

1. Define the SDK and libraries location.
2. Specify the location of the board support package.  You can choose from predefined
BSPs from within the SDK bsp directory or create your own.
3. Specify the output file names: one for debug and the other for release.
4. Include other resources such as headers, sources, libraries or paths.

# Build Instructions

There are two build configurations: one for debug and another for release.  The
configuration to be build is defined by the variable DEBUG.  When DEBUG is defined,
the debug configuration is selected and if it is left undefined, then the release
configuration is selected.  The output target will be located in either the debug or
the release directory.

## Debug Configuration
* make DEBUG=1
* make DEBUG=1 clean

## Release Configuration
* make
* make clean

