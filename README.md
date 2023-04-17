# Into
This project controls Mitsubishi minisplits with an ESP-32 via MQTT.o

Commands generally are sent back via MQTT. You can reach it other ways - like via telnet or HTTP - but MQTT is best for general-purpose automation. It will tell you the IP addresses on the MQTT messages so it's pretty easy.

The only thing you need to do is install the firmware, then give each one a unqiue name (via USB/serial connection) first-time. It also supports OTA firmware updates

More docs to follow - but a few things to note are:

1. Wifi credentials need to be defined as environment variables. See `CMakeLists.txt`
2. You need to add 3 files for MQTT credentails in `main`,client.key, client.crt and CA.crt. If you don't want to use SSL for your MQTT - remove them
3. You can telnet to the ESP and do some basic commands - but this is insecure. You probably want to disable this
4. Your device needs to be set up for OTA updates w/ 2MB partitions. Use OTA tool. You can ask for an automatic update, or it will try at startup and like every 15 hours thereafter. You probably want to change the address to the update server or it will grab my latest release.
5. The project version number is in `CMakeLists.txt`. *Use the defined format*. Up-rev this to indiate somethine is NEWER than what's running. If you mess this up, you may find that when you build and install new firmware, it opts to use an OTA image instead. If confused, use ESP's otatool.py to erase the OTA data. (But beware - it will try to update!)

# NOTE
Values configured for Menuconfig section for "Minisplit config" does not actually do anything yet.


# SETUP

1. for OTA - go into `menuconfig` and under `Parition Table` make sure you set `Partition Table (Factory app, two OTA definitions)`
2. In `menuconfig` under `serial flasher config` make sure flash is at least `4 MB` (or greater)
3. Do a `otatool.py erase_otadata` and make sure that all works
