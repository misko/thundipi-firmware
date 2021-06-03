#!/bin/bash
python=/usr/local/bin/python3.6
bgbuild="/Applications/Simplicity Studio.app/Contents/Eclipse/developer/sdks/gecko_sdk_suite/v3.1/protocol/bluetooth/bin/gatt/bgbuild.py"
$python "$bgbuild" ./config/btconf/gatt_configuration.btconf ./config/btconf/*.xml
mv gatt_db.[ch] ./autogen/
