#-------------------------------------------------------------------
# Config.py file to allow both Arduino (by USB) and MQTT interaction
# Use this file in conjunction with the modified FlowMonitor.py file
# This file is modified version from rtlindne/RaspberryPints
# RandR+ RaspberryPints Fork
#-------------------------------------------------------------------

config = {}

# the dir everything lives in
config['pints.dir'] = '/var/www/html/'
#the serial port to the alamode
#config['flowmon.port'] = '/dev/ttyS0'
#The following line is for serial over USB
##config['flowmon.port' ] = '/dev/ttyACM0'
#The following line is for serial over bluetooth
##config['flowmon.port'] = '/dev/rfcomm0'

#logging settings for flowmon
config['flowmon.debug' ] = False#True

#logging settings for pintdispatch
config['dispatch.debug' ] = False#True

#logging settings for load cells
config['loadcell.debug' ] = False#True

#logging settings for iSpindel Devices
config['iSpindel.debug' ] = False#True

#Use fake monitoring to report flow
config['dispatch.debugMonitoring' ] = False

# MQTT Connection Settings
# config['flowmon.port'] = 'MQTT'           # Tell flowmon to use MQTT instead of serial
config['mqtt.host'] = 'localhost'           # MQTT broker address
config['mqtt.port'] = '1883'                # MQTT broker port
config['mqtt.user'] = 'RaspberryPints'      # MQTT username
config['mqtt.password'] = 'MQTT_PW'         # MQTT password (Match with PW on MCU Sketch)

config['flowmon.port'] = '/dev/ttyACM0'
