# Lora Messages
# This struct contains data for sendndig to LoRaWAN or recieved form LoRaWAN.
#

uint64 timestamp	# time since system start (microseconds)

uint8 len
uint8[58] data

# TOPICS lora_outgoing_message lora_incoming_message

