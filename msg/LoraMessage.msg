# Lora Messages
# This struct contains data for sendndig to LoRaWAN or recieved form LoRaWAN.
#

uint64 timestamp	# time since system start (microseconds)

#spreading factor, values of enum selected from LMIC - not used for incoming message
uint8 LORA_SF12 = 0
uint8 LORA_SF11 = 1
uint8 LORA_SF10 = 2
uint8 LORA_SF9 = 3
uint8 LORA_SF8 = 4
uint8 LORA_SF7 = 5
uint8 sf


uint8 len
uint8[58] data

# TOPICS lora_outgoing_message lora_incoming_message

