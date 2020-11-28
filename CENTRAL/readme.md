## Firmware

This firmware implements Central role in BLE communication. In this setup, the CENTRAL role corresponds both to INITIATOR ble role and RECEIVER role. Once the board is setted up, the boards start scanning (with parameter @ref **BLE SCANNING PARAMETER**) for advertiser events and will automatically connect with boards whose "name" is @ref **m_target_periph_name**.
Once connected, the 2 boards exchange some parameters defiing the connection(@ref **BLE CONNECTION PARAMETER**) and the central search for CUSTOM UUID (the caractheristic on which it will activate the notification). 
Moreover the 2 boards will toggle PHY link to 1Mbps to 2Mbps and they will toggle the maximum ATT packet length to @ref **NRF_SDH_BLE_GATT_MAX_MTU_SIZE** (ATTENTION: this value **CONTAINS** the 3B of ATT header but not the 4B of L2CAP header).

Once the notification for the custom UUID is activate, the CENTRAL start receiving data from peripheral: if enable (**enable_reconstruction**), the packet will be reconstruct and put in out on PIN_OUT gpio pin. The 2 boards will continue exchanging data, until the central does not turn off the notification (pressing the correspondant button)

# FLAG
1) enable_uart: the received packet is written on uart port
2) enable_pin_out: Toggle of PIN_OUT_DEBUG pin for delay studies

# BUTTONS
1) Activation of notification (write 0x0100 on CCCD of custom carachteristic)
2) Notification off
3) Write a value on custom caracheteristic (not used)

#LED
1)	ON: configuration completed, starts scanning
	OFF:	configuration not completed
2)	ON:		Connected to peer
	OFF:	Disconnected to peer
3)	ON:		Notification on 
	OFF:	Notification off

#GPIO
(1,1): PIN_OUT (used for ATC events reconstruction)
(1,3): PA_PIN (toggle of pin when POWER AMPLIFIER is on)
(1,4): LNA_PIN (toggle of pin when Low Noise Amplifier is on)
(1,5): PIN_OUT_DEBUG (toggle of pin when a packet is correctly received)

# USEFUL DOCUMENTATION
Have a look at both documentation for s140 Softdevice (which implements BLE stack) and NRF5_SDK_17.0.0

