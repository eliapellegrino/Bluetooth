## Firmware
This version implements PERIPHERAL role, TRANSMITTER in our scenario, and also ADVERTISER ROLE.
Once the boards is set up, it starts advertising with name @ref **DEVICE_NAME** and parameter @ref **// BLE ADVERTISING PARAMETERS**. 
Once the 2 boards are connected and ready for transmission, it waits for central to write 0x0100 to CCCD of custom carachteristic (notification on): when this occur, the board start listening for rising edge on PIN_IN (connected to ATC board output) and save a timestamp of that events w.r.t. a timer started from zero (@ref **TIMER_ACQUISITION**) each time the antenna turns on (the idea is to restart the timer and synchronize it with the **CONNECTION INTERVAL**). Every connection interval time, the packet will be sent to the CENTRAL/RECEIVER board and the process starts again until the RECEIVER doesn't turn off the notification.




# FLAG
1) enable_uart: the transmitted packet is written on uart port
2) enable_pin_out: Toggle of PIN_OUT_PRE/PIN_OUT_POST pin for delay studies

# BUTTONS
Used only for debug purpose

# LED
1)	ON: configuration completed, starts advertising
	OFF:	configuration not completed
2)	ON:		Connected to central
	OFF:	Disconnected from central
3)	ON:		Notification on 
	OFF:	Notification off

#GPIO
(1,1): PIN_IN (connected to ATC events)
(1,3): PA_PIN (toggle of pin when POWER AMPLIFIER is on)
(1,4): LNA_PIN (toggle of pin when Low Noise Amplifier is on)
(1,5): PIN_OUT__PRE (toggle of pin when the packet is put on queue for trasmission)
(1,6)_ PIN_OUT_POST (toggle of pin when the packet is correctly been sent. **ATTENTION: this events is triggered the next connection windows: this is because the notifications aren't ACKed and so the only way in which the BLE stack can know if the packet has been correctly sent is to wait for next packet and look at SN/NESN flag**)

# USEFUL DOCUMENTATION
Have a look at both documentation for s140 Softdevice (which implements BLE stack) and NRF5_SDK_17.0.0

# BUG
If enable_rtc = true, The rtc will be used instead of TIMER_ACQUISITION: the events beetween 2 windows will be delayed by 8 rtc_clocks cycles.
