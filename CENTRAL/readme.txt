---------------------------------------FIRMWARE--------------------------------------------------------
Ogni volta che viene ricevuto un  pacchetto si cerca di ricostruirlo (da testare con oscilloscopio)TIMER_OUTPUT: alla scadenza di questo timer viene aumentato un contatore per contare gli eventuali eventi
In sdk_config.h, MAX_PACKET_SIZE indica la dimensione del pacchetto da ricevere
E' stato abilitato il data length extension e cambiato l'ATT MTU in modo da poter ricevere fino a 244B ad evento di connessione

FLAG:
	1) enable_uart: viene scritto in seriale il pacchetto mandato
	2) enable_pin_out: toggle del pin per il studio di time of flight (vedo miei esempi/delay) (effettuato a trasmissione completata)

BOTTONI:
	1) Attivazione notifiche (scrive il CCCD della caratteristica a 0100)
	2) Disattivazione notifiche
	3) Scrive un valore sulla caratteristica del peer (non più usato)
LED:
	1)	ON:		configurazione della stack (Bluetooh Low Energy) completata: scanning
		OFF:	in fase di configurazione
	2)	ON:		connesso al peripheral
		OFF:	disconesso dal peripheral
	3)	ON:		notifiche attivate
		OFF:	notifiche disattivate
GPIO:
	PIN 1.1: PIN_OUT: out per il segnale ricostruito
	PIN 1.13: PIN_OUT_DEBUG: usato per studi di time of flight (da abilitare con l'apposito flag)

-------------------------------------------------------BUG---------------------------------------------

