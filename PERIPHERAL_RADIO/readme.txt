---------------------------------------FIRMWARE--------------------------------------------------------
Nuova versione del PERIPHERAL. A differenza di PERIPHERAL_TIMER (che mandava il pacchetto quando un timer della durata uguale all'evento di connessione scadeva), ora viene mandato (o meglio incodato) il pacchetto quando l'antenna è accesa in caso di notifiche attivate. Questo è fattibile per il fatto che la Nordic ha messo a disposizione un interrupt tot microsecondi (decisi da noi, @ref radio_notification_handler)  prima che l'antenna effettivamete si accenda.
In sdk_config.h, MAX_PACKET_SIZE indica la dimensione del pacchetto da mandare
E' stato abilitato il data length extension e cambiato l'ATT MTU in modo da poter mandare fino a 244B ad evento di connessione
FLAG:
	1) enable_rtc: viene usato l'RTC al posto di TIMER_ACQUISITION (non pià usato)
	2) enable_uart: viene scritto in seriale il pacchetto mandato
	3) enable_pin_out: toggle del pin per il studio di time of flight (vedo miei esempi/delay) (effettuato a trasmissione completata)

BOTTONI:
	1) Start timer (TIMER_NOTIFICATION) per simulare onda quadra in ingresso
	2) Stop timer (TIME_NOTIFICATION)
LED:
	1)	ON:		configurazione della stack (Bluetooh Low Energy) completata: advertising
		OFF:	in fase di configurazione
	2)	ON:		connesso al central
		OFF:	disconesso dal central
	3)	ON:		notifiche attivate
		OFF:	notifiche disattivate
	4)	ON:		TIMER_NOTIFICATION on (solo per simulazione onda quadra)
		OFF:	TIMER_NOTIFICATION off 
GPIO:
	PIN 1.1: PIN_IN: in per il segnale ATC d'ingresso
	PIN 1.2: PIN_OUT_OSC: out per l'oscilloscopio per triggerare il segnale e poter mediare in maniera corretta: viene messo a 1 in timer_acquisition_event_handler e riportato a 0 a trasmissione avvenuta (in generale viene messo a 1 quando vogliamo triggerare il segnale)
	PIN 1.3: on quando l'antenna è accesa in trasmissione
	PIN 1.4: on quando l'antenna è accesa in ricezione

-------------------------------------------------------BUG---------------------------------------------
Se viene abilitato l'RTC la durata degli eventi a cavallo delle varie finestre di acquisizione differisce del valore reale - 8 colpi di clock

