---------------------------------------FIRMWARE--------------------------------------------------------
Viene implementata una finestra di acquisizione di WINDOW_LENGTH ms, e alla fine di essa viene mandato il pacchetto contenente il timestamp dell'evento (simulato con un timer TIMER_NOTIFICATION o in laboratorio tramite generatori di segnali sul PIN_IN) relativo alla finestra di acquisizione. In questo caso viene usato un timer a 8 bit @31250Hz (TIMER_ACQUISITION) (in quanto la finestra di acquisizione dura quanto il conn_interval 7.5ms). In alternativa al TIMER_ACQUISITION si può abilitare anche l'RTC (tramite il flag enable_rtc).
In sdk_config.h, MAX_PACKET_SIZE indica la dimensione del pacchetto da mandare
E' stato abilitato il data length extension e cambiato l'ATT MTU in modo da poter mandare fino a 244B ad evento di connessione
TIMER_ACQUISITON startato ad inizio connessione e stoppato in caso di connessione: ogni volta che scade viene mandato il pacchetto con i relativi timestamp e viene aumentato un contatore per tenere conto del numero di eventi di connessione (per una eventuale ricostruzione totale del pacchetto da inzio connessione)

FLAG:
	1) enable_rtc: viene usato l'RTC al posto di TIMER_ACQUISITION
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
	PIN 1.13: PIN_OUT_DEBUG: usato per studi di time of flight (da abilitare con l'apposito flag)

-------------------------------------------------------BUG---------------------------------------------
Se viene abilitato l'RTC la durata degli eventi a cavallo delle varie finestre di acquisizione differisce del valore reale - 8 colpi di clock

