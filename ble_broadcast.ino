#include <SPI.h>
#include <EEPROM.h>
#include <lib_aci.h>
#include <aci_setup.h>
#include "OneWireTerm.h"
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <avr/sleep.h>
/**
 Put the nRF8001 setup in the RAM of the nRF8001.
 */
#include "services.h"
#include "measurments.h"

/**
 Include the services_lock.h to put the setup in the OTP memory of the nRF8001.
 This would mean that the setup cannot be changed once put in.
 However this removes the need to do the setup of the nRF8001 on every reset.
 */

#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
static services_pipe_type_mapping_t services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
#define NUMBER_OF_PIPES 0
static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif
#define TIMEOUT 33
static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;

static struct aci_state_t aci_state;
static hal_aci_evt_t aci_data;
static hal_aci_data_t aci_cmd;
//temperature
static int16_t temperature = 0;
static int16_t temperatureDHT = 0;
static int16_t temperatureDS = 0;
static uint16_t humidity = 0;
static uint32_t pressure = 0;
static OneWire term (7);
static DHT dht (2, DHT22);
static Adafruit_BMP085 bmp;
static byte measure = 0;
static bool setup_required = false;

#define ADV_INTERVAL 8000

/* Define how assert should function in the BLE library */
void
__ble_assert (const char *file, uint16_t line)
{
  Serial.print ("ERROR ");
  Serial.print (file);
  Serial.print (": ");
  Serial.print (line);
  Serial.print ("\n");
  while (1)
    ;
}

void
Timer1start ()
{

  // Setup Timer1 overflow to fire every 4000ms
  //   period [sec] = (1 / f_clock [sec]) * prescale * (count)
  //                  (1/16000000)  * 1024 * (count) = 4000 ms

  TCCR1B = 0x00;        // Disable Timer1 while we set it up

  TCNT1H = TIMEOUT;          // Approx 4000ms when prescaler is set to 1024
  TCNT1L = 0;
  TIFR1 = 0x00;        // Timer1 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK1 = 0x01;        // Timer1 INT Reg: Timer1 Overflow Interrupt Enable
  TCCR1A = 0x00;        // Timer1 Control Reg A: Wave Gen Mode normal
  TCCR1B = 0x05;        // Timer1 Control Reg B: Timer Prescaler set to 1024
}

void
Timer1stop ()
{
  TCCR1B = 0x00;
  TIMSK1 = 0x00;
}

int
clipToPWM (int value, int begin, int end, int pwmMax = 255)
{
  int32_t range = end - begin;
  int32_t scale = value - begin;

  int32_t pwm = scale * pwmMax / range;

  //clip
  if (pwm > pwmMax)
    {
      return pwmMax;
    }
  if (pwm < 0)
    {
      return 0;
    }
  return pwm;
}

/*** FUNC
 Name:       Timer1 ISR
 Function:   Handles the Timer1-overflow interrupt
 FUNC ***/
ISR(TIMER1_OVF_vect)
{
  static int count = 0;
  switch (count++ % 3)
    {
    case 0:
      measure = TEMPERATURE;
      break;
    case 1:
      measure = PRESSURE;
      break;
    case 2:
      measure = HUMIDITY;
      break;
    }

  TCNT1H = TIMEOUT;    // Approx 4000 ms - Reload
  TCNT1L = 0;
  TIFR1 = 0x00;    // timer1 int flag reg: clear timer overflow flag

}
;

void
disableACDC ()
{
  ADCSRA = ADCSRA & B01111111;
  ACSR = B10000000;
}

void
setup (void)
{
  disableACDC ();
  Timer1stop ();

  Serial.begin (115200);

  dht.begin ();
  bmp.begin ();

  //RGB

  //Wait until the serial port is available (useful only for the Leonardo)
  //As the Leonardo board is not reseted every time you open the Serial Monitor
#if defined (__AVR_ATmega32U4__)
  while(!Serial)
    {}
  delay(5000);  //5 seconds delay for enabling to see the start up comments on the serial board
#elif defined(__PIC32MX__)
  delay(1000);
#endif
  Serial.println (F("Arduino setup"));

  if (NULL != services_pipe_type_mapping)
    {
      aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
    }
  else
    {
      aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
    }
  aci_state.aci_setup_info.number_of_pipes = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs = (hal_aci_data_t*) setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs = NB_SETUP_MESSAGES;

  /*
   Tell the ACI library, the MCU to nRF8001 pin connections.
   The Active pin is optional and can be marked UNUSED
   */
  aci_state.aci_pins.board_name = BOARD_DEFAULT; //See board.h for details
  aci_state.aci_pins.reqn_pin = 9;
  aci_state.aci_pins.rdyn_pin = 3;
  aci_state.aci_pins.mosi_pin = MOSI;
  aci_state.aci_pins.miso_pin = MISO;
  aci_state.aci_pins.sck_pin = SCK;

  aci_state.aci_pins.spi_clock_divider = SPI_CLOCK_DIV8; //SPI_CLOCK_DIV8  = 2MHz SPI speed
							 //SPI_CLOCK_DIV16 = 1MHz SPI speed

  aci_state.aci_pins.reset_pin = 4;
  aci_state.aci_pins.active_pin = UNUSED;
  aci_state.aci_pins.optional_chip_sel_pin = UNUSED;

  aci_state.aci_pins.interface_is_interrupt = true;
  aci_state.aci_pins.interrupt_number = digitalPinToInterrupt(3);

  //We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
  //and initialize the data structures required to setup the nRF8001
  lib_aci_init (&aci_state, false);

}

void
measurment_loop ()
{
  bool hasCredits = aci_state.data_credit_available > 0;
  if (measure)
    {
      Serial.print (F("Credits available :"));
      Serial.println (aci_state.data_credit_available);

      if (!hasCredits)
	{
	  Serial.println (F("Not enough credits -skipping"));
	}
    }
  if (measure & PRESSURE)
    {
      Serial.println (F("Measuring Pressure"));
      pressure = bmp.readPressure () * 10;
      Serial.println (pressure);
      lib_aci_set_local_data (&aci_state, PIPE_WEATHER_STATION_PRESSURE_SET, (uint8_t*) (&pressure), sizeof(pressure));
      if (lib_aci_is_pipe_available (&aci_state, PIPE_WEATHER_STATION_PRESSURE_TX))
	{
	  if (hasCredits && !lib_aci_send_data (PIPE_WEATHER_STATION_PRESSURE_TX, (uint8_t*) (&pressure), sizeof(pressure)))
	    {
	      Serial.println (F("Data Not Send"));
	      return;
	    }
	  else
	    {
	      aci_state.data_credit_available--;
	    }
	}
      measure &= ~PRESSURE;
      return;
    }
  if (measure & HUMIDITY)
    {
      Serial.println (F("Measuring Humidity"));
      humidity = (dht.readHumidity () * 100.0);
      Serial.println (humidity, 10);
      lib_aci_set_local_data (&aci_state, PIPE_WEATHER_STATION_HUMIDITY_SET, (uint8_t*) (&humidity), sizeof(humidity));
      if (lib_aci_is_pipe_available (&aci_state, PIPE_WEATHER_STATION_HUMIDITY_TX))
	{
	  if (hasCredits && !lib_aci_send_data (PIPE_WEATHER_STATION_HUMIDITY_TX, (uint8_t*) (&humidity), sizeof(humidity)))
	    {
	      Serial.println (F("Data Not Send"));
	      return;
	    }
	  else
	    {
	      aci_state.data_credit_available--;
	    }
	}

      measure &= ~HUMIDITY;
      return;
    }
  if (measure & TEMPERATURE)
    {
      Serial.println (F("Measuring Temperature"));
      temperatureDS = readTemp (term);
      temperatureDHT = (dht.readTemperature () * 100.0);
      Serial.println (F("T DS, T DHT"));
      Serial.println (temperatureDS, 10);
      Serial.println (temperatureDHT, 10);
      temperature = temperatureDS;
      lib_aci_set_local_data (&aci_state, PIPE_WEATHER_STATION_TEMPERATURE_SET, (uint8_t*) (&temperature),
			      sizeof(temperature));
      if (lib_aci_is_pipe_available (&aci_state, PIPE_WEATHER_STATION_TEMPERATURE_TX))
	{
	  if (hasCredits
	      && !lib_aci_send_data (PIPE_WEATHER_STATION_TEMPERATURE_TX, (uint8_t*) (&temperature), sizeof(temperature)))
	    {
	      Serial.println (F("Data Not Send"));
	      return;
	    }
	  else
	    {
	      aci_state.data_credit_available--;
	    }

	}
      measure &= ~TEMPERATURE;
      return;
    }

}

void
ble_loop ()
{
  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get (&aci_state, &aci_data))
    {
      aci_evt_t* aci_evt;
      aci_evt = &aci_data.evt;
      Serial.println(aci_evt->evt_opcode);
      switch (aci_evt->evt_opcode)
	{

	/**
	 As soon as you reset the nRF8001 you will get an ACI Device Started Event
	 */
	case ACI_EVT_DEVICE_STARTED:
	  {
	    aci_state.data_credit_available = aci_evt->params.device_started.credit_available;
	    switch (aci_evt->params.device_started.device_mode)
	      {
	      case ACI_DEVICE_SETUP:
		/**
		 When the device is in the setup mode
		 */
		Serial.println (F("Evt Device Started: Setup"));
		setup_required = true;
		break;
	      case ACI_DEVICE_STANDBY:
		Serial.println (F("Evt Device Started: Standby"));
		lib_aci_connect (0, ADV_INTERVAL);
		Serial.println (F("waiting for connection started"));
		//See ACI Broadcast in the data sheet of the nRF8001
		//While broadcasting (non_connectable) interval of 100ms is the minimum possible
		//To stop the broadcasting before the timeout use the
		//lib_aci_radio_reset to soft reset the radio
		//See ACI RadioReset in the datasheet of the nRF8001
		break;
	      }
	  }
	  break; //ACI Device Started Event
	case ACI_EVT_CMD_RSP:
	  //If an ACI command response event comes with an error -> stop
	  if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
	    {
	      //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
	      //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
	      //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
	      Serial.print (F("ACI Command "));
	      Serial.println (aci_evt->params.cmd_rsp.cmd_opcode, HEX);
	      Serial.println (F("Evt Cmd respone: Error. Arduino is in an while(1); loop"));
	      while (1)
		;

	    }
	  break;
	case ACI_EVT_CONNECTED:
	  Serial.println (F("Evt Connected"));
	  Timer1start ();
	  break;
	case ACI_EVT_PIPE_STATUS:
	  Serial.println (F("Evt Pipe Status"));
	  break;
	case ACI_EVT_DISCONNECTED:
	  if (ACI_STATUS_ERROR_ADVT_TIMEOUT == aci_evt->params.disconnected.aci_status)
	    {
	      Serial.println (F("Broadcasting timed out"));
	    }
	  else
	    {
	      Serial.println (F("Evt Disconnected. Link Loss"));
	    }
	  Timer1stop ();
	  measure = 0;
	  lib_aci_connect (0, ADV_INTERVAL);
	  break;
	case ACI_EVT_DATA_RECEIVED:
	  Serial.print (F("Data received on Pipe #: 0x"));
	  Serial.println (aci_evt->params.data_received.rx_data.pipe_number, HEX);
	  Serial.print (F("Length of data received: 0x"));
	  Serial.println (aci_evt->len - 2, HEX);
	  break;
	case ACI_EVT_HW_ERROR:
	  Serial.println (F("HW error: "));
	  Serial.println (aci_evt->params.hw_error.line_num, DEC);
	  for (uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
	    {
	      Serial.write (aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
	    }
	  Serial.println ();
	  break;
	case ACI_EVT_DATA_CREDIT:
	  Serial.print (F("Data credit returned: "));
	  Serial.println (aci_evt->params.data_credit.credit);
	  aci_state.data_credit_available += aci_evt->params.data_credit.credit;
	  break;
	}
    }
  else
    {
      //Serial.println(F("No ACI Events available"));
      // No event in the ACI Event queue
      // Arduino can go to sleep now
      // Wakeup from sleep from the RDYN line
    }
  /* setup_required is set to true when the device starts up and enters setup mode.
   * It indicates that do_aci_setup() should be called. The flag should be cleared if
   * do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE.
   */
  if (setup_required)
    {
      if (SETUP_SUCCESS == do_aci_setup (&aci_state))
	{
	  setup_required = false;
	}
    }
}
void
sleepIfPossible ()
{
  if (measure == 0)
    {
      set_sleep_mode(SLEEP_MODE_IDLE);
      sleep_mode();
    }
}

void
loop ()
{
  ble_loop ();
  measurment_loop ();
  sleepIfPossible();
}

