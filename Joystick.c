#include "Joystick.h"

USB_JoystickReport_Input_t current_state;

void SetupSpi(void) {
  memset(&current_state, 0, sizeof(USB_JoystickReport_Input_t));
  current_state.LX = STICK_CENTER;
  current_state.LY = STICK_CENTER;
  current_state.RX = STICK_CENTER;
  current_state.RY = STICK_CENTER;
  current_state.HAT = HAT_CENTER;

  DDRD |= (1 << 6);           // enable LED as output
  DDRB = DDRB | (1 << DDB6);  // set MISO as output TODO maybe don't need this

  // SPI Type: Slave
  // SPI Clock Rate: 2000,000 kHz
  // SPI Clock Phase: Cycle Start
  // SPI Clock Polarity: Low
  // SPI Data Order: MSB First
  SPCR = (0 << SPIE) | (1 << SPE) | (0 << DORD) | (0 << MSTR) | (0 << CPOL) |
         (0 << CPHA) | (0 << SPR1) | (0 << SPR0);
  SPSR = (0 << SPI2X);
}

void ledOn(void) { PORTD |= (1 << 6); }
void ledOff(void) { PORTD &= ~(1 << 6); }

int targetSpiByte = 0;
char spiMessage[2];

#define KJ_SPI_TYPE 0
#define KJ_SPI_DATA 1

#define KJ_SPI_TYPE_BUTTON 0
#define KJ_SPI_TYPE_LX 1
#define KJ_SPI_TYPE_LY 2
#define KJ_SPI_TYPE_RX 3
#define KJ_SPI_TYPE_RY 4

void ReadSpi(void) {
  while (SPSR & (1 << SPIF)) {
    // If the interupt flag on the status register is set,
    // we have a byte ready to read
    spiMessage[targetSpiByte] = SPDR;
    targetSpiByte ^= 1;

    if (targetSpiByte == 0) {  // at the beginning of a new message
      ledOn();
      switch (spiMessage[0]) {
        case KJ_SPI_TYPE_BUTTON: {
          current_state.Button =
              current_state.Button ^ (1 << spiMessage[KJ_SPI_DATA]);
          break;
        }
        case KJ_SPI_TYPE_LX: {
          current_state.LX = spiMessage[KJ_SPI_DATA];
          break;
        }
        case KJ_SPI_TYPE_LY: {
          current_state.LY = spiMessage[KJ_SPI_DATA];
          break;
        }
        case KJ_SPI_TYPE_RX: {
          current_state.RX = spiMessage[KJ_SPI_DATA];
          break;
        }
        case KJ_SPI_TYPE_RY: {
          current_state.RY = spiMessage[KJ_SPI_DATA];
          break;
        }
      }
    } else {
      ledOff();
    }
  }
}

// Main entry point.
int main(void) {
  // We'll start by performing hardware and peripheral setup.
  SetupHardware();
  // Spi setup
  SetupSpi();
  // We'll then enable global interrupts for our use.
  GlobalInterruptEnable();
  // Once that's done, we'll enter an infinite loop.
  for (;;) {
    ReadSpi();
    // We need to run our task to process and deliver data for our IN and OUT
    // endpoints.
    HID_Task();
    // We also need to run the main USB management task.
    USB_USBTask();
  }
}

// Configures hardware and peripherals, such as the USB peripherals.
void SetupHardware(void) {
  // We need to disable watchdog if enabled by bootloader/fuses.
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  // We need to disable clock division before initializing the USB hardware.
  clock_prescale_set(clock_div_1);
  // We can then initialize our hardware and peripherals, including the USB
  // stack.

#ifdef ALERT_WHEN_DONE
// Both PORTD and PORTB will be used for the optional LED flashing and buzzer.
#warning LED and Buzzer functionality enabled. All pins on both PORTB and \
PORTD will toggle when printing is done.
  DDRD = 0xFF;  // Teensy uses PORTD
  PORTD = 0x0;
  // We'll just flash all pins on both ports since the UNO R3
  DDRB = 0xFF;  // uses PORTB. Micro can use either or, but both give us 2 LEDs
  PORTB = 0x0;  // The ATmega328P on the UNO will be resetting, so unplug it?
#endif
  // The USB stack should be initialized last.
  USB_Init();
}

// Fired to indicate that the device is enumerating.
void EVENT_USB_Device_Connect(void) {
  // We can indicate that we're enumerating here (via status LEDs, sound,
  // etc.).
}

// Fired to indicate that the device is no longer connected to a host.
void EVENT_USB_Device_Disconnect(void) {
  // We can indicate that our device is not ready (via status LEDs, sound,
  // etc.).
}

// Fired when the host set the current configuration of the USB device after
// enumeration.
void EVENT_USB_Device_ConfigurationChanged(void) {
  bool ConfigSuccess = true;

  // We setup the HID report endpoints.
  ConfigSuccess &= Endpoint_ConfigureEndpoint(
      JOYSTICK_OUT_EPADDR, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);
  ConfigSuccess &= Endpoint_ConfigureEndpoint(
      JOYSTICK_IN_EPADDR, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);

  // We can read ConfigSuccess to indicate a success or failure at this point.
}

// Process control requests sent to the device from the USB host.
void EVENT_USB_Device_ControlRequest(void) {
  // We can handle two control requests: a GetReport and a SetReport.

  // Not used here, it looks like we don't receive control request from the
  // Switch.
}

// Process and deliver data from IN and OUT endpoints.
void HID_Task(void) {
  // If the device isn't connected and properly configured, we can't do
  // anything here.
  if (USB_DeviceState != DEVICE_STATE_Configured) return;

  // We'll start with the OUT endpoint.
  Endpoint_SelectEndpoint(JOYSTICK_OUT_EPADDR);
  // We'll check to see if we received something on the OUT endpoint.
  if (Endpoint_IsOUTReceived()) {
    // If we did, and the packet has data, we'll react to it.
    if (Endpoint_IsReadWriteAllowed()) {
      // We'll create a place to store our data received from the host.
      USB_JoystickReport_Output_t JoystickOutputData;
      // We'll then take in that data, setting it up in our storage.
      while (Endpoint_Read_Stream_LE(&JoystickOutputData,
                                     sizeof(JoystickOutputData),
                                     NULL) != ENDPOINT_RWSTREAM_NoError)
        ;
      // At this point, we can react to this data.

      // However, since we're not doing anything with this data, we abandon
      // it.
    }
    // Regardless of whether we reacted to the data, we acknowledge an OUT
    // packet on this endpoint.
    Endpoint_ClearOUT();
  }

  // We'll then move on to the IN endpoint.
  Endpoint_SelectEndpoint(JOYSTICK_IN_EPADDR);
  // We first check to see if the host is ready to accept data.
  if (Endpoint_IsINReady()) {
    // We'll create an empty report.
    USB_JoystickReport_Input_t JoystickInputData;
    // We'll then populate this report with what we want to send to the host.
    GetNextReport(&JoystickInputData);
    // Once populated, we can output this data to the host. We do this by
    // first writing the data to the control stream.
    while (Endpoint_Write_Stream_LE(&JoystickInputData,
                                    sizeof(JoystickInputData),
                                    NULL) != ENDPOINT_RWSTREAM_NoError)
      ;
    // We then send an IN packet on this endpoint.
    Endpoint_ClearIN();
  }
}

typedef enum {
  SYNC_CONTROLLER,
  SYNC_POSITION,
  BREATHE,
  PROCESS,
  CLEANUP,
  DONE
} State_t;
State_t state = SYNC_CONTROLLER;

#define ECHOES 2
int echoes = 0;
USB_JoystickReport_Input_t last_report;

int report_count = 0;
int xpos = 0;
int ypos = 0;
int bufindex = 0;
int duration_count = 0;
int portsval = 0;

// Prepare the next report for the host.
void GetNextReport(USB_JoystickReport_Input_t* const ReportData) {
  // Prepare an empty report
  memset(ReportData, 0, sizeof(USB_JoystickReport_Input_t));
  ReportData->LX = STICK_CENTER;
  ReportData->LY = STICK_CENTER;
  ReportData->RX = STICK_CENTER;
  ReportData->RY = STICK_CENTER;
  ReportData->HAT = HAT_CENTER;

  // If we have pending echos to send, just send the last report another time
  // and decrement echoes
  if (echoes > 0) {
    memcpy(ReportData, &last_report, sizeof(USB_JoystickReport_Input_t));
    echoes--;
    return;
  }

  // States and moves management
  switch (state) {
    case SYNC_CONTROLLER:
      state = BREATHE;
      break;

    case SYNC_POSITION:  // TODO this is dead code
      bufindex = 0;

      ReportData->Button = 0;
      ReportData->LX = STICK_CENTER;
      ReportData->LY = STICK_CENTER;
      ReportData->RX = STICK_CENTER;
      ReportData->RY = STICK_CENTER;
      ReportData->HAT = HAT_CENTER;

      state = BREATHE;
      break;

    case BREATHE:  // This seems to be a dummy step to just take a break from
                   // a response?
      state = PROCESS;
      break;

    case PROCESS:
      ReportData->LY = current_state.LY;
      ReportData->LX = current_state.LX;
      ReportData->RX = current_state.RX;
      ReportData->RY = current_state.RY;
      ReportData->HAT = current_state.HAT;
      ReportData->Button = current_state.Button;
  }

  // Prepare to echo this report
  memcpy(&last_report, ReportData, sizeof(USB_JoystickReport_Input_t));
  echoes = ECHOES;
}
