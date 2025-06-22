/* USB Host Shield 2.0 board quality control routine */
/* To see the output set your terminal speed to 115200 */
/* for GPIO test to pass you need to connect GPIN0 to GPOUT7, GPIN1 to GPOUT6, etc. */
/* otherwise press any key after getting GPIO error to complete the test */
/**/
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <../../../../hardware/pic32/libraries/SPI/SPI.h> // Hack to use the SPI library
#endif
#include <SPI.h> // Hack to use the SPI library

/* variables */
uint8_t rcode;
uint8_t usbstate;
uint8_t laststate;
//uint8_t buf[sizeof(USB_DEVICE_DESCRIPTOR)];
USB_DEVICE_DESCRIPTOR buf;

/* objects */
USB Usb;
//TODO: add GPIO_HEX
//USBHub hub(&Usb);

void setup() {
        xil_printf("Circuits At Home 2011\n");
        xil_printf("USB Host Shield Quality Control Routine\n");
        /* SPI quick test - check revision register */
        xil_printf("Reading REVISION register...\n");
        Usb.Init(); // Initializes SPI, we don't care about the return value here
        {
                uint8_t tmpbyte = Usb.regRd(rREVISION);
                switch(tmpbyte) {
                        case( 0x01): //rev.01
                                xil_printf("Die revision: 01\n");
                                // E_Notify(PSTR("01"), 0x80);
                                break;
                        case( 0x12): //rev.02
                                xil_printf("Die revision: 02\n");
                                // E_Notify(PSTR("02"), 0x80);
                                break;
                        case( 0x13): //rev.03
                                xil_printf("Die revision: 03\n");                        
                                // E_Notify(PSTR("03"), 0x80);
                                break;
                        default:
                                xil_printf("invalid. Value returned: %x \n", tmpbyte);
                                halt55();
                                break;
                }//switch( tmpbyte...
        }//check revision register
        /* SPI long test */
        {
                xil_printf("SPI long test. Transfers 1MB of data. Each dot is 64K\n");
                uint8_t sample_wr = 0;
                uint8_t sample_rd = 0;
                uint8_t gpinpol_copy = Usb.regRd(rGPINPOL);
                for(uint8_t i = 0; i < 16; i++) {
#ifdef ESP8266
                        yield(); // needed in order to reset the watchdog timer on the ESP8266
#endif
                        for(uint16_t j = 0; j < 65535; j++) {
                                Usb.regWr(rGPINPOL, sample_wr);
                                sample_rd = Usb.regRd(rGPINPOL);
                                if(sample_rd != sample_wr) {
                                        xil_printf("Test failed. Value written: %x Value read: %x \n", sample_wr, sample_rd);
                                        halt55();
                                }//if( sample_rd != sample_wr..
                                sample_wr++;
                        }//for( uint16_t j...
                        xil_printf(".");
                }//for( uint8_t i...
                Usb.regWr(rGPINPOL, gpinpol_copy);
                xil_printf(" SPI long test passed\n");
        }//SPI long test
        /* GPIO test */
        /* in order to simplify board layout, GPIN pins on text fixture are connected to GPOUT */
        /* in reverse order, i.e, GPIN0 is connected to GPOUT7, GPIN1 to GPOUT6, etc. */
        {
                uint8_t tmpbyte;
                xil_printf("GPIO test. Connect GPIN0 to GPOUT7, GPIN1 to GPOUT6, and so on \n");
                for(uint8_t sample_gpio = 0; sample_gpio < 255; sample_gpio++) {
#ifdef ESP8266
                        yield(); // needed in order to reset the watchdog timer on the ESP8266
#endif
                        Usb.gpioWr(sample_gpio);
                        tmpbyte = Usb.gpioRd();
                        /* bit reversing code copied vetbatim from http://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious */
                        tmpbyte = ((tmpbyte * 0x0802LU & 0x22110LU) | (tmpbyte * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
                        if(sample_gpio != tmpbyte) {
                                xil_printf("Test failed. Value written: %x Value read: %x \n", sample_gpio, tmpbyte);
                                press_any_key();
                                break;
                        }//if( sample_gpio != tmpbyte...
                }//for( uint8_t sample_gpio...
                E_Notify(PSTR("\r\nGPIO test passed."), 0x80);
        }//GPIO test
        /* PLL test. Stops/starts MAX3421E oscillator several times */
        {
                xil_printf("PLL test. 100 chip resets will be performed\n");
                /* check current state of the oscillator */
                if(!(Usb.regRd(rUSBIRQ) & bmOSCOKIRQ)) { //wrong state - should be on
                        xil_printf("Current oscillator state unexpected.\n");
                        press_any_key();
                }
                /* Restart oscillator */
                xil_printf("Resetting oscillator\n");
                for(uint16_t i = 0; i < 100; i++) {
#ifdef ESP8266
                        yield(); // needed in order to reset the watchdog timer on the ESP8266
#endif
                        E_Notify(PSTR("\rReset number "), 0x80);
                        Serial.print(i, DEC);
                        Usb.regWr(rUSBCTL, bmCHIPRES); //reset
                        if(Usb.regRd(rUSBIRQ) & bmOSCOKIRQ) { //wrong state - should be off
                                E_Notify(PSTR("\r\nCurrent oscillator state unexpected."), 0x80);
                                halt55();
                        }
                        Usb.regWr(rUSBCTL, 0x00); //release from reset
                        uint16_t j = 0;
                        for(j = 1; j < 65535; j++) { //tracking off to on time
                                if(Usb.regRd(rUSBIRQ) & bmOSCOKIRQ) {
                                        E_Notify(PSTR(" Time to stabilize - "), 0x80);
                                        Serial.print(j, DEC);
                                        E_Notify(PSTR(" cycles\r\n"), 0x80);
                                        break;
                                }
                        }//for( uint16_t j = 0; j < 65535; j++
                        if(j == 0) {
                                E_Notify(PSTR("PLL failed to stabilize"), 0x80);
                                press_any_key();
                        }
                }//for( uint8_t i = 0; i < 255; i++

        }//PLL test
        /* initializing USB stack */
        if(Usb.Init() == -1) {
                E_Notify(PSTR("\r\nOSCOKIRQ failed to assert"), 0x80);
                halt55();
        }
        E_Notify(PSTR("\r\nChecking USB device communication.\r\n"), 0x80);
}

void loop() {
        delay(200);
        Usb.Task();
        usbstate = Usb.getUsbTaskState();
        if(usbstate != laststate) {
                laststate = usbstate;
                /**/
                switch(usbstate) {
                        case( USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE):
                                E_Notify(PSTR("\r\nWaiting for device..."), 0x80);
                                break;
                        case( USB_ATTACHED_SUBSTATE_RESET_DEVICE):
                                E_Notify(PSTR("\r\nDevice connected. Resetting..."), 0x80);
                                break;
                        case( USB_ATTACHED_SUBSTATE_WAIT_SOF):
                                E_Notify(PSTR("\r\nReset complete. Waiting for the first SOF..."), 0x80);
                                break;
                        case( USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE):
                                E_Notify(PSTR("\r\nSOF generation started. Enumerating device..."), 0x80);
                                break;
                        case( USB_STATE_ADDRESSING):
                                E_Notify(PSTR("\r\nSetting device address..."), 0x80);
                                break;
                        case( USB_STATE_RUNNING):
                                E_Notify(PSTR("\r\nGetting device descriptor"), 0x80);
                                rcode = Usb.getDevDescr(1, 0, sizeof (USB_DEVICE_DESCRIPTOR), (uint8_t*) & buf);

                                if(rcode) {
                                        E_Notify(PSTR("\r\nError reading device descriptor. Error code "), 0x80);
                                        print_hex(rcode, 8);
                                } else {
                                        /**/
                                        E_Notify(PSTR("\r\nDescriptor Length:\t"), 0x80);
                                        print_hex(buf.bLength, 8);
                                        E_Notify(PSTR("\r\nDescriptor type:\t"), 0x80);
                                        print_hex(buf.bDescriptorType, 8);
                                        E_Notify(PSTR("\r\nUSB version:\t\t"), 0x80);
                                        print_hex(buf.bcdUSB, 16);
                                        E_Notify(PSTR("\r\nDevice class:\t\t"), 0x80);
                                        print_hex(buf.bDeviceClass, 8);
                                        E_Notify(PSTR("\r\nDevice Subclass:\t"), 0x80);
                                        print_hex(buf.bDeviceSubClass, 8);
                                        E_Notify(PSTR("\r\nDevice Protocol:\t"), 0x80);
                                        print_hex(buf.bDeviceProtocol, 8);
                                        E_Notify(PSTR("\r\nMax.packet size:\t"), 0x80);
                                        print_hex(buf.bMaxPacketSize0, 8);
                                        E_Notify(PSTR("\r\nVendor  ID:\t\t"), 0x80);
                                        print_hex(buf.idVendor, 16);
                                        E_Notify(PSTR("\r\nProduct ID:\t\t"), 0x80);
                                        print_hex(buf.idProduct, 16);
                                        E_Notify(PSTR("\r\nRevision ID:\t\t"), 0x80);
                                        print_hex(buf.bcdDevice, 16);
                                        E_Notify(PSTR("\r\nMfg.string index:\t"), 0x80);
                                        print_hex(buf.iManufacturer, 8);
                                        E_Notify(PSTR("\r\nProd.string index:\t"), 0x80);
                                        print_hex(buf.iProduct, 8);
                                        E_Notify(PSTR("\r\nSerial number index:\t"), 0x80);
                                        print_hex(buf.iSerialNumber, 8);
                                        E_Notify(PSTR("\r\nNumber of conf.:\t"), 0x80);
                                        print_hex(buf.bNumConfigurations, 8);
                                        /**/
                                        E_Notify(PSTR("\r\n\nAll tests passed. Press RESET to restart test"), 0x80);
                                        while(1) {
#ifdef ESP8266
                                                yield(); // needed in order to reset the watchdog timer on the ESP8266
#endif
                                        }
                                }
                                break;
                        case( USB_STATE_ERROR):
                                E_Notify(PSTR("\r\nUSB state machine reached error state"), 0x80);
                                break;

                        default:
                                break;
                }//switch( usbstate...
        }
}//loop()...

/* constantly transmits 0x55 via SPI to aid probing */
void halt55() {

        xil_printf("Unrecoverable error - test halted!!\n");
        xil_printf("0x55 pattern is transmitted via SPI\n");
        xil_printf("Press RESET to restart test\n");

        while(1) {
                Usb.regWr(0x55, 0x55);
#ifdef ESP8266
                yield(); // needed in order to reset the watchdog timer on the ESP8266
#endif
        }
}

/* prints hex numbers with leading zeroes */
void printHex (u32 data, unsigned channel)
{
	XGpio_DiscreteWrite (&Gpio_hex, channel, data);
}

/* prints "Press any key" and returns when key is pressed */
void press_any_key() {
        xil_printf("Press any key to continue...\n");
        while(Serial.available() <= 0) { // wait for input //TODO: FIX FIX FIX FIX FIX FIX
#ifdef ESP8266
                yield(); // needed in order to reset the watchdog timer on the ESP8266
#endif
        }
        Serial.read(); //empty input buffer
        return;
}
