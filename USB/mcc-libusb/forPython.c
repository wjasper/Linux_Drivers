
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#include <fcntl.h>
#include <ctype.h>

#include "pmd.h"
#include "usb-1208LS.h"
#include "usb-temp.h"


#define channel_6 6 // Pt100
#define channel_7 7 // Pt100
#define nchan_TC 6	// for Thermocouples type K
#define channel_0 0	// 7+8/CH4+5/AI4+5  voltage signal
#define channel_2 2	// 1+2/CH0+1/AI0+1  voltage signal


///////////////////////////////////
// global structures and variables
struct rec{
	int zeit;
	float wert;
};
struct rec my_rec;


int flag; // für fcntl 
hid_device *hid_usb1208LS;
hid_device *hid_usbTemp;


///////////////////////////////////////
// Declarations of the function body
int initialize_usbTemp();
extern float volts_FS(const int gain, const signed short num);

int initialize_usb1208LS();
extern signed short usbAIn_USB1208LS(hid_device *hid_usb1208LS, uint8_t channel, uint8_t range);
extern float volts_LS( const int gain, const signed short num );

void initialize_ChannelConfigurations();


unsigned int time_difference(char LINK[]);




int main(int argc, char **argv)
{
  int ch;
  
  //~ int mikros = 1000000;
  //~ struct timespec ts;
  //~ ts.tv_sec = mikros/ 1000000L;
  //~ ts.tv_nsec = (mikros % 1000000L) * 1000L;
  
  signed short svalue_0;	// usb1208LS
  signed short svalue_2;
  uint8_t gain = BP_20_00V;
  
  float temperature_Pt100_Ch6;	// usb-temp
  float temperature_Pt100_Ch7;
  float temperature_TC_array[nchan_TC];
  
  initialize_usb1208LS();	// Initialisierungsfunktionen
  initialize_usbTemp();
  initialize_ChannelConfigurations();

    
  while (1){
	  printf("\nUSB-TEMP and USB-1208LS  Testing\n");
	  printf("\nType i to start measurement\n");
	  printf("\nType e to stop programm\n");
		while((ch = getchar()) == '\0' || ch == '\n');
		
		switch(tolower(ch)) {
		case 'i':
			flag = fcntl(fileno(stdin), F_GETFL);
			fcntl(0, F_SETFL, flag | O_NONBLOCK);
				do {
					unsigned int zeit= (unsigned int)time(NULL); // UNIX-Timestamp
					time_t raw_time = zeit; 
					svalue_0 = usbAIn_USB1208LS(hid_usb1208LS, channel_0, gain);
					svalue_2 = usbAIn_USB1208LS(hid_usb1208LS, channel_2, gain);

					usbTinScan_USBTEMP(hid_usbTemp, CH0, nchan_TC-1, 0, temperature_TC_array);
					usbTin_USBTEMP(hid_usbTemp, channel_6, 0, &temperature_Pt100_Ch6);
					usbTin_USBTEMP(hid_usbTemp, channel_7, 0, &temperature_Pt100_Ch7);
					
					 
					printf("Channel %d: UNIX-Timestamp %u, Zeit %s, value = %#hx, valuE = %d, %.2fV\n",channel_0, zeit,ctime(&raw_time), svalue_0, svalue_0, volts_LS(gain, svalue_0));
					printf("Channel %d: UNIX-Timestamp %u, Zeit %s, value = %#hx, value = %d, %.2fV\n",channel_2, zeit,ctime(&raw_time), svalue_2, svalue_2, volts_LS(gain, svalue_2));
					int i;
					for ( i = 0; i < nchan_TC; i++ ) {
						printf("Channel %d:  %.3f degress Celsius\n", i, temperature_TC_array[i]);								
					}
					printf("Channel: %d  %.3f degress Celsius, Channel: %d  %.3f degrees CEEatelsius \n", channel_6, temperature_Pt100_Ch6, channel_7, temperature_Pt100_Ch7);
					
	
	
						
				printf("\n");
				//~ sleep(1);
				usleep(1000000);
				//~ nanosleep(&ts, (struct timespec*)NULL);
			  } while (!isalpha(getchar()));
			  
			  fcntl(fileno(stdin), F_SETFL, flag);
		break;
		
							
		case 'e':
			  hid_close(hid_usb1208LS);
			  hid_exit();
			  hid_close(hid_usbTemp);
			  hid_exit();
			  exit(0);
		break;
		}
		return 0;
	}
	
}

	




void initialize_ChannelConfigurations(){
	float R0, A, B, C;      // RTD: Callendar-Van Dusen coefficients
	float value;
	int i;
	for ( i = 0; i < nchan_TC; i++ ) {	
		usbSetItem_USBTEMP(hid_usbTemp, i/2, SENSOR_TYPE, THERMOCOUPLE);
		usbSetItem_USBTEMP(hid_usbTemp, i/2, EXCITATION, EXCITATION_OFF);
		usbSetItem_USBTEMP(hid_usbTemp, i/2, i%2+CH_0_TC, TYPE_K);
	}
	unsigned short ch_RTD;
	for (ch_RTD=6; ch_RTD <8; ch_RTD++){
		usbSetItem_USBTEMP(hid_usbTemp, ch_RTD/2, SENSOR_TYPE, RTD);
		usbSetItem_USBTEMP(hid_usbTemp, ch_RTD/2, CONNECTION_TYPE, FOUR_WIRE);
		usbSetItem_USBTEMP(hid_usbTemp, ch_RTD/2, EXCITATION, MU_A_210);
		usbSetItem_USBTEMP(hid_usbTemp, ch_RTD/2, CH_0_GAIN + ch_RTD%2, 0x2);          // Set 0 - 0.5V for RTD	//0x2 entsprich GAIN_4x

		R0 = 100.;
		A = .003908;	// Schon gegebene Werte
		B = -5.8019E-7;
		C = -4.2735E-12;
		//A = 0.0039083; 		// laut Datenblatt nach DIN EN 60751
		//B = -5.775E-7;
		//C = -4.183E-12;			
		usbSetItem_USBTEMP(hid_usbTemp, ch_RTD/2, CH_0_COEF_0 + ch_RTD%2, R0);         // R0 value
		usbGetItem_USBTEMP(hid_usbTemp, ch_RTD/2, CH_0_COEF_0 + ch_RTD%2, &value);
		usbSetItem_USBTEMP(hid_usbTemp, ch_RTD/2, CH_0_COEF_1 + ch_RTD%2, A);          // Callendar-Van Dusen Coefficient A
		usbGetItem_USBTEMP(hid_usbTemp, ch_RTD/2, CH_0_COEF_1 + ch_RTD%2, &value);
		usbSetItem_USBTEMP(hid_usbTemp, ch_RTD/2, CH_0_COEF_2 + ch_RTD%2, B);          // Callendar-Van Dusen Coefficient B
		usbGetItem_USBTEMP(hid_usbTemp, ch_RTD/2, CH_0_COEF_2 + ch_RTD%2, &value);
		usbSetItem_USBTEMP(hid_usbTemp, ch_RTD/2, CH_0_COEF_3 + ch_RTD%2, C);          // Callendar-Van Dusen Coefficient C
		usbGetItem_USBTEMP(hid_usbTemp, ch_RTD/2, CH_0_COEF_3 + ch_RTD%2, &value);
	}				
}

int initialize_usbTemp(){
	hid_usbTemp = 0x0;
	int returnValue = 0;
	int ret = hid_init();
	
	if (ret < 0) {
		fprintf(stderr, "hid_init failed with return code %d\n", ret);
		returnValue = -1;
	}

	if ((hid_usbTemp = hid_open(MCC_VID, USB_TEMP_PID, NULL)) > 0) {
		printf("USB-TEMP Device is found!\n");
	} else {
		fprintf(stderr, "USB-TEMP not found.\n");
		returnValue = -1;
		//exit(1);
	}
	/* config mask 0x01 means all inputs */
	usbDConfigPort_USBTEMP(hid_usbTemp, DIO_DIR_OUT);
	usbDOut_USBTEMP(hid_usbTemp, 0x00);
	return returnValue;
}


int initialize_usb1208LS(){
	hid_usb1208LS=0x0;
	int to_return = 0;
	int ret = hid_init();

	if (ret < 0) {
		fprintf(stderr, "hid_init failed with return code %d\n", ret);
		to_return = -1;
	}

	if ((hid_usb1208LS = hid_open(MCC_VID, USB1208LS_PID, NULL)) >  0) {
		printf("USB-1208LS Device is found!\n");
	} else {
		fprintf(stderr, "USB-1208LS not found.\n");
		to_return = -1;
		//~ exit(1);
	}
	/* config mask 0x01 means all inputs */
	usbDConfigPort_USB1208LS(hid_usb1208LS, DIO_PORTB, DIO_DIR_IN);
	usbDConfigPort_USB1208LS(hid_usb1208LS, DIO_PORTA, DIO_DIR_OUT);
	usbDOut_USB1208LS(hid_usb1208LS, DIO_PORTA, 0x01);
	usbDOut_USB1208LS(hid_usb1208LS, DIO_PORTA, 0x01);
	return to_return; 
}
