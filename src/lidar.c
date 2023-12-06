#include "lidar.h"
#include "uart_print.h"

#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#include <inc/hw_uart.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_memmap.h>
#include <common/tm4c123gh6pm.h>

#define TX_PIN (1 << 7)
#define RX_PIN (1 << 6)
#define LID_PINS TX_PIN | RX_PIN

#define LID_BAUD 230400

#define LID_BASE UART2_BASE

//defined bytes for the lidar commands
#define COMMAND_MAGIC		0xA5
#define COMMAND_START		0x60
#define COMMAND_STOP		0x65
#define COMMAND_DEV_INFO	0x90
#define COMMAND_HEALTH		0x92
#define COMMAND_RESET		0x40

#define RESPONSE_HEADER_MAGIC 0x5AA5
#define SCAN_MAGIC 0x55AA
#define SCAN_MAGIC_0 0xAA
#define SCAN_MAGIC_1 0x55

#define DEBUG_LID
/*
void process_point_buff(){
	for(int i = 0; i < POINTS_BUF_SIZE; i++){
		
		if(receiving_point_buffer[i] != 0) continue;
		//Find closest point to the "left" that has a value
		int leftidx = (i == 0) ? (POINTS_BUF_SIZE-1) : (i-1);
		bool from_top = (leftidx > 0);
		while ((receiving_point_buffer[leftidx] == 0)) {
			leftidx--;
			if (leftidx < 0) {
				if(from_top){
					receiving_point_buffer[i] = 1; // No data was collected from lidar???
					continue;
				} else {
					leftidx = (POINTS_BUF_SIZE-1);
					from_top = true;
				}
			}
		}
		
		//Find closest point to the right that has a value
		int rightidx = (i+1 < POINTS_BUF_SIZE) ? (i+1) : 0;
		bool from_0 = !rightidx;
		while(receiving_point_buffer[rightidx] == 0) {
			if(++rightidx >= POINTS_BUF_SIZE){
				if(from_0) {
					receiving_point_buffer[i] = 1; // No data was collected from lidar?
					continue;
				} else {
					rightidx = 0;
					from_0 = true;
				}
			}
		}
		receiving_point_buffer[i] = (receiving_point_buffer[leftidx] + receiving_point_buffer[rightidx])/2;
	}
}*/

void process_points(uint16_t *scan_points){

	for(int i = 0; i < 360; i++) {
		printlf("%d:%d\n", i, scan_points[i]);
		scan_points[i] = 0;
	}
}

void setup_lidar_comms(void){
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_UART2)){};
	GPIO_PORTD_LOCK_R = 0x4C4F434B;
	GPIO_PORTD_CR_R |= 0xFF;
	GPIO_PORTD_DEN_R |= 0xFF;
	GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & ~(GPIO_PCTL_PD7_M | GPIO_PCTL_PD6_M)) | GPIO_PCTL_PD7_U2TX | GPIO_PCTL_PD6_U2RX;
	ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, LID_PINS, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
	ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, RX_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
	ROM_GPIODirModeSet(GPIO_PORTD_BASE, LID_PINS, GPIO_DIR_MODE_HW);
	//ROM_UARTClockSourceSet(LID_BASE, UART_CLOCK_SYSTEM);
	ROM_UARTConfigSetExpClk(LID_BASE, ROM_SysCtlClockGet(), LID_BAUD, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	
}

void start_lidar_scan(void) {
	ROM_UARTCharPut(LID_BASE, COMMAND_MAGIC);
	ROM_UARTCharPut(LID_BASE, COMMAND_START);
}

void stop_lidar_scan(void) {
	ROM_UARTCharPut(LID_BASE, COMMAND_MAGIC);
	ROM_UARTCharPut(LID_BASE, COMMAND_STOP);
}

void clear_lidar_IO(void){
	while(ROM_UARTCharsAvail(LID_BASE)){
		ROM_UARTCharGet(LID_BASE);
		ROM_UARTRxErrorClear(LID_BASE);
	}
}

uint16_t map_to_degree(uint16_t start, uint16_t end, uint16_t num){
	float delta = ((float)end)/64.0f - ((float)start)/64.0f;
	float degree = (float)start + (delta * num);
	
	#ifdef DEBUG_LID
	if(degree < 0 || degree >360) printlf("BAD ANGLE?\n");
	#endif
	return (uint16_t)degree;
}

uint16_t convert_to_mm(uint16_t point){
	return point >> 2; //divide by 4
}

//Returns true if header fails checksum & other verification
bool check_header(ScanHeader *head) {
	if(head->header.start_code != SCAN_MAGIC) return true; //incorret start word
	if(!(head->header.start_angle & 0x1)) return true; //incorrect angle data
	if(!(head->header.end_angle & 0x1)) return true; //incorrect angle data
	
	uint16_t checksum = 0;
	//size minus 2 - don't include checksum word
	for(size_t i = 0; i < sizeof(*head) - 2; ) {
		uint8_t low = head->bytes[i++];
		uint8_t high = head->bytes[i++];
		checksum ^= (high << 8) | low;
	}
	
	return (checksum != head->header.checksum);
}

void process_packets(void) {
	static enum {COMM_WAITING, RECEIVING_HEADER, WAITING_SCAN_HEADER, RECEIVING_SCAN_HEADER, RECEIVING_SCAN_DATA, RECEIVING_HEALTH_REPORT} comm_state = WAITING_SCAN_HEADER;
	static uint32_t current_byte = 0;
	static ScanHeader current_scan;
	static uint32_t current_point = 0;
	static uint16_t scan_points[360];
	
	while(ROM_UARTCharsAvail(LID_BASE)) {
		uint8_t received = ROM_UARTCharGet(LID_BASE);
		if(ROM_UARTRxErrorGet(UART2_BASE)) goto reset;
		
		switch(comm_state) {
		//Haven't started a data packet yet - look for magic header
			case WAITING_SCAN_HEADER:{
				//Is this the first, or second byte in the header magic number
				switch(current_byte++) {
					case 0:
						if(received != SCAN_MAGIC_0) goto reset;
						current_scan.bytes[0] = received;
					break;
				
					case 1:
						if(received != SCAN_MAGIC_1) goto reset;
						current_scan.bytes[1] = received;
						comm_state = RECEIVING_SCAN_HEADER;
					break;
				}
			} break;
		
			case RECEIVING_SCAN_HEADER:{
				current_scan.bytes[current_byte++] = received;
				
				//Still receiving the header
				if(current_byte < sizeof(current_scan)) continue;
				
				if(check_header(&current_scan)) goto reset;
				
				//Start of a new scan sequence - process the last, complete, scan sequence
				if(current_scan.header.type == START_PACKET) {
					process_points(scan_points);
					goto reset;
				}
				
				//Header is good, read in the points
				current_byte = 0;
				current_point = 0;
				comm_state = RECEIVING_SCAN_DATA;
			} break;
			
			case RECEIVING_SCAN_DATA:{
				static uint8_t low, high;
				
				switch(current_byte){
					case 0:
						low = received;
					break;
					
					case 1: 
						high = received;
						uint16_t point = (high << 8) | low;
						uint16_t angle = map_to_degree(current_scan.header.start_angle>>1, current_scan.header.end_angle>>1, current_point);
						
						//Only read in the first point at a particular degree. Overwrite "0" points
						if(!scan_points[angle]) scan_points[angle] = convert_to_mm(point);
						
						//If we've read in all the points available in this packet, reset for the next packet.
						if(++current_point >= current_scan.header.sample_count) goto reset;
					break;
				}
				
			} break;
		}
	}
	return;
	
	//HOT TAKE: Exception handling is just goto in fancy clothes
	reset:
	ROM_UARTRxErrorClear(LID_BASE);
	current_byte = 0;
	current_point = 0;
	comm_state = WAITING_SCAN_HEADER;
	#ifdef DEBUG_LID
	printlf("R");
	#endif
}











