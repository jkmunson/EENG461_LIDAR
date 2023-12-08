#include "lidar.h"
#include "uart_print.h"
#include "timers.h"

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

uint16_t g_points[361];
uint16_t g_conditioned_points[361];
uint16_t g_pt_buf[361];

//VERY IMPORTANT: This function has a time budget of 40000 cycles, to avoid losing data in the fifo
void process_points(){
	static int32_t last_time = 0;
	//#### GNU extension: nested helper functions
	bool has_holes(uint16_t *points) {
		for(int i = 0; i < 360; i++) if(!points[i]) return true;
		return false;
	}
	
	int count_points(void) {
		int count = 0;
		for(int i = 0; i < 360; i++) {
			if(g_points[i]) {
				count++;
			}
		}
		return count;
	}
	
	//reminder, c does not have a modulo operator. It has a remainder operator.
	int bounded_idx(int idx) {
		while(idx < 0) idx += 360;
		return idx % 360;
	}
	
	//Wipe the point buffer every second - prevent overly stale points accumulating
	if(last_time != uptime_seconds){
		//printlf("Wiping buffer\n");
		for(int j = 0; j < 360; j++) {
			g_points[j] = 0;
		}
		last_time = uptime_seconds;
	}
	
	if(count_points() < 180) return; //Want at least this many points before we process this

	uint16_t *temp = g_pt_buf;
	for(int i = 0; i < 360; i++) temp[i] = g_points[i];
	int iter = 0;
	while(has_holes(temp)) {
		int left = 0, right, mid; //indexes of start and end of a gap
		// find a point that is missing
		while(temp[left]) left++;
		while(!temp[bounded_idx(left-1)]) left--;
		left -= 1;
		right = left;
		while(!temp[bounded_idx(right+1)]) right++; // walk to right, find last empty point
		right += 1;
		left = bounded_idx(left);
		right = bounded_idx(right);
		int diff = bounded_idx(right - left);
		mid = bounded_idx(left + diff/2);
		temp[mid] = (temp[left] + temp[right])/2;
	}
	for(int i = 0; i < 360; i++) g_conditioned_points[i] = temp[i];
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

uint16_t map_to_degree(uint16_t start, uint16_t end, uint16_t count, uint16_t num){
	float delta = (((float)end)/64.0f - ((float)start)/64.0f)/(float)count;
	float degree = ((float)start)/64.0f + (delta * num);
	if(delta < 0 || degree < 0 || degree > 359) return 360; // 360 is a safe, do nothing location to write points that have angles that don't make sense.
	return (uint16_t)degree;
}

uint16_t convert_to_mm(uint16_t point){
	return point >> 2; //divide by 4
}

//Returns true if header fails verification
bool check_header(ScanHeader *head) {
	//printlf("Magic\n");
	if(head->header.start_code != SCAN_MAGIC) return true; //incorret start word
	//printlf("Start A\n");
	if(!(head->header.start_angle & 0x1)) return true; //incorrect angle data
	//printlf("End A\n");
	if(!(head->header.end_angle & 0x1)) return true; //incorrect angle data
	return false;
}

//calculates checksum of header
uint16_t checksum_header(ScanHeader *head) {
	uint16_t checksum = 0;
	//size minus 2 - don't include checksum word
	for(size_t i = 0; i < sizeof(*head) - 2; ) {
		uint8_t low = head->bytes[i++];
		uint8_t high = head->bytes[i++];
		checksum ^= (high << 8) | low;
	}

	return checksum;
}

void process_packets(void) {
	static enum {COMM_WAITING, RECEIVING_HEADER, WAITING_SCAN_HEADER, RECEIVING_SCAN_HEADER, RECEIVING_SCAN_DATA, RECEIVING_HEALTH_REPORT} comm_state = WAITING_SCAN_HEADER;
	static uint32_t current_byte = 0;
	static ScanHeader current_scan;
	static uint32_t current_point = 0;
	static uint16_t checksum = 0;
	
	while(ROM_UARTCharsAvail(LID_BASE)) {
		uint8_t received = ROM_UARTCharGet(LID_BASE);
		if(ROM_UARTRxErrorGet(UART2_BASE)) {
			#ifdef DEBUG_LID
			//printlf("x");
			#endif
			goto reset;
		}
		
		switch(comm_state) {
		//Haven't started a data packet yet - look for magic header
			case WAITING_SCAN_HEADER:{
				//Is this the first, or second byte in the header magic number
				switch(current_byte++) {
					case 0:
						if(received != SCAN_MAGIC_0) goto reset;
						//printlf("B0\n");
						current_scan.bytes[0] = received;
					break;
				
					case 1:
						if(received != SCAN_MAGIC_1) goto reset;
						current_scan.bytes[1] = received;
						comm_state = RECEIVING_SCAN_HEADER;
						//printlf("B1\n");
					break;
				}
			} break;
		
			case RECEIVING_SCAN_HEADER:{
				current_scan.bytes[current_byte++] = received;
				//printlf("h%d\n",current_byte);
				//Still receiving the header
				if(current_byte < sizeof(current_scan)) continue;
				
				if(check_header(&current_scan)) goto reset;
				
				//Start of a new scan sequence - process the last, complete, scan sequence
				if(current_scan.header.type == START_PACKET) {
					process_points();
					goto reset;
				}
				
				//Header is good, read in the points
				current_byte = 0;
				current_point = 0;
				checksum = checksum_header(&current_scan);
				comm_state = RECEIVING_SCAN_DATA;
			} break;
			
			case RECEIVING_SCAN_DATA:{
				static uint8_t low, high;
				
				switch(current_byte){
					case 0:
						low = received;
						current_byte = 1;
					break;
					
					case 1: 
						current_byte = 0;
						high = received;
						uint16_t point = ((high&0xff) << 8) | low; //High is actually the 7 leftmost bits.
						checksum ^= point;
						if(!(point & 0x80)) goto skip;
						uint16_t angle = map_to_degree(current_scan.header.start_angle>>1, current_scan.header.end_angle>>1, current_scan.header.sample_count, current_point);
						//printlf("P%d:A%d:L%d\n", current_point, angle, point);
						//Only read in the first point at a particular degree. Overwrite "0" points
						uint16_t tmp = convert_to_mm(point);
						//These values appear mixed into otherwise valid data: maybe they are meant to mean something? In any case, these points are JUNK almost always
						//if(tmp == 1684 || tmp == 8192 || tmp == 3223 || tmp == 256 || tmp == 16 || tmp == 360 || tmp == 3636 || tmp > 4000) goto skip;
						if(tmp) g_points[angle] = tmp;
						skip:
						//If we've read in all the points available in this packet, reset for the next packet.
						if(++current_point >= current_scan.header.sample_count) {
							if(checksum != current_scan.header.checksum) ;//printlf("Bad Checksum\n");
							goto reset;
						}
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
	//printlf("R\n");
	#endif
}











