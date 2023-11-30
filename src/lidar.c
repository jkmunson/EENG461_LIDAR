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

//defined bytes for the lidar
#define MAGIC		0xA5
#define START		0x60
#define STOP		0x65
#define DEV_INFO	0x90
#define HEALTH		0x92
#define RESET		0x40

#define HEADER_MAGIC 0x5AA5
#define SCAN_MAGIC 0x55AA

uint32_t points_buf_0[POINTS_BUF_SIZE];
uint32_t points_buf_1[POINTS_BUF_SIZE];
uint32_t points_smooth[POINTS_BUF_SIZE];
uint32_t *active_point_buffer;
uint32_t *receiving_point_buffer;

void zero_point_buf(void) {
	for(int i = 0; i < POINTS_BUF_SIZE; i++){
		receiving_point_buffer[i] = 0;
	}
}

//Fill in any missing points with the average of the closest points to the left and the right.
//Unfortunately the lidar has significant "holes" in it's data.
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
		int rightidx = (i < POINTS_BUF_SIZE) ? (i+1) : 0;
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
	ROM_UARTConfigSetExpClk(LID_BASE, ROM_SysCtlClockGet(), LID_BAUD, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	
	receiving_point_buffer = points_smooth;
	zero_point_buf();
	receiving_point_buffer = points_buf_0;
	zero_point_buf();
	receiving_point_buffer[0] = 1;
	process_point_buff();
	active_point_buffer = points_buf_0;
	receiving_point_buffer = points_buf_1;
	zero_point_buf();
	
	
}

void start_lidar_scan(void) {
	ROM_UARTCharPut(LID_BASE, MAGIC);
	ROM_UARTCharPut(LID_BASE, START);
}

void stop_lidar_scan(void) {
	ROM_UARTCharPut(LID_BASE, MAGIC);
	ROM_UARTCharPut(LID_BASE, STOP);
}

void clear_lidar_IO(void){
	while(ROM_UARTCharsAvail(LID_BASE)){
		ROM_UARTCharGet(LID_BASE);
	}
}

int angle_map(int32_t start, int32_t end, int count, int position){
	const int32_t delta = (end > start) ? (end - start) : (start - end);
	const int32_t delta_per_count = delta/count;
	return (((position*delta_per_count) + start) / 64) % 360;
}

void print_angles(void) {
	for(int i = 0; i<360; i++){
		points_smooth[i] = (points_smooth[i]*3 + active_point_buffer[i])/4;
	}
	printlf("\033c");
	for(int i = 15; i < 360; i+=15) {
		printlf("%d:%d\n", i, points_smooth[i]);
	}
}

void process_packets(void) {
	static enum {COMM_WAITING, RECEIVING_HEADER, WAITING_SCAN_HEADER, RECEIVING_SCAN_HEADER, RECEIVING_SCAN_DATA, RECEIVING_HEALTH_REPORT} comm_state = COMM_WAITING;
	static uint16_t current_byte;
	static PacketHeader current_message;
	static ScanHeader current_scan;
	
	//while(ROM_UARTCharsAvail(LID_BASE)) {
	//	ROM_UARTCharPut(UART0_BASE,ROM_UARTCharGet(LID_BASE));
	//} return;
	
	while(ROM_UARTCharsAvail(LID_BASE)) {
		switch(comm_state) {
			case COMM_WAITING:{
				char tmp = ROM_UARTCharGet(LID_BASE);
				if(tmp != 0xA5 && tmp != 0xAA ) break; // Really we're looking for 0xA5, but is 0xAA is seen, this will lead to a reset.
				comm_state = RECEIVING_HEADER;
				current_byte = 1;
				current_message.bytes[0] = 0xA5;
				printlf("Header Started\n");
			} break;
			
			case RECEIVING_HEADER: {
				current_message.bytes[current_byte++] = ROM_UARTCharGet(LID_BASE);
				//printlf(" %d : %d\n", current_message.bytes[current_byte-1], current_byte-1);
				//If we're not done receiving the header, continue
				if(current_byte < sizeof(current_message)) break; 
				
				//process the header
				if(current_message.header.start_code != HEADER_MAGIC) {
					printlf("Unexpected Header %d, resetting\n", current_message.header.start_code);
					stop_lidar_scan();
					clear_lidar_IO();
					ROM_SysCtlDelay(20000);
					clear_lidar_IO();
					start_lidar_scan();
					comm_state = COMM_WAITING;
					break;
				} else {
					printlf("Got valid header!\n");
				}
				
				switch(current_message.header.type){
					case TYPE_SCAN:
						comm_state = WAITING_SCAN_HEADER;
						current_byte = 0;
						printlf("Header is start of scan\n");
					break;
					
					case TYPE_HEALTH_STATUS:
						comm_state = RECEIVING_HEALTH_REPORT;
					break;
					
					default:
						printlf("Unexpected message type %d. Discarding...\n", (uint32_t)current_message.header.type);
						comm_state = COMM_WAITING;
					break;
				}
			} break;
			
			case WAITING_SCAN_HEADER:{
				if(ROM_UARTCharGet(LID_BASE) != 0xAA) break;
				comm_state = RECEIVING_SCAN_HEADER;
				current_byte = 1;
				current_scan.bytes[0] = 0xAA;
			} break;
				
			case RECEIVING_SCAN_HEADER: {
				current_scan.bytes[current_byte++] = ROM_UARTCharGet(LID_BASE);
				//ROM_UARTCharPut(UART0_BASE, current_scan.bytes[current_byte-1]);
				//If we're not done receiving the header, continue
				if(current_byte < sizeof(current_scan)) break;
				//printlf("Got scan header\n");
				if(current_scan.header.start_code != SCAN_MAGIC){
					//printlf("Scan header invalid! Reset\n");
					//stop_lidar_scan();
					//start_lidar_scan();
					comm_state = WAITING_SCAN_HEADER;
					current_byte=0;
					return;
				}
				if(current_scan.header.type == START_PACKET) {
					//Swap buffers
					//printlf("START PACKET\n");
					process_point_buff();
					uint32_t *temp = active_point_buffer;
					active_point_buffer = receiving_point_buffer;
					receiving_point_buffer = temp;
					current_byte = 0;
					zero_point_buf();
					comm_state = WAITING_SCAN_HEADER;
					print_angles();
					
				} else {
					//printlf("Receiving Data\n");
					comm_state = RECEIVING_SCAN_DATA;
					current_byte = 0;
				}
			} break;
			
			case RECEIVING_SCAN_DATA:{
				//printlf("byte%d/%d\n", current_byte, current_scan.header.sample_count*2);
				static uint8_t lsb, msb;
				switch(current_byte % 2){
					case 0:
						lsb = ROM_UARTCharGet(LID_BASE);
					break;
					
					case 1:
						msb = ROM_UARTCharGet(LID_BASE);
						uint32_t val = ((msb << 8) | lsb) >> 2;
						//printlf("%d ",val);
						int angle = angle_map(current_scan.header.start_angle, current_scan.header.end_angle, current_scan.header.sample_count, current_byte>>1);
						//Average in points that map to the same integer angle.
						if (receiving_point_buffer[angle]) {
							receiving_point_buffer[angle] = (receiving_point_buffer[angle] + val) >> 1;
						} else {
							receiving_point_buffer[angle] = val;
						}
					break;
				};
				if((++current_byte)>>1 == current_scan.header.sample_count){
					current_byte = 0;
					comm_state = WAITING_SCAN_HEADER;
				}
			} break;
			
			case RECEIVING_HEALTH_REPORT:
				//stub
				comm_state = COMM_WAITING;
			break;
			
			default:
				printlf("Invalid comm state - resetting");
				comm_state = COMM_WAITING;
			break;
		}
	}
}
