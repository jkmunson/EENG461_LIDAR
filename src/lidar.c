#include "lidar.h"
#include "uart_print.h"

#define TX_PIN (1 << 7)
#define RX_PIN (1 << 6)
#define LID_PINS TX_PIN | RX_PIN

#define LID_BAUD 230400

//defined bytes for the lidar
#define MAGIC		0xA5
#define START		0x60
#define STOP		0x65
#define DEV_INFO	0x90
#define HEALTH		0x92
#define RESET		0x40

#define HEADER_MAGIC 0xA55A

#define POINTS_BUF_SIZE 360

uint32_t points_buf_0[POINTS_BUF_SIZE];
uint32_t points_buf_1[POINTS_BUF_SIZE];
uint32_t *active_point_buffer;
uint32_t *receiving_point_buffer;

typedef union {
	uint8_t *bytes;
	struct __attribute__((packed)) {
		uint16_t start_code;
		uint32_t length:30;
		enum {MODE_SINGLE = 0, MODE_CONTINUOUS = 1} mode:2;
		enum {TYPE_SCAN=0x81, TYPE_HEALTH_STATUS=0x06} type:8;
	} header;
} PacketHeader;

void zero_point_buf() {
	for(int i = 0; i < POINTS_BUF_SIZE; i++){
		receiving_point_buffer[i] = 0;
	}
}

//Fill in any missing points with the average of the closest points to the left and the right.
//While the lidar is operating correctly, this should basically never happen.
void process_point_buff(){
	for(int i = 1; i < POINTS_BUF_SIZE; i++){
		if(receiving_point_buffer[i] != 0) continue;
		//Find closest point to the "left" that has a value
		int leftidx = (i == 0) ? (POINTS_BUF_SIZE-1) : (i-1);
		bool from_top = (leftidx > 0);
		while ((receiving_point_buffer[leftidx] == 0)) {
			if (--leftidx < 0) {
				if(from_top){
					receiving_point_buffer[i] = 1; // No data was collected from lidar???
					continue;
				} else {
					leftidx = (POINTS_BUF_SIZE-1);
				}
			}
		}
		//Find closest point to the right that has a value
		int rightidx = (i < POINTS_BUF_SIZE) ? (i+1) : 0;
		bool from_0 = !rightidx;
		while(receiving_point_buffer[rightidx] == 0)) {
			if(++rightidx => POINTS_BUF_SIZE){
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
	
	ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, LID_PINS, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
	ROM_GPIODirModeSet(GPIO_PORTD_BASE, LID_PINS, GPIO_DIR_MODE_HW);
	ROM_UARTConfigSetExpClk(UART2_BASE, ROM_SysCtlClockGet(), LID_BAUD,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void start_lidar_scan(void) {
	ROM_UARTCharPut(UART1_BASE, MAGIC);
	ROM_UARTCharPut(UART1_BASE, START);
}

void process_packets(void) {
	static enum {COMM_WAITING, RECEIVING_HEADER, RECEIVING_SCAN_HEADER, RECEIVING_SCAN_DATA, RECEIVING_HEALTH_REPORT} comm_state = COMM_WAITING;
	static int current_byte;
	static PacketHeader current_message;
	static ScanHeader current_scan;
	
	while(ROM_UARTCharsAvail(UART2_BASE) {
	
		switch(comm_state) {
			case COMM_WAITING:{
				comm_state = RECEIVING_HEADER;
				current_byte = 0;
			} break;
			
			case RECEIVING_HEADER: {
				current_message.bytes[current_byte++] = ROM_UARTCharGet(UART2_BASE);
				
				//If we're not done receiving the header, continue
				if(current_byte < sizeof(current_message)) break; 
				
				//process the header
				if(current_message.header.start_code != HEADER_MAGIC) {
					printlf("Unexpected Header %d\n", current_message.header.start_code);
					//To recover, wait until the next packet start. (LONG!)
					while(ROM_UARTCharGet(UART2_BASE) != 0xA5){};
					current_message.bytes[0] = 0xA5;
					current_byte = 1;
					break;
				}
				
				switch(current_message.header.type){
					case TYPE_SCAN:
						comm_state = RECEIVING_SCAN_HEADER;
						current_byte = 0;
					break;
					
					case TYPE_HEALTH_STATUS:
						comm_state = RECEIVING_HEALTH_REPORT;
					break;
					
					default:
						printlf("Unexpected message type %d. Discarding...\n", (uint32_t)current_message.header.type);
						while(ROM_UARTCharGet(UART2_BASE) != 0xA5){};
						current_message.bytes[0] = 0xA5;
						current_byte = 1;
					break;
				}
			} break;

				
			case RECEIVING_SCAN_HEADER: {
				current_scan.bytes[current_byte++] = ROM_UARTCharGet(UART2_BASE);
				
				//If we're not done receiving the header, continue
				if(current_byte < sizeof(current_scan)) break;
				
				comm_state = RECEIVING_SCAN_DATA;
				current_byte = 0;
			} break;
			
			case RECEIVING_SCAN_DATA:
				
			break;
			
			case RECEIVING_HEALTH_REPORT:
				//stub
				while(ROM_UARTCharGet(UART2_BASE) != 0xA5){};
				current_message.bytes[0] = 0xA5;
				current_byte = 1;
				
			break;
			
			default:
				printlf("Invalid comm state - resetting");
				comm_state = COMM_WAITING;
			break;
		}
	
}