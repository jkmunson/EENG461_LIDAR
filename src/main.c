#include <stdint.h>
#include <stdbool.h>

#include <inc/hw_uart.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_memmap.h>
#include <common/tm4c123gh6pm.h>
#include <driverlib/rom.h>
#include <driverlib/sysctl.h>

#include "main.h"
#include "timers.h"
#include "uart_print.h"
#include "lidar.h"

#define Disable_Interrupts() __asm("CPSID I")
#define Enable_Interrupts() 


int main (void) {
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	configureDebounceTimer();
	setup_uart_printer();
	//printlf("Init Lidar Comms\n");
	setup_lidar_comms();
	//printlf("Stopping Lidar scan\n");
	stop_lidar_scan();
	//printlf("Flushing Lidar buffer\n");
	ROM_SysCtlDelay(1000000);
	clear_lidar_IO();
	
	//Speed the lidar up to max
	ROM_UARTCharPut(UART2_BASE, 0xA5);
	ROM_UARTCharPut(UART2_BASE, 0x0B);
	ROM_UARTCharPut(UART2_BASE, 0xA5);
	ROM_UARTCharPut(UART2_BASE, 0x0B);
	ROM_UARTCharPut(UART2_BASE, 0xA5);
	ROM_UARTCharPut(UART2_BASE, 0x0B);
	ROM_UARTCharPut(UART2_BASE, 0xA5);
	ROM_UARTCharPut(UART2_BASE, 0x0B);
	clear_lidar_IO();
	ROM_SysCtlDelay(1000000);
	//printlf("Starting Scan\n");
	start_lidar_scan();
	//printlf("Looping\n");
	int i = 0;
	while(1) {
		printlf("%d:%d\n", i, g_conditioned_points[i]);
		process_packets();
		i = (i+1)%360;
		if(!i) {
			bool drive_okay = false;
			//Check if good to drive - Average distance in cone of 90 deg in front must be > 1m
			int32_t avg_forward_distance = 0;
			for(int j = 135; j < 225; j++){
				avg_forward_distance += g_conditioned_points[j];
			}
			avg_forward_distance = avg_forward_distance / 90;
			if(avg_forward_distance > 1500) drive_okay = true;
			
			//Calculate best forward angle
			uint16_t *possible_angles = g_pt_buf; //Re-using this global buffer
			int n = 0;
			for(int j = 90; j < 270; j+= 5, n++){
				int32_t avg_distance = 0;
				for(int k = 0; k < 10; k++) avg_distance += g_conditioned_points[j+k] + g_conditioned_points[j-k];
				possible_angles[n] = avg_distance/20;
			}
			int top = n;
			int mid_pos = n/2;
			int best_angle = mid_pos;
			uint16_t best_value = possible_angles[mid_pos];
			for(;n;--n) {
				int position_mod = (n-mid_pos) > 0 ? (n-mid_pos)/2 : (mid_pos-n)/2;
				//printlf("%d/", position_mod);
				if(possible_angles[n]/position_mod > best_value) best_value = possible_angles[n]/position_mod, best_angle = n;
			}
			
			if(possible_angles[mid_pos] > 8000) best_angle = mid_pos;
			
			
			int turn_direction = ((int8_t)((best_angle*5)-90) > 0) ? 1 : 1;
			int8_t mod = 0;
			//extra adjustment to help avoid obstacles
			if (possible_angles[0] < 500 ) mod += 30;
			if (possible_angles[top-1] < 500) mod += -30;
			if (possible_angles[(top-1)/4] < 1000 ) mod += 20;
			if (possible_angles[((top-1)*3)/4] < 1000) mod += -20;
			//ROM_UARTCharPut(UART0_BASE, drive_okay ? (int8_t)((best_angle*5)-90) + mod : 120*turn_direction + mod);
			printlf("*%d|%d|%d\n", (best_angle*5)-90, (uint32_t) best_value, (int32_t)drive_okay);
		}
	}
	
	return (0);
}

/*//printlf("%d:%d\n", ROM_UARTCharGet(UART2_BASE), ROM_UARTRxErrorGet(UART2_BASE));
		//ROM_UARTCharPut(UART0_BASE, ROM_UARTCharGet(UART2_BASE));
		printlf("%d\n", ROM_UARTRxErrorGet(UART2_BASE), ROM_UARTCharGet(UART2_BASE));
		//ROM_UARTCharPut(UART0_BASE, ROM_UARTRxErrorGet(UART2_BASE));
		ROM_UARTRxErrorClear(UART2_BASE);*/
