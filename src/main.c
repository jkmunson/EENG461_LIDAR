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
	
	setup_uart_printer();
	printlf("Init Lidar Comms\n");
	setup_lidar_comms();
	printlf("Stopping Lidar scan\n");
	stop_lidar_scan();
	printlf("Flushing Lidar buffer\n");
	clear_lidar_IO();
	__asm("CPSIE I");
	ROM_SysCtlDelay(20000000);
	//ROM_SysCtlDelay(20000000);
	//ROM_SysCtlDelay(20000000);
	//ROM_SysCtlDelay(20000000);
	printlf("Starting lidar scan#####################\n");
	clear_lidar_IO();
	ROM_SysCtlDelay(20000);
	ROM_UARTCharPut(UART2_BASE, 0xA5);
	ROM_UARTCharPut(UART2_BASE, 0x0B);
	ROM_UARTCharPut(UART2_BASE, 0xA5);
	ROM_UARTCharPut(UART2_BASE, 0x0B);
	ROM_UARTCharPut(UART2_BASE, 0xA5);
	ROM_UARTCharPut(UART2_BASE, 0x0B);
	ROM_UARTCharPut(UART2_BASE, 0xA5);
	ROM_UARTCharPut(UART2_BASE, 0x0B);
	clear_lidar_IO();
	start_lidar_scan();
	
	while(1) {
		process_packets();
	}
	
	return (0);
}
