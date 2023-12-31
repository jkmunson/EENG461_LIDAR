#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

extern uint16_t g_points[361]; //Most up to date points, up to 1 second stale. position 360 is "dead letter" box
extern uint16_t g_pt_buf[361];
extern uint16_t g_conditioned_points[361]; //Holes drawn in

struct __attribute__((packed)) HeaderFields{
	uint16_t start_code;
	uint32_t length:30;
	enum {MODE_SINGLE = 0, MODE_CONTINUOUS = 1} mode:2;
	enum {TYPE_SCAN=0x81, TYPE_HEALTH_STATUS=0x06} type:8;
};
typedef union {
	uint8_t bytes[sizeof(struct HeaderFields)];
	struct HeaderFields header;
} PacketHeader;


struct __attribute__((packed)) ScanHeaderFields{
	uint16_t start_code;
	enum {START_PACKET=0x01, DATA_PACKET=0x00} type:8;
	uint8_t sample_count;
	uint16_t start_angle;
	uint16_t end_angle;
	uint16_t checksum; //(2-byte xor)
};
typedef union {
	uint8_t bytes[sizeof(struct ScanHeaderFields)];
	struct ScanHeaderFields header;
} ScanHeader;


void setup_lidar_comms(void);
void start_lidar_scan(void);
void stop_lidar_scan(void);
void process_packets(void);
void clear_lidar_IO(void);
