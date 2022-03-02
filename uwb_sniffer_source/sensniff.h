
#include <stm32f4xx_hal.h>
#include <stdlib.h>
#include <deca_types.h>

/*!
 *  @fn	sensniff_rx_frame()
 *
 *  @brief	This function sends a received frame to a receiver running sensniff
 *
 *  @param rx_buffer - The frame content as a rx buffer
 *  @param frame_len - The length of the frame
 *
 *  no return value
 */
void sensniff_rx_frame(uint8_t* rx_buffer, int frame_len);


/*!
 * @fn sensniff_rx_frame_timestamp()
 *
 * @brief This functions sends a received frame to a receiver over UART running sensniff. The function also takes a timestamp value that will be transmitted as well.
 *
 * @param rx_buffer -  The frame content
 * @param frame_len - The length of the frame
 * @param timestamp - The timestamp when the frame has been received.
 */
void sensniff_rx_frame_timestamp(uint8_t* rx_buffer, int frame_len, uint64_t timestamp);


/*!
 *  @fn	sensniff_debug_log()
 *
 *  @brief	This function sends a debug message to a receiver that runs sensniff
 *
 *  @param data - Message data, this should be a NULL terminated string
 *
 *  no return value
 */
void sensniff_debug_log(unsigned char *data);

struct Sensniff_command {
	uint32_t magic_header;
	uint8_t version;
	uint8_t cmd;
	uint16_t data_length;
	uint8_t *data;
};

struct Sensniff_command* sensniff_receive();

#define USE_SENSNIFF_FOR_LOGS 1
