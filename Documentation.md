# Documentation 
In this file we document the source code of the sniffer in a high-level manner. 

## Sniffer code 
The code hardware consists of four c files and their respective header. 

### main.c 
The main file is mainly retrieved from the STM samples. We did only change that it launches the sniffer code after the initial setup. 

### uwb_sniffer.c 

This file contains the code for the sniffer. The sniffer is started by executing the  `sniff_uwb()` function. 
The `configure()` function sets up the DWM3000EVB and configures the necessary parameters. 
The `receive()` function is called in an infinite loop to receive UWB frames. All received frames are handled by the `handle_receive()` function. In this function the frame's content is forwarded to the host using UART communication. The `handle_error()` function logs errors during frame reception using the UART communication. 

Since the DWM3000EVB uses picosecond-level accuracy the 40-bit counter overflows approximately every 20s. To keep consistent timestamps, we use the function `get_timestamps()` to handle overflows and synchronize the timestamps with the system time. 

### uart_send.c 
This file contains all functions necessary to send UART code. To loose as little performance as possible we use non-blocking UART with a send buffer. 

### sensniff.c
This file contains the necessary code to make our UART communication compatible with the sensiff Python project that receives our frames and forwards them to a pipe which is accessed from Wireshark. 