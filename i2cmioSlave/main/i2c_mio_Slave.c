/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/i2c.h"

/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a BH1750 light sensor(GY-30 module) for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP32 chip.
 *
 * Pin assignment:
 *
 * - slave :
 *    GPIO25 is assigned as the data signal of i2c slave port
 *    GPIO26 is assigned as the clock signal of i2c slave port
 * - master:
 *    GPIO18 is assigned as the data signal of i2c master port
 *    GPIO19 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect GPIO18 with GPIO25
 * - connect GPIO19 with GPIO26
 * - connect sda/scl of sensor with GPIO18/GPIO19
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 * - i2c master(ESP32) will write data to i2c slave(ESP32).
 * - i2c master(ESP32) will read data from i2c slave(ESP32).
 */

#define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     26              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        30             /*!< delay time between different test items */

#define I2C_EXAMPLE_SLAVE1_SCL_IO           22               /*!<gpio number for i2c slave clock  */
#define I2C_EXAMPLE_SLAVE1_SDA_IO           23               /*!<gpio number for i2c slave data */
#define I2C_EXAMPLE_SLAVE1_NUM              I2C_NUM_0        /*!<I2C port number for slave dev */
#define I2C_EXAMPLE_SLAVE_TX_BUF_LEN       (DATA_LENGTH)  /*!<I2C slave tx buffer size */
#define I2C_EXAMPLE_SLAVE_RX_BUF_LEN       (DATA_LENGTH)  /*!<I2C slave rx buffer size */

#define I2C_EXAMPLE_SLAVE2_SCL_IO           18               /*!<gpio number for i2c slave clock  */
#define I2C_EXAMPLE_SLAVE2_SDA_IO           19               /*!<gpio number for i2c slave data */
#define I2C_EXAMPLE_SLAVE2_NUM              I2C_NUM_1        /*!<I2C port number for slave dev */
//#define I2C_EXAMPLE_SLAVE2_TX_BUF_LEN       (DATA_LENGTH)  /*!<I2C slave tx buffer size */
//#define I2C_EXAMPLE_SLAVE2_RX_BUF_LEN       (DATA_LENGTH)  /*!<I2C slave rx buffer size */

#define ESP_SLAVE1_ADDR                     0x33             /*!< ESP32 slave address, you can set any 7bit value */
#define ESP_SLAVE2_ADDR                     0x27             /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

SemaphoreHandle_t print_mux = NULL;

/**
 * @brief i2c slave initialization
 */
static void i2c_example_slave_init(i2c_port_t port, uint16_t slave_addr, int sda_io_num, int scl_io_num)
{
    int i2c_slave_port = port;
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = sda_io_num;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = scl_io_num;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = slave_addr;
    i2c_param_config(i2c_slave_port, &conf_slave);
    i2c_driver_install(i2c_slave_port, conf_slave.mode,
                       I2C_EXAMPLE_SLAVE_RX_BUF_LEN,
                       I2C_EXAMPLE_SLAVE_TX_BUF_LEN, 0);
}

/**
 * @brief test function to show buffer
 */
/*static void disp_buf(uint8_t* buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if (( i + 1 ) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}*/

static void i2c_test_task(void* arg)
{
    int i = 0;
    //int ret;
    uint32_t task_idx = (uint32_t) arg;
    uint8_t* data1 = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t* data2 = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t* data_play;
    int size1, size2, size_play;
        //---------------------------------------------------
        //xSemaphoreTake(print_mux, portMAX_DELAY);
        //we need to fill  the slave buffer so that master can read later
    while(1){
		for (i = 0; i <= RW_TEST_LENGTH; i++){
		   data1[i]=0;
		   data2[i]=0;
		}

		size1 = i2c_slave_read_buffer(I2C_EXAMPLE_SLAVE1_NUM, data1, RW_TEST_LENGTH, 1000 / portTICK_RATE_MS);
		size2 = i2c_slave_read_buffer(I2C_EXAMPLE_SLAVE2_NUM, data2, RW_TEST_LENGTH, 1000 / portTICK_RATE_MS);
		printf("*********************************************************\n");
		printf("----Slave_id[1] Slave read: [%d] bytes ----\n", size1);
		printf("----Slave_id[2] Slave read: [%d] bytes ----\n", size2);
		
		for (int j=1; j<=2; j++){

			if (j==1){
				size_play=size1;
				data_play=data1;
			} else {
			    size_play=size2;
			    data_play=data2;
			}

			if (size_play>0){
				printf("\r\n===============================DATA [%d]=====================\r\n",j);
				printf(" Slave % d read data: %s\r\n", j , data_play);

				for (i = 0; i < size_play; i++){
					data_play[i]=data_play[i]+32;
				}

				if (j==2){
					for (i = 0; i < size_play; i++){
						data_play[i]=data_play[size_play-i-1];
					}
				}

				printf("\r\n\r\nSlave[%d] sending new data to Master.....: %s",j,data_play);

				printf("\r\n===============================END [%d]=====================\r\n",j);


				i2c_port_t port_sel;
				if (j==1){
					port_sel=I2C_NUM_0;
				} else {
					port_sel=I2C_NUM_1;
				}
				size_t d_size = i2c_slave_write_buffer(port_sel, data_play, size_play, 1000 / portTICK_RATE_MS);


				if (d_size == 0) {
					printf("i2c slave tx buffer full\n");
				}
			}
		} //for
		vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);
	} //while (1)
} //i2c_test_task

void app_main()
{
    print_mux = xSemaphoreCreateMutex();
    i2c_example_slave_init(I2C_EXAMPLE_SLAVE1_NUM,ESP_SLAVE1_ADDR,I2C_EXAMPLE_SLAVE1_SDA_IO,I2C_EXAMPLE_SLAVE1_SCL_IO);
    i2c_example_slave_init(I2C_EXAMPLE_SLAVE2_NUM,ESP_SLAVE2_ADDR,I2C_EXAMPLE_SLAVE2_SDA_IO,I2C_EXAMPLE_SLAVE2_SCL_IO);
    xTaskCreate(i2c_test_task, "i2c_test_task", 1024 * 2, (void* ) 0, 10, NULL);
}
