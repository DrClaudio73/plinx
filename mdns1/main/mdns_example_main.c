/* MDNS-SD Query and advertise Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mdns.h"
#include "driver/gpio.h"
//#include <sys/socket.h>
#include "lwip/sockets.h"
//#include <netdb.h>

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

#define EXAMPLE_MDNS_HOSTNAME CONFIG_MDNS_HOSTNAME
#define EXAMPLE_MDNS_INSTANCE CONFIG_MDNS_INSTANCE

#define MAXLINE 80

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int IP4_CONNECTED_BIT = BIT0;
const int IP6_CONNECTED_BIT = BIT1;

static const char *TAG = "mdns-test";
static bool auto_reconnect = true;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        printf("\n\n\n SYSTEM_EVENT_STA_START\n\n\n\r\n");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, IP4_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        xEventGroupSetBits(wifi_event_group, IP6_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        if (auto_reconnect) {
            esp_wifi_connect();
        }
        xEventGroupClearBits(wifi_event_group, IP4_CONNECTED_BIT | IP6_CONNECTED_BIT);
        break;
    default:
        break;
    }
    mdns_handle_system_event(ctx, event);
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void initialise_mdns(void)
{
    //initialize mDNS
    ESP_ERROR_CHECK( mdns_init() );
    //set mDNS hostname (required if you want to advertise services)
    ESP_ERROR_CHECK( mdns_hostname_set(EXAMPLE_MDNS_HOSTNAME) );
    //set default mDNS instance name
    ESP_ERROR_CHECK( mdns_instance_name_set(EXAMPLE_MDNS_INSTANCE) );

    //structure with TXT records
    mdns_txt_item_t serviceTxtData[3] = {
        {"board","esp32"},
        {"u","user"},
        {"p","password"}
    };

    //initialize service
    ESP_ERROR_CHECK( mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, serviceTxtData, 3) );
    //add another TXT item
    ESP_ERROR_CHECK( mdns_service_txt_item_set("_http", "_tcp", "path", "/foobar") );
    //change TXT item value
    ESP_ERROR_CHECK( mdns_service_txt_item_set("_http", "_tcp", "u", "admin") );
}

static const char * if_str[] = {"STA", "AP", "ETH", "MAX"};
static const char * ip_protocol_str[] = {"V4", "V6", "MAX"};

static void mdns_print_results(mdns_result_t * results){
    mdns_result_t * r = results;
    mdns_ip_addr_t * a = NULL;
    int i = 1, t;
    while(r){
        printf("%d: Interface: %s, Type: %s\n", i++, if_str[r->tcpip_if], ip_protocol_str[r->ip_protocol]);
        if(r->instance_name){
            printf("  PTR : %s\n", r->instance_name);
        }
        if(r->hostname){
            printf("  SRV : %s.local:%u\n", r->hostname, r->port);
        }
        if(r->txt_count){
            printf("  TXT : [%d] ", r->txt_count);
            //mdns_txt_item_t* tct = r->txt;
            //printf("%s",tct->key);

            for(t=0; t<r->txt_count; t++){
            	if (r->txt[t].value==NULL){
            		printf("%s=; ", r->txt[t].key);
            	}
            	else{
            		printf("%s=%s; ", r->txt[t].key, r->txt[t].value);
            	}
            }
            printf("\n");
        }
        a = r->addr;
        while(a){
        	//printf("a->addr.type: %d\r\n",a->addr.type);
            if(a->addr.type == 6 /*MDNS_IP_PROTOCOL_V6*/){
                printf("  AAAA: " IPV6STR "\n", IPV62STR(a->addr.u_addr.ip6));
            } else {
                printf("  A   : " IPSTR "\n", IP2STR(&(a->addr.u_addr.ip4)));
            }
            a = a->next;
        }
        r = r->next;
    }

}

static void query_mdns_service(const char * service_name, const char * proto)
{
    ESP_LOGI(TAG, "\r\nQuery PTR: %s.%s.local\r\n", service_name, proto);

    mdns_result_t * results = NULL;
    esp_err_t err = mdns_query_ptr(service_name, proto, 3000, 20,  &results);
    if(err){
        ESP_LOGE(TAG, "Query Failed: %s", esp_err_to_name(err));
        return;
    }
    if(!results){
        ESP_LOGW(TAG, "No results found!");
        return;
    }

    mdns_print_results(results);
    mdns_query_results_free(results);
}

static void query_mdns_host(const char * host_name, char* host_IP)
{
    ESP_LOGI(TAG, "\r\n\r\nQuery A: %s.local\r\n\r\n", host_name);

    struct ip4_addr addr;
    addr.addr = 0;

    esp_err_t err = mdns_query_a(host_name, 2000,  &addr);
    if(err){
        if(err == ESP_ERR_NOT_FOUND){
            ESP_LOGW(TAG, "%s: Host was not found!", esp_err_to_name(err));
            return;
        }
        ESP_LOGE(TAG, "Query Failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, IPSTR, IP2STR(&addr));
    printf("\r\nIndirizzo risolto: ");
    printf(IPSTR, IP2STR(&addr));
    printf("\n");
    snprintf(host_IP, MAXLINE, IPSTR, IP2STR(&addr));
}

static void initialise_button(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pin_bit_mask = 1;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
}


static void check_button(void)
{
    //static bool old_level = true;
    //bool new_level = gpio_get_level(GPIO_NUM_0);
    int sock_fd;
    int nread=0;
    struct sockaddr_in serv_add;
    char buffer[MAXLINE], host_IP[MAXLINE];
    memset(host_IP,0,sizeof(host_IP));
 
    if (1) {
        //query_mdns_host("claudioPC",host_IP);
	memset(host_IP,0,sizeof(host_IP));
        query_mdns_host("ilmioesp32mdns0",host_IP);
        //query_mdns_service("_arduino", "_tcp");
        query_mdns_service("_http", "_tcp");
        //query_mdns_service("_printer", "_tcp");
        //query_mdns_service("_ipp", "_tcp");
        //query_mdns_service("_afpovertcp", "_tcp");
        //query_mdns_service("_smb", "_tcp");
        query_mdns_service("_ftp", "_tcp");
        //query_mdns_service("_nfs", "_tcp");
    }
    //old_level = new_level;
    /* create connection socket */
    if ( (sock_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
	perror("Socket creation error");
	//return -1;
	close(sock_fd);
	return;
    }
    printf("Socket created for: %s\n",host_IP);
    /* initialize address */
    memset((void *) &serv_add, 0, sizeof(serv_add)); /* clear server address */
    serv_add.sin_family = AF_INET;                   /* address type is INET */
    serv_add.sin_port = htons(80);                   /* daytime port is 13 */
    /* build address using inet_pton */
    if ( (inet_pton(AF_INET, host_IP, &serv_add.sin_addr)) <= 0) {
	perror("Address creation error");
	close(sock_fd);
	return ;
    }
    printf("Address created\n");
    /* extablish connection */
    if (connect(sock_fd, (struct sockaddr *)&serv_add, sizeof(serv_add)) < 0) {
	perror("Connection error");
	close(sock_fd);
	return ;
    }
    printf("Connection created\n");

    // read daytime from server 
    printf("\nprinting received msg...\n");
    while ( (nread = read(sock_fd, buffer, MAXLINE)) > 0) {
	printf("read %d bytes from socket\n", nread);
	buffer[nread]=0;
	if (fputs(buffer,stdout) == EOF) {           //write daytime 
	    perror("fputs error");
	    close(sock_fd);
	    return ;
         }
    }
    printf("\nend of received msg...\n");

/*
    int total=0,r,i;
    printf("\nprinting received msg...\n");
    do {
	bzero(buffer, sizeof(buffer));
	r = read(sock_fd, buffer, sizeof(buffer) - 1);
	total+=r;
	for(int i = 0; i < r; i++) {
	   putchar(buffer[i]);
	}
    } while(r > 0);
   printf("\nend of received msg...\n");
 */	
    /* error on read */
    if (nread < 0) {
	perror("Read error");
	close(sock_fd);
	return ;
    }
    /* normal exit */
    printf("\nnormal exit\n");
    close(sock_fd);
    return ;

}

static void mdns_example_task(void *pvParameters)
{
    /* Wait for the callback to set the CONNECTED_BIT in the event group. */
    xEventGroupWaitBits(wifi_event_group, IP4_CONNECTED_BIT | IP6_CONNECTED_BIT,
                     false, true, portMAX_DELAY);
    while(1) {
        check_button();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_mdns();
    initialise_wifi();
    initialise_button();
    xTaskCreate(&mdns_example_task, "mdns_example_task", 2048, NULL, 5, NULL);
}
