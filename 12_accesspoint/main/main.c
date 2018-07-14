#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "mdns.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
//#include "sys/socket.h"
//#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "esp_partition.h"
#include "esp_err.h"

// set AP CONFIG values
#ifdef CONFIG_AP_HIDE_SSID
	#define CONFIG_AP_SSID_HIDDEN 1
#else
	#define CONFIG_AP_SSID_HIDDEN 0
#endif	
#ifdef CONFIG_WIFI_AUTH_OPEN
	#define CONFIG_AP_AUTHMODE WIFI_AUTH_OPEN
#endif
#ifdef CONFIG_WIFI_AUTH_WEP
	#define CONFIG_AP_AUTHMODE WIFI_AUTH_WEP
#endif
#ifdef CONFIG_WIFI_AUTH_WPA_PSK
	#define CONFIG_AP_AUTHMODE WIFI_AUTH_WPA_PSK
#endif
#ifdef CONFIG_WIFI_AUTH_WPA2_PSK
	#define CONFIG_AP_AUTHMODE WIFI_AUTH_WPA2_PSK
#endif
#ifdef CONFIG_WIFI_AUTH_WPA_WPA2_PSK
	#define CONFIG_AP_AUTHMODE WIFI_AUTH_WPA_WPA2_PSK
#endif
#ifdef CONFIG_WIFI_AUTH_WPA2_ENTERPRISE
	#define CONFIG_AP_AUTHMODE WIFI_AUTH_WPA2_ENTERPRISE
#endif

//mdns
#define EXAMPLE_MDNS_HOSTNAME CONFIG_MDNS_HOSTNAME
#define EXAMPLE_MDNS_INSTANCE CONFIG_MDNS_INSTANCE
static const char *TAG = "mdns-test";

// Event group
static EventGroupHandle_t event_group;
const int STA_CONNECTED_BIT = BIT0;
const int STA_DISCONNECTED_BIT = BIT1;
const int AP_STARTED_BIT = BIT2;
int restart_mdns=1;

nvs_handle my_nvs_handle;
int resets_num=0;
int write_errs_num=0;

// AP event handler
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
		
    case SYSTEM_EVENT_AP_START:
		printf("\nAccess point started\n");
		xEventGroupSetBits(event_group, AP_STARTED_BIT);
		break;
		
	case SYSTEM_EVENT_AP_STACONNECTED:
		printf("\nNew STA disconnected\n");
		xEventGroupSetBits(event_group, STA_CONNECTED_BIT);
		tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_AP);
		break;

	case SYSTEM_EVENT_AP_STADISCONNECTED:
		printf("\nSTA disconnected\n");
		xEventGroupSetBits(event_group, STA_DISCONNECTED_BIT);
		break;		
    
	default:
        break;
    }
    mdns_handle_system_event(ctx, event);
	return ESP_OK;
}


// print the list of connected stations
void printStationList() 
{
	printf(" Connected stations:\n");
	printf("--------------------------------------------------\n");
	
	wifi_sta_list_t wifi_sta_list;
	tcpip_adapter_sta_list_t adapter_sta_list;
   
	memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
	memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));
   
	ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&wifi_sta_list));	
	ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list));
	
	for(int i = 0; i < adapter_sta_list.num; i++) {
		
		tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
         printf("%d - mac: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x - IP: %s\n", i + 1,
				station.mac[0], station.mac[1], station.mac[2],
				station.mac[3], station.mac[4], station.mac[5],
				ip4addr_ntoa(&(station.ip)));
	}
	
	printf("\n");
}


// Monitor task, receive Wifi AP events
void monitor_task(void *pvParameter)
{
	while(1) {
		EventBits_t staBits = xEventGroupWaitBits(event_group, STA_CONNECTED_BIT | STA_DISCONNECTED_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
		if((staBits & STA_CONNECTED_BIT) != 0) printf("New station connected\n\n");
		else printf("A station disconnected\n\n");
	}
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

static void query_mdns_host(const char * host_name)
{
    ESP_LOGI(TAG, "\r\n\r\nQuery A: %s.local\r\n\r\n", host_name);

    struct ip4_addr addr;
    addr.addr = 0;

    esp_err_t err = mdns_query_a(host_name, 2000,  &addr);
    if(err){
        if(err == ESP_ERR_NOT_FOUND){
            ESP_LOGW(TAG, "%s: Host was not found!", esp_err_to_name(err));
    		printf("\r\nmdns will not be freed...\r\n");
            //mdns_free();
            //restart_mdns=1;
            return;
        }
        ESP_LOGE(TAG, "Query Failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, IPSTR, IP2STR(&addr));
}

static void check_Miomdns(void)
{
    //static bool old_level = true;
    //bool new_level = gpio_get_level(GPIO_NUM_0);
	query_mdns_host("claudioPC");
	if (restart_mdns!=0){
		printf("\r\nmdns has been freed reinitializing it...\r\n");
		initialise_mdns();
		restart_mdns=0;
	}
	query_mdns_host("ilmioesp32mdns1");
	query_mdns_service("_arduino", "_tcp");
	query_mdns_service("_http", "_tcp");
   // query_mdns_service("_printer", "_tcp");
	//query_mdns_service("_ipp", "_tcp");
	//query_mdns_service("_afpovertcp", "_tcp");
   // query_mdns_service("_smb", "_tcp");
	query_mdns_service("_ftp", "_tcp");
   // query_mdns_service("_nfs", "_tcp");
//old_level = new_level;
}


void testEndianess()
{
    int i, val = 0xABCDEF01;
    char * ptr;

    printf("Using value %X\n", val);
    ptr = (char *) &val;
    for (i=0; i<sizeof(int); i++) {
	printf("val[%d]=%2hhX\n", i, ptr[i]);
    }
}

void dayTimeServerTask(void *pvParameter){
/*
 * Variables definition
 */
    int list_fd, conn_fd;
    int rcvReqsNum=0;
    struct sockaddr_in serv_add;
    char buffer[599];
    time_t timeval;

    /* create socket */
    if ( (list_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		perror("Socket creation error");
		//exit(-1);
    }
    /* initialize address */
    memset((void *)&serv_add, 0, sizeof(serv_add)); /* clear server address */
    serv_add.sin_family = AF_INET;                  /* address type is INET */
    serv_add.sin_port = htons(80);                  /* daytime port is 13 */
    serv_add.sin_addr.s_addr = htonl(INADDR_ANY);   /* connect from anywhere */
    /* bind socket */
    if (bind(list_fd, (struct sockaddr *)&serv_add, sizeof(serv_add)) < 0) {
		printf("bind error\n");
		close(list_fd);
		return;
		//exit(-1);
    }
    /* listen on socket */
    if (listen(list_fd, 20) < 0 ) {;
		printf("listen error\n");
		close(list_fd);
		return;
		//exit(-1);
    }
    /* write daytime to client */
    struct sockaddr_in cli_add;
    in_addr_t cli_in_addr;

    socklen_t size_cli_addr;

    while (1) {
    	//size_cli_addr=sizeof(cli_in_addr);
		if ( (conn_fd = accept(list_fd, (struct sockaddr *)&cli_add, &size_cli_addr)) <0 ) {
			printf("accept error\n");
			close(conn_fd);
			continue;
			//exit(-1);
		}
		cli_in_addr=cli_add.sin_addr.s_addr;
		char bufcliaddr[50];
		memset(bufcliaddr,0,sizeof(bufcliaddr));

		printf("cli_add->len: %d\n", cli_add.sin_len);
		printf("cli_add->sin_family: %d\n", ntohs(cli_add.sin_family));
		printf("cli_add->sin_port: %d\n", ntohs(cli_add.sin_port));
		inet_ntop(AF_INET,&cli_in_addr,&bufcliaddr,sizeof(bufcliaddr));
		printf("cli_add->len: %s\n", bufcliaddr);

		timeval = time(NULL);
		rcvReqsNum+=1;
		char timestrg[800];
		memset(timestrg,0,sizeof(timestrg));
		snprintf(timestrg, sizeof(timestrg),"%.24s", ctime(&timeval));
		snprintf(buffer, sizeof(buffer), "HTTP/1.1 200 OK\nServer: esp32\nContent-Type: text/html\nConnection: close\r\n\r\n<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><title>Instant Update Project by ccattaneo</title></head><body><header><h1 style=\"color:green;\">%.24s</h1>\r\n\r\n</header><footer><p>Received %d requests</p><p>Reset %d times</p><p>Write %d errors</p></footer></body></html>\r\n", timestrg, rcvReqsNum, resets_num, write_errs_num);
		//printf("\r\nSending day time to Client: %.24s\r\nReceived %d requests\r\n", timestrg,rcvReqsNum);
		printf("%s", buffer);

		/*
		time_t now;
		time(&now);
		printf("current time: %d\r\n",(int) now);
		struct tm timeinfo;

		localtime_r(&now,&timeinfo);
		char buffertime[100];
		strftime(buffertime, sizeof(buffertime), "%d/%m/%Y %H:%M:%S", &timeinfo);
		printf("\r\nToday is %s\r\n", buffertime);
		*/
		printf("printing received msg\n");

		int r;
		char recv_buf[2000];
		//do {
		  memset(recv_buf, 0, sizeof(recv_buf));
		  r = read(conn_fd, recv_buf, sizeof(recv_buf) - 1);
		  printf("\n-----------------------------------\nread returned with %d values read\n-----------------------------------\n",r);
		  for(int i = 0; i < r; i++) {
		     putchar(recv_buf[i]);
		  }
		//} while(r < 0); //attenzione non dovrebbe mai iterare!!!!!!!!!!

		printf("sending reply msg\n");

		if ( (write(conn_fd, buffer, strlen(buffer))) < 0 ) {
			printf("write error\n");
			/*
			write_errs_num=write_errs_num+1;
			//check number of write errors
		    esp_err_t err = nvs_get_i32(my_nvs_handle, "write_errs_num" , &write_errs_num);
			if(err != ESP_OK) {
				if(err == ESP_ERR_NVS_NOT_FOUND) printf("\nKey %s not found\n", "write_errs_num");
				else printf("\nError in nvs_get_i32! (%04X)\n", err);
			}
			printf("\nValue stored in NVS for key %s is %d\n", "write_errs_num", write_errs_num);

			resets_num=resets_num+1;

		    err = nvs_set_i32(my_nvs_handle, "write_errs_num", write_errs_num);

		    if (err != ESP_OK) {
				printf("\nError in nvs_set_i32! (%04X)\n", err);
			}

			err = nvs_commit(my_nvs_handle);
			if (err != ESP_OK) {
				printf("\nError in commit! (%04X)\n", err);
			}

			printf("\nValue %d stored in NVS with key %s\n", write_errs_num, "write_errs_num");
			*/
			//close(conn_fd);
			//return;
			//exit(-1);
		}
		close(conn_fd);
		//close(list_fd);
    } //while (1)
    close(list_fd);
    /* normal exit */
    //exit(0);
}

// Station list task, print station list and query mdns results every 5 seconds
void station_list_task(void *pvParameter)
{
	while(1) {
		EventBits_t apBits = xEventGroupWaitBits(event_group, AP_STARTED_BIT, pdFALSE, pdFALSE, 100 / portTICK_RATE_MS/*portMAX_DELAY*/);
		printStationList();
		printf("\r\nevent_group =%x \r\n", (uint8_t) apBits);
		//if((apBits & AP_STARTED_BIT) != 0) {check_Miomdns();}
		//else {printf("Not STARTED AP mdns query avoided!!!!!");}
		//testEndianess();
		vTaskDelay(5000 / portTICK_RATE_MS);
	}
}

// Main application
void app_main()
{	
	// disable the default wifi logging
	//esp_log_level_set("wifi", ESP_LOG_NONE);
	
	// create the event group to handle wifi events
	event_group = xEventGroupCreate();
	
	// initialize NVS
	ESP_ERROR_CHECK(nvs_flash_init());

	// initialize the mdns service
	initialise_mdns();
	restart_mdns=0;

	//set time
    struct timeval now;
    int rc;

    now.tv_sec=1539997999 /*866208142*/;
    now.tv_usec=290944;
    rc=settimeofday(&now, NULL);
    if(rc==0) {
        printf("settimeofday() successful.\n");
    }
    else {
        printf("settimeofday() failed, errno = %d\n",errno);
    }

	// initialize the tcp stack
	tcpip_adapter_init();

	// stop DHCP server
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
	
	// assign a static IP to the network interface
	tcpip_adapter_ip_info_t info;
	char dest[50];
	memset(dest,0,sizeof(dest));

    memset(&info, 0, sizeof(info));
	IP4_ADDR(&info.ip, 192, 168, 10, 1);
    IP4_ADDR(&info.gw, 192, 168, 10, 1);
    IP4_ADDR(&info.netmask, 255, 255, 255, 0);
	ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));
	
	inet_ntop(AF_INET, &info.ip.addr, &dest, sizeof(dest));
	printf("\r\nsetting IP of AP: %s\r\n",dest);

	// start the DHCP server   
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
	// initialize the wifi event handler
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	
	// initialize the wifi stack in AccessPoint mode with config in RAM
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

	// configure the wifi connection and start the interface
	wifi_config_t ap_config = {
        .ap = {
            .ssid = CONFIG_AP_SSID,
            .password = CONFIG_AP_PASSWORD,
			.ssid_len = 0,
			.channel = CONFIG_AP_CHANNEL,
			.authmode = CONFIG_AP_AUTHMODE,
			.ssid_hidden = CONFIG_AP_SSID_HIDDEN,
			.max_connection = CONFIG_AP_MAX_CONNECTIONS,
			.beacon_interval = CONFIG_AP_BEACON_INTERVAL,			
        },
    };
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    
	// start the wifi interface
	ESP_ERROR_CHECK(esp_wifi_start());
	printf("Starting access point, SSID=%s\n", CONFIG_AP_SSID);
	// open the partition in RW mode
	esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_nvs_handle);
	if (err != ESP_OK) {
		printf("FATAL ERROR: Unable to open NVS\n");
		while(1) vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	printf("NVS open OK\n");
	
	//check number of restarts
    err = nvs_get_i32(my_nvs_handle, "resets_num" , &resets_num);
	if(err != ESP_OK) {
		if(err == ESP_ERR_NVS_NOT_FOUND) printf("\nKey %s not found\n", "resets_num");
			else printf("\nError in nvs_get_i32! (%04X)\n", err);
			//return;
	}
	printf("\nValue stored in NVS for key %s is %d\n", "resets_num", resets_num);

	resets_num=resets_num+1;

    err = nvs_set_i32(my_nvs_handle, "resets_num", resets_num);

    if (err != ESP_OK) {
		printf("\nError in nvs_set_i32! (%04X)\n", err);
	}

	err = nvs_commit(my_nvs_handle);
	if (err != ESP_OK) {
		printf("\nError in commit! (%04X)\n", err);
			//return;
	}

	printf("\nValue %d stored in NVS with key %s\n", resets_num, "resets_num");

	// start the main task
    //xTaskCreate(&monitor_task, "monitor_task", 2048, NULL, 5, NULL);
	//xTaskCreate(&station_list_task, "station_list_task", 2048, NULL, 6, NULL);
	xTaskCreate(&dayTimeServerTask, "daytime_server_task", 8192, NULL, 7, NULL);
}
