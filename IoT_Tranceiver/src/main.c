#include <stdio.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/spsc_lockfree.h>
#include <zephyr/drivers/flash.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/net/dns_resolve.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/sntp.h>
#include <zephyr/drivers/cellular.h>


#define SNTP_SERVER		("216.239.35.4") // Google's public NTP server time.google.com
// #define SAMPLE_TEST_ENDPOINT_UDP_ECHO_PORT	(7780)
// #define SAMPLE_TEST_ENDPOINT_UDP_RECEIVE_PORT	(7781)
// #define SAMPLE_TEST_PACKET_SIZE			(1024)
// #define SAMPLE_TEST_ECHO_PACKETS		(16)
// #define SAMPLE_TEST_TRANSMIT_PACKETS		(128)


int err, ret;
const struct device *modem = DEVICE_DT_GET(DT_ALIAS(modem));
struct sntp_time ts;
struct sockaddr_in sntp_server;
// const struct device *flash_dev = device_get_binding(DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);

const struct bt_le_scan_param scan_param = BT_LE_SCAN_PARAM_INIT(BT_LE_SCAN_TYPE_PASSIVE, BT_LE_SCAN_OPT_FILTER_DUPLICATE, BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_WINDOW);
const struct bt_le_scan_param scan_paramRunning = BT_LE_SCAN_PARAM_INIT(BT_LE_SCAN_TYPE_PASSIVE, BT_LE_SCAN_OPT_FILTER_ACCEPT_LIST, BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_WINDOW);





typedef struct 
{
  uint8_t noIdea[5];
  uint16_t companyID;
  uint8_t frameType;
  uint8_t config[3];
  uint8_t seqID;
  uint16_t counter;
  uint16_t batteryLvl;
  uint16_t adc_val1;
  uint16_t adc_val2;
  bt_addr_le_t addr;
  int8_t rssi;
}__attribute__((packed)) makeen_data_ADC;

typedef struct
{
  uint8_t seqID;
  uint16_t runtimeCounter;
  uint16_t batteryLvl;
  uint16_t adc_val1;
  uint16_t adc_val2;
  bt_addr_le_t addr;
  int8_t rssi;
  uint16_t timestamp;
}__attribute__((packed)) makeen_data_filtered;


SPSC_DEFINE(spscBuf1, makeen_data_ADC, 16);
SPSC_DEFINE(spscBuf2, makeen_data_filtered, 16);



static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, struct net_buf_simple *buf)
{ 
  uint8_t * pData=(uint8_t*) buf->data;
  makeen_data_ADC *mdata=(makeen_data_ADC*) pData;
  
  mdata->batteryLvl = mdata->batteryLvl>>8 | mdata->batteryLvl<<8; // flip bytes in batteryLvl
  if (mdata->companyID == 0x017F && mdata->config[1] == 0x03)
  {
    mdata->addr = *addr;
    mdata->rssi = rssi;
    makeen_data_ADC * empty_spot_pointer = spsc_acquire(&spscBuf1);
    if (empty_spot_pointer != NULL)
    {
      memcpy(empty_spot_pointer, mdata, sizeof(makeen_data_ADC));
      spsc_produce(&spscBuf1);      
    } else {
      printk("No space in spscBuf1\n");
    } 
  }
}



void bt_rec_task(void)
{
  printk("Bluetooth receiving task started\n");
  while (1)
  {
    while (!bt_is_ready())
    {
      printk("Starting Scanner\n");
      err = bt_enable(NULL);
      if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
      }
      err = bt_le_scan_start(&scan_param, scan_cb);
      if (err) {
        printk("Bluetooth scan_start failed (err %d)\n", err);
      }
    } 
    if (bt_is_ready()) {
      printk("Bluetooth initialized - killing thread\n");
      k_thread_abort(k_current_get());
    }
  }
}


void timestamp_task(void)
{
  printk("Timestamping task started\n");
  //Connect to LTE
  pm_device_action_run(modem, PM_DEVICE_ACTION_RESUME); //Turn on modem


  

  while (1)
  {
    sntp_server.sin_family = AF_INET;
    sntp_server.sin_port = htons(123);
    ret = net_addr_pton(AF_INET, SNTP_SERVER, &sntp_server.sin_addr);
    if (ret < 0) {
      printk("Invalid address: %d\n", ret);
      // printk(sntp_server.sin_addr);
      printk("\n");
    }
    // ret = sntp_simple(SNTP_SERVER, 5000, &ts);

    ret = sntp_init(NULL, (struct sockaddr *)&sntp_server, sizeof(sntp_server));
    if (ret < 0) {
      printk("SNTP init failed: %d\n", ret);
    } else {
      printk("SNTP init succeeded\n");
    }
  

  k_sleep(K_SECONDS(1));
  if (ret < 0) {
    printk("SNTP query failed: %d\n", ret);
  } else {
    printk("SNTP query succeeded\n");
    printk("Timestamp: %u.%06u\n", ts.seconds, ts.fraction);
  }
    // if (spsc_consumable(&spscBuf1))
    // {
    //   makeen_data_ADC * dataptr = spsc_consume(&spscBuf1);
    //   if (dataptr != NULL)
    //   {
    //     makeen_data_filtered *tmpStruct = k_malloc(sizeof(makeen_data_filtered));
    //     tmpStruct->seqID = dataptr->seqID;
    //     tmpStruct->runtimeCounter = dataptr->counter;
    //     tmpStruct->batteryLvl = dataptr->batteryLvl;
    //     tmpStruct->adc_val1 = dataptr->adc_val1;
    //     tmpStruct->adc_val2 = dataptr->adc_val2;
    //     tmpStruct->addr = dataptr->addr;
    //     tmpStruct->rssi = dataptr->rssi;

    //     tmpStruct->timestamp = k_uptime_get_32();

    //     spsc_release(&spscBuf1);

    //     makeen_data_filtered * empty_spot_pointer = spsc_acquire(&spscBuf2);
    //     if (empty_spot_pointer != NULL)
    //     {
    //       memcpy(empty_spot_pointer, tmpStruct, sizeof(makeen_data_filtered));
    //       spsc_produce(&spscBuf2);      
    //     } else {
    //       printk("No space in spscBuf2\n");
    //     } 
    //   }
    // } else {
    //   k_yield();
    // }
  }
}


void decipher_task(void)
{
  printk("Decipher task started\n");

  while (1) {
    if (spsc_consumable(&spscBuf2) > 0)
    {
      makeen_data_filtered * dataptr = spsc_consume(&spscBuf2);
      if (dataptr != NULL)
      {

        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&dataptr->addr, addr_str, sizeof(addr_str));
        printk("Addr: %s\n", addr_str);
        printk("SeqID: %d\n", dataptr->seqID);
        printk("RuntimeCounter: %d\n", dataptr->runtimeCounter);
        printk("BatteryLvl: %d\n", dataptr->batteryLvl);
        printk("ADC1: %d\n", dataptr->adc_val1);
        printk("ADC2: %d\n", dataptr->adc_val2);
        printk("RSSI: %d\n", dataptr->rssi);
        printk("Timestamp: %d\n", dataptr->timestamp);
        spsc_release(&spscBuf2);
      }
    } else {
      k_yield();
    }
  }
}


void MQTT_task(void)
{
  printk("MQTT task started\n");
  while (1)
  {
    if (true) {
      //TODO FLASH STORAGE
      // flash_write(flash_dev, 0, "Hello", 5);
    }
  }
}

#define STACKSIZE 1024
#define PRIORITY 7
K_THREAD_DEFINE(bluetooth_task_id, STACKSIZE*2, bt_rec_task, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(timestamp_task_id, STACKSIZE*2, timestamp_task, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(decipher_task_id, STACKSIZE, decipher_task, NULL, NULL, NULL, 7, 0, 0);
// K_THREAD_DEFINE(MQTT_task_id, STACKSIZE, MQTT_task, NULL, NULL, NULL, 7, 0, 0);
