#include <stdio.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/spsc_lockfree.h>



int err;




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
  uint8_t rssi;
  uint16_t timestamp;
}__attribute__((packed)) makeen_data_filtered;


SPSC_DEFINE(spscBuf1, makeen_data_ADC, 16);
SPSC_DEFINE(spscBuf2, makeen_data_filtered, 16);



static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, struct net_buf_simple *buf)
{
  // printk("Scan callback\n");
  // char addr_str[BT_ADDR_LE_STR_LEN];
  // bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

  
  
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




#if 1
int main(void) {

  

  printk("Starting Scanner - main\n");

  err = bt_enable(NULL);
  
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return 0;
  } else {
    printk("Bluetooth initialized\n");
  }


  err = bt_le_scan_start(&scan_param, scan_cb);
   if (err) {
    printk("Bluetooth scan_start failed (err %d)\n", err);
    return 0;
  } else {
    printk("Bluetooth scan started\n");
  }


}
#endif

void bt_rec_task(void)
{
  printk("Bluetooth receiving task started\n");
  // while (!bt_is_ready())
  // {
  //   printk("Starting Scanner\n");
  //   err = bt_enable(NULL);
  //   if (err) {
  //     printk("Bluetooth init failed (err %d)\n", err);
  //   }
  //   err = bt_le_scan_start(&scan_param, scan_cb);
  //   if (err) {
  //     printk("Bluetooth scan_start failed (err %d)\n", err);
  //   }
    
  //   k_msleep(100);
  // } 
  if (bt_is_ready()) {
    printk("Bluetooth initialized - killing thread\n");
    // k_thread_suspend(k_current_get());
    k_thread_abort(k_current_get());
  }

  // while (1)
  // {

  //   printk("Scanning...\n");
  //   k_msleep(100);
  // }

  
}


void timestamp_task(void)
{
  printk("Timestamping task started\n");
  //Connect to LTE
  //TODO

  while (1)
  {
    if (spsc_consumable(&spscBuf1))
    {
      printk("Consumable available for timestamping\n");
      makeen_data_ADC * dataptr = spsc_consume(&spscBuf1);
      if (dataptr != NULL)
      {
        // printk("dataptr not null\n");
        makeen_data_filtered *tmpStruct = k_malloc(sizeof(makeen_data_filtered));
        tmpStruct->seqID = dataptr->seqID;
        tmpStruct->runtimeCounter = dataptr->counter;
        tmpStruct->batteryLvl = dataptr->batteryLvl;
        tmpStruct->adc_val1 = dataptr->adc_val1;
        tmpStruct->adc_val2 = dataptr->adc_val2;
        tmpStruct->addr = dataptr->addr;
        tmpStruct->rssi = dataptr->rssi;

        tmpStruct->timestamp = k_uptime_get_32();

        spsc_release(&spscBuf1);

        makeen_data_filtered * empty_spot_pointer = spsc_acquire(&spscBuf2);
        if (empty_spot_pointer != NULL)
        {
          memcpy(empty_spot_pointer, tmpStruct, sizeof(makeen_data_filtered));
          spsc_produce(&spscBuf2);      
        } else {
          printk("No space in spscBuf2\n");
        } 
      }
    } else {
      // printk("No consumable available for timestamping\n");
      //release the thread
      //TODO

      k_msleep(100);

      
    }
  }
}




void decipher_task(void)
{
  printk("Deciphering\n");

  while (1) {
    if (spsc_consumable(&spscBuf2) > 0)
    {
      // printk("Consumable available for deciphering\n");
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
      // printk("No consumable available for deciphering\n");
      //release the thread
      //TODO
      k_msleep(100);
    }
  }
}

#define STACKSIZE 1024
#define PRIORITY 7
K_THREAD_DEFINE(bluetooth_task, STACKSIZE, bt_rec_task, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(timestamp_task_id, STACKSIZE, timestamp_task, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(decipher_task_id, STACKSIZE, decipher_task, NULL, NULL, NULL, 7, 0, 0);
