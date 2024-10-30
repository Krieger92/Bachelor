#include <stdio.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>



bt_addr_le_t macAdresseGeneric, macAdresses[10];
int err;
uint8_t seqIDs [10]={0};

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
  uint16_t x_freq;
  uint16_t x_vel;
  uint16_t y_freq;
  uint16_t y_vel;
  uint16_t z_freq;
  uint16_t z_vel;
}__attribute__((packed)) makeen_data;



static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, struct net_buf_simple *buf)
{
  char addr_str[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

  
  uint8_t * pData=(uint8_t*) buf->data;
  makeen_data *mdata=(makeen_data*) pData;
  mdata->companyID = mdata->companyID>>8 | mdata->companyID<<8; // flip bytes in companyID
  mdata->batteryLvl = mdata->batteryLvl>>8 | mdata->batteryLvl<<8; // flip bytes in batteryLvl

  //print the mac address and the rssi
  printk("MAC: %s    RSSI: %d\n", addr_str, rssi);


}


void bt_ready_cb(int err)
{
  printk("Bluetooth CB Initialized\n");
}


int main(void) {

  printk("Starting Scanner\n");

  err = bt_enable(bt_ready_cb);
  
#if 0
  while (err < 0) 
  {   printk("Bluetooth init failed (err %d)\n", err);
    k_sleep(K_SECONDS(10));
    if (bt_is_ready()) {
    printk("Bluetooth initialized\n");
  } else {
    printk("Bluetooth not initialized - trying bt_enable\n");
  }
    err = bt_enable(NULL);


    if (bt_is_ready()) {
      printk("Bluetooth initialized\n");
    }
    
    
  }
#endif
  err = bt_le_scan_start(&scan_param, scan_cb);
   if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return 0;
  }
  printk("Bluetooth initialized\n");


  while(1) {

    if (!bt_is_ready()) {
      printk("Bluetooth not initialized - trying bt_enable\n");
    }

    printk("Scanning...\n");
    
    k_sleep(K_SECONDS(5));
  }

}


