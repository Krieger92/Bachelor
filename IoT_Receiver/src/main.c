#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>



bt_addr_le_t macAdresseGeneric, macAdresses[10];
char c;
int err, operationMode = 0, isScanning = 0, toggleReadout[10] = {1}, RAInt = 0;
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

  if (operationMode == 0)
  {
  
  
    if (addr->a.val[5] == 0xD0 && addr->a.val[4] == 0x83 && addr->a.val[3] == 0xD4 && addr->a.val[2] == 0x00 && addr->a.val[1] == 0x11)
    {
      
      printk("[DEVICE]: %s  ::  ", addr_str); // mac adress
      printk("RSSI: %d  ::  ", rssi); // RSSI
      printk("BATTERY LVL: %04X  ::  ", mdata->batteryLvl); // battery lvl
      printk("SEQUENCE ID: %02X\n\n", mdata->seqID); // sequence id
        
      
      
    }
  }  
  else if (operationMode == 1)
  {
    for (size_t i = 0; i < 10; i++)
    {
      if (macAdresses[i].a.val[0] == addr->a.val[0] && toggleReadout[i] == 1 && seqIDs[i] != mdata->seqID)
      {
          printk("[DEVICE]: %s  ::  ", addr_str); // mac adress
          printk("RSSI: %d  ::  ", rssi); // RSSI
          printk("BATTERY LVL: %04X  ::  ", mdata->batteryLvl); // battery lvl
          printk("SEQUENCE ID: %02X\n", mdata->seqID); // sequence id
          printk("DECODED DATA ARRAY: X_FREQ: %02X  ::  X_VEL %02X  ::  Y_FREQ: %02X  ::  Y_VEL: %02X  ::  Z_FREQ: %02X  ::  Z_VEL: %02X \n \n", 
            mdata->x_freq, mdata->x_vel, mdata->y_freq, mdata->y_vel, mdata->z_freq, mdata->z_vel); // data array
          seqIDs[i] = mdata->seqID;
      }
    }


    #if 0
    printk("Data: ");
    for (int i = 0; i < buf->len; i++)
    {
      printk("%02X ", buf->data[i]);
    } 
    printk("\n");
    #endif
  }
   
  
}




int main(void) {
  macAdresseGeneric.a.val[5] = 0xD0;
  macAdresseGeneric.a.val[4] = 0x83; 
  macAdresseGeneric.a.val[3] = 0xD4; 
  macAdresseGeneric.a.val[2] = 0x00; 
  macAdresseGeneric.a.val[1] = 0x11;

  err = bt_enable(NULL);
  

  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return 0;
  }
  printk("Bluetooth initialized\n");




}

