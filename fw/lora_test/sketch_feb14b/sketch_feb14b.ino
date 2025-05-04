

#include "llcc68_hal.h"
#include "llcc68.h"


#define LORA_BUSY     PA9
#define LORA_ANT_PWR  PA10
#define LORA_DIO1     PB1   //interrupt pin

#define LED     PA0
#define BUTTON  PA1


#define TX_MODE
//#define RX_MODE


volatile bool rec_interrupt_flag = false; // Counter for interrupts


void gpio_init(void){
  pinMode(LORA_BUSY,INPUT);
  pinMode(LORA_ANT_PWR,OUTPUT);
  pinMode(LORA_DIO1,INPUT_PULLUP);   //default status is pull-down
  pinMode(LED,OUTPUT);
  pinMode(BUTTON,INPUT_PULLUP);

  digitalWrite(LED,LOW);
  digitalWrite(LORA_ANT_PWR,HIGH);
}

void lora_isr(void){
  digitalWrite(LED,HIGH);
  llcc68_clear_irq_status(nullptr,0x0002);
  rec_interrupt_flag = true;
}

void setup(){
  gpio_init();
  Serial.begin(115200);
  llcc68_hal_init();
  llcc68_hal_reset(NULL);
  digitalWrite(LED,HIGH);
  delay(2000);    //this delay is necessary after llcc68 reset
  digitalWrite(LED,LOW);
  
  attachInterrupt(digitalPinToInterrupt(LORA_DIO1), lora_isr, RISING);

  uint8_t stat = digitalRead(LORA_BUSY);
  if(stat){
    Serial.println("Device is in sleep mode");
  }else{
    Serial.println("device is in stdby_rc mode");
  }
}


#ifdef TX_MODE

void loop(){

  static bool run_once = true;
  static uint8_t data_buf[10]={0x01,0x02,0x03,0x04,0x05,0x06,0X07,0X08,0X09,0X0A};

  if(run_once){
    run_once = false;
    //check the table 13-21 from LLCC68 datasheet.
    llcc68_pa_cfg_params_t pa_cfg;
    pa_cfg.device_sel = 0X00;   //deviceSel is reserved and has always the value 0x00.
    pa_cfg.hp_max = 0X03;
    pa_cfg.pa_duty_cycle = 0x02;
    pa_cfg.pa_lut = 0x01;     //paLut is reserved and has always the value 0x01.

    llcc68_mod_params_lora_t lora_mod;
    lora_mod.bw = LLCC68_LORA_BW_500;
    lora_mod.cr = LLCC68_LORA_CR_4_5;
    lora_mod.sf = LLCC68_LORA_SF8;
    
    int8_t power_in_db = 0x04;    //output 14dBm based on the regulation in China.
    llcc68_ramp_time_t ramp_time = LLCC68_RAMP_10_US;

    uint8_t tx_base_addr = 0;
    uint8_t rx_base_addr = 0;

    llcc68_pkt_params_lora_t lora_packet_para;
    lora_packet_para.crc_is_on = true;
    lora_packet_para.header_type = LLCC68_LORA_PKT_EXPLICIT;
    lora_packet_para.invert_iq_is_on = true;
    lora_packet_para.pld_len_in_bytes = 50;
    lora_packet_para.preamble_len_in_symb = 8;

    uint8_t reg_buffer[10];

    llcc68_set_pkt_type(nullptr,LLCC68_PKT_TYPE_LORA);
    llcc68_set_rf_freq(nullptr,868000000);    //868MHz in China
    llcc68_set_buffer_base_address(nullptr,0,0);
    llcc68_set_pa_cfg(nullptr,&pa_cfg);
    llcc68_set_tx_params(nullptr,power_in_db,ramp_time);
    llcc68_set_buffer_base_address(nullptr,tx_base_addr,rx_base_addr);
    
    llcc68_set_lora_mod_params(nullptr,&lora_mod);
    llcc68_set_lora_pkt_params(nullptr,&lora_packet_para);
    llcc68_set_dio2_as_rf_sw_ctrl(nullptr,true);
    //llcc68_write_register(nullptr,0X01,reg_buffer,sizeof(reg_buffer));
    //uint16_t irqMask = 0x002F|0x002F;   
    //llcc68_set_dio_irq_params(nullptr,irqMask,irqMask,0,0);
  }
  delay(2000);
  digitalWrite(LED,HIGH);
  static uint8_t press_counter = 0;
  press_counter +=1;
  data_buf[0] = press_counter;
  llcc68_write_buffer(nullptr,0,data_buf,sizeof(data_buf));
  delay(10);
  llcc68_set_tx(nullptr,100);
  digitalWrite(LED,LOW);
  /*
  if(digitalRead(PA1)==0){
    delay(20);
    digitalWrite(PA0,true);
    if(digitalRead(PA1)==0){
      while(digitalRead(PA1)==0);
      Serial.println("send lora data out");
      static uint8_t press_counter = 0;
      press_counter +=1;
      data_buf[0] = press_counter;
      llcc68_write_buffer(nullptr,0,data_buf,sizeof(data_buf));
      delay(10);
      llcc68_set_tx(nullptr,100);
      delay(100);
      digitalWrite(PA0,false);
    }
  }
  */
}
#endif 


#ifdef RX_MODE

void loop(){



  static bool run_once = true;
  static uint8_t data_buf[10]={0x01,0x02,0x03,0x04,0x05,0x06,0X07,0X08,0X09,0X0A};
  static uint8_t tx_base_addr = 0;
  static uint8_t rx_base_addr = 0x0f;
  if(run_once){
    run_once = false;
    //check the table 13-21 from LLCC68 datasheet.
    llcc68_pa_cfg_params_t pa_cfg;
    pa_cfg.device_sel = 0X00;   //deviceSel is reserved and has always the value 0x00.
    pa_cfg.hp_max = 0X03;
    pa_cfg.pa_duty_cycle = 0x02;
    pa_cfg.pa_lut = 0x01;     //paLut is reserved and has always the value 0x01.

    llcc68_mod_params_lora_t lora_mod;
    lora_mod.bw = LLCC68_LORA_BW_500;
    lora_mod.cr = LLCC68_LORA_CR_4_5;
    lora_mod.sf = LLCC68_LORA_SF8;
    
    int8_t power_in_db = 0x04;    //output 14dBm based on the regulation in China.
    llcc68_ramp_time_t ramp_time = LLCC68_RAMP_10_US;

    uint8_t tx_base_addr = 0;
    uint8_t rx_base_addr = 0x0f;

    for(char i=0;i<10;i++){
      data_buf[i] = 0;
    }
    llcc68_pkt_params_lora_t lora_packet_para;
    lora_packet_para.crc_is_on = true;
    lora_packet_para.header_type = LLCC68_LORA_PKT_EXPLICIT;
    lora_packet_para.invert_iq_is_on = true;
    lora_packet_para.pld_len_in_bytes = 50;
    lora_packet_para.preamble_len_in_symb = 8;

    uint8_t reg_buffer[10];

    llcc68_set_pkt_type(nullptr,LLCC68_PKT_TYPE_LORA);
    llcc68_set_rf_freq(nullptr,868000000);    //868MHz in China
    llcc68_set_pa_cfg(nullptr,&pa_cfg);
    llcc68_set_tx_params(nullptr,power_in_db,ramp_time);
    llcc68_set_buffer_base_address(nullptr,tx_base_addr,rx_base_addr);
    llcc68_read_buffer(nullptr,rx_base_addr,data_buf,sizeof(data_buf));
    llcc68_set_lora_mod_params(nullptr,&lora_mod);
    llcc68_set_lora_pkt_params(nullptr,&lora_packet_para);
    llcc68_set_dio2_as_rf_sw_ctrl(nullptr,true);
    
    uint16_t irqMask = 0x0002;    //only need to support packagetransmission completed and Packet received irq
    llcc68_set_dio_irq_params(nullptr,irqMask,irqMask,0,0);
    //llcc68_write_register(nullptr,0X01,reg_buffer,sizeof(reg_buffer));

    llcc68_set_rx(nullptr,LLCC68_RX_SINGLE_MODE); // LLCC68_RX_SINGLE_MODE
  }
 // delay(300);   //!!!!!!!!!!!!!!!!!!!fuck. don't simply delete this delay. 
  if(rec_interrupt_flag){
    rec_interrupt_flag = false;
    llcc68_set_rx(nullptr,LLCC68_RX_SINGLE_MODE);
    llcc68_set_buffer_base_address(nullptr,tx_base_addr,rx_base_addr);
    llcc68_rx_buffer_status_t rx_buf_stat;
    llcc68_get_rx_buffer_status(nullptr,&rx_buf_stat);
    if(rx_buf_stat.pld_len_in_bytes!=0){
      //Serial.printf("package received and the quantity is %d and the start address is %d\r\n",rx_buf_stat.pld_len_in_bytes,rx_buf_stat.buffer_start_pointer);
      llcc68_read_buffer(nullptr,rx_base_addr,data_buf,sizeof(data_buf));
      if(data_buf[0]!=0){
        //digitalWrite(PA0,true);
        for(char i=0;i<10;i++){
          Serial.print(data_buf[i]);
          Serial.print('-');
        }
        Serial.println(' ');
        //digitalWrite(PA0,false);
        for(char i=0;i<10;i++){
          data_buf[i] = 0;
        }
      }
    }
    digitalWrite(LED,LOW);
  }



}


#endif 

























/*

void loop(){


  static bool run_once = true;
  static uint8_t data_buf[10]={0x01,0x02,0x03,0x04,0x05,0x06,0X07,0X08,0X09,0X0A};

  if(run_once){
    run_once = false;
    //check the table 13-21 from LLCC68 datasheet.
    llcc68_pa_cfg_params_t pa_cfg;
    pa_cfg.device_sel = 0X00;   //deviceSel is reserved and has always the value 0x00.
    pa_cfg.hp_max = 0X03;
    pa_cfg.pa_duty_cycle = 0x02;
    pa_cfg.pa_lut = 0x01;     //paLut is reserved and has always the value 0x01.

    llcc68_mod_params_lora_t lora_mod;
    lora_mod.bw = LLCC68_LORA_BW_500;
    lora_mod.cr = LLCC68_LORA_CR_4_8;
    lora_mod.sf = LLCC68_LORA_SF5;
    
    int8_t power_in_db = 0x04;    //output 14dBm based on the regulation in China.
    llcc68_ramp_time_t ramp_time = LLCC68_RAMP_10_US;

    uint8_t tx_base_addr = 0;
    uint8_t rx_base_addr = 0;

#ifdef RX_MODE
    for(char i=0;i<10;i++){
      data_buf[i] = 0;
    }
#endif
  
    llcc68_pkt_params_lora_t lora_packet_para;
    lora_packet_para.crc_is_on = true;
    lora_packet_para.header_type = LLCC68_LORA_PKT_EXPLICIT;
    lora_packet_para.invert_iq_is_on = true;
    lora_packet_para.pld_len_in_bytes = 50;
    lora_packet_para.preamble_len_in_symb = 8;

    uint8_t reg_buffer[10];

    llcc68_set_pkt_type(nullptr,LLCC68_PKT_TYPE_LORA);
    llcc68_set_rf_freq(nullptr,868000000);    //868MHz in China
    llcc68_set_buffer_base_address(nullptr,0,0);
    llcc68_set_pa_cfg(nullptr,&pa_cfg);
    llcc68_set_tx_params(nullptr,power_in_db,ramp_time);
    llcc68_set_buffer_base_address(nullptr,tx_base_addr,rx_base_addr);
    llcc68_read_buffer(nullptr,0,data_buf,sizeof(data_buf));
    llcc68_set_lora_mod_params(nullptr,&lora_mod);
    llcc68_set_lora_pkt_params(nullptr,&lora_packet_para);
    llcc68_set_dio2_as_rf_sw_ctrl(nullptr,true);
    //llcc68_write_register(nullptr,0X01,reg_buffer,sizeof(reg_buffer));
  }

#ifdef RX_MODE
  
  llcc68_set_rx(nullptr,LLCC68_RX_SINGLE_MODE);
  delay(1000);
  if(data_buf[0]!=0){
    digitalWrite(PA0,true);
    for(char i=0;i<10;i++){
      Serial.print(data_buf[i]);
      Serial.print('-');
    }
    Serial.println(' ');
    digitalWrite(PA0,false);
  }

#endif

#ifdef TX_MODE
  if(digitalRead(PA1)==0){
    delay(20);
    digitalWrite(PA0,true);
    if(digitalRead(PA1)==0){
      while(digitalRead(PA1)==0);
      Serial.println("send lora data out");
      llcc68_set_tx(nullptr,100);
      delay(100);
      digitalWrite(PA0,false);
    }
  }
#endif







  llcc68_chip_status_t chip_stat;
  llcc68_set_standby(nullptr,LLCC68_STANDBY_CFG_XOSC);
  delay(100);
  llcc68_get_status(nullptr,&chip_stat);
  Serial.println(chip_stat.chip_mode);
  delay(100);
  llcc68_set_standby(nullptr,LLCC68_STANDBY_CFG_RC);
  delay(100);
  llcc68_get_status(nullptr,&chip_stat);
  Serial.println(chip_stat.chip_mode);
  delay(100);

}

*/
