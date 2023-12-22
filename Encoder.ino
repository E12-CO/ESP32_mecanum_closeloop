#include <ESP32Encoder.h>

ESP32Encoder enc_lf;
ESP32Encoder enc_lb;
ESP32Encoder enc_rf;
ESP32Encoder enc_rb;

#define ENC_LF_A  36
#define ENC_LF_B  39

#define ENC_RF_A  22
#define ENC_RF_B  23

#define ENC_LB_A  34
#define ENC_LB_B  35

#define ENC_RB_A  19
#define ENC_RB_B  21

void Encoder_Init(){
  enc_lf.attachFullQuad(ENC_LF_A, ENC_LF_B);
  enc_lb.attachFullQuad(ENC_LB_A, ENC_LB_B);
  enc_rf.attachFullQuad(ENC_RF_A, ENC_RF_B);
  enc_rb.attachFullQuad(ENC_RB_A, ENC_RB_B);
  
  enc_lf.setCount(0);
  enc_lb.setCount(0);
  enc_rf.setCount(0);
  enc_rb.setCount(0);
}

long GetEncoder(uint8_t i){
  switch (i){
    case 1:
      return enc_lf.getCount();
    case 2:
      return enc_rf.getCount();
    case 3:
      return enc_lb.getCount();
    case 4:
      return enc_rb.getCount();      
  }  
}
