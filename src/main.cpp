#include <Arduino.h>
#include <entry.h>
#include <Maixduino_OV7740.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <stdio.h>
#ifdef __cplusplus
extern "C"
{
#endif
#include "kpu.h"
#ifdef __cplusplus
}
#endif
#include "image_process.h"
#include "region_layer.h"
#include "w25qxx.h"
#include "gpio.h"
#include "fft.h"
#include "fpioa.h"
#include "sysctl.h"
#include "dmac.h"
#include "fpioa.h"
#include "sdcard.h"
#include "ff.h"
#include "plic.h"
#include "uarths.h"
#include "kendryte-standalone-sdk/lib/bsp/include/bsp.h"

#define INCBIN_STYLE INCBIN_STYLE_SNAKE
#define INCBIN_PREFIX
#include "incbin.h"
#include "KPUDevice.h"

#define PLL0_OUTPUT_FREQ 800000000UL
#define PLL1_OUTPUT_FREQ 400000000UL

#ifdef M5STICKV
#define AXP192_ADDR 0x34
#define PIN_SDA 29
#define PIN_SCL 28
#endif

#define DATA_SAVE 1

INCBIN(model, "predict.kmodel");
CKPUDevice *device = NULL;
Maixduino_OV7740 camera(FRAMESIZE_QVGA, PIXFORMAT_YUV422);
TFT_eSPI lcd;

#define WIDTH 280
#define HEIGHT 240
#define FFT_N 512U
#define FRAME_LEN FFT_N
#define SAMPLING_FREQUENCY 16000

uint16_t rx_buf[FRAME_LEN * FFT_N];

// fft
#define FFT_FORWARD_SHIFT 0x0U
#define FFT_BACKWARD_SHIFT 0x1ffU

uint64_t time_last = 0U;
uint64_t time_now = 0U;
int time_count = 0;

typedef enum _complex_mode
{
  FFT_HARD = 0,
  FFT_SOFT = 1,
  FFT_COMPLEX_MAX,
} complex_mode_t;

fft_data_t fft_in_data[FFT_N];
fft_data_t fft_out_data[FFT_N];
complex_hard_t data_hard[FFT_N * FFT_N] = {0};
uint32_t g_rx_dma_buf[FRAME_LEN * 2 * 2];
uint16_t g_lcd_gram[WIDTH * HEIGHT] __attribute__((aligned(64)));
float hard_power[FFT_N * FFT_N];

int main1(void *parameter);

void io_mux_init(void)
{
  fpioa_set_function(30, FUNC_SPI0_SCLK);
  fpioa_set_function(33, FUNC_SPI0_D0);
  fpioa_set_function(31, FUNC_SPI0_D1);
  //	fpioa_set_function(32, FUNC_GPIOHS7);
  pinMode(32, OUTPUT);

  fpioa_set_function(32, FUNC_SPI0_SS3);
}

void FFT(int offset)
{
  for (int i = 0; i < FFT_N / 2; i++)
  {
    fft_in_data[i].I1 = 0;
    fft_in_data[i].R1 = rx_buf[2 * i + offset] - 32768;
    fft_in_data[i].I2 = 0;
    fft_in_data[i].R2 = rx_buf[2 + i + 1 + offset] - 32768;
  }
  fft_complex_uint16_dma(DMAC_CHANNEL1, DMAC_CHANNEL2, FFT_FORWARD_SHIFT, FFT_DIR_FORWARD, (uint64_t *)fft_in_data, FFT_N, (uint64_t *)fft_out_data);
  for (int i = 0; i < FFT_N / 2; i++)
  {
    data_hard[2 * i + offset].imag = fft_out_data[i].I1;
    data_hard[2 * i + offset].real = fft_out_data[i].R1;
    data_hard[2 * i + 1 + offset].imag = fft_out_data[i].I2;
    data_hard[2 * i + 1 + offset].real = fft_out_data[i].R2;
  }
}

#define SWAP_16(x) ((x >> 8 & 0xff) | (x << 8))

void update_image_fft(float *hard_power, float pw_max, uint32_t *pImage, uint32_t color, uint32_t bkg_color)
{
  uint32_t bcolor = SWAP_16((bkg_color << 16)) | SWAP_16(bkg_color);
  uint32_t fcolor = SWAP_16((color << 16)) | SWAP_16(color);

  int i = 0;
  int h[80];

  int x = 0;

  for (i = 0; i < 80; i++)
  {
    h[i] = 120 * (hard_power[i]) / pw_max;

    if (h[i] > 120)
      h[i] = 120;
    if (h[i] < 0)
      h[i] = 0;
  }

  for (i = 0; i < 80; i++)
  { // 53* 38640/512 => ~4000Hz
    x = i * 2;
    for (int y = 0; y < 120; y++)
    {
      if (y < (120 - h[i + 2]))
      {
        pImage[x + y * 2 * 140] = bcolor;
        pImage[x + 1 + y * 2 * 140] = bcolor;
        pImage[x + (y * 2 + 1) * 140] = bcolor;
        pImage[x + 1 + (y * 2 + 1) * 140] = bcolor;
      }
      else
      {
        pImage[x + y * 2 * 140] = fcolor;
        pImage[x + 1 + y * 2 * 140] = fcolor;
        pImage[x + (y * 2 + 1) * 140] = bcolor;
        pImage[x + 1 + (y * 2 + 1) * 140] = bcolor;
      }
    }
  }
}

void drawFft(int offset)
{
  FFT(offset);
  int i = 0;
  float pmax = 10;
  for (i = 0; i < FFT_N / 2; i++)
  {
    hard_power[i] = sqrt(data_hard[i].real * data_hard[i].real + data_hard[i].imag * data_hard[i].imag);

    //Convert to dBFS
    hard_power[i] = 20 * log(2 * hard_power[i] / FFT_N);

    if (hard_power[i] > pmax)
      pmax = hard_power[i];
  }
  update_image_fft(hard_power, 200 /*MAX range dBFS*/, (uint32_t *)g_lcd_gram, TFT_BLUE, TFT_BLACK);
  lcd.pushImage(0, 0, WIDTH, HEIGHT, (uint16_t *)g_lcd_gram);
}

bool axp192_init()
{
  Serial.printf("AXP192 init.\n");
  sysctl_set_power_mode(SYSCTL_POWER_BANK3, SYSCTL_POWER_V33);

  Wire.begin((uint8_t)PIN_SDA, (uint8_t)PIN_SCL, 400000);
  Wire.beginTransmission(AXP192_ADDR);
  int err = Wire.endTransmission();
  if (err)
  {
    Serial.printf("Power management ic not found.\n");
    return false;
  }
  Serial.printf("AXP192 found.\n");

  // Clear the interrupts
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(0x46);
  Wire.write(0xFF);
  Wire.endTransmission();
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(0x23);
  Wire.write(0x08); //K210_VCore(DCDC2) set to 0.9V
  Wire.endTransmission();
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(0x33);
  Wire.write(0xC1); //190mA Charging Current
  Wire.endTransmission();
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(0x36);
  Wire.write(0x6C); //4s shutdown
  Wire.endTransmission();
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(0x91);
  Wire.write(0xF0); //LCD Backlight: GPIO0 3.3V
  Wire.endTransmission();
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(0x90);
  Wire.write(0x02); //GPIO LDO mode
  Wire.endTransmission();
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(0x28);
  Wire.write(0xF0); //VDD2.8V net: LDO2 3.3V,  VDD 1.5V net: LDO3 1.8V
  Wire.endTransmission();
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(0x27);
  Wire.write(0x2C); //VDD1.8V net:  DC-DC3 1.8V
  Wire.endTransmission();
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(0x12);
  Wire.write(0xFF); //open all power and EXTEN
  Wire.endTransmission();
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(0x23);
  Wire.write(0x08); //VDD 0.9v net: DC-DC2 0.9V
  Wire.endTransmission();
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(0x31);
  Wire.write(0x03); //Cutoff voltage 3.2V
  Wire.endTransmission();
  Wire.beginTransmission(AXP192_ADDR);
  Wire.write(0x39);
  Wire.write(0xFC); //Turnoff Temp Protect (Sensor not exist!)
  Wire.endTransmission();

  fpioa_set_function(23, (fpioa_function_t)(FUNC_GPIOHS0 + 26));
  gpiohs_set_drive_mode(26, GPIO_DM_OUTPUT);
  gpiohs_set_pin(26, GPIO_PV_HIGH); //Disable VBUS As Input, BAT->5V Boost->VBUS->Charing Cycle

  msleep(20);
  return true;
}

int ad_convert()
{
  int ret = 0;
  //AD変換開始準備
  for (int j = 0; j < 7; j++)
  {
    digitalWrite(34, HIGH);
    digitalWrite(34, LOW);
  }
  //データ取り込み(12bit入力)
  for (int j = 0; j < 12; j++)
  {
    digitalWrite(34, HIGH);
    digitalWrite(34, LOW);
    ret = (ret << 1) + digitalRead(35);
  }
  //最後取り込み終了。FPGAが初期ステートに戻る
  digitalWrite(34, HIGH);
  digitalWrite(34, LOW);
  return ret;
}

void setup()
{
  // put your setup code here, to run once:
  io_mux_init();
  pll_init();
  sysctl_pll_set_freq(SYSCTL_PLL0, PLL0_OUTPUT_FREQ);
  sysctl_pll_set_freq(SYSCTL_PLL1, PLL1_OUTPUT_FREQ);
  plic_init();
  uarths_init();
  Serial.begin(115200);
  axp192_init();

  /* flash init */
  Serial.printf("flash init\n");
  w25qxx_init(3, 0);
  w25qxx_enable_quad_mode();
  /* LCD init */
  Serial.printf("lcd init\n");
  lcd.begin();
  lcd.setRotation(1);
  lcd.fillScreen(TFT_BLACK);
  /* DVP init */
  Serial.printf("DVP init\n");
  if (!camera.begin())
  {
    Serial.printf("camera init fail\n");
    while (true)
      ;
  }
  else
  {
    Serial.printf("camera init success\n");
  }
  camera.run(true);
  /* enable global interrupt */
  sysctl_enable_irq();
  /* system start */
  Serial.printf("System start\n");
  time_last = sysctl_get_time_us();
  time_now = sysctl_get_time_us();
  time_count = 0;

  uint8_t *model_data_align = model_data;
  device = new CKPUDevice(DMAC_CHANNEL5, model_data_align);

  pinMode(34, OUTPUT);
  pinMode(35, INPUT_PULLUP);
  /*
  Serial.printf("sd card init.\n");

  if (sdcard_init())
  {
    Serial.printf("SD card err\n");
  }
  if (fs_init())
  {
    Serial.printf("FAT32 err\n");
  }
*/
  pinMode(36, INPUT_PULLUP);

  dmac_init();
  // second core enabled entry function is main1
  register_core1(main1, NULL);
}

int buffSwap = 0;
int offset = 0;
int file_count = 0;

void sample()
{

  Serial.println("wait for serial signal.");
  Serial.println(Serial.readString());
  file_count++;
  Serial.println("start sampling.\n");
  auto start = micros();
  for (int j = 0; j < FFT_N / 2; j++)
  {
    for (int i = 0; i < FFT_N; i++)
    {
      auto startone = micros();
      int addr = i + j * FFT_N;
      rx_buf[addr] = ad_convert();
      auto endone = micros();
      auto elp = endone - startone;
      if (elp < 17)
      {
        delayMicroseconds(17 - elp);
      }
    }
  }
  auto end = micros();
  for (int j = 0; j < FFT_N / 2; j++)
  {
    FFT(j * FFT_N);
  }
  Serial.printf("end sampling.\n");
  //drawFft(0);
  auto elipse = float(end - start);
  float speed = elipse / (FFT_N * (FFT_N / 2));
  Serial.printf("start data.\n");
  Serial.printf("sps : %.2f[hz]\n", 1000000 / speed);
  Serial.printf("raw data\n");
  for (int i = 0; i < FFT_N / 2; i++)
  {
    Serial.write((uint8_t *)&rx_buf[i * FFT_N], sizeof(uint16_t) * FFT_N);
  }
  Serial.printf("fft data\n");
  for (int i = 0; i < FFT_N / 2; i++)
  {
    Serial.write((uint8_t *)&data_hard[i * FFT_N], sizeof(complex_hard_t) * FFT_N / 2);
  }
  Serial.printf("end data.");
}

float data[2][1][255];
#define RUNNING

void runnning()
{
  Serial.printf("sampling data.\n");
  for (int i = 0; i < FFT_N; i++)
  {
    auto startone = micros();
    int addr = i;
    rx_buf[addr] = ad_convert();
    auto endone = micros();
    auto elp = endone - startone;
    if (elp < 17)
    {
      delayMicroseconds(17 - elp);
    }
  }
  FFT(0);
  for(int i = 0;i<255;i++)
  {
    data[1][0][i] = data_hard[i+1].imag;
    data[0][0][i] = data_hard[i+1].real;
  }
  Serial.printf("run kpu forward.\n");
  device->runForward((uint8_t*)data);
  sleep(1);
}

void loop()
{
#ifdef RUNNING
  runnning();
#else
  sample();
#endif
}

void loop1()
{
#ifdef RUNNING
  float *result;
  size_t size;
  char resultView[256];
  if(device->isDone()==false)
  {
    while(!device->isDone());
    device->getOutput(0,(uint8_t**)&result,&size);
    float max = -1;
    int index = 0;
    for(int i = 0;i<4;i++)
    {
      if(max < result[i])
      {
        max = result[i];
        index = i;
      }
    }
    lcd.fillScreen(TFT_BLACK);
    sprintf(resultView,"Device Location : %d",index+1);
    lcd.drawString(resultView,50,80);
    Serial.printf("%s\n",resultView);
    for(int i = 0;i<4;i++){
      sprintf(resultView,"Class%d : %3.3f",i+1,result[i]);
      lcd.drawString(resultView,50,80+20*(i+1));
      Serial.printf("%s\n",resultView);
    }
//    sleep(1);
  }
#else
  sleep(100);
#endif
}
//2nd core entry program
int main1(void *parameter)
{
  while (1)
  {
    loop1();
  }
}