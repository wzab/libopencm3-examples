/*
 * Simple ADC based on STM32.
 * Written by Wojciech M. Zabolotny <wzab01@gmail.com>
 * Copyright (C) 2022 Wojciech M. Zabolotny <wzab01@gmail.com> 
 * Based on work of Gareth McMullin
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * and on a book by Warren Gay: "Beginning STM32, Developing with 
 * FreeRTOS, libopencm3 and GCC" <https://doi.org/10.1007/978-1-4842-3624-6>
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>

/* Define this to include the DFU APP interface. */
//#define INCLUDE_DFU_INTERFACE

#ifdef INCLUDE_DFU_INTERFACE
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/dfu.h>
#endif

static uint8_t nof_chan = 0;
static uint8_t chans[ADC_SQR_MAX_CHANNELS_REGULAR];
static uint16_t adc_data[ADC_SQR_MAX_CHANNELS_REGULAR];
static uint32_t smp_prescaler = 0;
static uint32_t smp_period = 0;

static uint8_t tx_buf[64];
static uint8_t tx_buf_busy = 0;
static uint8_t tx_error = 0;
static uint8_t tx_end = 0;

static usbd_device *usbd_dev;

const struct usb_device_descriptor dev_descr = {
  .bLength = USB_DT_DEVICE_SIZE,
  .bDescriptorType = USB_DT_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = 0,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = 64,
  .idVendor = 0xabba,
  .idProduct = 0x5301,
  .bcdDevice = 0x0200,
  .iManufacturer = 1,
  .iProduct = 2,
  .iSerialNumber = 3,
  .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor adc_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
  },{

    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
  },{

    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
  }};

const struct usb_interface_descriptor adc_iface = {
  .bLength = USB_DT_INTERFACE_SIZE,
  .bDescriptorType = USB_DT_INTERFACE,
  .bInterfaceNumber = 0,
  .bAlternateSetting = 0,
  .bNumEndpoints = 3,
  .bInterfaceClass = USB_CLASS_VENDOR,
  .bInterfaceSubClass = 0xff,
  .bInterfaceProtocol = 0xff,
  .iInterface = 0,

  .endpoint = adc_endp,

};

#ifdef INCLUDE_DFU_INTERFACE
const struct usb_dfu_descriptor dfu_function = {
  .bLength = sizeof(struct usb_dfu_descriptor),
  .bDescriptorType = DFU_FUNCTIONAL,
  .bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
  .wDetachTimeout = 255,
  .wTransferSize = 1024,
  .bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor dfu_iface = {
  .bLength = USB_DT_INTERFACE_SIZE,
  .bDescriptorType = USB_DT_INTERFACE,
  .bInterfaceNumber = 1,
  .bAlternateSetting = 0,
  .bNumEndpoints = 1,
  .bInterfaceClass = 0xFE,
  .bInterfaceSubClass = 1,
  .bInterfaceProtocol = 1,
  .iInterface = 0,

  .extra = &dfu_function,
  .extralen = sizeof(dfu_function),
};
#endif

const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = &adc_iface,
#ifdef INCLUDE_DFU_INTERFACE
  }, {
    .num_altsetting = 1,
    .altsetting = &dfu_iface,
#endif
  }};

const struct usb_config_descriptor config = {
  .bLength = USB_DT_CONFIGURATION_SIZE,
  .bDescriptorType = USB_DT_CONFIGURATION,
  .wTotalLength = 0,
#ifdef INCLUDE_DFU_INTERFACE
  .bNumInterfaces = 2,
#else
  .bNumInterfaces = 1,
#endif
  .bConfigurationValue = 1,
  .iConfiguration = 0,
  .bmAttributes = 0xC0,
  .bMaxPower = 0x32,

  .interface = ifaces,
};

static const char *usb_strings[] = {
  "WZab SWIS",
  "ADC Demo",
  "DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

#ifdef INCLUDE_DFU_INTERFACE
static void dfu_detach_complete(usbd_device *dev, struct usb_setup_data *req)
{
  (void)req;
  (void)dev;

  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO10);
  gpio_set(GPIOA, GPIO10);
  scb_reset_core();
}

static enum usbd_request_return_codes dfu_control_request(usbd_device *dev, 
  struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
  void (**complete)(usbd_device *, struct usb_setup_data *))
{
  (void)buf;
  (void)len;
  (void)dev;

  if ((req->bmRequestType != 0x21) || (req->bRequest != DFU_DETACH))
    return USBD_REQ_NOTSUPP; /* Only accept class request. */

  *complete = dfu_detach_complete;

  return USBD_REQ_HANDLED;
}
#endif

static uint16_t read_adc(uint8_t channel) {
  adc_set_sample_time(ADC1,channel,ADC_SMPR_SMP_1DOT5CYC);
  adc_set_regular_sequence(ADC1,1,&channel);
  adc_start_conversion_direct(ADC1);
  while ( !adc_eoc(ADC1) ) {};
  return adc_read_regular(ADC1);
}

static void wz_adc_set_chans(uint8_t *buf, int len)
{
  //The first byte should contain the number of channels
  uint8_t nch = buf[0];
  nof_chan = 0;
  if(nch != len-1) {
    char msg[]="E:wrong length";
    usbd_ep_write_packet(usbd_dev,0x82,msg,sizeof(msg));	        
    return;
  }
  for(uint8_t i = 0; i < nch ; i++) {
    if (buf[i+1] > ADC_CHANNEL18) {
      char msg[]="E:wrong channel";
      usbd_ep_write_packet(usbd_dev,0x82,msg,sizeof(msg));	        
      return;
    }
    chans[i] = buf[i+1];
  }
  nof_chan = nch;
  {
    char msg[]="OK";
    usbd_ep_write_packet(usbd_dev,0x82,msg,sizeof(msg));	        
    return;
  }
}

static void wz_adc_set_freq(uint8_t *buf, int len)
{
  if(len != 8) {
    char msg[]="E:wrong length";
    usbd_ep_write_packet(usbd_dev,0x82,msg,sizeof(msg));	        
    return;
  }
  smp_prescaler = buf[0]+256*(buf[1]+256*(buf[2]+256*(uint32_t)buf[3]));
  smp_period = buf[4]+256*(buf[5]+256*(buf[6]+256*(uint32_t)buf[7]));
  {
    char msg[]="OK";
    usbd_ep_write_packet(usbd_dev,0x82,msg,sizeof(msg));	        
    return;
  }
}


static void wz_adc_stop(void)
{
  timer_disable_counter(TIM3);
  adc_disable_dma(ADC1);
  tx_end = 1;
  {
    char msg[]="OK";
    usbd_ep_write_packet(usbd_dev,0x82,msg,sizeof(msg));	        
    return;
  }     
}

static void wz_adc_read(void)
{
  usbd_ep_write_packet(usbd_dev,0x82,adc_data,2*nof_chan);	        
  return;
}

static void wz_tim3_read(void)
{
  uint32_t val = timer_get_counter(TIM3);
  usbd_ep_write_packet(usbd_dev,0x82,&val,sizeof(val));	        
  return;
}


static void wz_adc_start(void)
{
  adc_disable_dma(ADC1);
  tx_buf_busy = 0;
  tx_error = 0;
  // Prepare DMA
  dma_disable_channel(DMA1, DMA_CHANNEL1);
 
  dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
 
  dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
 
  dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
  dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t) &ADC_DR(ADC1));
 
  dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t) &adc_data);
  dma_set_number_of_data(DMA1, DMA_CHANNEL1, nof_chan);
 
  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
    
  adc_set_regular_sequence(ADC1,nof_chan,chans);
  adc_set_single_conversion_mode(ADC1);
  adc_enable_scan_mode(ADC1);
  adc_enable_dma(ADC1);
  dma_enable_channel(DMA1, DMA_CHANNEL1);
  //delay(100);
  adc_enable_external_trigger_regular(ADC1,ADC_CR2_EXTSEL_TIM3_TRGO);
  // Prepare the TIMER3 for operation
  timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_continuous_mode(TIM3);
  timer_set_prescaler(TIM3,smp_prescaler);
  timer_set_period(TIM3,smp_period);
  timer_set_clock_division(TIM3,0);
  timer_set_master_mode(TIM3, TIM_CR2_MMS_UPDATE);
  timer_enable_counter(TIM3);
  {
    char msg[]="OK";
    char msg2[]="START";
    usbd_ep_write_packet(usbd_dev,0x82,msg,sizeof(msg));	        
    while(!usbd_ep_write_packet(usbd_dev,0x83,msg2,sizeof(msg2))){};	        
    return;
  }
}

static void adccfg_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  (void)ep;
  uint8_t buf[64];
  int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);
  if(len) {
    //Check the type of command
    switch(buf[0]) {
    case 0: // Stop
      wz_adc_stop();
      break;
    case 1: // Set channels
      wz_adc_set_chans(&buf[1],len-1);
      break;
    case 2: // Set frequency
      wz_adc_set_freq(&buf[1],len-1);
      break;
    case 3: // Start
      wz_adc_start();
      break;
    case 4: // Read data
      wz_adc_read();
      break;
    case 5: // Read TIM3
      wz_tim3_read();
      break;
    default: // Unknown command
      {
	char msg[]="E:Unknown command!";
	usbd_ep_write_packet(usbd_dev,0x82,msg,sizeof(msg));	        
      }
    } 
  }
}

void dma1_channel1_isr(void) {
  dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_IFCR_CGIF1);
  gpio_toggle(GPIOC,GPIO13);
  //Here we copy the data to the transmission buffer
  if(tx_buf_busy) {
    //Overrun
    tx_error = 1;
  } else {
    tx_buf[0]='D';
    memcpy(&tx_buf[1],adc_data,2*nof_chan);
    tx_buf_busy = 1;
  }
}

static void adc_set_config(usbd_device *dev, uint16_t wValue)
{
  (void)wValue;
  (void)dev;

  usbd_ep_setup(dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, adccfg_data_rx_cb);
  usbd_ep_setup(dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
  usbd_ep_setup(dev, 0x83, USB_ENDPOINT_ATTR_BULK, 64, NULL);
  /*usbd_register_control_callback(
    dev,
    USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
    USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
    adc_control_request);*/
#ifdef INCLUDE_DFU_INTERFACE
  usbd_register_control_callback(
				 dev,
				 USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				 USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				 dfu_control_request);
#endif

  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
  /* SysTick interrupt every N clock pulses: set reload to N-1 */
  systick_set_reload(99999);
  systick_interrupt_enable();
  systick_counter_enable();
}

int main(void)
{
  rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_HSI_48MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_set_mode(GPIOA,
		GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_ANALOG, // Analog mode
		GPIO0|GPIO1|GPIO2|GPIO3|GPIO4|GPIO5|GPIO6|GPIO7); // PA0 .. PA7
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_set_mode(GPIOC,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO13); // PC13
	
  // Initialize DMA
  rcc_periph_clock_enable(RCC_DMA1);
  // Initialize TIMER1
  rcc_periph_clock_enable(RCC_TIM3);
  rcc_periph_reset_pulse(RST_TIM3);
  // Initialize ADC:
  rcc_peripheral_enable_clock(&RCC_APB2ENR,RCC_APB2ENR_ADC1EN);
  adc_power_off(ADC1);
  rcc_peripheral_reset(&RCC_APB2RSTR,RCC_APB2RSTR_ADC1RST);
  rcc_peripheral_clear_reset(&RCC_APB2RSTR,RCC_APB2RSTR_ADC1RST);
  rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);// Set. 12MHz, Max. 14MHz
  adc_set_dual_mode(ADC_CR1_DUALMOD_IND); // Independent mode
  adc_disable_scan_mode(ADC1);
  adc_set_right_aligned(ADC1);
  adc_set_single_conversion_mode(ADC1);
  adc_set_sample_time(ADC1,ADC_CHANNEL_TEMP,ADC_SMPR_SMP_239DOT5CYC);
  adc_set_sample_time(ADC1,ADC_CHANNEL_VREF,ADC_SMPR_SMP_239DOT5CYC);
  adc_enable_temperature_sensor();
  adc_power_on(ADC1);
  adc_reset_calibration(ADC1);
  adc_calibrate_async(ADC1);
  while (adc_is_calibrating(ADC1)) {};
  nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
  nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 0);

 
  /*
   * This is a somewhat common cheap hack to trigger device re-enumeration
   * on startup.  Assuming a fixed external pullup on D+, (For USB-FS)
   * setting the pin to output, and driving it explicitly low effectively
   * "removes" the pullup.  The subsequent USB init will "take over" the
   * pin, and it will appear as a proper pullup to the host.
   * The magic delay is somewhat arbitrary, no guarantees on USBIF
   * compliance here, but "it works" in most places.
   */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
  gpio_clear(GPIOA, GPIO12);
  for (unsigned i = 0; i < 800000; i++) {
    __asm__("nop");
  }

  usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
  usbd_register_set_config_callback(usbd_dev, adc_set_config);

  while (1){
    usbd_poll(usbd_dev);
    //Check if there is ADC data frame to be sent
    if(tx_error) {
      char msg[] = "E:OVR";
      wz_adc_stop();
      tx_buf_busy = 0;
      tx_error = 0;
      while(!usbd_ep_write_packet(usbd_dev,0x83,msg,sizeof(msg))) {};
    } else if(tx_buf_busy){
      while(!usbd_ep_write_packet(usbd_dev,0x83,tx_buf,1+2*nof_chan)){};
      tx_buf_busy = 0;           		      
    }
    if(tx_end) {
       char msg[] = "O:END";  
       while(!usbd_ep_write_packet(usbd_dev,0x83,msg,sizeof(msg))){}; 
       tx_end = 0;   
    }
  }
}

void sys_tick_handler(void)
{
  static int x = 0;
  static int dir = 1;
  uint8_t buf[4] = {0, 0, 0, 0};

  buf[1] = dir;
  x += dir;
  if (x > 30)
    dir = -dir;
  if (x < -30)
    dir = -dir;

  //	usbd_ep_write_packet(usbd_dev, 0x81, buf, 4);
}
