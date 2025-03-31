/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "tflora.hpp"
#include <stdio.h>

TFLORA *tflora=nullptr;

using namespace time_literals;

TFLORA::TFLORA(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio)
{
/*	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}*/

  PX4_INFO("drdy: %d", (int)_drdy_gpio );

  busy_enabled=0;
  irq_enabled=1;
  irq_time=0;
  outMsg.timestamp=0;

  if(tflora==nullptr)
    tflora=this;
  else
    PX4_ERR("tflora already exist.. error state...");
}

TFLORA::~TFLORA()
{
	tflora=nullptr;
}

int TFLORA::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	lora_outgoing_message_sub.set_interval_us(50_ms);
  lora_incoming_message_pub.advertise();

	if (!lora_outgoing_message_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return PX4_ERROR;
	}

  if(parseConfig()!=0){
		PX4_ERR("error parsing config file tflora.txt");
		return PX4_ERROR;
	}

  RadioInterruptConfigure();

  //init libraray
  init_lmic();

	return 0;
}


void TFLORA::exit_and_cleanup()
{
	RadioInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

/*void ICM20689::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}*/

#define REG_LORASYNCWORDLSB     0x0741
int TFLORA::probe()
{
  PX4_INFO ("TFLORA Probe!");
  if(this->ReadReg(REG_LORASYNCWORDLSB) == 0x24)
  {
    PX4_INFO ("I see the tflora!");
    return PX4_OK;
  }

	return PX4_ERROR;
}

void TFLORA::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

  //check message
  if (lora_outgoing_message_sub.updated()) 
  {
		if (lora_outgoing_message_sub.copy(&outMsg)) 
    {
			PX4_INFO("recieved lora message for send");
    }
  }

	switch (_state) {
  case STATE::SLEEPING_FOREVER:
    if (outMsg.timestamp > 0)
    {
      //send message...
      PX4_INFO("sending lora message...");
  
      LMIC_setDrTxpow(ordinary_dr, KEEP_TXPOWADJ);

      LMIC_setTxData2(1, outMsg.data, outMsg.len, 0);
      outMsg.timestamp=0;//means message sended...

      _state=STATE::JOB_DONE;
      ScheduleNow();
    }    
    break;

  case STATE::JOB_DONE:        
    currentJob = os_popJob();    
    if(currentJob!=NULL)
    {
      _state=STATE::WAIT_FOR_JOB_TIME;       
      ScheduleAt(currentJob->deadline*US_PER_OSTICK);
    } 
    else 
    {
      _state=STATE::SLEEPING_FOREVER;
    }
    break;

  case STATE::WAIT_FOR_JOB_TIME:
      if( currentJob == NULL)
        PX4_ERR("No job!");
      
      //TODO: if job time, process job
      int diff=currentJob->deadline*US_PER_OSTICK-now;
      if( diff > 2) //not sure if good enougth...
      {        
        PX4_INFO("Not job time... jobTime:%ld ... now: %lld.. diff: %d",
                  currentJob->deadline*US_PER_OSTICK,
                  now,
                  diff);
        return;
      }

      if(currentJob->flags & OSJOB_FLAG_IRQDISABLED) 
          hal_disableIRQs();
      
      os_runJob(currentJob);

      currentJob=NULL;
      _state=STATE::JOB_DONE;
      ScheduleNow();

    break;
  }
}

int TFLORA::RadioInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<TFLORA *>(arg)->RadioInterrupt();
	return 0;
}

bool TFLORA::RadioInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, true, false, false, &RadioInterruptCallback, this) == 0;
}

bool TFLORA::RadioInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

void TFLORA::RadioInterrupt()
{
  if(busy_enabled==1)//busy pin is shared with irq..
    	return;

  if(irq_enabled==0)//library wont be disturbet, call irq callback later, when it enable irq..
  {
    irq_time = hal_ticks();
    return;
  }

  radio_irq_handler(0, hal_ticks());
  ScheduleNow();
}

void TFLORA::spi_select()
{
  busy_enabled=1; 
  select();
}

uint8_t TFLORA::spi_transfer(uint8_t data)
{ 
  return transfer(data);
}

void TFLORA::spi_deselect()
{
  busy_enabled=0; 
  deselect();
}

void TFLORA::hal_disableIrq()
{
  irq_enabled=0;
};

void TFLORA::hal_enableIrq()
{
  if(irq_time!=0)
  {
    radio_irq_handler(0, irq_time);
    ScheduleNow();
    irq_time=0;
  }
  irq_enabled=1;
}

void TFLORA::processDownlink(uint8_t *data, int dataLen)
{
   
    for(int i=0;i<dataLen;i++)
    {
        printf("%0x ", (int)(data[i]));
    }
    printf("\n");

    lora_message_s msg{};
    msg.timestamp = hrt_absolute_time();    
    msg.len = dataLen;
    if(msg.len>58)
      msg.len=58;
    for(int i=0;i<msg.len;i++)
      msg.data[i]=data[i];
    lora_incoming_message_pub.publish(msg);

}

int TFLORA::readKey(FILE * f,uint8_t *output, int len)
{
  int i=0;
  while(i<2*len)
  {
    char c;
    int n = -1;
    if(1!=fread(&c,1,1,f))
      return -1;

    if(c==' ')
      continue;

    if(c=='\n'||c=='\r')
      return -1;

    if (c >= '0' && c <= '9')
        n = c - '0';
    else if (c >= 'a' && c <= 'f')
        n = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F')
        n = c - 'A' + 10;
    else
    {
        fprintf(stderr, "Unexpected character: '%c'\n", c);
        return -1;
    }

    if (n == -1)
        return -1;

    if (i % 2 == 0)
        *output = n << 4;
    else
        *output++ |= n;

    i++;
  }
  return 0;
}

int TFLORA::parseConfig()
{

	FILE* conf_fd = fopen(CONF_FILE, "r");
  if(conf_fd==NULL)
  {
    fprintf(stderr,"Cannot open config file: %s",CONF_FILE);
    return -1;
  }  

  char k;
  uint8_t state=0; //0 - line begin, 1 - wait for '\n'
  while(1)
  {
    if(fread(&k,1,1,conf_fd)!=1)
      break;

    if(state==0)
    {
      if(k=='\n')
        continue;

      char sec;
      if(fread(&sec,1,1,conf_fd)!=1)
        break;

      if(sec=='\n')
        continue;

      if(sec!=':')
        k='\0';
      
      if(k=='a')
        if(readKey(conf_fd,APPSKEY, 16)!=0)
        {
          printf("error reading application key");
        }

      if(k=='n')
        if(readKey(conf_fd,NWKSKEY, 16)!=0)
        {
          printf("error reading network key");
        }

      if(k=='d')
      {
        DEVADDR=0;
        uint8_t b[4];
        if(readKey(conf_fd,b, 4)!=0)
        {
          printf("error reading dev addr");
        }
        else
        { 
          DEVADDR = ((uint32_t) b[0]) << 24 \
          | ((uint32_t) b[1]) << 16 \
          | ((uint32_t) b[2]) << 8 \
          | ((uint32_t) b[3]);
        }
      }

      state=1;
      continue;    
    }
    
    if(state==1)
    {
      if(k=='\n')
        state=0;
    }    

  }

  fclose(conf_fd);

  printf("app: ");
  for(int i=0;i<16;i++)
    printf("%02x ",NWKSKEY[i]);
  printf("\n");

  printf("net: ");
  for(int i=0;i<16;i++)
    printf("%02x ",APPSKEY[i]);
  printf("\n");

  printf("dev: %lu ",DEVADDR);
  for(int i=3;i>=0;i--)
    printf("%02x ",(uint8_t)((DEVADDR>> i*8) & 0xFF));
  printf("\n");

  return 0;
}


#define CMD_READREGISTER                0x1D
void TFLORA::ReadRegs (uint16_t addr, uint8_t* data, uint8_t len) {
    spi_select();
    spi_transfer(CMD_READREGISTER);
    spi_transfer(addr >> 8);
    spi_transfer(addr);
    spi_transfer(0x00); // NOP
    for (uint8_t i = 0; i < len; i++) {
        data[i] = spi_transfer(0x00);
    }
    spi_deselect();
}

uint8_t TFLORA::ReadReg (uint16_t addr) {
    uint8_t val;
    ReadRegs(addr, &val, 1);
    return val;
}

void TFLORA::wait_busy_pin()
{
    if(busy_enabled==0)
    {
      PX4_ERR("Busy pin disabled now, but waiting for it..");
      return;
    }

    uint64_t counter=0;
    while(counter < 100000000ul 
       && stm32_gpioread(GPIO_PORTI|GPIO_PIN10)) //TODO drdy pin!
    {
      counter++;
    }

    if(counter>=100000000ul)
      printf("wait.. for busy... timeout: %lld \n",counter);
}

void TFLORA::init_lmic()
{

    PX4_INFO("init_lmic");

    // LMIC init
    os_init(nullptr);
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7_BW250)); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(EU_DR_FSK,  EU_DR_FSK));      // g2-band

    // TTN uses SF9 at 869.525Mhz for its RX2 window (frequency is
    // default LoRaWAN, SF is different, but set them both to be
    // explicit).
    LMIC.dn2Freq = 869525000;
    LMIC.dn2Dr = EU_DR_SF9;

    // Set data rate for uplink
    LMIC_setDrTxpow(ordinary_dr, KEEP_TXPOWADJ);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    printf("Off to the loop!\n");
}

