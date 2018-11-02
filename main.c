/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORMOSIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUMOSING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * MOSISCLAIMED. IN NO EVENT SHALL NORMOSIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY MOSIRECT, INMOSIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUMOSING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUMOSING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"

#define BIT8_VALID(BYTE)                  ((BYTE&0x80)? 1: 0)

void delay(uint32_t ms)
{
	
		while(ms--)
		{
			int i = 1000;
			while(i--);		
		}
}


void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
   

}


const int CSN  	= 29; 
const int MISO 	= 28;  
const int MOSI 	= 4;   
const int CLK  	= 3;


static void csn_low(void)
{
    nrf_gpio_pin_write(CSN, 0);
}

static void csn_high(void)
{
    nrf_gpio_pin_write(CSN, 1);
}

static uint8_t spi_rw(uint8_t value)
{
    for(int i=0; i<8; i++)          
    {
      nrf_gpio_pin_write(MOSI, BIT8_VALID(value));        
      value <<= 1;            
      nrf_gpio_pin_write(CLK, 1);                
      value |= nrf_gpio_pin_read(MISO);         
      nrf_gpio_pin_write(CLK, 0);              
      nrf_delay_us(2);
    }
    return(value);           
}


static void spi_drv_init(void)
{
#ifdef SOFT_SPI
    nrf_gpio_cfg_input(MISO, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_output(CSN);
    nrf_gpio_cfg_output(MOSI);
    nrf_gpio_cfg_output(CLK);
    nrf_gpio_pin_write(CSN, 1);         // SPI禁止
    nrf_gpio_pin_write(CLK, 0);         // SPI时钟置低
	
#else	
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;//29
    spi_config.miso_pin = SPI_MISO_PIN;//28
    spi_config.mosi_pin = SPI_MOSI_PIN;//4
    spi_config.sck_pin  = SPI_SCK_PIN;//3
	
	spi_config.frequency = NRF_DRV_SPI_FREQ_125K;
	spi_config.mode         = NRF_DRV_SPI_MODE_0;
	spi_config.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
	
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
#endif
	
}


uint8_t spi_read_one(uint8_t addr)
{
#ifdef SOFT_SPI
	uint8_t ret = 0;
	csn_low();
	spi_rw(0x0b);
	ret = spi_rw(addr);
	csn_high();
	return ret;
#else
	uint8_t m_tx_buf[2] = {0x0b, addr};
	uint8_t m_rx_buf[2] = {0};
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 2));
	while (!spi_xfer_done)
	{
		__WFE();
	}
	return m_rx_buf[1];
#endif
}

int16_t spi_read_two(uint8_t addr)
{
#ifdef SOTF_SPI
	int16_t ret = 0;
	uint16_t ret_l = 0;
	uint16_t ret_h = 0;
	csn_low();
	spi_rw(0x0b);
	spi_rw(addr);
	ret_l = spi_rw(0);
	ret_h = spi_rw(0);
	csn_high();
	ret_h <<= 8;
	ret = ret_l + ret_h;
	return ret;
#else
	uint8_t m_tx_buf[4] = {0x0b, addr, 0, 0};
	uint8_t m_rx_buf[4] = {0};
	int16_t ret = 0;
	uint16_t ret_l = 0;
	uint16_t ret_h = 0;
	
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 4, m_rx_buf, 4));
	while (!spi_xfer_done)
	{
		__WFE();
	}
	ret_l = m_rx_buf[2];
	ret_h = m_rx_buf[3];
	ret_h <<= 8;
	ret = ret_l + ret_h;
	return ret;
#endif
}

void spi_write_one(uint8_t addr, uint8_t value)
{
#ifdef SOFT_SPI
	csn_low();
	spi_rw(0x0A);
	spi_rw(addr);
	spi_rw(value);
	csn_high();
#else
	uint8_t m_tx_buf[3] = {0x0A, addr, value};
	uint8_t m_rx_buf[3] = {0};
	
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 3, m_rx_buf, 3));
	while (!spi_xfer_done)
	{
		__WFE();
	}
#endif
}

void spi_write_two(uint8_t addr, int16_t twoRegValue)
{
#ifdef SOFT_SPI
	uint8_t twoRegValueH = twoRegValue >> 8;
	uint8_t twoRegValueL = twoRegValue;
	csn_low();
	spi_rw(0x0A);
	spi_rw(addr);
	spi_rw(twoRegValueL);
	spi_rw(twoRegValueH);
	csn_high();
#else
	uint8_t twoRegValueH = twoRegValue >> 8;
	uint8_t twoRegValueL = twoRegValue;
	
	uint8_t m_tx_buf[4] = {0x0A, addr, twoRegValueL, twoRegValueH};
	uint8_t m_rx_buf[4] = {0};
	
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 4, m_rx_buf, 4));
	while (!spi_xfer_done)
	{
		__WFE();
	}
#endif
}



void begin(void) 
{
	// soft reset
	spi_write_one(0x1F, 0x52);  // Write to SOFT RESET, "R"
	delay(10000);
	NRF_LOG_INFO("Soft Reset\n");
} 

void beginMeasure()
{
	uint8_t temp = spi_read_one(0x2D);	// read Reg 2D before modifying for measure mode

	// turn on measurement mode
	uint8_t tempwrite = temp | 0x02;			// turn on measurement bit in Reg 2D
	spi_write_one(0x2D, tempwrite); // Write to POWER_CTL_REG, Measurement Mode
		
  	delay(1000);	
	temp = spi_read_one(0x2D);
	NRF_LOG_INFO("Reg 2D after = %d", temp);	
}

int16_t readXData()
{
	int16_t XDATA = spi_read_two(0x0E);
	
	return XDATA;
}

int16_t readYData()
{
	int16_t YDATA = spi_read_two(0x10);
	return YDATA;
}

int16_t readZData()
{
	int16_t ZDATA = spi_read_two(0x12);
	return ZDATA;
}

int16_t readTemp()
{
	int16_t TEMP = spi_read_two(0x14);
	NRF_LOG_INFO("TEMP = %d",TEMP);
	return TEMP;
}



int main(void)
{
   

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();


	spi_drv_init();
    NRF_LOG_INFO("SPI example started.");
	begin();
	beginMeasure();
	NRF_LOG_FLUSH();
    while (1)
    {
        short x = readXData();
		short y = readYData();
		short z = readZData();
		
		NRF_LOG_INFO("x %d,y %d,z %d",x,y,z);	
        NRF_LOG_FLUSH();
    }
}
