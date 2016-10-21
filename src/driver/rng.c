/**
 ****************************************************************************************
 *
 * @file rng.c
 *
 * @brief RNG (Random Nubmer Generator) driver for QN9020.
 *
 * Copyright(C) 2016 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup  RNG
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rng.h"
#if CONFIG_ENABLE_DRIVER_RNG==TRUE
#ifdef BLE_PRJ
#include "usr_design.h"
#endif
#include "adc.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
 
uint32_t rng_get(void)
{
    uint32_t rand = 0;
    int16_t buf[16];
    adc_read_configuration read_cfg;
    
    // Initialize ADC
    adc_init(ADC_DIFF_WITHOUT_BUF_DRV, ADC_CLK_1000000, ADC_INT_REF, ADC_12BIT);

    // Configure ADC mode
    read_cfg.trig_src = ADC_TRIG_SOFT;
    read_cfg.mode = CONTINUE_MOD;
    read_cfg.start_ch = RNG;
    read_cfg.end_ch = RNG;
    
    // Collect data and construct rand
    for(uint8_t i=0; i<32; i++)
    {
        adc_read(&read_cfg, buf, 16, NULL);
        
        for(uint8_t j=0; j<16; j++)
        {
            rand ^= (buf[j] & 0x1);
        }
        
        rand <<= 1;
    }
    
    // Disable ADC clock and power
    adc_clock_off();
    adc_power_off();
    
    return rand;
}


#endif /* CONFIG_ENABLE_DRIVER_RNG==TRUE */
/// @} RNG
