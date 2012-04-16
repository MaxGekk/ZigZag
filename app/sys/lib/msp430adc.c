/*! @file   msp430adc.c
 *  @brief  Реализация интерфейса работы с АЦП12 в msp430
 *  @author Max Gekk
 *  @date   апрель 2007
 *  @version 1
 */   

#include    <_zigzag.h>
#include    <msp430.h>
#include    <zzADC.h>
#include    <syszig.h>
#include    <msp430adc.h>

#include    <platform.h>

static uint16_t    irq_mask;

result_t    adc_init( adc_mode_t mode, ... )
{
    ADC12CTL0 &= ~ENC;
    ADC12IE = 0x00;
    
    if( mode == SINGLE_CONVERSION ) {
        //ADC12CTL0  = ADC12ON | REFON | SHT0_16 | REF2_5V | MSC;
        ADC12CTL0  = ADC12ON | SHT0_16 | REF2_5V | MSC;
        ADC12CTL1  = CSTARTADDR_0 | SHS_00 | SHP | ADC12DIV_1 | ADC12SSEL_00 | CONSEQ_00;
        //ADC12MCTL0 = SREF_001 + INCH_0000 + EOS;
        ADC12MCTL0 = SREF_010 + INCH_0000 + EOS;

        ADC12IE |= 0x01;
    } else
        return EINVAL;  

    ADC12IFG = 0x00;

    return ENOERR;
}

result_t    adc_start( bool_t only_prepare )
{
    ADC12CTL0 &= ~ENC;
    //ADC12CTL0 |= (REFON | ADC12ON);
    ADC12CTL0 |= ADC12ON;

    if( only_prepare )
        return ENOERR;

    ADC12CTL0 |= ( ENC | ADC12SC );

    return ENOERR;
}

int32_t     adc_read( uint16_t channel )
{
    uint16_t i, im;

    for( i=0, im=1; i<ADC12MEM_TOTAL; i++, im <<= 1 )
        if( (im & irq_mask)&&( (adc12mctl[i]&INCH_x) == channel ) ) {
            irq_mask &= ~im;
            return adc12mem[i];
        }

    return ENOTFOUND;
}

result_t    adc_stop( bool_t force )
{
    if( ADC12CTL1 & ADC12BUSY ) {
        if( !force )
            return EBUSY;
    }
    ADC12CTL0 &= ~ENC;
    ADC12CTL0 &= ~( REFON | ADC12ON | ADC12SC );

    return ENOERR;
}

IRQ_HANDLER( ADC12_IRQ )
{
    uint16_t iv;

    iv = ADC12IV;

    if( ( 0x6 <= iv )&&( iv <= 0x24 ) ) {
        const uint16_t  channel = adc12mctl[(iv - 0x6)/2] & INCH_x;
        irq_mask = irq_mask | ADC12IFG;
        ADC12IFG = 0;
        _event_emit( ALL_OBJOFFSET, PRIORITY_HIGH-1, EV_ADC, (unidata_t)channel );
        return;
    }
}

