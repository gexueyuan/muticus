#include <string.h>
#include <rtthread.h>
#include "board.h"

#ifdef RT_USING_SPI
#include "stm32f20x_40x_spi.h"

/*
SPI2_MOSI: PB15
SPI2_MISO: PB14
SPI2_SCK : PB13

CS0: PG10  SPI FLASH
CS1: PB12  TOUCH
CS2: PG7   WIFI
*/
static void rt_hw_spi2_init(void)
{
    /* register spi bus */
    {
        static struct stm32_spi_bus stm32_spi;
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        /*!< SPI SCK pin configuration */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        /* Connect alternate function */
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

        stm32_spi_register(SPI2, &stm32_spi, "spi2");
    }

//    /* attach cs */
//    {
//        static struct rt_spi_device spi_device;
//        static struct stm32_spi_cs  spi_cs;

//        GPIO_InitTypeDef GPIO_InitStructure;

//        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

//        /* spi21: PG10 */
//        spi_cs.GPIOx = GPIOB;
//        spi_cs.GPIO_Pin = GPIO_Pin_12;
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

//        GPIO_InitStructure.GPIO_Pin = spi_cs.GPIO_Pin;
//        GPIO_SetBits(spi_cs.GPIOx, spi_cs.GPIO_Pin);
//        GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);

//        rt_spi_bus_attach_device(&spi_device, "MOTION", "spi2", (void*)&spi_cs);
//    }

}

static void rt_hw_spi1_init(void)
{
    /* register spi bus */
    {
        static struct stm32_spi_bus stm32_spi;
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        /*!< SPI SCK pin configuration */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        /* Connect alternate function */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

        stm32_spi_register(SPI1, &stm32_spi, "spi1");
    }

    /* attach cs */
    {
        static struct rt_spi_device spi_device;
        static struct stm32_spi_cs  spi_cs;

        GPIO_InitTypeDef GPIO_InitStructure;

        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        /* spi21: PG10 */
        spi_cs.GPIOx = GPIOE;
        spi_cs.GPIO_Pin = GPIO_Pin_3;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

        GPIO_InitStructure.GPIO_Pin = spi_cs.GPIO_Pin;
        GPIO_SetBits(spi_cs.GPIOx, spi_cs.GPIO_Pin);
        GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);

        rt_spi_bus_attach_device(&spi_device, "MOTION", "spi1", (void*)&spi_cs);
    }
}


#endif /* RT_USING_SPI */


static uint8_t device_eletronic_signature[12];

void des_init(void)
{
    #define DES_BASE 0x1FFF7A10

    volatile uint32_t *des_reg = (uint32_t *)DES_BASE;

    memcpy(&device_eletronic_signature[0], (void *)des_reg, 4);
    memcpy(&device_eletronic_signature[4], (void *)(des_reg+1), 4);
    memcpy(&device_eletronic_signature[8], (void *)(des_reg+2), 4);
}

uint8_t des(int offset)
{
    return device_eletronic_signature[offset];
}

void rt_platform_init(void)
{
	#ifdef RT_USING_SPI
		//rt_hw_spi2_init();
		//rt_hw_spi1_init();
	#endif /* RT_USING_SPI */
	
    rt_device_init_all();
    des_init();
}

