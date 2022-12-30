/* ==========================================================
 *    ___             _   _     _
 *   / _ \ _ __   ___| |_| |__ (_)_ __ __  __
 *  | | | | '_ \ / _ \ __| '_ \| | '_ \\ \/ /
 *  | |_| | | | |  __/ |_| | | | | | | |>  <
 *   \___/|_| |_|\___|\__|_| |_|_|_| |_/_/\_\
 *
 * Copyright Onethinx, 2018
 * All Rights Reserved
 *
 * UNPUBLISHED, LICENSED SOFTWARE.
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Onethinx BV
 *
 * ========================================================== */
#include "OnethinxCore01.h"
#include "DemoKit01.h"
#include "LoRaWAN_keys.h"
#include "cy_pdl.h"
#include "cycfg.h"
#include "stdio.h"
#include "stdlib.h"
#include "cy_syslib.h"
#include "cy_device_headers.h"
#include "cy_mcwdt.h"
#include "cy_sysint.h"

//#include "cy_syslib.c"
#include "cy_sar.h"
#include "cy_syslib.h"

#include "inttypes.h"
#define DigInpuntCount 									5		//button isr count based, om de x aantal stuur bericht
#define DigInpuntTime									2		//button isr time based in min, om de x min stuur bericht

/* Go to ../OnethinxCore/LoRaWAN_keys.h and fill in the fields of the TTN_OTAAkeys structure */
coreConfiguration_t	coreConfig = {
	.Join.KeysPtr = 		&TTN_OTAAkeys,
	.Join.DataRate =		DR_0,
	.Join.Power =			PWR_MAX,
	.Join.MAXTries = 		1,														//aantal connecteer pogingen
	.TX.Confirmed = 		true,
	.TX.DataRate = 			DR_0,
	.TX.Power = 			PWR_MAX,
	.TX.FPort = 			1,
};

/* OnethinxCore uses the following structures and variables, which should be defined globally */
coreStatus_t status;
errorStatus_t errorStatus;

typedef uint8_t byte;
typedef uint16_t word;

/*global variables*/
char RXbuffer[64], TXbuffer[16], buffer[64], Tijdelijkbuffer[11];
bool once = false, alive_flag = false, alarm_flag = false, dailya_flag = false, batterij_flag = false;
int32 adc_result = 0;															//analoge sensor 1, pin 10.2
int16 adc1_result = 0x1010;															//analoge sensor 2, pin 10.3
int16 adc2_result = 0x1010; 														//analoge sensor 3, geen vrije pin op module
int16 minval = 0, maxval = 0, teller = 0, uren, buttoncount = 0;
byte sec, min;
byte button_case = 2;
int32 som = 0, gemid;
uint32 tijd = 0;

cy_stc_scb_uart_context_t uartContext;
cy_stc_rtc_config_t dateTime; 														//rtc struct voor get time

/*mcwdt config om .c1Match te kunnen callen - luisteren*/
cy_stc_mcwdt_config_t mcwdtconfig =
{
	.c0Match = 65535U,
	.c1Match = 900U,																	//30 = 1 minuut
	.c0Mode = CY_MCWDT_MODE_NONE,
	.c1Mode = CY_MCWDT_MODE_INT,
	.c2ToggleBit = 16U,
	.c2Mode = CY_MCWDT_MODE_NONE,
	.c0ClearOnMatch = false,
	.c1ClearOnMatch = true,
	.c0c1Cascade = true,
	.c1c2Cascade = false,
};

/*mcwdt config tijd bijhouden + sensoren uitlezen*/
cy_stc_mcwdt_config_t mcwdtconfig1 =
{
	.c0Match = 32768,
	.c1Match = 20U,																	//60 = 1 minuut, enkel deze waarde aanpassen
	.c0Mode = CY_MCWDT_MODE_NONE,
	.c1Mode = CY_MCWDT_MODE_INT,
	.c2ToggleBit = 16U,
	.c2Mode = CY_MCWDT_MODE_NONE,
	.c0ClearOnMatch = true,
	.c1ClearOnMatch = true,
	.c0c1Cascade = true,
	.c1c2Cascade = false,
};

/*RTC alarm1 config*/
cy_stc_rtc_alarm_t alarmconfigdaily = //telkens opnieuw configureren bij elke full reset
{
	    .sec            =   00u,								 //waarde seconde
	    .secEn          =   CY_RTC_ALARM_ENABLE,				 //seconde match enable
	    .min            =   00u,								 //waarde min
	    .minEn          =   CY_RTC_ALARM_ENABLE,				 //min match enable
	    .hour           =   00u,								 //waarde uur
	    .hourEn         =   CY_RTC_ALARM_ENABLE,				 //uur match enable
	    .dayOfWeek      =   01u,								 //waarde dag
	    .dayOfWeekEn    =   CY_RTC_ALARM_DISABLE,				 //dag match enable
	    .date           =   01u,								 //waarde datum
	    .dateEn         =   CY_RTC_ALARM_DISABLE,				 //datum match enable
	    .month          =   01u,								 //waarde maand
	    .monthEn        =   CY_RTC_ALARM_DISABLE,				 //maand match enable
	    .almEn          =   CY_RTC_ALARM_ENABLE					 //alarm enable
};

/*RTC alarm2 config*/
cy_stc_rtc_alarm_t alarmconfigbat = //telkens opnieuw configureren bij elke full reset
{
	    .sec            =   40u,                                  //waarde seconde
	    .secEn          =   CY_RTC_ALARM_ENABLE,                  //seconde match enable
	    .min            =   02u,                                  //waarde min
	    .minEn          =   CY_RTC_ALARM_ENABLE,                  //min match enable
	    .hour           =   23u,                                  //waarde uur
	    .hourEn         =   CY_RTC_ALARM_ENABLE,                 //uur match enable
	    .dayOfWeek      =   01u,                                  //waarde dag
	    .dayOfWeekEn    =   CY_RTC_ALARM_ENABLE,                 //dag match enable
	    .date           =   01u,                                  //waarde datum
	    .dateEn         =   CY_RTC_ALARM_ENABLE,                 //datum match enable
	    .month          =   01u,                                  //waarde maand
	    .monthEn        =   CY_RTC_ALARM_ENABLE,                 //maand match enable
	    .almEn          =   CY_RTC_ALARM_ENABLE                   //alarm enable
};

/*button config*/
const cy_stc_sysint_t PIN_BUTTON_ISR_CFG =
{
	.intrSrc = ioss_interrupts_gpio_0_IRQn,
	.intrPriority = 3
};

const cy_stc_sysint_t BUTTON1_ISR_CFG =
{
	.intrSrc = ioss_interrupts_gpio_9_IRQn,
	.intrPriority = 3
};

const cy_stc_sysint_t BUTTON2_ISR_CFG =
{
	.intrSrc = ioss_interrupts_gpio_9_IRQn,
	.intrPriority = 3
};

const cy_stc_sysint_t BUTTON3_ISR_CFG =
{
	.intrSrc = ioss_interrupts_gpio_9_IRQn,
	.intrPriority = 3
};

const cy_stc_sysint_t BUTTON4_ISR_CFG =
{
	.intrSrc = ioss_interrupts_gpio_9_IRQn,
	.intrPriority = 3
};

/*MCWDT ISR config*/
const cy_stc_sysint_t MCWDT_ISR_CFG = {
    .intrSrc = srss_interrupt_mcwdt_0_IRQn,
    .intrPriority = 7
};

/*MCWDT1 ISR config*/
const cy_stc_sysint_t MCWDT_ISR_CFG1 = {
    .intrSrc = srss_interrupt_mcwdt_1_IRQn,
    .intrPriority = 7										   	//intrpriority 3 bits => max: dec = 7, bin = 111
};

/* RTC ISR config */
const cy_stc_sysint_t RTC_ISR_CFG =
{
	.intrSrc = srss_interrupt_backup_IRQn,
	.intrPriority = 7
};

/*function prototypes*/
void MCWDT_Interrupt_Handler(void);
void MCWDT_init(void);
void MCWDT_deinit(void);
void MCWDT_Interrupt_Handler1(void);
void MCWDT_init1(void);
void Tijd_bijhouden(void);
void PinButtonInterruptHandler(void);
void RTC_Interrupt_Handler(void);
void RTC_Init(void);
void MinMaxSorteer(void);
void Leds_uit(void);

/*Message struct analoog*/
typedef struct message
{
	byte MsgID_analoog; 	// Message Identification Value = 0x09
	int16 VBat; 			// Geen idee hoe te verkrijgen
	int16 AnalogIn1;
	int16 AnalogIn2;
	int16 Distance; 		// Geen idee hoe te verkrijgen
	int16 Analogin4;
}amessage;

/*Message struct min/max, eigen struct*/
typedef struct minmax  		//struct message is voorlopig gemaakt voor 1 sensor, voor meerdere => meerdere min/max waardes variables
{
	char MsdID_minmax;
	uint16_t minwaarde;
	uint16_t maxwaarde;
	uint16_t gemiddelde;
	byte seconde;
	byte minuten;
	word uur;
}minmaxwaarde;

typedef struct dagelijks {
	byte MSGID_daily;
	byte Status;
	int8 MinTemp;
	int8 Maxtemp;
	byte MinHum;
	byte MaxHum;
	byte MaxBaro;
	byte MinBaro;
	word RunHrs;
	word KM;
}TDailyRepMsg;

typedef struct dig1in {
	byte MsgID_Dig1;
	byte Mode;
	byte RepMode;
	word Counter;
	uint32_t RunTimer;
	byte State;

}TdigIn1Msg;

int main(void)
 {
	/* initialize hardware configuration + pins */
	init_cycfg_all();
	init_cycfg_pins();

	/* a delay is necessary to prevent spurious joins in the debug procedure */
	CyDelay(1000);

    /* Set up internal routing, pins, and clock-to-peripheral connections */
	init_cycfg_peripherals();

	/* Turn on AREF, then initialize SAR-ADC */
	Cy_SAR_DeInit(ADC_HW, true );
	Cy_SysAnalog_Enable();
    Cy_SAR_Init(ADC_HW, &ADC_config);

    /*interrupt knop*/
    Cy_SysInt_Init(&PIN_BUTTON_ISR_CFG, &PinButtonInterruptHandler );
    NVIC_ClearPendingIRQ(PIN_BUTTON_ISR_CFG.intrSrc);
    NVIC_EnableIRQ(PIN_BUTTON_ISR_CFG.intrSrc);

	/* enable global interrupts. */
	__enable_irq();

	LED_B_SET(LED_OFF);

	/*UART, pins 10_0 and 10_1 */
	Cy_SCB_UART_Init( UART_HW, &UART_config, &uartContext);
	Cy_SCB_UART_Enable( UART_HW );
	Cy_SCB_UART_PutString(UART_HW, "\r------------  LoRaWAN end node prototype  -------------\n" );

	RTC_Init();
	MCWDT_init1();

	/* initialize radio with parameters in coreConfig */
	status = LoRaWAN_Init(&coreConfig);

	/* display the LoRaWAN stack version */
	sprintf( buffer, "\rOnethinx LoRaWAN stack version: 0x%x\n", (int) status.system.version );
	Cy_SCB_UART_PutString(UART_HW, buffer );

	/* send join using parameters in coreConfig, blocks until either success or MAXtries (max tries, top page) */
	status = LoRaWAN_Join(true);

	/* check for successful join */
	if (!status.mac.isJoined)
	{
		Cy_SCB_UART_PutString(UART_HW, "\rDEVICE NOT JOINED\n" );

		MCWDT_init();

		LED_B_SET(LED_OFF);
		LED_R_SET(LED_OFF);
		//Cy_SysPm_SystemEnterHibernate();
		Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
	}

	else
	{
		LED_B_SET(LED_OFF);
		sprintf( buffer, "\rDevice joined network, device address: 0x%x\n", (int) status.mac.devAddr );
		Cy_SCB_UART_PutString(UART_HW, buffer );
		/*delay before first message will be sent */
		CyDelay(2000);
	}

	/*Turn on the SAR hardware. */
	Cy_SAR_Enable(ADC_HW);


	/* main loop */
	for(;;)
	{
		if(status.mac.isJoined)
		{

			   if(alarm_flag == true)										 		//ENKEL BIJ ALARM TRIGGER
				{
					if(dailya_flag == true )
					{
						Cy_SCB_UART_PutString(UART_HW, "\rdaily loop\n");

						status = LoRaWAN_Send((uint8_t*)TXbuffer, 11, true);		//daily payload send => bij het gebruiken van de onethinx struct, verander naar size 12

						Leds_uit();

						dailya_flag = false;
						alarm_flag = false;

						Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
					}
					else
					{
						Cy_SCB_UART_PutString(UART_HW, "\rbatterij loop\n");

						TXbuffer[0]=04;
						TXbuffer[1]=04;
						status = LoRaWAN_Send((uint8_t*)TXbuffer, 2, true);			//batterij payload send

						Leds_uit();

						batterij_flag = false;
						alarm_flag = false;

						Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
					}
				}

			    else if(Tijdelijkbuffer[0] != 0) 									//als er geen connectie is, daily payload (TXbuffer) = tijdelijkbuffer
				{
					status = LoRaWAN_Send((uint8_t *)Tijdelijkbuffer, 11, true);
					memset(Tijdelijkbuffer, 0, 11);									//na versturen, terug op 0 zetten
				}

			    else if(batterij_flag == true) 										//opnieuw verzenden van batterij message, geen connectie gehad.
			    {
			    	TXbuffer[0]=05;
			    	TXbuffer[1]=05;
			    	status = LoRaWAN_Send((uint8_t*)TXbuffer, 2, true);

			    	batterij_flag = false;
			    }

				else if(alive_flag == true) 										//ENKEL BIJ MCWDT SENSOR TRIGGER
				{
					alive_flag = false;

					Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
				}

				else //NORMALE VERLOOP PROGRAMMA
				{
					LED_B_SET(LED_ON);

					for(int i=0;i < 3;i++) 											 //stuurt 3 keer bericht voor luisteren naar downlink berichten
					{
						/*Startmessage om te receiven*/
						TXbuffer[0] = 0;
						status = LoRaWAN_Send((uint8_t *) TXbuffer, 1, true);

						CyDelay(1000);
						Cy_SCB_UART_PutString(UART_HW, "\r*********L04D1NG***********\n");

						CyDelay(5000);												//wachttijd tussen luisterbericht

						if(status.mac.messageReceived)
						{goto message;}
					}

					/*check message*/
					if(status.mac.messageReceived)
					{ message:
						/*ontvangen message uart*/
						Cy_SCB_UART_PutString(UART_HW, "\rMessage_ontvangen: ");
						LoRaWAN_GetRXdata((uint16_t *) RXbuffer, status.mac.bytesToRead);

						sprintf(buffer, "waarde %x\n", RXbuffer[0]);
						Cy_SCB_UART_PutString(UART_HW, buffer);

						switch(RXbuffer[0])
							{

							case 0x01:

								MCWDT_deinit();
								/*Sensor1, Sensor2 en Sensor3 ADC channel 0 & 1 & 2 as 32bit unsigned integer*/
								Cy_SAR_StartConvert(ADC_HW, CY_SAR_START_CONVERT_CONTINUOUS); //CY_SAR_START_CONVERT_SINGLE_SHOT
								Cy_SAR_IsEndConversion(ADC_HW, CY_SAR_WAIT_FOR_RESULT);
								adc_result = Cy_SAR_GetResult16(ADC_HW, 0);					  //sensor1
								adc1_result = Cy_SAR_GetResult16(ADC_HW, 1);				  //sensor2
								adc2_result = Cy_SAR_GetResult16(ADC_HW, 2);				  //sensor3

								/*output UART */
								sprintf( buffer, "\rSensor1_waarde: deci = %d / hex = %x\n", (int)adc_result, (int)adc_result );
								Cy_SCB_UART_PutString(UART_HW, buffer );
								sprintf( buffer, "\rSensor2_waarde: deci = %d / hex = %x\n", (int)adc1_result, (int)adc1_result );
								Cy_SCB_UART_PutString(UART_HW, buffer );
								sprintf( buffer, "\rExtra_waarde: deci = %d / hex = %x\n", (int)adc2_result,(int)adc2_result );
								Cy_SCB_UART_PutString(UART_HW, buffer );

								/*struct fill*/
								amessage analogemessage = {9,25,adc_result,adc1_result,250,adc2_result};

								/*debug print*/
								sprintf(buffer, "\ranalogemessage: %x %x %x %x %x %x\n", analogemessage.MsgID_analoog, analogemessage.VBat, analogemessage.AnalogIn1, analogemessage.AnalogIn2, analogemessage.Distance, analogemessage.Analogin4);
								Cy_SCB_UART_PutString(UART_HW, buffer );

								/*concatenate message*/
								TXbuffer[0] = analogemessage.MsgID_analoog;
								TXbuffer[1] = analogemessage.VBat;
								TXbuffer[2] = analogemessage.VBat >> 8;
								TXbuffer[3] = analogemessage.AnalogIn1;
								TXbuffer[4] = analogemessage.AnalogIn1 >> 8;
								TXbuffer[5] = analogemessage.AnalogIn2;
								TXbuffer[6] = analogemessage.AnalogIn2 >> 8;
								TXbuffer[7] = analogemessage.Distance;
								TXbuffer[8] = analogemessage.Distance >> 8;
								TXbuffer[9] = analogemessage.Analogin4;
								TXbuffer[10] = analogemessage.Analogin4 >> 8;

								/*debug print*/
								sprintf(buffer,"\rTXbuffer: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",TXbuffer[0],TXbuffer[1],TXbuffer[2],TXbuffer[3],TXbuffer[4],TXbuffer[5],TXbuffer[6],TXbuffer[7],TXbuffer[8],TXbuffer[9],TXbuffer[10],TXbuffer[11],TXbuffer[12],TXbuffer[13],TXbuffer[14],TXbuffer[15]);
								Cy_SCB_UART_PutString(UART_HW, buffer );

								sprintf(buffer, "\rMCWDT trigger tijd (minuten) in deci: %d / hex %x\n", RXbuffer[1], RXbuffer[1]);
								Cy_SCB_UART_PutString(UART_HW, buffer );

								/*send LoRaWAN*/
								status = LoRaWAN_Send((uint8_t *) TXbuffer, 16, true);

								MCWDT_init();

								Leds_uit();

								Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);

								break;

							case 0x03:

								MCWDT_deinit();

								/*debug print*/
								sprintf(buffer, "\rMCWDT systeem trigger tijd (minuten) in deci: %d / hex: %x\n", RXbuffer[1], RXbuffer[1]);
								Cy_SCB_UART_PutString(UART_HW, buffer);

								/*indicatie bericht, stuurt terug naar server 06 + X, X = ingestelde tijd */
								TXbuffer[0] = 0x06;
								TXbuffer[1] = RXbuffer[1];
								status = LoRaWAN_Send((uint8_t *) TXbuffer, 2, true);

								/*counter match time*/
								mcwdtconfig.c1Match = 30U * RXbuffer[1];

								MCWDT_init();

								Leds_uit();

								Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);

								break;

							case 0x04:

								MCWDT_deinit();

								gemid = som/teller;
								sec = (tijd%60);
								min = ((tijd/60)%60);
								uren = (tijd/3600);

								sprintf(buffer, "\rHuidige MinValue: %d & MaxValue: %d &AvgValue: %ld\n", minval, maxval, gemid);
								Cy_SCB_UART_PutString(UART_HW, buffer );

								TXbuffer[0] = 4;
								TXbuffer[1] = minval;
								TXbuffer[2] = minval >> 8;
								TXbuffer[3] = maxval;
 								TXbuffer[4] = maxval >> 8;
								TXbuffer[5] = gemid;
								TXbuffer[6] = gemid >>8;
								TXbuffer[7] = sec;
								TXbuffer[8] = min;
								TXbuffer[9] = uren;
								TXbuffer[10] = uren >> 8;


								status = LoRaWAN_Send((uint8_t*)TXbuffer, 11, true);

								MCWDT_init();

								Leds_uit();

								Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);

								break;

							case 0x05:

								MCWDT_deinit();

								once = false;
								som = 0;
								teller = 0;

								MCWDT_init();

								Leds_uit();

								Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);

								break;

							case 0x06:

								MCWDT_deinit();

								button_case = RXbuffer[1];

								MCWDT_init();

								Leds_uit();

								Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
								break;

							default:
								Cy_SCB_UART_PutString(UART_HW, "\r==========Commands list===========\n");
								Cy_SCB_UART_PutString(UART_HW, "\r=--------------------------------=\n");
								Cy_SCB_UART_PutString(UART_HW, "\r==0x01: message analoge sensoren==\n");
								Cy_SCB_UART_PutString(UART_HW, "\r==0x03 + 0x..:MCWDT sys wake	  ==\n");
								Cy_SCB_UART_PutString(UART_HW, "\r==0x04: huidige min/max/avg/tijd==\n");
								Cy_SCB_UART_PutString(UART_HW, "\r==0x05: manueel min/max reset   ==\n");
								Cy_SCB_UART_PutString(UART_HW, "\r==0x06 + 0x:01,02,03 button ISR ==\n");
								Cy_SCB_UART_PutString(UART_HW, "\r==================================\n");

								Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);

								break;
							}
				}

					/*geen bericht na 3 maal = go to deep sleep*/
					else
					{
						MCWDT_deinit();

						Leds_uit();

						MCWDT_init();

						Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
					}
				}
		}



		else if (!status.mac.isJoined)

		{
			Cy_SCB_UART_PutString(UART_HW, "\rReconnecting...\n");

			/* send join using parameters in coreConfig, blocks until either success or MAXtries */
			status = LoRaWAN_Join(true);

			//notcon_flag = true

			if(!status.mac.isJoined)
			{
				if(alive_flag == true)
				{
					alive_flag = false;
					Cy_SCB_UART_PutString(UART_HW,"\rnot connected alive loop\n");			//debug print
					Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
				}

				else if(dailya_flag == true)
				{
						memcpy(Tijdelijkbuffer, TXbuffer, 11); //strcpy/strlcpy/strncpy kopieert niet alles

						sprintf(buffer, "\rTijdelijkbuffer: %x %x %x %x %x %x %x %x %x %x %x\n", Tijdelijkbuffer[0],Tijdelijkbuffer[1],Tijdelijkbuffer[2],Tijdelijkbuffer[3],Tijdelijkbuffer[4],Tijdelijkbuffer[5],Tijdelijkbuffer[6],Tijdelijkbuffer[7],Tijdelijkbuffer[8],Tijdelijkbuffer[9],Tijdelijkbuffer[10]);
						Cy_SCB_UART_PutString(UART_HW, buffer );

						/*Cy_SCB_UART_PutString(UART_HW,"\rreset_tijdelijkbuffer\n");

						memset(Tijdelijkbuffer, 0, 11);

						sprintf(buffer, "\rTijdelijkbuffer: %x %x %x %x %x %x %x %x %x %x %x\n", Tijdelijkbuffer[0],Tijdelijkbuffer[1],Tijdelijkbuffer[2],Tijdelijkbuffer[3],Tijdelijkbuffer[4],Tijdelijkbuffer[5],Tijdelijkbuffer[6],Tijdelijkbuffer[7],Tijdelijkbuffer[8],Tijdelijkbuffer[9],Tijdelijkbuffer[10]);
						Cy_SCB_UART_PutString(UART_HW, buffer );*/

						dailya_flag = false;
						Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
				}

				else
				{
					Cy_SCB_UART_PutString(UART_HW,"\rMCWDT_Loop\n");						//debug print
					Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
				}
			}

		}


	}
}
/***********************************************************************************************************************************************************************/
/*																			MCWDT LUISTER FUNCTIONS 															       */
/***********************************************************************************************************************************************************************/

void MCWDT_init(void)
{
    Cy_MCWDT_Init(MCWDT_HW, &mcwdtconfig);
    Cy_MCWDT_Enable(MCWDT_HW, CY_MCWDT_CTR0, 93 /* 2 LFCLK cycles */);
    Cy_MCWDT_Enable(MCWDT_HW, CY_MCWDT_CTR1, 93 /* 2 LFCLK cycles */);

    Cy_MCWDT_SetInterruptMask(MCWDT_HW, CY_MCWDT_CTR1);

    Cy_SysInt_Init(&MCWDT_ISR_CFG, &MCWDT_Interrupt_Handler);

    NVIC_ClearPendingIRQ(MCWDT_ISR_CFG.intrSrc);
    NVIC_EnableIRQ(MCWDT_ISR_CFG.intrSrc);
}

void MCWDT_Interrupt_Handler(void)
{
    Cy_MCWDT_ClearInterrupt(MCWDT_HW, CY_MCWDT_CTR1);
    NVIC_ClearPendingIRQ(MCWDT_ISR_CFG.intrSrc);

    alive_flag = false;
    alarm_flag = false;

    Cy_SCB_UART_PutString(UART_HW, "\rMCWDT wake up\n"); //debug
}

void MCWDT_deinit(void)
{
	Cy_MCWDT_Disable(MCWDT_HW, CY_MCWDT_CTR0, 93 /* 2 LFCLK cycles */);
	Cy_MCWDT_Disable(MCWDT_HW, CY_MCWDT_CTR1, 93 /* 2 LFCLK cycles */);

	Cy_MCWDT_DeInit(MCWDT_HW);
}
/***********************************************************************************************************************************************************************/
/*																		MCWDT ALIVE TIME FUNCTIONS																       */
/***********************************************************************************************************************************************************************/

void MCWDT_init1(void)
{
    Cy_MCWDT_Init(MCWDT1_HW, &mcwdtconfig1);
    Cy_MCWDT_Enable(MCWDT1_HW, CY_MCWDT_CTR0, 93 /* 2 LFCLK cycles */);
    Cy_MCWDT_Enable(MCWDT1_HW, CY_MCWDT_CTR1, 93 /* 2 LFCLK cycles */);

    Cy_MCWDT_SetInterruptMask(MCWDT1_HW, CY_MCWDT_CTR1);

    Cy_SysInt_Init(&MCWDT_ISR_CFG1, &MCWDT_Interrupt_Handler1);

    NVIC_ClearPendingIRQ(MCWDT_ISR_CFG1.intrSrc);
    NVIC_EnableIRQ(MCWDT_ISR_CFG1.intrSrc);

}

void MCWDT_Interrupt_Handler1(void)
{
    Cy_MCWDT_ClearInterrupt(MCWDT1_HW, CY_MCWDT_CTR1);
    NVIC_ClearPendingIRQ(MCWDT_ISR_CFG1.intrSrc);

    Tijd_bijhouden();

	/*Sensor1, Sensor2 en Sensor3 ADC channel 0 & 1 & 2 as 32bit unsigned integer*/
	Cy_SAR_StartConvert(ADC_HW, CY_SAR_START_CONVERT_CONTINUOUS); //CY_SAR_START_CONVERT_SINGLE_SHOT
	Cy_SAR_IsEndConversion(ADC_HW, CY_SAR_WAIT_FOR_RESULT);
	adc_result = Cy_SAR_GetResult16(ADC_HW, 0);					  //sensor1
	//adc1_result = Cy_SAR_GetResult16(ADC_HW, 1);				  //sensor2
	//adc2_result = Cy_SAR_GetResult16(ADC_HW, 2);				  //sensor3

	/*gemiddeld*/
	som = adc_result;
	teller ++;
	gemid = som/teller;

	/*min/max*/
	MinMaxSorteer();

	sprintf(buffer, "\rAlivetrigger som: %ld, teller: %d\n",som, teller); 										//debug print
	Cy_SCB_UART_PutString(UART_HW, buffer );


	sprintf(buffer, "\rAlivetrigger MinValue: %d, MaxValue: %d, AvgValue: %ld\n", minval, maxval, gemid); 		//debug print
	Cy_SCB_UART_PutString(UART_HW, buffer );

	alive_flag = true;
	alarm_flag = false;
}
/***********************************************************************************************************************************************************************/
/*																			interrupt handler knop																	   */
/***********************************************************************************************************************************************************************/

void PinButtonInterruptHandler( void )
{
	CyDelay(200);    											//tegen debounce
	Cy_GPIO_ClearInterrupt(PIN_BUTTON_PORT, PIN_BUTTON_PIN);

	switch(button_case)
	{
	case 0x01:

		Cy_SCB_UART_PutString(UART_HW, "\rbutton een: direct\n");
		buttoncount ++;

		sprintf(buffer,"\rButtoncount: %d\n", buttoncount);
		Cy_SCB_UART_PutString(UART_HW, buffer);

		//TXbuffer[0] = ...;													//Struct maken + vullen zoals bij case 3
		status = LoRaWAN_Send((uint8_t*)TXbuffer, 10, true);

		break;

	case 0x02:

		Cy_SCB_UART_PutString(UART_HW, "\rbutton twee: time based\n");
		buttoncount++;

		sprintf(buffer,"\rButtoncount: %d\n", buttoncount);
		Cy_SCB_UART_PutString(UART_HW, buffer);

		if( (((tijd/60)%60) % DigInpuntTime) == 0)
		{
			Cy_SCB_UART_PutString(UART_HW, "\rstuur timebased bericht\n");

			TdigIn1Msg DigIn1Msg = {12, 2, 3, buttoncount, tijd, 1};

			TXbuffer[0] = DigIn1Msg.MsgID_Dig1;
			TXbuffer[1] = DigIn1Msg.Mode;
			TXbuffer[2] = DigIn1Msg.RepMode;
			TXbuffer[3] = DigIn1Msg.Counter;
			TXbuffer[4] = DigIn1Msg.Counter >> 8;
			TXbuffer[5] = DigIn1Msg.RunTimer;
			TXbuffer[6] = DigIn1Msg.RunTimer >> 8;
			TXbuffer[7]	= DigIn1Msg.RunTimer >> 16;
			TXbuffer[8] = DigIn1Msg.RunTimer >> 24;
			TXbuffer[9] = DigIn1Msg.State;

			status = LoRaWAN_Send((uint8_t*)TXbuffer, 10, true);

			buttoncount = 0;
		}

		break;

	case 0x03:
		Cy_SCB_UART_PutString(UART_HW, "\rbutton drie: countbased\n");
		buttoncount++;

		sprintf(buffer,"\rButtoncount: %d\n", buttoncount);
		Cy_SCB_UART_PutString(UART_HW, buffer);

		if((buttoncount % DigInpuntCount) == 0)
		 {

			Cy_SCB_UART_PutString(UART_HW, "\rstuur countbased bericht\n");

			TdigIn1Msg DigIn1Msg = {12, 2, 3, buttoncount, tijd, 1};

			TXbuffer[0] = DigIn1Msg.MsgID_Dig1;
			TXbuffer[1] = DigIn1Msg.Mode;
			TXbuffer[2] = DigIn1Msg.RepMode;
			TXbuffer[3] = DigIn1Msg.Counter;
			TXbuffer[4] = DigIn1Msg.Counter >> 8;
			TXbuffer[5] = DigIn1Msg.RunTimer;
			TXbuffer[6] = DigIn1Msg.RunTimer >> 8;
			TXbuffer[7]	= DigIn1Msg.RunTimer >> 16;
			TXbuffer[8] = DigIn1Msg.RunTimer >> 24;
			TXbuffer[9] = DigIn1Msg.State;

			status = LoRaWAN_Send((uint8_t*)TXbuffer, 10, true);

		 }
		break;

	default:

		Cy_SCB_UART_PutString(UART_HW, "\rbutton default\n");

		break;
	}

	alive_flag = false;
	alarm_flag = false;

}
/***********************************************************************************************************************************************************************/
/*																			 	Min/Max sorteren																	   */
/***********************************************************************************************************************************************************************/

void MinMaxSorteer (void) //voorlopig voor 1 sensor, voor meerdere => meedere minval/maxval koppelen aan verschillende adc_results
{
	if (once == false)
	{
		minval = adc_result;
		maxval = adc_result;

		once = true;
	}

	if(adc_result <= minval)
	{
		minval = adc_result;

	}
	else if(adc_result >= maxval)
	{
		maxval = adc_result;
	}
	else
	{
		Cy_SCB_UART_PutString(UART_HW, "\rgetal is niet kleiner of groter dan huidige min/max\n");
	}
}
/***********************************************************************************************************************************************************************/
/*																			Tijd bijhouden																			   */
/***********************************************************************************************************************************************************************/

void Tijd_bijhouden(void)
{
	tijd = tijd + mcwdtconfig1.c1Match;

	/*debug*/
	sprintf(buffer,"\rActivetime: %ld uur, %ld min, %ld sec\n", (tijd/3600), ((tijd/60)%60), (tijd%60));
	Cy_SCB_UART_PutString(UART_HW, buffer );

}
/***********************************************************************************************************************************************************************/
/*																			RTC FUNCTIONS																			   */
/***********************************************************************************************************************************************************************/

void RTC_Interrupt_Handler(void)
{
	Cy_RTC_Interrupt(0, false);
}

void RTC_Init(void)
{
    Cy_RTC_Init(&RTC_config);
    Cy_RTC_SetInterruptMask(CY_RTC_INTR_ALARM1|CY_RTC_INTR_ALARM2);
	Cy_SysInt_Init(&RTC_ISR_CFG,&RTC_Interrupt_Handler);
	NVIC_EnableIRQ(RTC_ISR_CFG.intrSrc);

	Cy_RTC_SetDateAndTime(&RTC_config);

    while(Cy_RTC_SetAlarmDateAndTime(&alarmconfigdaily,CY_RTC_ALARM_1) != CY_RET_SUCCESS); 			//set alarm lukt niet altijd in 1 try => while()
    while(Cy_RTC_SetAlarmDateAndTime(&alarmconfigbat,CY_RTC_ALARM_2) != CY_RET_SUCCESS);
}

/*RTC alarm 1 = daily min/max waarde + alive time message, voorlopig voor 1 sensor, voor meerdere => meerdere variables, message struct aanpassen*/
void Cy_RTC_Alarm1Interrupt1(void)
{
	sprintf(buffer, "\rALARM1_OK\n");																//debug print
	Cy_SCB_UART_PutString(UART_HW, buffer);

	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);
	NVIC_ClearPendingIRQ(RTC_ISR_CFG.intrSrc);

	once = false; 																					//reset min/max
	gemid = som/teller;

	sec = (tijd%60);
	min = ((tijd/60)%60);
	uren = (tijd/3600);

	minmaxwaarde maxmin = {6,minval,maxval,gemid,sec,min, uren}; 									//vul struct message min/max

	TXbuffer[0] = maxmin.MsdID_minmax;
	TXbuffer[1] = maxmin.minwaarde;																	//LSB
	TXbuffer[2] = maxmin.minwaarde >>8; 															//MSB
	TXbuffer[3] = maxmin.maxwaarde; 																//LSB
	TXbuffer[4] = maxmin.maxwaarde >>8; 															//MSB
	TXbuffer[5] = maxmin.gemiddelde;																//LSB
	TXbuffer[6] = maxmin.gemiddelde >> 8;															//MSB
	TXbuffer[7] = maxmin.seconde; 																	//sec
	TXbuffer[8] = maxmin.minuten;																	//min
	TXbuffer[9] = maxmin.uur;																		//LSB uur
	TXbuffer[10] = maxmin.uur >> 8;																	//MSB uur

	/*TDailyRepMsg dagwaardes = {11, 0, minval, maxval, 0, 0, 0, 0, uren, 0};					    //comment bovenste struct en gebruik deze, voor de Italks daily struct

	TXbuffer[0] = dagwaardes.MSGID_daily;
	TXbuffer[1] = dagwaardes.Status;
	TXbuffer[2] = dagwaardes.MinTemp;
	TXbuffer[3] = dagwaardes.Maxtemp;
	TXbuffer[4] = dagwaardes.MinHum;
	TXbuffer[5] = dagwaardes.MaxHum;
	TXbuffer[6] = dagwaardes.MaxBaro;
	TXbuffer[7] = dagwaardes.MinBaro;
	TXbuffer[8] = dagwaardes.RunHrs;
	TXbuffer[9] = dagwaardes.RunHrs >> 8;
	TXbuffer[10] = dagwaardes.KM;
	TXbuffer[11] = dagwaardes.KM >> 8;
	*/


	sprintf(buffer, "\rTXbuffer: %x %x %x %x %x %x %x %x %x %x %x\n", TXbuffer[0],TXbuffer[1],TXbuffer[2],TXbuffer[3],TXbuffer[4],TXbuffer[5],TXbuffer[6],TXbuffer[7],TXbuffer[8],TXbuffer[9],TXbuffer[10]);
	Cy_SCB_UART_PutString(UART_HW, buffer );

	alarm_flag = true;
	dailya_flag = true;

	/*reset variables voor gemiddelde*/
	gemid = 0;
	som = 0;
	teller = 0;
}

/*RTC alarm2 = batterij niveau message*/
void Cy_RTC_Alarm2Interrupt(void)
{
	/*debug print*/
	sprintf(buffer, "\rALARM2_OK\n");
	Cy_SCB_UART_PutString(UART_HW, buffer );

	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM2);
	NVIC_ClearPendingIRQ(RTC_ISR_CFG.intrSrc);

	alarm_flag = true;
	batterij_flag = true;
}

/***********************************************************************************************************************************************************************/
/*																			LEDS UIT																			       */
/***********************************************************************************************************************************************************************/
void Leds_uit(void)
{
	LED_B_SET(LED_OFF);
	LED_R_SET(LED_OFF);
}

