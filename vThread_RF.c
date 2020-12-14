
#define _GNU_SOURCE
#include <sched.h>
#include <string.h>
#include <math.h>
#ifndef BUILDOPT_MACOS
#include <malloc.h>
#endif

#include "vThread_RF.h"
#include "NPAL_thread.h"
#include "NPAL_engines.h"
#include "../../ARCH/COMMON/common_lib.h"
#include "vThreadIx_rfSrch.h"

#define VRF_FREQOFFSET_STEPSIZE		100

#define VRF_RTCHG_FREQOFFSET		0x01
#define VRF_RTCHG_GAIN				0x02
#define VRF_RTCHG_SAMPLINGRATE		0x04


#define VRF_SRCHG_BUFFEROUT_SUBF	2	
#define VRF_LOOPTIME_AVERAGE_COEFF	7

#define BN


/* virtual H/W for RF 
- Assuming that there is only one RF in the UE
- virtual H/W has one real RF device and has HAL code for controling this real HW
*/


/*     ------- RF HW related ------          */

//1. SDR RF interface
static openair0_device vrf_rfdevice; 	//rf device object
static openair0_config_t vrf_rfDevcfg[MAX_CARDS];
static uint8_t vrf_rfHwOn; 				//RF device on/off status
static double vrf_rxcFreq[4];			//current RX center frequency
static double vrf_txcFreq[4];			//current TX center frequency

//2. TX/RX chain vHW interface
static int32_t ***vrf_rxdata;
static int32_t ***vrf_txdata;
static int32_t **vrf_rxdata_bu;

static int32_t vrf_oversampled;


/* initial configuration parameters for SDR RF------ */
static unsigned int vrf_mmapped_dma;
static clock_source_t vrf_clksrc = internal;
static char vrf_config_file[1024];
static char* vrf_usrpArg = NULL;



/*     ------- HAL - HW interface related ------          */
const halRfic_woReg_t* vrf_HALix_in; 	//write only registers
halRfic_roReg_t* vrf_HALix_out;			//read only registers
halRfic_rwReg_t* vrf_HALix_buf;			//read/write registers
pthread_mutex_t* vrf_muPtr_rfHal;		//MUTEX for HAL interface
pthread_cond_t* vrf_csPtr_rfHal;		//condition signal for off -> on

typedef struct {
	uint8_t samplingMode;
	int 	samplingChangeRate_Exp; // sampling change rate (2^N)
	uint32_t samples_in_subframe;
	uint32_t samples_in_bank;
	uint32_t samples_in_slot;
	uint16_t slots_per_subframe;
	int 	 nb_slot_frame;
} vrf_sampleConfig_t;

static struct {
	int freq_offset; //delta
	int tx_sample_advance;
	int sync_offset;
	int rx_offset;
	uint8_t rxMode;
	uint8_t txMode;
	vrf_sampleConfig_t sampleConfig;
	int frame_cnt;
} vrf_rtCfg; //RFIC HW abstraction


/*  ------- other variables used in this vHW  ---------          */


/* ---------- HW - HW interfaces -----------       */

/* 1. RF - SRCH */
static vrfSrch_rfReg_t			vrfsrch_rfReg;
vrfSrch_srchReg_t*				vrfsrch_srchRegPtr=NULL;
vrfSrch_sharedReg_t				vrfsrch_sharedReg;


/* -------- 1. Initializing function codes ----------- */


//interface configuration between RF and SRCH
vrfSrch_rfReg_t* vrf_configRfReg(void)
{
	return &vrfsrch_rfReg;
}


vrfSrch_sharedReg_t* vrf_configSrchSharedReg(void)
{
	return &vrfsrch_sharedReg;
}



//initiation function for virtual RF H/W called by API
void vrf_initHwParams(unsigned int Vmmapped_dmam, int VclkSrc, char* inConfigFile, char* inUsrpArg)
{
	vrf_mmapped_dma = Vmmapped_dmam;

	vrf_clksrc = (clock_source_t)VclkSrc;

	if (inConfigFile != NULL)
		memcpy(vrf_config_file, inConfigFile, 1024*sizeof(char));
	if (inUsrpArg != NULL)
	{
		vrf_usrpArg = malloc((strlen(inUsrpArg)+1)*sizeof(char));
		strcpy(vrf_usrpArg, inUsrpArg);
	}
}

double vrf_calcSampleRate(uint8_t samplingMode)
{
	switch(samplingMode)
	{
		case VRF_SAMPLEMODE_192:
			return 1.92e6;
		
		case VRF_SAMPLEMODE_768:
			return 7.68e6;
			
		case VRF_SAMPLEMODE_1536:
			return 15.36e6;

		case VRF_SAMPLEMODE_3072:
			return 30.72e6;

		#ifdef BN
		case VRF_SAMPLEMODE_6144:
			return 61.44e6;
		#endif 
		
		default:
			printf("[HW RF] WARNING : unknown sample rate %i (default 30.72e6 is set)\n", vrf_HALix_in->samplingMode);
			break;
	}

	return 30.72e6;
}


//initializing the abstraction variable for USRP
//called within this vHW
static void vrf_init_rfDevCfg(void) {

    int card;
    int i;

    for (card=0; card<MAX_CARDS; card++)
	{
        vrf_rfDevcfg[card].mmapped_dma = vrf_mmapped_dma;
        vrf_rfDevcfg[card].configFilename = NULL;
  		vrf_rfDevcfg[card].duplex_mode = duplex_mode_FDD;
		vrf_rfDevcfg[card].Mod_id = 0;
	
		vrf_rfDevcfg[card].clock_source = vrf_clksrc;
		vrf_rfDevcfg[card].digital_freq_offset = 0;
	
		vrf_rfDevcfg[card].tx_num_channels = min(2,HW_NB_TXANT);
		vrf_rfDevcfg[card].rx_num_channels = min(2,HW_NB_RXANT);
	
		for (i=0; i<4; i++)
		{
    		vrf_rfDevcfg[card].tx_freq[i]=0.0;
    		vrf_rfDevcfg[card].rx_freq[i]=0.0;	  
	  		vrf_rfDevcfg[card].autocal[i] = 1;
	  		vrf_rfDevcfg[card].tx_gain[i] = 0.0;
	  		vrf_rfDevcfg[card].rx_gain[i] = 0.0;
		}
		
		vrf_rfDevcfg[card].configFilename = vrf_config_file;
		if (vrf_usrpArg)
		{
			vrf_rfDevcfg[card].sdr_addrs = vrf_usrpArg;
		}
    }
}

static void vrf_resetRtCfg(void)
{
	vrf_rtCfg.freq_offset = 0;
	vrf_rtCfg.sync_offset = 0;
	vrf_rtCfg.rx_offset = 0;
	vrf_rfDevcfg[0].digital_freq_offset = 0;
	vrf_HALix_out->digital_freq_offset = 0;

	AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");
	vrf_HALix_buf->gainChanged = 0;
	AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");
}

//configuration function for off-on procedures
static void vrf_configInit(void)
{
	//check other configurations
	int i;	

	AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");

	vrf_rfDevcfg[0].sample_rate = vrf_calcSampleRate(vrf_HALix_in->samplingMode);

	vrf_rtCfg.sampleConfig.samplingMode = vrf_HALix_in->samplingMode;
	vrf_rtCfg.sampleConfig.samples_in_subframe = vrf_HALix_in->samples_per_frame/10;
	vrf_rtCfg.sampleConfig.samples_in_slot = vrf_HALix_in->samples_per_slot;
	vrf_rtCfg.sampleConfig.slots_per_subframe = vrf_rtCfg.sampleConfig.samples_in_subframe/vrf_rtCfg.sampleConfig.samples_in_slot;
	vrf_rtCfg.sampleConfig.nb_slot_frame = HW_NB_SUBF_IN_FRAME * vrf_rtCfg.sampleConfig.slots_per_subframe;
	vrf_rtCfg.sampleConfig.samples_in_bank = vrf_HALix_in->samples_in_bank;
	vrf_rtCfg.sampleConfig.samplingChangeRate_Exp = 0;
	
	
	for (i=0;i<HW_NB_TXANT;i++)
	{
		vrf_rfDevcfg[0].tx_freq[i] = vrf_HALix_in->tx_freq[i];
		vrf_txcFreq[i] = vrf_HALix_in->tx_freq[i];
	}
	for (i=0;i<HW_NB_RXANT;i++)
	{
		vrf_rfDevcfg[0].rx_freq[i] = vrf_HALix_in->rx_freq[i];
		vrf_rxcFreq[i] = vrf_HALix_in->rx_freq[i];
	}

	vrf_rfDevcfg[0].rx_bw = vrf_HALix_in->rx_bw;
	vrf_rfDevcfg[0].tx_bw = vrf_HALix_in->tx_bw;

	for (i=0;i<HW_NB_RXANT;i++)
	{
		vrf_rfDevcfg[0].rx_gain[i] = vrf_HALix_in->rx_gain[i];
		vrf_rfDevcfg[0].rx_gain_offset[i] = vrf_HALix_in->rx_gain_offset[i];
	}
	for (i=0;i<HW_NB_TXANT;i++)
	{
		vrf_rfDevcfg[0].tx_gain[i] = vrf_HALix_in->tx_gain[i];
	}
	AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");

	vrf_resetRtCfg();
}









/* -------- 2. real-time processing codes ----------- */

//calculating rx offset drift decision based on rx_offset register
//inner function
int vrf_computeSamplesShift(void)
{
	
  	// compute TO compensation that should be applied for this frame
  	//HAL - HW compensation cmd first, then RF-SRCH compensation cmd later
	if (vrf_rtCfg.rx_offset > 0)
	{
		return -1;
	}
	else if (vrf_rtCfg.rx_offset < 0)
	{
		return 1;
	}
	else
	{
		//command from srcher
		AssertFatal( 0 == pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
		int tComandSrch = vrfsrch_sharedReg.timeOffset;
		AssertFatal( 0 == pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");
			
		if (tComandSrch > 0)
		{
			return -1;
		}
		else if (tComandSrch < 0)
		{
			return 1;
		}
	}
  
  	return 0;
}

//calculating the frequency offset input (into USRP)
//inner function 
static int vrf_calc_freqOffsetIn(int freq_offset)
{
	int truncFreq;

	truncFreq = (freq_offset/VRF_FREQOFFSET_STEPSIZE)*VRF_FREQOFFSET_STEPSIZE;
	return truncFreq;
}


//main function for processing the real-time register command of this virtual RF
//inner function

static uint8_t vrf_configRealTimeSync(void)
{
	uint8_t flag_cfg = 0;

	//reading register area and get values if changed
	AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");
	// -- RX offsets
	//jumping offset
	if (vrf_HALix_buf->sync_offset != 0)
	{
		vrf_rtCfg.sync_offset = vrf_HALix_buf->sync_offset;
		vrf_HALix_buf->sync_offset = 0;
		flag_cfg = 1;
	}
	AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");

	return flag_cfg;
}


static uint8_t vrf_configRealTime(void)
{
	uint8_t flag_cfg = 0;
	int i;

	//reading register area and get values if changed
	AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");
	
	// -- RX offsets
	//jumping offset
	if (vrf_HALix_buf->sync_offset != 0)
	{
		vrf_rtCfg.sync_offset = vrf_HALix_buf->sync_offset;
		vrf_HALix_buf->sync_offset = 0;
	}

	//sampling mode change : needs to be dealt first
	if (vrf_HALix_in->samplingMode != vrf_rtCfg.sampleConfig.samplingMode)
	{
		
		vrf_rfDevcfg[0].sample_rate = vrf_calcSampleRate(vrf_HALix_in->samplingMode);

		vrf_rtCfg.sampleConfig.samplingChangeRate_Exp = vrf_HALix_in->samplingMode - vrf_rtCfg.sampleConfig.samplingMode;
		vrf_rtCfg.sampleConfig.samplingMode = vrf_HALix_in->samplingMode;
		vrf_rtCfg.sampleConfig.samples_in_subframe = vrf_HALix_in->samples_per_frame/10;
		vrf_rtCfg.sampleConfig.samples_in_slot = vrf_HALix_in->samples_per_slot;
		vrf_rtCfg.sampleConfig.slots_per_subframe = vrf_rtCfg.sampleConfig.samples_in_subframe/vrf_rtCfg.sampleConfig.samples_in_slot;
		vrf_rtCfg.sampleConfig.nb_slot_frame = HW_NB_SUBF_IN_FRAME * vrf_rtCfg.sampleConfig.slots_per_subframe;
		vrf_rtCfg.sampleConfig.samples_in_bank = vrf_HALix_in->samples_in_bank;

		LOG_E(PHY, "detected sampling mode change at register level (%i)\n", vrf_rtCfg.sampleConfig.samplingMode);
		
		flag_cfg |= VRF_RTCHG_SAMPLINGRATE;
	}
	else
	{
	
		// -- Frequency offset (AFC)
		if (vrf_rtCfg.freq_offset != vrf_HALix_in->freq_offset)
		{
			vrf_rtCfg.freq_offset = vrf_HALix_in->freq_offset;
			flag_cfg |= VRF_RTCHG_FREQOFFSET;
		}
		
		// -- TX timing
		if (vrf_rtCfg.tx_sample_advance!= vrf_HALix_in->tx_sample_advance)
		{
			vrf_rtCfg.tx_sample_advance = vrf_HALix_in->tx_sample_advance;
		}
		
		//real time drift offset
		if (vrf_HALix_in->rx_offset != vrf_rtCfg.rx_offset)
		{
			vrf_rtCfg.rx_offset = vrf_HALix_in->rx_offset;
		}
		
		// -- Gains
		if (vrf_HALix_buf->gainChanged == 1)
		{
			for (i=0;i<HW_NB_RXANT;i++)
			{
				vrf_rfDevcfg[0].rx_gain[i] = vrf_HALix_in->rx_gain[i];
				vrf_rfDevcfg[0].rx_gain_offset[i] = vrf_HALix_in->rx_gain_offset[i];
			}
			for (i=0;i<HW_NB_TXANT;i++)
			{
				vrf_rfDevcfg[0].tx_gain[i] = vrf_HALix_in->tx_gain[i];
			}
			vrf_HALix_buf->gainChanged = 0;
			flag_cfg |= VRF_RTCHG_GAIN;
		}
	}
	
	// -- modes
	vrf_rtCfg.rxMode = vrf_HALix_in->rxMode;
	vrf_rtCfg.txMode = vrf_HALix_in->txMode;
	vrf_rtCfg.sampleConfig.samples_in_bank = vrf_HALix_in->samples_in_bank;
  	AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");

	if (flag_cfg & VRF_RTCHG_SAMPLINGRATE)
	{
		vrf_resetRtCfg();
	}
	//frequency offset (center freq + drift offset) config on SDR RF
	if (vrf_rtCfg.rxMode == VRF_RXMODE_SRCH && (flag_cfg & VRF_RTCHG_FREQOFFSET) )
	{
		int inFreq = vrf_calc_freqOffsetIn(vrf_rtCfg.freq_offset);

		if (vrf_rfDevcfg[0].digital_freq_offset != inFreq)
		{
			vrf_rfDevcfg[0].digital_freq_offset = inFreq;
			vrf_rfdevice.adjust_rx_freq_func(&vrf_rfdevice, vrf_rfDevcfg);

			vrf_HALix_out->digital_freq_offset = vrf_rfDevcfg[0].digital_freq_offset; //output the Doffset to the HAL layer
		}
	}
	
	//gain setting
	if (flag_cfg & VRF_RTCHG_GAIN)
	{
		vrf_rfdevice.trx_set_gains_func(&vrf_rfdevice, vrf_rfDevcfg);
	}
	
	return flag_cfg;
}

//reading on/off register in real-time
//inner function
static uint8_t vrf_readOnOff(void)
{
	uint8_t onOff;
	
	AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading onOff");
	onOff = vrf_HALix_in->onoff;
	AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading onOff");

	return onOff;
}


void vrf_updateFrameNum(void)
{
	vrf_rtCfg.frame_cnt = (vrf_rtCfg.frame_cnt + vrf_rtCfg.sampleConfig.samples_in_bank/vrf_rtCfg.sampleConfig.samples_in_subframe/HW_NB_SUBF_IN_FRAME)%1024;
}








/* --------- 3. USRP trx function (all inner functions) -------- */
//throw away samples of one frame
uint8_t vrf_trashBank(openair0_timestamp *timestamp)
{
	uint8_t undersampled=0;
	void *dummy_rx[HW_NB_RXANT];
	int trashSize = vrf_rtCfg.sampleConfig.samples_in_bank/vrf_rtCfg.sampleConfig.samples_in_subframe;

	for (int i=0; i<HW_NB_RXANT; i++)
		malloc16(dummy_rx[i], vrf_rtCfg.sampleConfig.samples_in_subframe*sizeof(int));

	for (int sf=0; sf<trashSize; sf++)
	{
		int trashUnit = vrf_rtCfg.sampleConfig.samples_in_subframe;
		if (vrf_oversampled > 0)
		{
			if (trashUnit > vrf_oversampled)
			{
				trashUnit -= vrf_oversampled;
				vrf_oversampled = 0;
			}
			else
				vrf_oversampled -= trashUnit;
			
			undersampled = 1;
		}

		if (trashUnit != 0)
		{
			vrf_rfdevice.trx_read_func(&vrf_rfdevice,
		                           timestamp,
		                           dummy_rx,
		                           trashUnit,
		                           HW_NB_RXANT);
		}
	}

	for (int i=0; i<HW_NB_RXANT; i++)
		free(dummy_rx[i]);

	return undersampled;
	
}


//throw away samples till the next frame boundary
void vrf_syncInFrame(openair0_timestamp *timestamp, int sync_offset)
{
	printf("[VRF] Resynchronizing RX by %d samples (samples in slot:%i, samples in bank : %i)\n", sync_offset, vrf_rtCfg.sampleConfig.samples_in_slot, vrf_rtCfg.sampleConfig.samples_in_bank);

	if (sync_offset < 0)
	{
		sync_offset = sync_offset + vrf_rtCfg.sampleConfig.samples_in_bank;
		vrf_updateFrameNum();
	}
	
	for ( int size = sync_offset ; size > 0 ; size -= vrf_rtCfg.sampleConfig.samples_in_slot )
	{
  		int unitTransfer = size > vrf_rtCfg.sampleConfig.samples_in_slot ? vrf_rtCfg.sampleConfig.samples_in_slot : size ;

  		AssertFatal(unitTransfer == vrf_rfdevice.trx_read_func(&vrf_rfdevice,
						                                         timestamp,
						                                         (void **)vrf_rxdata[0],
						                                         unitTransfer,
						                                         HW_NB_RXANT),
					"");
	}
}


void vrf_adjustTrashBank(openair0_timestamp *timestamp, int sampleTime)
{
	void *rxp[HW_NB_RXANT];
	int sample_diff = vhw_absreal(sampleTime);
	
	if (sampleTime != 0)
	{
		for (int i=0; i<HW_NB_RXANT; i++)
			malloc16(rxp[i],vrf_rtCfg.sampleConfig.samples_in_subframe*sizeof(int));
		
		//sampling rate reduced //need more samples
		if (sampleTime > 0)
		{
	 		AssertFatal(sample_diff == vrf_rfdevice.trx_read_func(&vrf_rfdevice,
						                                         timestamp,
						                                         rxp,
						                                         sample_diff,
						                                         HW_NB_RXANT),
						"");
			//LOG_E(PHY, "adjustTrashBank : additional sampling : %i\n", sample_diff);
		}
		//sampling rate increased //over sampled
		else if (sampleTime < 0)
		{
			//sample reset (no shift is needed)
			for (int i=0; i<HW_NB_RXANT; i++)
			{
				memset(&vrf_rxdata_bu[i][0], 0, sample_diff*sizeof(int));
			}
			vrf_oversampled = sample_diff;

			//LOG_E(PHY, "adjustTrashBank : setting oversample as %i\n", sample_diff);
		}
		
		for (int i=0;i<HW_NB_RXANT;i++)
			free(rxp[i]);
	}
	else
	{
		LOG_E(PHY, "[VRF][WARNING] failed to adjust bank ! sampletime is invalid (%i)\n", sampleTime);
	}
	
}




void vrf_adjustBank(openair0_timestamp *timestamp, int sampleTime)
{
	void *rxp[HW_NB_RXANT];
	int sample_diff = vhw_absreal(sampleTime);
	uint32_t trash_samples;


	if (vrf_rtCfg.sampleConfig.samplingChangeRate_Exp > 0)
	{
		trash_samples = (sample_diff >> vrf_rtCfg.sampleConfig.samplingChangeRate_Exp);
	}
	else
	{
		//trash_samples = (sample_diff << (-vrf_rtCfg.sampleConfig.samplingChangeRate_Exp));
		trash_samples = sample_diff / ((0x01<< (-vrf_rtCfg.sampleConfig.samplingChangeRate_Exp) ) -1);
	}

	
	if (sampleTime != 0)
	{
		
		//sampling rate reduced //need more samples
		if (sampleTime > 0)
		{
			int readSize = sampleTime;

			
			//sample shift (shift to left)
			for (int i=0; i<HW_NB_RXANT; i++)
			{
				memset(&vrf_rxdata[0][i][0], 0, sampleTime*sizeof(int));
				memcpy(&vrf_rxdata[0][i][(trash_samples - sampleTime)], 
					   &vrf_rxdata[0][i][trash_samples], 
					   (vrf_rtCfg.sampleConfig.samples_in_bank-trash_samples)*sizeof(int));
			}

			//additional sample
			for (int i=0; i<HW_NB_RXANT; i++)
		  		rxp[i] = ((void *)&vrf_rxdata[0][i][vrf_rtCfg.sampleConfig.samples_in_bank-sampleTime]);

	 		AssertFatal(readSize == vrf_rfdevice.trx_read_func(&vrf_rfdevice,
						                                         timestamp,
						                                         rxp,
						                                         readSize,
						                                         HW_NB_RXANT),
						"");

			//LOG_E(PHY, "adjustBank result : chg rate : %i, sample time : %i, sample diff : %i, trash_sample : %i shifted start point : %i, cpy size : %i\n", 
			//	vrf_rtCfg.sampleConfig.samplingChangeRate_Exp, sampleTime, sample_diff, trash_samples, (trash_samples - sampleTime), (vrf_rtCfg.sampleConfig.samples_in_bank-trash_samples));
		}
		//sampling rate increased //over sampled
		else if (sampleTime < 0)
		{
			int sample_start = trash_samples + sample_diff;
			//sample reset (no shift is needed)
			for (int i=0; i<HW_NB_RXANT; i++)
			{				
				memcpy(&vrf_rxdata_bu[i][0], &vrf_rxdata[0][i][trash_samples], (vrf_rtCfg.sampleConfig.samples_in_bank-trash_samples)*sizeof(int));
				
				memset(&vrf_rxdata[0][i][0], 0, vrf_rtCfg.sampleConfig.samples_in_bank*sizeof(int));
				memcpy(&vrf_rxdata[0][i][sample_start], &vrf_rxdata_bu[i][0], (vrf_rtCfg.sampleConfig.samples_in_bank - sample_start)*sizeof(int));
				memcpy(&vrf_rxdata_bu[i][0], &vrf_rxdata_bu[i][vrf_rtCfg.sampleConfig.samples_in_bank - sample_start], sample_diff*sizeof(int));
			}
			
			vrf_oversampled = sample_diff;

			//LOG_E(PHY, "adjustBank result : chg rate : %i, sample time : %i, sample diff : %i, trash_sample : %i, backuped sample start : %i, cpy size : %i\n", 
			//	vrf_rtCfg.sampleConfig.samplingChangeRate_Exp, sampleTime, sample_diff, trash_samples, vrf_rtCfg.sampleConfig.samples_in_bank - sample_diff, (vrf_rtCfg.sampleConfig.samples_in_bank-trash_samples-sample_diff));
		}
	}
	else
	{
		LOG_E(PHY, "[VRF][WARNING] failed to adjust bank ! sampletime is invalid (%i)\n", sampleTime);
	}
	
}

//read one slot samples
void vrf_readBank(openair0_timestamp *timestamp)
{
	void *rxp[HW_NB_RXANT];
	int readSize = vrf_rtCfg.sampleConfig.samples_in_bank/vrf_rtCfg.sampleConfig.samples_in_subframe;
	for(int x=0; x<readSize; x++)
	{
		for (int i=0; i<HW_NB_RXANT; i++)
	  		rxp[i] = ((void *)&vrf_rxdata[0][i][0]) + 4*x*vrf_rtCfg.sampleConfig.samples_in_subframe;

 		AssertFatal( vrf_rtCfg.sampleConfig.samples_in_subframe == vrf_rfdevice.trx_read_func(&vrf_rfdevice,
						                                        timestamp,
						                                        rxp,
						                                        vrf_rtCfg.sampleConfig.samples_in_subframe,
						                                        HW_NB_RXANT), 
					"");

	}
	vrfsrch_rfReg.roReg.bankSize[0] = vrf_rtCfg.sampleConfig.samples_in_bank;
}

int vrf_checkBankStatus(int processId)
{
	int processStatus;

	AssertFatal( 0 == pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
	processStatus = (vrfsrch_sharedReg.processBitmap & (1<<processId));
	AssertFatal( 0 == pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");

	return (processStatus);
}

int vrf_checkSrchStatus(void)
{
	AssertFatal ( 0== pthread_mutex_lock(&vrfsrch_srchRegPtr->roReg.regMutex), ""); //SYNC register mutex
	uint8_t srcherStatus = vrfsrch_srchRegPtr->roReg.hwStatus;
	AssertFatal ( 0== pthread_mutex_unlock(&vrfsrch_srchRegPtr->roReg.regMutex), ""); //SYNC register mutex
	
	if (srcherStatus != hwIx_on)
		return -1;

	return 0;
}



void vrf_irqSrch(int processId)
{
	AssertFatal( 0 == pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
	vrfsrch_sharedReg.processBitmap |= (1<<processId);
	AssertFatal( 0 == pthread_cond_signal(&vrfsrch_srchRegPtr->woReg.irq_srchInst), "");
	AssertFatal( 0 == pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");
}


int32_t vrf_calcSampleTime(int time_us)
{
	int sample_out = (int)(time_us*(vrf_rfDevcfg[0].sample_rate/1000000.0));
	return (sample_out - sample_out%8);
}

static int16_t __attribute__((aligned(16))) minusW4[4][8] = {
	 {32767, 0, 32767, 0, 32767, 0, 32767, 0},
	 {32767, 0, 0, 32767, -32768, 0, 0, -32768}, 
	 {32767, 0, -32768, 0, 32767, 0, -32768, 0},
	 {32767, 0, 0, -32768, -32768, 0, 0, 32767}
};

static int16_t __attribute__((aligned(16))) shuffleW4[4][8] = {
	 {0, 32767, 0, 32767, 0, 32767, 0, 32767},
	 {0, 32767, -32768, 0, 0, -32768, 32767, 0}, 
	 {0, 32767, 0, -32768, 0, 32767, 0, -32768},
	 {0, 32767, 32767, 0, 0, -32768, -32768, 0}
};

int16_t* FFT4(int16_t *x, int16_t *y) 
{ 
	 int32_t* x32, *pcumulre, *pcumulim; 
	 __m128i x128, mmcumulre1, mmcumulim1, mmcumulre2, mmcumulim2, mmtmpre, mmtmpim, *result_FFT4;  
	 __m128i *Wptr; 


	 x32 = (int32_t*)x;
	 result_FFT4 = (__m128i*)y;
	 
	 mmcumulre1= _mm_setzero_si128();
	 mmcumulim1= _mm_setzero_si128();
	 
	 for(int i=0;i<4;i++)
	 { 
		  x128 = _mm_set_epi32(x32[i], x32[i], x32[i], x32[i]);
		  Wptr = (__m128i*)minusW4[i];
		  mmtmpre=_mm_madd_epi16(x128,Wptr[0]); 


		  Wptr = (__m128i*)shuffleW4[i];
		  mmtmpim=_mm_madd_epi16(x128,Wptr[0]); 

		  mmtmpre=_mm_srai_epi32(mmtmpre, 15);
		  mmtmpim=_mm_srai_epi32(mmtmpim, 15);
		   
		  mmcumulre1 = _mm_add_epi32(mmcumulre1, mmtmpre);
		  mmcumulim1 = _mm_add_epi32(mmcumulim1, mmtmpim);
 	 }

	 mmcumulre2=_mm_packs_epi32(mmcumulre1, mmcumulre1); 
	 mmcumulim2=_mm_packs_epi32(mmcumulim1, mmcumulim1);

	 result_FFT4[0] =_mm_unpacklo_epi16(mmcumulre2, mmcumulim2);
	 	 
	 _mm_empty();
	 _m_empty();
	 
	 return((int16_t*) result_FFT4);


}

void dft16(int16_t *x,int16_t *y);
#define shiftright_int16_simd256(a,shift) _mm256_srai_epi16(a,shift)
void dft64(int16_t *x,int16_t *y,int scale);
void vhw_dumpDataAndExit(short* ptr, int size, const char* filepath, const char* msg);

void vrf_RSSIsniffing(uint8_t samplingMode, double rx_Freq, double rx_bw, int Nfft, double rxgain, float *RSSI)
{	
	int64_t sumRSSI;
	int N_repeat;
	void *rxp[HW_NB_RXANT];
	int16_t* vrf_rxdata_ptr = &vrf_rxdata[0][0][0]; 
	int16_t vrf_rxdata_power[VRF_NB_MAXSAMPLES]; 
	__m256i vrf_rxdata_power_256[VRF_NB_MAXSAMPLES/8] __attribute__((aligned(32)));
	__m256i *vrf_rxdata_256_ptr __attribute__((aligned(16))) = vrf_rxdata_ptr;	
	int32_t* vrf_rxdata_power_p32;
	int Nsamples;
	int fftsize, mvf_tap, Noffset;
	int max_mvav = 0;
	int max_index = 0;
	int16_t *pdetect_rxdata; 
	int16_t detect_rxdataF[10000] __attribute__((aligned(32)));
	int16_t *pdetect_rxdataF; 
	int32_t detect_rxdataF_power[64] = { 0,};
	
	vhw_sysTimeStamp_t onTimer;
	vhw_setTimeStamp(&onTimer);

	if( !(Nfft == 1 || Nfft == 4 || Nfft == 16 || Nfft == 64) )
	{
		LOG_E(PHY, "[ERROR][sniffing] wrong devided section size!! : %i\n", Nfft);
		return -1;
	}

	switch (samplingMode) {
		case 4:
			fftsize = 1024;
			break;
		case 3:
			fftsize = 512;
			break;
		case 2:
			fftsize = 256;
			break;
		case 0:
			fftsize = 64;
			break;
		}

	Nsamples = vrf_calcSampleRate(samplingMode)*2/100; 
	mvf_tap = fftsize*4;
	N_repeat = mvf_tap/Nfft; 
	int64_t mvav[VRF_NB_MAXSAMPLES]; 
	int16_t detect_rxdata[mvf_tap*2] __attribute__((aligned(32)));
	
	//power of rxdata
	for(int i=0; i < Nsamples/8; i++ )
		vrf_rxdata_power_256[i] = _mm256_madd_epi16(vrf_rxdata_256_ptr[i],vrf_rxdata_256_ptr[i]);		
	
	vrf_rxdata_power_p32 = vrf_rxdata_power_256;
	
	for(int i=0; i<mvf_tap; i++)
		mvav[0] += vrf_rxdata_power_p32[i];

	for(int i=1; i<(Nsamples - mvf_tap); i++)
	{
		mvav[i] = mvav[i - 1] - vrf_rxdata_power_p32[i - 1] + vrf_rxdata_power_p32[i + mvf_tap - 1]; 
		if(mvav[i] > max_mvav)
		{
			max_mvav = mvav[i];
			max_index = i;
		}			
	}
	
	Noffset = max_index*2 - fftsize*2;
	for(int i=0; i<mvf_tap*2; i++)
		detect_rxdata[i] = vrf_rxdata_ptr[i + Noffset];
	
	pdetect_rxdata = (int16_t*)detect_rxdata;
	pdetect_rxdataF = (int16_t*)detect_rxdataF;

	for(int i=0; i <N_repeat; i++)
	{
		//make function for get dft responding to fftsize(N)!
		 switch (Nfft) {
		 	case 1:
				pdetect_rxdataF = pdetect_rxdata;
		 		break;
		 	case 4:
				FFT4(pdetect_rxdata + i*2*Nfft, pdetect_rxdataF + i*2*Nfft);	
				break;
			case 16:
			  	dft16(pdetect_rxdata + i*2*Nfft, pdetect_rxdataF + i*2*Nfft);
			   	break;
			case 64:
			 	dft64(pdetect_rxdata + i*2*Nfft, pdetect_rxdataF + i*2*Nfft,1);
				break;
		 	}
		 for(int j=0; j<Nfft; j++)
		{
			detect_rxdataF_power[j] += (int64_t)(pdetect_rxdataF[i*Nfft*2 + 2*j]*pdetect_rxdataF[i*Nfft*2 + 2*j] + pdetect_rxdataF[i*Nfft*2 + (2*j+1)]*pdetect_rxdataF[i*Nfft*2 + (2*j+1)]);//SIMD쓰는게 나으려나..
		}
	}

	for(int i=0; i<Nfft; i++)
		RSSI[i] = 10*log10(((double)detect_rxdataF_power[i]/32768)/N_repeat)-rxgain;

	FILE *fp = fopen("./timing_BN_SIMDfull","a");
	int time_us = vhw_getTime_us(&onTimer);
	fprintf(fp, "%d ", time_us);
	for(int i=0; i<Nfft; i++)
		fprintf(fp, "%3.1f ", RSSI[i]);
	fprintf(fp,"\n");
	fclose(fp);
	
	vhw_printTimeStamp(&onTimer, "RSSIsniffing time");
	
	#ifdef DEBUG
	for(int i=0; i<Nfft; i++)
	{
		LOG_E(PHY, "\x1b[36m RSSI[%i] : %f, ", i, RSSI[i]);
	}
	printf("\n");
	#endif
	

	return;
}



void *vRFIC_mainThread(void *arg)
{
	uint8_t onOffCfg = 0;
	openair0_timestamp timestamp;
	void *rxp[HW_NB_RXANT], *txp[HW_NB_TXANT];
	int i;
	char threadname[128];
	int rx_offset_diff;
	vhw_sysTimeStamp_t loopTimer;
	int avSamplingTime=-1;
	int srchTimeAvCoeff=VRF_LOOPTIME_AVERAGE_COEFF;
	uint8_t flag_rtcfg;

	vrf_rxdata = (int32_t***) vhw_malloc16_clear( VRF_NB_SAMPLEBANK * sizeof(int32_t**) );
	for (int i=0;i<VRF_NB_SAMPLEBANK;i++)
	{
		vrf_rxdata[i] = (int32_t**) vhw_malloc16_clear( HW_NB_RXANT * sizeof(int32_t*) );
		for (int j=0;j<HW_NB_RXANT;j++)
		{
			vrf_rxdata[i][j] = (int32_t*) vhw_malloc16_clear( VRF_NB_MAXSAMPLES * sizeof(int32_t) );
		}
	}

	vrf_rxdata_bu = (int32_t**) vhw_malloc16_clear( HW_NB_RXANT * sizeof(int32_t*) );
	for (int j=0;j<HW_NB_RXANT;j++)
	{
		vrf_rxdata_bu[j] = (int32_t*) vhw_malloc16_clear( VRF_NB_MAXSAMPLES * sizeof(int32_t) );
	}

	vrf_txdata = (int32_t***) vhw_malloc16_clear( VRF_NB_SAMPLEBANK * sizeof(int32_t**) );
	for (int i=0;i<VRF_NB_SAMPLEBANK;i++)
	{
		vrf_txdata[i] = (int32_t**) vhw_malloc16_clear( HW_NB_RXANT * sizeof(int32_t*) );
		for (int j=0;j<HW_NB_RXANT;j++)
		{
			vrf_txdata[i][j] = (int32_t*) vhw_malloc16_clear( VRF_NB_MAXSAMPLES * sizeof(int32_t) );
		}
	}

	

	// -------------- HAL interface configuration -------------------
	vrf_HALix_in = &( ((ix_halRfic_t*)arg)->woReg  );
	vrf_HALix_out = &( ((ix_halRfic_t*)arg)->roReg );
	vrf_HALix_buf = &( ((ix_halRfic_t*)arg)->rwReg );
	vrf_muPtr_rfHal = (pthread_mutex_t*)&(((ix_halRfic_t*)arg)->mutex_rfHal);
	vrf_csPtr_rfHal = (pthread_cond_t*)&(((ix_halRfic_t*)arg)->cond_rfHal);

	vrf_HALix_out->hwStatus = rfst_null;
	vrf_HALix_out->ptr_rxdata = (const int32_t***) vrf_rxdata;

	vrf_HALix_out->ptr_txdata = (const int32_t***) vrf_txdata;


	//HW interface configuration
	// vs. Srcher
	vrfsrch_rfReg.roReg.rxData = (int***) vrf_rxdata;
	vrfsrch_rfReg.roReg.bankSize = vhw_malloc16_clear( VRF_NB_SAMPLEBANK * sizeof(uint32_t));
	vrfsrch_rfReg.roReg.hfslot_nb = vhw_malloc16_clear( VRF_NB_SAMPLEBANK * sizeof(int16_t));
	vrfsrch_rfReg.roReg.hfframe_nb = vhw_malloc16_clear( VRF_NB_SAMPLEBANK * sizeof(int));
	vrfsrch_rfReg.roReg.rxGain = vhw_malloc16_clear( VRF_NB_SAMPLEBANK * sizeof(int));
	vrfsrch_rfReg.roReg.slot_offset = vhw_malloc16_clear( VRF_NB_SAMPLEBANK * sizeof(int));
	vrfsrch_rfReg.roReg.hwStatus = hwIx_null;
	pthread_mutex_init(&(vrfsrch_rfReg.roReg.regMutex), NULL);

	vrfsrch_sharedReg.timeOffset = 0;
	vrfsrch_sharedReg.freqOffset = 0;
	vrfsrch_sharedReg.processBitmap= 0;
	pthread_mutex_init(&(vrfsrch_sharedReg.sharedMutex), NULL);

	AssertFatal ( NULL != (vrfsrch_srchRegPtr = vsrch_configSrchReg()), "[vHW][ERROR] error in configuration RF-SRCH register : pointer is NULL!\n");

	
	// --------------- thread setting -------------------
	sprintf(threadname, "RF virtual HW thread on UE side");
	vhw_initvThread(100000, 500000, HW_FIFO_PRIORITY, threadname);

	
	int slot_nr=-1;

	//SDR RF initialization
	vrf_init_rfDevCfg();
	vrf_configInit();
	
	AssertFatal(0== openair0_device_load(&vrf_rfdevice, &vrf_rfDevcfg[0]), "");	
	
	vrf_rfdevice.host_type = RAU_HOST;
	vrf_HALix_out->hwStatus = rfst_off;
	AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_rfReg.roReg.regMutex)), "");
	vrfsrch_rfReg.roReg.hwStatus = hwIx_off;
	AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_rfReg.roReg.regMutex)), "");

	printf("[vHW] RF virtual HW is initialized, waiting for on signal\n");
	
	//outer loop for off waiting
	while (!NPAL_checkEnd())
	{
		//off state loop (stay until on signal comes)
		AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "");
		while (vrf_HALix_in->onoff == 0)
		      pthread_cond_wait( vrf_csPtr_rfHal, vrf_muPtr_rfHal );
		AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "");

		
		//check on/off register and process it
		//if register is 'on', then do static configuration and apply to the SDR RF device (init and start)
		if (vrf_rfHwOn == 0)
		{
			vhw_sysTimeStamp_t onTimer;
			vhw_setTimeStamp(&onTimer);
			
		  	vrf_configInit();			
			
			vrf_rfdevice.trx_configure_on(&vrf_rfdevice, vrf_rfDevcfg);
			AssertFatal(vrf_rfdevice.trx_start_func(&vrf_rfdevice) == 0, "Could not start the RF device\n");

			AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "");
			vrf_HALix_out->hwStatus = rfst_on;
			AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "");
			AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_rfReg.roReg.regMutex)), "");
			vrfsrch_rfReg.roReg.hwStatus = hwIx_on;
			AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_rfReg.roReg.regMutex)), "");
			vrf_rfHwOn = 1;
			vrf_rtCfg.frame_cnt = 0;

			printf("[vHW] >>>>>>>>>>>>>>>>> RF virtual HW is on! (carrier DL freq : %lf) - on time : %4d us\n", 
				vrf_rfDevcfg[0].rx_freq[0], vhw_getTime_us(&onTimer));
		}

		
		//inner loop for on operation
		while ( !NPAL_checkEnd() && 
				 vrf_rfHwOn == 1 && 
				 (onOffCfg = vrf_readOnOff()) == 1 )
		{

			flag_rtcfg = vrf_configRealTimeSync();


			//frame number offset
			AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
			if (vrfsrch_sharedReg.frameOffset != 0)
			{
				vrf_rtCfg.frame_cnt = (vrf_rtCfg.frame_cnt + vrfsrch_sharedReg.frameOffset + 1024)%1024;
				LOG_E(PHY, "[vRF] frame configuration : %i (%i)\n", vrf_rtCfg.frame_cnt, vrfsrch_sharedReg.frameOffset);				
				vrfsrch_sharedReg.frameOffset = 0;
			}
			AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");

			//timing compensation
			if (flag_rtcfg == 1)
			{
				vrf_syncInFrame(&timestamp, vrf_rtCfg.sync_offset);
				vrf_rtCfg.sync_offset = 0;

				//force to initialize the timing compensation command from srcher (HAL cmd first!)
				AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
				vrfsrch_sharedReg.timeOffset = 0;
				AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");				
			}
			else if (vrf_rtCfg.rxMode == VRF_RXMODE_SRCH || vrf_rtCfg.rxMode == VRF_RXMODE_SNIFF )
			{
				AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
				int timeOffset = vrfsrch_sharedReg.timeOffset;
				AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");	
				if (timeOffset != 0)
				{
					LOG_E(PHY, "[vRF] timing compensation command from srcher : %i\n", timeOffset);
					vrf_syncInFrame(&timestamp, timeOffset);
					AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
					vrfsrch_sharedReg.timeOffset = 0;
					AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");
				}
			}

		
			//read the real-time configure register and apply in real-time
			flag_rtcfg = vrf_configRealTime();


			//frequency compensation
			if (vrf_rtCfg.rxMode == VRF_RXMODE_SYNC)
			{
				AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
				int freq_offset_cmd = vrfsrch_sharedReg.freqOffset;
				AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");	

				if (freq_offset_cmd != 0)
				{
					int inFreq = vrf_calc_freqOffsetIn(vrf_rfDevcfg[0].digital_freq_offset+freq_offset_cmd);

					if (vrf_rfDevcfg[0].digital_freq_offset != inFreq)
					{
						//LOG_E(PHY, "[vRF] FO in online mode : %i\n", inFreq);
						
						vrf_rfDevcfg[0].digital_freq_offset = inFreq;
						vrf_rfdevice.adjust_rx_freq_func(&vrf_rfdevice, vrf_rfDevcfg);

						AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
						vrfsrch_sharedReg.freqOffset = 0; //output the Doffset to the HAL layer
						AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");	
					}
				}
			}

			if (vrf_rtCfg.rxMode == VRF_RXMODE_SRCH )
			{
				vhw_setTimeStamp(&loopTimer);
			}

			//sampling rate change
			if (flag_rtcfg & VRF_RTCHG_SAMPLINGRATE)
			{
				//off first : off only when Power optimization is off
				//AssertFatal(vrf_rfdevice.trx_stop_func(&vrf_rfdevice) == 0, "Could not stop the RF device\n");
				//then on
				vrf_rfdevice.trx_configure_on(&vrf_rfdevice, vrf_rfDevcfg);
				AssertFatal(vrf_rfdevice.trx_start_func(&vrf_rfdevice) == 0, "Could not start the RF device\n");
				
				LOG_E(PHY, "[VRF] Sampling rate is changed (new sampling rate : %i)\n", 
					vrf_rtCfg.sampleConfig.samplingMode);
			}


			//mode 1 : not synchronized (maybe initial cell search case)
			if (vrf_rtCfg.rxMode == VRF_RXMODE_SRCH)
			{
				int time_oneLoop;
				uint8_t underSampled=0;

			    if (vrf_checkBankStatus(0) == 0)
				{  	// we can invoke the synch
			      	// grab 10 ms of signal and wakeup synch thread
					vrf_readBank(&timestamp);
					
					time_oneLoop = vhw_getTime_us(&loopTimer);

					if (vrf_checkSrchStatus() == 0)
					{
						if (avSamplingTime != 0 && (flag_rtcfg & VRF_RTCHG_SAMPLINGRATE))
						{
							int time_sample = vrf_calcSampleTime(avSamplingTime- time_oneLoop);
							//LOG_E(PHY, "loop time (read) : %i, timestamp : %lli\n", time_oneLoop, timestamp);
							vrf_adjustBank(&timestamp, time_sample);
						}

						
						AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_rfReg.roReg.regMutex)), "");
						vrfsrch_rfReg.roReg.hfframe_nb[0] = (vrf_rtCfg.frame_cnt<<1);	
						vrfsrch_rfReg.roReg.rxGain[0] = vrf_rfDevcfg[0].rx_gain[0] - vrf_rfDevcfg[0].rx_gain_offset[0];
						AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_rfReg.roReg.regMutex)), "");

					
						vrf_irqSrch(0);
					}
			    } 
				else
				{
		      		// grab 10 ms of signal into dummy buffer to wait result of sync detection
					underSampled = vrf_trashBank(&timestamp);

					time_oneLoop = vhw_getTime_us(&loopTimer);
					
					if (avSamplingTime != 0 && (flag_rtcfg & VRF_RTCHG_SAMPLINGRATE))
					{
						int time_sample = vrf_calcSampleTime(avSamplingTime - time_oneLoop);
						//LOG_E(PHY, "loop time (trash) : %i, timestamp : %lli\n", time_oneLoop, timestamp);
						vrf_adjustTrashBank(&timestamp, time_sample);
					}
		    	}
				vrf_updateFrameNum();

				if ((flag_rtcfg & VRF_RTCHG_SAMPLINGRATE) == 0 &&
					underSampled == 0)
				{
					if (avSamplingTime==-1)
						avSamplingTime = 0;
					else if (avSamplingTime == 0)
						avSamplingTime = time_oneLoop;
					else
						avSamplingTime = avSamplingTime*(srchTimeAvCoeff/10.0) + time_oneLoop*(1-srchTimeAvCoeff/10.0);

					//LOG_E(PHY, "sampling time : %i %i, oversample : %i, timestamp : %lli\n", avSamplingTime, time_oneLoop, vrf_oversampled, timestamp);
				}
				
				flag_rtcfg = 0;

		  		continue;
			}
			//mode2: for RSSI sniffing
			else if(vrf_rtCfg.rxMode == VRF_RXMODE_SNIFF)
			{

				vrf_readBank(&timestamp);
				
				AssertFatal(vrf_rfdevice.trx_stop_func(&vrf_rfdevice) == 0, "Could not stop the RF device\n");
				AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "");
				vrf_HALix_out->hwStatus = rfst_off;
				AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "");
				AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_rfReg.roReg.regMutex)), "");
				vrfsrch_rfReg.roReg.hwStatus = hwIx_off;
				AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_rfReg.roReg.regMutex)), "");
				vrf_rfHwOn = 0;
				
				vrf_RSSIsniffing(vrf_HALix_in->samplingMode, vrf_HALix_in->rx_freq[0], vrf_HALix_in->rx_bw, vrf_HALix_in->Nfft, (vrf_rfDevcfg[0].rx_gain[0] - vrf_rfDevcfg[0].rx_gain_offset[0]), vrf_HALix_out->ptr_RSSI);

		  		continue;
			}

			//mode 2-2 : synchronized, and not the first RX stream
			// 2-2-1 : multiple thread management for channel processings a(TXRX seems to be processed by 2 threads)
			slot_nr++;
			// 2-2-2 : timing calculation (subframe, frame, slot numbers, ...)
			//current timing setting

			//frame numbering (increment of 1 after subframe num goes back to 0)
			if (slot_nr == vrf_rtCfg.sampleConfig.nb_slot_frame)
			{
				slot_nr %= vrf_rtCfg.sampleConfig.nb_slot_frame;
				vrf_rtCfg.frame_cnt = (vrf_rtCfg.frame_cnt + 1)%1024;
			}

			// 2-2-3 : RX buffering and TX symbol determination
			//pointer setting
			//shifted structure (first CP+symbol in the first subframe is received initially
			//for other slot, get second symbol ~ the first symbol at the next slot
			//last slot, get one less symbol, and then get one long CP symbol
	  		for (i=0; i<HW_NB_RXANT; i++)
	    		rxp[i] = (void *)&vrf_rxdata[0][i][slot_nr*vrf_rtCfg.sampleConfig.samples_in_bank];
	  		for (i=0; i<HW_NB_TXANT; i++)
	    		txp[i] = (void *)&vrf_txdata[0][i][((slot_nr+2)%HW_NB_SUBF_IN_FRAME)*vrf_rtCfg.sampleConfig.samples_in_bank];


			//read/write size determination (based on slot number)
	  		int readBlockSize, writeBlockSize;
		    readBlockSize = vrf_rtCfg.sampleConfig.samples_in_bank;
			writeBlockSize = vrf_rtCfg.sampleConfig.samples_in_bank;
			if (slot_nr == (vrf_rtCfg.sampleConfig.nb_slot_frame - 1) ) //??
			{
			    rx_offset_diff = vrf_computeSamplesShift();
			    readBlockSize -= rx_offset_diff;
			    writeBlockSize -= rx_offset_diff;
	  		}
			// 2-2-4 : RX/TX buffering
	  		AssertFatal(readBlockSize ==
	              		vrf_rfdevice.trx_read_func(&vrf_rfdevice,
			                                         &timestamp,
			                                         rxp,
			                                         readBlockSize,
			                                         HW_NB_RXANT),"");
			vrfsrch_rfReg.roReg.bankSize[0] = vrf_rtCfg.sampleConfig.samples_in_bank;
			if (vrf_rtCfg.txMode == VRF_TXMODE_INSYNC)
			{
	  			AssertFatal( writeBlockSize ==
	               		 vrf_rfdevice.trx_write_func(&vrf_rfdevice,
									                   timestamp+
									                   2*vrf_rtCfg.sampleConfig.samples_in_bank-
									                   //(UE->frame_parms.ofdm_symbol_size - UE->frame_parms.nb_prefix_samples0) -
									                   vrf_rfDevcfg[0].tx_sample_advance + vrf_rtCfg.tx_sample_advance,
									                   txp,
									                   writeBlockSize,
									                   HW_NB_TXANT,
									                   1),"");
			}

			// 2-2-5 : passing to the TXRX process
			// operate on thread sf mod 2

			//interrupt srcher for online synch
			if (vrf_checkBankStatus(0) == 0)
			{  	
				AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_rfReg.roReg.regMutex)), "");
				vrfsrch_rfReg.roReg.hfslot_nb[0] = slot_nr%(vrf_rtCfg.sampleConfig.nb_slot_frame/2);					//half frame slot index
				vrfsrch_rfReg.roReg.hfframe_nb[0] = (vrf_rtCfg.frame_cnt<<1) + slot_nr/(vrf_rtCfg.sampleConfig.nb_slot_frame/2);	//half frame count (SFN *2 + hf)
				vrfsrch_rfReg.roReg.slot_offset[0] = slot_nr*vrf_rtCfg.sampleConfig.samples_in_slot;
				vrfsrch_rfReg.roReg.rxGain[0] = vrf_rfDevcfg[0].rx_gain[0] - vrf_rfDevcfg[0].rx_gain_offset[0];
				AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_rfReg.roReg.regMutex)), "");
				vrf_irqSrch(0);
			}
		} //HW on loop

		if (vrf_rfHwOn == 1)
		{
			AssertFatal(vrf_rfdevice.trx_stop_func(&vrf_rfdevice) == 0, "Could not stop the RF device\n");
			AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "");
			vrf_HALix_out->hwStatus = rfst_off;
			AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "");
			AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_rfReg.roReg.regMutex)), "");
			vrfsrch_rfReg.roReg.hwStatus = hwIx_off;
			AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_rfReg.roReg.regMutex)), "");
			vrf_rfHwOn = 0;

			avSamplingTime = -1;

			printf("[vHW] RF virtual HW is off!\n");
		}
	}

	if (vrf_rfdevice.trx_end_func)
		  vrf_rfdevice.trx_end_func(&vrf_rfdevice);

	return NULL;
}
