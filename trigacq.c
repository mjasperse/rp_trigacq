#include "fpga.h"
#include "calib.h"
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>

#define SIGNAL_LENGTH OSC_FPGA_SIG_LEN

rp_calib_params_t calibr;

int process_sig(float *output, int *in_cha_signal, int *in_chb_signal, int dec_factor)
{
    float smpl_period = c_osc_fpga_smpl_period * dec_factor;
    float t_factor = smpl_period * 1e3 /*ms*/;
    int   in_idx, t_idx, wr_ptr_curr;
	
	// NB: NO TRIGGER ADJUSTMENT
    osc_fpga_get_wr_ptr(&wr_ptr_curr, &in_idx);
    
	float adc_max_v1 = osc_fpga_calc_adc_max_v(calibr.fe_ch1_fs_g_hi,0);
	float adc_max_v2 = osc_fpga_calc_adc_max_v(calibr.fe_ch2_fs_g_hi,0);
	
	for(t_idx=0; t_idx < SIGNAL_LENGTH; ++t_idx, ++in_idx)
	{
        /* Wrap the pointer */
		if(in_idx >= OSC_FPGA_SIG_LEN) in_idx = in_idx % OSC_FPGA_SIG_LEN;
		/* Write the output array */
		output[0] = t_idx * t_factor;
		output[1] = osc_fpga_cnv_cnt_to_v(in_cha_signal[in_idx], adc_max_v1, calibr.fe_ch1_dc_offs/*calib_dc_off*/, 0/*user_dc_off*/);
		output[2] = osc_fpga_cnv_cnt_to_v(in_chb_signal[in_idx], adc_max_v2, calibr.fe_ch2_dc_offs/*calib_dc_off*/, 0/*user_dc_off*/);
        output += 3;
	}

    return 0;
}


int main(int nargs, char** args)
{	
	// initialise device
	fprintf(stderr,"Initialising\n");
	assert(osc_fpga_init()==0);
	fprintf(stderr,"Resetting\n");
	osc_fpga_reset();
	fprintf(stderr,"Calibrating\n");
	rp_read_calib_params(&calibr);
	
	// configure acquisition settings
	int trig_chan = 1;
	int decim_enum = 0;
	if (nargs>1)
		decim_enum = atoi(args[1]);
	if ((decim_enum<0)||(decim_enum> 5))
		decim_enum = 0;
	int decim = osc_fpga_cnv_time_range_to_dec(decim_enum);
	osc_fpga_update_params(0, trig_chan, 0, 
                           0, 0.5, decim_enum,
                           1, 1,
                           0, 0,
                           0, 0,
                           0, 0,
                           0, 0,
                           0);
	float fs = 125.0e6 / decim;
	fprintf(stderr,"Decim %u, expected time %.2f ms\n", decim, 1e9/fs);
	
	// GO!!
	int trig = osc_fpga_cnv_trig_source(0 /*WAIT*/, trig_chan, 0 /*PGT*/);
	fprintf(stderr,"Arming trigger\n");
	osc_fpga_arm_trigger();
	osc_fpga_set_trigger(trig);
	usleep(1);
	fprintf(stderr,"Waiting for data...  ");
	
	// wait for trigger
	int retries = 60*100; // 60s
	while (retries && !osc_fpga_triggered())
	{
		usleep(10000);
		--retries;
		if (retries % 100 == 0)
			fprintf(stderr, "#");
	}
	fprintf(stderr,"\n");
	// waited too long, abort
	if (retries == 0)
	{
		fprintf(stderr,"Aborted acquisition.\n");
		osc_fpga_exit();
		return 1;
	}
	
	// get pointer to data
	int *ptrA, *ptrB;
	osc_fpga_get_sig_ptr(&ptrA, &ptrB);
	fprintf(stderr,"Data pointers %p %p\n", ptrA, ptrB);
	// copy data to local memory
	float *output = calloc(SIGNAL_LENGTH, 3*sizeof(float));
	process_sig(output, ptrA, ptrB, osc_fpga_cnv_time_range_to_dec(decim_enum));
	// binary ejection to stdout
	fwrite(output,3*sizeof(float),SIGNAL_LENGTH,stdout);
	fprintf(stderr,"Done\n");
	
	// clean-up
	free(output);
	osc_fpga_exit();
	return 0;
}
