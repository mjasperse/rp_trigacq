#include "fpga.h"
#include "calib.h"
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#define SIGNAL_LENGTH OSC_FPGA_SIG_LEN

rp_calib_params_t calibr;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
volatile int retries;

// dirty dirty global variables
#define MASTER_CLOCK 125.0e6
float fs = MASTER_CLOCK;
float dt = 1/MASTER_CLOCK;
float *dataA = NULL;
float *dataB = NULL;

int serve(int port); // forward declaration

int process_sig(int *in_cha_signal, int *in_chb_signal, int dec_factor)
{
    float smpl_period = c_osc_fpga_smpl_period * dec_factor;
    dt = smpl_period /*ms*/;
    
	// NB: NO TRIGGER ADJUSTMENT
    int in_idx, t_idx, wr_ptr_curr;
	osc_fpga_get_wr_ptr(&wr_ptr_curr, &in_idx);
	fprintf(stderr,"Trigger locations %i %i\n", wr_ptr_curr, in_idx);
    
	float adc_max_v1 = osc_fpga_calc_adc_max_v(calibr.fe_ch1_fs_g_hi,0);
	float adc_max_v2 = osc_fpga_calc_adc_max_v(calibr.fe_ch2_fs_g_hi,0);
	
	if (!dataA)	dataA = calloc(SIGNAL_LENGTH, sizeof(float));
	if (!dataB)	dataB = calloc(SIGNAL_LENGTH, sizeof(float));
	for (t_idx=0; t_idx < SIGNAL_LENGTH; ++t_idx, ++in_idx)
	{
        /* Wrap the pointer */
		if(in_idx >= OSC_FPGA_SIG_LEN) in_idx = in_idx % OSC_FPGA_SIG_LEN;
		/* Write the output array */
		dataA[t_idx] = osc_fpga_cnv_cnt_to_v(in_cha_signal[in_idx], adc_max_v1, calibr.fe_ch1_dc_offs/*calib_dc_off*/, 0/*user_dc_off*/);
		dataB[t_idx] = osc_fpga_cnv_cnt_to_v(in_chb_signal[in_idx], adc_max_v2, calibr.fe_ch2_dc_offs/*calib_dc_off*/, 0/*user_dc_off*/);
	}
    return 0;
}

int acquire(int trig_chan, int decim_enum, int timeout)
{
	int ret = 0;
	int decim = osc_fpga_cnv_time_range_to_dec(decim_enum);
	osc_fpga_update_params(0, trig_chan, 0, 
                           0, 0.5, decim_enum,
                           1, 1,
                           0, 0,
                           0, 0,
                           0, 0,
                           0, 0,
                           0);
	fs = MASTER_CLOCK / decim;
	fprintf(stderr,"Decim %u, expected time %.2f ms\n", decim, 1e9/fs);
	
	// GO!!
	int trig = osc_fpga_cnv_trig_source(0 /*WAIT*/, trig_chan, 0 /*PGT*/);
	fprintf(stderr,"Arming trigger\n");
	osc_fpga_arm_trigger();
	osc_fpga_set_trigger(trig);
	usleep(1);
	fprintf(stderr,"Waiting for data (%is timeout)...  ", timeout);
	
	// wait for trigger
	pthread_mutex_lock(&mutex);
	if (retries < 0) ret = -1;
	retries = timeout*100;	// timeout in s
	pthread_mutex_unlock(&mutex);
	
	while (!ret && !osc_fpga_triggered())
	{
		usleep(10000);
		if (retries % 100 == 0)
			fprintf(stderr, "#");
		pthread_mutex_lock(&mutex);
		--retries;
		if (retries <= 0)
		{
			fprintf(stderr," >> Timed out! ");
			ret = -1;
		}
		pthread_mutex_unlock(&mutex);
	}
	fprintf(stderr,"\n");
	// waited too long, abort
	if (ret < 0)
	{
		fprintf(stderr,"Aborted acquisition.  ");
		return -1;
	}
	
	// get pointer to data
	int *ptrA, *ptrB;
	osc_fpga_get_sig_ptr(&ptrA, &ptrB);
	fprintf(stderr,"Data pointers %p %p\n", ptrA, ptrB);
	// transform to analog units
	fprintf(stderr,"Processing signal\n");
	process_sig(ptrA, ptrB, osc_fpga_cnv_time_range_to_dec(decim_enum));
	fprintf(stderr,"Done processing\n");
	return SIGNAL_LENGTH;
}

#include <signal.h>

int main(int nargs, char** args)
{	
	int ret = 0;
	// initialise device
	fprintf(stderr,"Initialising\n");
	assert(osc_fpga_init()==0);
	fprintf(stderr,"Resetting\n");
	osc_fpga_reset();
	fprintf(stderr,"Calibrating\n");
	rp_read_calib_params(&calibr);
	
	if ((nargs>1) && ((strcmp(args[1],"-s") == 0)||(strcmp(args[1],"--serve") == 0)))
	{
		// I hate my life
		signal(SIGPIPE, SIG_IGN);
		// now serve already
		int port = 5001;
		if (nargs>2) port = atoi(args[2]);
		ret = serve(port);
	}
	else
	{
		// configure acquisition settings
		int trig_chan = 1;
		int decim_enum = 0;
		if (nargs>1)
			decim_enum = atoi(args[1]);
		if ((decim_enum<0)||(decim_enum> 5))
			decim_enum = 0;
		// acquire the data
		ret = acquire(trig_chan, decim_enum, 60);
		float data[3];
		// write to stdout
		for (int i = 0; i < SIGNAL_LENGTH; ++i)
		{
			data[0] = i * dt;
			data[1] = dataA[i];
			data[2] = dataB[i];
			fwrite(data,3,sizeof(float),stdout);
		}
	}
	osc_fpga_exit();
	return ret;
}


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

void* socket_monitor(void* pdata)
{
	int fd = *(int*)pdata;
	fprintf(stderr,"Monitor started\n");
	// try to receive nothing
	while (1)
	{
		pthread_mutex_lock(&mutex);
		int val = -retries;
		// has acquisition stopped?
		if (val >= 0) break;
		// try to send a status message
		int ret = send(fd, &val, sizeof(val), 0);
		// did it work?
		if (ret < 0)
		{
			// comms are broken, so abort
			fprintf(stderr,"\n>> Comms failurs, %i ", errno);
			break;
		}
		pthread_mutex_unlock(&mutex);
		// take a nap
		usleep(100000);	// 100ms
	}
	if (retries == 0)
		fprintf(stderr,"\n>> Monitor trigger ");
	retries = 0;	// reset (also breaks acquistion loop)
	pthread_mutex_unlock(&mutex);
	return NULL;
}

// based on http://www.cs.ucsb.edu/~almeroth/classes/W01.176B/hw2/examples/tcp-server.c
int serve(int port)
{
	int ret = 0;
	int listenfd;
	struct sockaddr_in servaddr, cliaddr;
	char msg[1024];
	pthread_t monitor;
	
	fprintf(stderr,"Starting server\n");
	listenfd = socket(PF_INET, SOCK_STREAM, 0);
	if (listenfd < 0) {
		fprintf(stderr, "Failed to create socket, %d\n", errno);
		goto cleanup;
	}
	// ask kernel to reuse port where possible
	const int enabled = 1;
	setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
	
	bzero(&servaddr, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servaddr.sin_port = htons(port);
	fprintf(stderr,"Binding to port %i\n", port);
	ret = bind(listenfd, (struct sockaddr *)&servaddr, sizeof(servaddr));
	if (ret < 0) {
		fprintf(stderr, "Bind failed, %d\n", errno);
		goto cleanup;
	}
	
	ret = listen(listenfd, 1);
	if (ret < 0) {
		fprintf(stderr, "Listen failed, %d\n", errno);
		goto cleanup;
	}
	fprintf(stderr,"Listening for connection\n");
	
	char cliip[INET6_ADDRSTRLEN + 1];
	while (ret >= 0)
	{
		ret = 0;
		socklen_t clilen = sizeof(cliaddr);
		bzero(&clilen, sizeof(clilen));
		int fd = accept(listenfd, (struct sockaddr *)&cliaddr, &clilen);
		// http://stackoverflow.com/questions/2064636/getting-the-source-address-of-an-incoming-socket-connection
		inet_ntop(AF_INET, &cliaddr.sin_addr, cliip, sizeof(cliip));
		fprintf(stderr,">> Connected to '%s'\n",cliip);
		while (ret == 0)
		{
			int n = recv(fd, msg, 1024, 0);
			if (n <= 0) break;	// error/disconnection
			// ensure null-termination (not guaranteed)
			n = strnlen(msg,n);
			if (msg[n-1]=='\n')	--n;
			if (msg[n-1]=='\r') --n;
			msg[n] = '\0';
			// blank string?
			if (n == 0) continue;
			if (!strcmp(msg,"QUIT"))
			{
				fprintf(stderr, "Received QUIT\n");
				ret = -1;
				break;
			}
			// permit echo message
			if (!strncmp(msg,"ECHO",4))
			{
				fprintf(stderr, "Replied to ECHO\n");
				send(fd, msg, n, 0);
				continue;
			}
			// expect line beginning with ACQ
			if (!strncmp(msg,"ACQ",3))
			{
				int trig_chan = 1;
				int decim_enum = 0;
				int timeout = 60;
				sscanf(msg, "ACQ %i %i %i", &trig_chan, &decim_enum, &timeout);
				// launch the monitor thread
				retries = timeout*100;
				if (pthread_create(&monitor, NULL, &socket_monitor, &fd))
				{
					fprintf(stderr, "Failed to launch monitor thread\n");
					break;
				}
				// do acquisition
				ret = acquire(trig_chan,decim_enum,timeout);
				if (ret < 0)
					fprintf(stderr,"Acquisiton failed, %i\n", ret);
				// clean up
				pthread_mutex_lock(&mutex);
				retries = -1;
				pthread_mutex_unlock(&mutex);
				// monitor should definitely fire by now
				pthread_join(monitor,NULL);
				// transmit data to client
				if (ret > 0) 
				{
					fprintf(stderr, "Sending data to client\n");
					// MILLIONS OF PEACHES, PEACHES FOR ME, MILLIONS OF PEACHES, PEACHES FOR FREE
					int len = SIGNAL_LENGTH;
					ret = send(fd, &len, sizeof(int), 0);	// send # samples
					ret = send(fd, &fs, sizeof(float), 0);	// send sample rate
					int nb = len*sizeof(float);
					ret = send(fd, dataA, nb, 0);
					if (ret != nb)	perror("Failed to send Channel A");
					ret = send(fd, dataB, nb, 0);
					if (ret != nb)	perror("Failed to send Channel B");
				}
				// need to reset "ret" now else server will quit
				ret = 0;
			}
			else
			{
				fprintf(stderr, "Invalid command '%s'\n", msg);
				break;
			}
		}
		fprintf(stderr,"Disconnected\n");
		close(fd);
	}
	ret = 0;
cleanup:
	close(listenfd);
	return ret;
}
