/* keyd.c */
/*
This file is part of a program that implements a Software-Defined Radio.

Copyright (C) 2004, 2005, 2006 by Frank Brickle, AB2KT and Bob McGwier, N4HY

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

The authors can be reached by email at

ab2kt@arrl.net
or
rwmcgwier@comcast.net

or by paper mail at

The DTTS Microwave Society
6 Kathleen Place
Bridgewater, NJ 08807
*/

//#include <linux/rtc.h>
#include <signal.h>
#include <time.h>

#include <fromsys.h>
#include <banal.h>
#include <splitfields.h>
#include <datatypes.h>
#include <bufvec.h>
#include <cxops.h>
#include <ringb.h>
#include <oscillator.h>
#include <cwtones.h>
#include <pthread.h>
#include <semaphore.h>
#include <keyer.h>
#include <spottone.h>
#include <sdrexport.h>
unsigned int timerid = 0;

static pthread_mutex_t cs_cw_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t update_ok_mutex = PTHREAD_MUTEX_INITIALIZER;

REAL SAMP_RATE = 48000.0;
// # times key is sampled per sec
// > 64 requires root on Linux
int key_poll_period = 1;
//#define RTC_RATE (64)
BOOLEAN HiPerformance = FALSE;

// # samples generated during 1 clock tick at RTC_RATE
//#define TONE_SIZE (SAMP_RATE / RTC_RATE)
unsigned int TONE_SIZE = 48;
unsigned int SIZEBUF = 1024;
// ring buffer size; > 1 sec at this sr
//#define RING_SIZE (1<<020)
//#define RING_SIZE (1<<017)
#define RING_SIZE 16384

#define NANO_SEC 1000000000
#define CLOCKID CLOCK_REALTIME
#define SIG SIGRTMIN

KeyerState ks;
KeyerLogic kl;

static pthread_t play, key, timer;
sem_t *clock_fired, *keyer_started, *poll_fired;

ringb_float_t *lring, *rring;


CWToneGen gen;
static BOOLEAN playing = FALSE, iambic = FALSE, bug = FALSE, 
	cw_ring_reset = FALSE;
static REAL wpm = 18.0, freq = 600.0, ramp = 5.0, gain = 0.0;

void StartKeyer();

//------------------------------------------------------------

#ifndef INTERLEAVED
void
CWtoneExchange (float *bufl, float *bufr, int nframes)
{
  size_t numsamps, bytesize = sizeof (float) * nframes;
  
  if (cw_ring_reset)
    {
	  size_t reset_size = max(SIZEBUF,(unsigned)nframes);
      cw_ring_reset = FALSE;
 	  pthread_mutex_lock(&cs_cw_mutex);
      //ringb_float_restart (lring, reset_size);
      //ringb_float_restart (rring, reset_size);
	  ringb_float_reset(lring);
	  ringb_float_reset(rring);
      memset (bufl, 0, bytesize);
      memset (bufr, 0, bytesize);
	  pthread_mutex_unlock(&cs_cw_mutex);
      return;
    }
  if ((numsamps = ringb_float_read_space (lring)) < (size_t) nframes)
    {
      memset (bufl, 0, bytesize);
      memset (bufr, 0, bytesize);
/*      if (numsamps != 0)
	  {
          EnterCriticalSection (cs_cw);
		  ringb_float_read (lring, bufl + nframes - numsamps, numsamps);
		  ringb_float_read (rring, bufr + nframes - numsamps, numsamps);
		  LeaveCriticalSection (cs_cw);
	  }
	  */
    }
  else
    {
	  pthread_mutex_lock(&cs_cw_mutex);
      ringb_float_read (lring, bufl, nframes);
      ringb_float_read (rring, bufr, nframes);
	  pthread_mutex_unlock(&cs_cw_mutex);
    }
	
}

// generated tone -> output ringbuffer
void
send_tone (void)
{
  if (ringb_float_write_space (lring) < TONE_SIZE)
    {
      cw_ring_reset = TRUE;
    }
  else
    {
      int i;
      pthread_mutex_lock(&cs_cw_mutex);
	  correctIQ(gen->buf, tx.iqfix);
      for (i = 0; i < gen->size; i++)
	  {
		float l = (float) CXBreal (gen->buf, i),
			r = (float) CXBimag (gen->buf, i);
		ringb_float_write (lring, (float *) &l, 1);
		ringb_float_write (rring, (float *) &r, 1);
	  }
      pthread_mutex_unlock(&cs_cw_mutex);
    }
}

// silence -> output ringbuffer
void
send_silence (void)
{
  if (ringb_float_write_space (lring) < TONE_SIZE)
    {
      cw_ring_reset = TRUE;
    }
  else
    {
      int i;
      pthread_mutex_lock(&cs_cw_mutex);
      for (i = 0; i < gen->size; i++)
	  {
		float zero = 0.0;
		ringb_float_write (lring, &zero, 1);
		ringb_float_write (rring, &zero, 1);
	  }
      pthread_mutex_unlock(&cs_cw_mutex);
    }
}

//------------------------------------------------------------------------


#else
void
CWtoneExchange (float *bufl, int nframes, int numchans)
{
  size_t numframes, bytesize = sizeof (float) * nframes * numchans;
  float *locbuf = bufl;
  if ((numframes =
       ringb_float_read_space (lring)) < (size_t) numchans * nframes)
    {
      pthread_mutex_lock(&cs_cw_mutex);
      memset (bufl, 0, bytesize);
      if (numframes != 0)
		{
			int i, j;
			numframes /= numchans;

			for (i = 0; i < nframes - (int) numframes; i++)
			{
				float l, r;
				ringb_float_read (lring, &l, 1);
				ringb_float_read (lring, &r, 1);
				for (j = 0; j < numchans; j += 2, locbuf += 2)
				{
					locbuf[0] = l;
					locbuf[1] = r;
				}
			}
		}
      pthread_mutex_unlock(&cs_cw_mutex);
    }
  else
    {
      int i, j;
      pthread_mutex_lock(&cs_cw_mutex);
      for (i = 0; i < nframes; i++)
	  {
		float l, r;
		ringb_float_read (lring, &l, 1);
		ringb_float_read (lring, &r, 1);
		for (j = 0; j < numchans; j += 2, locbuf += 2)
			{
			locbuf[0] = l;
			locbuf[1] = r;
			}
	  }
      pthread_mutex_unlock(&cs_cw_mutex);
    }
}


// generated tone -> output ringbuffer
void
send_tone (void)
{
  if (ringb_float_write_space (lring) < 2 * TONE_SIZE)
    {
      //write(2, "overrun tone\n", 13);
      pthread_mutex_lock(&cs_cw_mutex);
      ringb_float_restart (lring, SIZEBUF);
      pthread_mutex_unlock(&cs_cw_mutex);
    }
  else
    {
      pthread_mutex_lock(&cs_cw_mutex);
      ringb_float_write (lring, (float *) CXBbase (gen->buf), 2 * gen->size);
#pragma message("info: need IQ correction in CW xmit path (kd5tfd 12 Nov 2006)") 
      pthread_mutex_lock(&cs_cw_mutex);
    }
}

// silence -> output ringbuffer
void
send_silence (void)
{
  if (ringb_float_write_space (lring) < 2 * TONE_SIZE)
    {
      //write(2, "overrun zero\n", 13);
      pthread_mutex_lock(&cs_cw_mutex);
      ringb_float_restart (lring, SIZEBUF);
      pthread_mutex_unlock(&cs_cw_mutex);
    }
  else
    {
      int i;
      pthread_mutex_lock(&cs_cw_mutex);
      for (i = 0; i < 2 * gen->size; i++)
	{
	  float zero = 0.0;
	  ringb_float_write (lring, &zero, 1);
	}
      pthread_mutex_unlock(&cs_cw_mutex);
    }
}

//------------------------------------------------------------------------

#endif
//------------------------------------------------------------

// sound/silence generation
// tone turned on/off asynchronously
static void
timer_callback(int sig, siginfo_t *si, void *uc)
{
  sem_post (poll_fired);
  signal(sig, SIG_IGN);
}



void
sound_thread_keyd (void)
{
  for (;;)
    {
      sem_wait (clock_fired);

      if (playing)
		{

			// CWTone keeps playing for awhile after it's turned off,
			// in order to allow for a decay envelope;
			// returns FALSE when it's actually done.
			playing = CWTone (gen);
            pthread_mutex_lock(&update_ok_mutex);
			send_tone ();
            pthread_mutex_unlock(&update_ok_mutex);
		}
		else
		{
			pthread_mutex_lock(&update_ok_mutex);
			send_silence ();
			// only let updates run when we've just generated silence
			pthread_mutex_unlock(&update_ok_mutex);
		}
    }

  pthread_exit (0);
}


BOOLEAN
read_key (REAL del, BOOLEAN dot, BOOLEAN dash)
{
  extern BOOLEAN read_straight_key (KeyerState ks, BOOLEAN keyed);
  extern BOOLEAN read_iambic_key (KeyerState ks, BOOLEAN dot,
				  BOOLEAN dash, KeyerLogic kl, REAL ticklen);


  if (bug)
    {
      if (dash)
	return read_straight_key (ks, dash);
      else
	return read_iambic_key (ks, dot, FALSE, kl, del);
    }
  if (iambic)
    return read_iambic_key (ks, dot, dash, kl, del);
  return read_straight_key (ks, dot | dash);
}

/// Main keyer function,  called by a thread in the C#
BOOLEAN dotkey = FALSE;
PRIVATE BOOLEAN __inline
whichkey (BOOLEAN dot, BOOLEAN dash)
{
  if (dotkey)
    return dot;
  return dash;
}
void
SetWhichKey (BOOLEAN isdot)
{
  if (isdot)
    dotkey = TRUE;
  else
    dotkey = FALSE;
}
void
key_thread_process (REAL del, BOOLEAN dash, BOOLEAN dot, BOOLEAN keyprog)
{
  BOOLEAN keydown;
  extern BOOLEAN read_straight_key (KeyerState ks, BOOLEAN keyed);
  // read key; tell keyer elapsed time since last call
  if (!keyprog)
    keydown = read_key (del, dot, dash);
  else
    keydown = read_straight_key (ks, whichkey (dot, dash));


  if (!playing && keydown)
    CWToneOn (gen), playing = TRUE;
  else if (playing && !keydown)
    CWToneOff (gen);

  sem_post (clock_fired);
}

BOOLEAN
KeyerPlaying ()
{
  return playing;
}

//------------------------------------------------------------------------


void
SetKeyerBug (BOOLEAN bg)
{
  pthread_mutex_lock(&update_ok_mutex);
  if (bg)
    {
      iambic = FALSE;
      ks->flag.mdlmdB = FALSE;
      ks->flag.memory.dah = FALSE;
      ks->flag.memory.dit = FALSE;
      bug = TRUE;
    }
  else
    bug = FALSE;
  pthread_mutex_unlock(&update_ok_mutex);

}
void
SetKeyerSpeed (REAL speed)
{
  pthread_mutex_lock(&update_ok_mutex);
  wpm = ks->wpm = speed;
  pthread_mutex_unlock(&update_ok_mutex);
}
void
SetKeyerWeight (int newweight)
{
  pthread_mutex_lock(&update_ok_mutex);
  ks->weight = newweight;
  pthread_mutex_unlock(&update_ok_mutex);
}
void
SetKeyerIambic (BOOLEAN setit)
{
  pthread_mutex_lock(&update_ok_mutex);
  if (setit)
    {
      iambic = TRUE;
      ks->flag.mdlmdB = TRUE;
      ks->flag.memory.dah = TRUE;
      ks->flag.memory.dit = TRUE;
    }
  else
    {
      iambic = FALSE;
      ks->flag.mdlmdB = FALSE;
      ks->flag.memory.dah = FALSE;
      ks->flag.memory.dit = FALSE;
    }
  pthread_mutex_unlock(&update_ok_mutex);
}
void
SetKeyerFreq (REAL newfreq)
{
  pthread_mutex_lock(&update_ok_mutex);
  freq = newfreq;
  setCWToneGenVals (gen, gain, freq, ramp, ramp);
  pthread_mutex_unlock(&update_ok_mutex);
}
void
SetKeyerGain (REAL newgain)
{
  if ((newgain >= 0.0) && (newgain <= 1.0))
    {
      pthread_mutex_lock(&update_ok_mutex);
      gain = (REAL) (20.0 * log10 (newgain));
      setCWToneGenVals (gen, gain, freq, ramp, ramp);
      pthread_mutex_unlock(&update_ok_mutex);
    }

}
void
SetKeyerRamp (REAL newramp)
{
  pthread_mutex_lock(&update_ok_mutex);
  ramp = newramp;
  setCWToneGenVals (gen, gain, freq, ramp, ramp);
  pthread_mutex_unlock(&update_ok_mutex);
}
void
SetKeyerMode (int newmode)
{
  pthread_mutex_lock(&update_ok_mutex);
  if (newmode == 1)
  {
    ks->mode = MODE_B;
	ks->flag.mdlmdB = TRUE;
  }
  if (newmode == 0)
  {
    ks->mode = MODE_A;
    ks->flag.mdlmdB = FALSE;
  }
  if (newmode == 2)
    iambic = FALSE;
  pthread_mutex_unlock(&update_ok_mutex);
}

void
SetKeyerDeBounce (int db)
{
  pthread_mutex_lock(&update_ok_mutex);
  ks->debounce = db;
  pthread_mutex_unlock(&update_ok_mutex);
}

void
SetKeyerRevPdl (BOOLEAN rvp)
{
  pthread_mutex_lock(&update_ok_mutex);
  ks->flag.revpdl = !rvp;
  pthread_mutex_unlock(&update_ok_mutex);
}

/*updateKeyer(REAL nfreq, BOOLEAN niambic, REAL ngain, REAL nramp, REAL nwpm,
			BOOLEAN revpdl, int weight, REAL SampleRate) {
	ks->flag.iambic = niambic;
	iambic = niambic;
	ks->flag.revpdl = revpdl;
	ks->weight = weight;
	wpm = nwpm;
	gain = ngain;
	ramp = nramp;
	freq = nfreq;
	gen->osc.freq = 2.0 * M_PI * freq / SampleRate;
} */
void
SetKeyerPerf (BOOLEAN hiperf)
{
  unsigned int tmp_timer;
  tmp_timer = timerid;
  if (timerid != 0)
    {
      pthread_mutex_lock(&update_ok_mutex);
      timeKillEvent ((unsigned int) timerid);
      timerid = 0;
      usleep(11000);
	  pthread_mutex_unlock(&update_ok_mutex);
    }
  delCWToneGen (gen);
  if (hiperf)
    {
      HiPerformance = TRUE;
      key_poll_period = 1;
      TONE_SIZE = 48;
    }
  else
    {
      HiPerformance = FALSE;
      key_poll_period = 5;
      TONE_SIZE = 240;
    }
  gen = newCWToneGen (gain, freq, ramp, ramp, TONE_SIZE, SAMP_RATE);
  if (tmp_timer != 0)
    {
      StartKeyer();
    }
}
void
NewKeyer (REAL freq, BOOLEAN niambic, REAL gain, REAL ramp, REAL wpm,
	  REAL SampleRate)
{

  BOOLEAN out;
  kl = newKeyerLogic ();
  ks = newKeyerState ();
  ks->flag.iambic = niambic;
  ks->flag.revpdl = TRUE;	// depends on port wiring
  ks->flag.autospace.khar = ks->flag.autospace.word = FALSE;
  ks->flag.mdlmdB = TRUE;
  ks->flag.memory.dah = TRUE;
  ks->flag.memory.dit = TRUE;
  ks->debounce = 1;		// could be more if sampled faster
  ks->mode = MODE_B;
  ks->weight = 50;
  ks->wpm = wpm;
  iambic = niambic;
#ifndef INTERLEAVED
  lring = ringb_float_create (RING_SIZE);
  rring = ringb_float_create (RING_SIZE);
#else
  lring = ringb_float_create (2 * RING_SIZE);
#endif
  sem_init (clock_fired, 0, 0);
  sem_init (poll_fired, 0, 0);
  sem_init (keyer_started, 0, 0);
  if (HiPerformance)
    {
      key_poll_period = 1;
      TONE_SIZE = 48 * (int) (uni.samplerate / 48000.0);
    }
  else
    {
      key_poll_period = 5;
      TONE_SIZE = 240 * (int) (uni.samplerate / 48000.0);
    }
  //------------------------------------------------------------

  gen = newCWToneGen (gain, freq, ramp, ramp, TONE_SIZE, SampleRate);

  //------------------------------------------------------------
//  if (timeSetEvent(5,1,(LPTIMECALLBACK)timer_callback,(unsigned long*)NULL,TIME_PERIODIC) == (unsigned int)NULL) {
//        fprintf(stderr,"Timer failed\n"),fflush(stderr);
//  }
}

void
StartKeyer ()
{
#ifndef INTERLEAVED
  ringb_float_restart (lring, SIZEBUF);
  ringb_float_restart (rring, SIZEBUF);
#else
  ringb_float_restart (lring, SIZEBUF);
#endif
    timer_t timerid;
    struct sigevent sev;
    struct itimerspec its;
    long long freq_nanosecs;
    sigset_t mask;
    struct sigaction sa;

   /* Establish handler for timer signal */
   printf("Establishing handler for signal %d\n", SIG);
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = timer_callback;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIG, &sa, NULL) == -1)
    {
        perror("sigaction");
        return;
    }

    /* Create the timer */
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIG;
    sev.sigev_value.sival_ptr = &timerid;
    if (timer_create(CLOCKID, &sev, &timerid) == -1)
    {
        perror("timer_create");
        return;
    }

   printf("timer ID is 0x%lx\n", (long) timerid);

    /* Start the timer */
    freq_nanosecs = NANO_SEC / (key_poll_period * NANO_SEC);
    its.it_value.tv_sec = freq_nanosecs / NANO_SEC;
    its.it_value.tv_nsec = freq_nanosecs % NANO_SEC;
    its.it_interval.tv_sec = its.it_value.tv_sec;
    its.it_interval.tv_nsec = its.it_value.tv_nsec;

    if (timer_settime(timerid, 0, &its, NULL) == -1)
    {
        perror("timer_settime");
        return;
    }
  sem_post (keyer_started);
}
void
StopKeyer ()
{
  pthread_mutex_lock(&update_ok_mutex);
  if (timerid)
    timeKillEvent ((unsigned int) timerid);
  pthread_mutex_unlock(&update_ok_mutex);
  timerid = 0;
}

BOOLEAN
KeyerRunning ()
{
  return (timerid != 0);
}

void
DeleteKeyer ()
{
  StopKeyer();
  if (clock_fired)
    {
      sem_destroy (clock_fired);
      clock_fired = NULL;
    }
  if (poll_fired)
    {
      sem_destroy (poll_fired);
      poll_fired = NULL;
    }
  if (keyer_started)
    {
      sem_destroy (keyer_started);
      keyer_started = NULL;
    }
  delCWToneGen (gen);
  delKeyerState (ks);
  delKeyerLogic (kl);
#ifndef INTERLEAVED
  ringb_float_free (lring);
  ringb_float_free (rring);
#else
  ringb_float_free (lring);
#endif
}
void
KeyerClockFireWait ()
{
  sem_wait (clock_fired);
}
void
KeyerClockFireRelease ()
{
  sem_post (clock_fired);
}
void
KeyerStartedWait ()
{
  sem_wait (keyer_started);
}
void
KeyerStartedRelease ()
{
  sem_post (keyer_started);
}
void
PollTimerWait ()
{
  sem_wait (poll_fired);
}
void
PollTimerRelease ()
{
  sem_post (poll_fired);
}
void
SetKeyerResetSize (unsigned int sizer)
{
  SIZEBUF = sizer;
  cw_ring_reset = TRUE;
}

void
CWRingRestart ()
{
  cw_ring_reset = TRUE;
}

void
SetKeyerSampleRate (REAL sr)
{
  int factor = (int) (sr / 48000.0f);
  if (HiPerformance)
    {
      key_poll_period = 1;
      TONE_SIZE = 48 * factor;
    }
  else
    {
      key_poll_period = 5;
      TONE_SIZE = 240 * factor;
    }
  SIZEBUF = 1024 * factor;

  delCWToneGen (gen);
  gen = newCWToneGen (gain, freq, ramp, ramp, TONE_SIZE, sr);
}

//------------------------------------------------------------------------
