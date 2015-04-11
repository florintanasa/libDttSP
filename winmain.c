/* winmain.c

This file is part of a program that implements a Software-Defined Radio.

Copyright (C) 2004, 2005, 2006-5 by Frank Brickle, AB2KT and Bob McGwier, N4HY

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

#include <common.h>
/////////////////////////////////////////////////////////////////////////

// elementary defaults
struct _loc loc;

/////////////////////////////////////////////////////////////////////////
// most of what little we know here about the inner loop,
// functionally speaking

extern void reset_meters(void);
extern void reset_spectrum(void);
extern void reset_counters(void);
extern void process_samples(float *, float *, float *, float *, int);
extern void setup_workspace(REAL rate,
                            int buflen,
                            SDRMODE mode,
                            char *wisdom,
                            int specsize, int numrecv, int cpdsize);
extern void destroy_workspace(void);

//========================================================================
#if 0

PRIVATE void
spectrum_thread (void)
{
    unsigned long NumBytesWritten;
    while (top.running)
    {
        sem_wait (&top.sync.pws.sem);
        sem_wait (&top.sync.upd.sem);
        compute_spectrum (&uni.spec);
        WriteFile (top.meas.spec.fd, (LPVOID) & uni.spec.label,
                   sizeof (int), &NumBytesWritten, NULL);
        WriteFile (top.meas.spec.fd, (LPVOID) uni.spec.output,
                   sizeof (float) * uni.spec.size, &NumBytesWritten, NULL);
        sem_post (&top.sync.upd.sem);
    }
    pthread_exit (0);
}

PRIVATE void
meter_thread (void)
{
    unsigned long NumBytesWritten;
    while (top.running)
    {
        sem_wait (&top.sync.mtr.sem);
        sem_wait (&top.sync.upd.sem);
        WriteFile (top.meas.mtr.fd, (LPVOID) & uni.meter.label, sizeof (int),
                   &NumBytesWritten, NULL);
        WriteFile (top.meas.mtr.fd, (LPVOID) & uni.meter.snap.rx,
                   sizeof (REAL) * MAXRX * RXMETERPTS, &NumBytesWritten, NULL);
        WriteFile (top.meas.mtr.fd, (LPVOID) & uni.meter.snap.tx,
                   sizeof (REAL) * TXMETERPTS, &NumBytesWritten, NULL);
        sem_post (&top.sync.upd.sem);
    }
    pthread_exit (0);
}
#endif
//========================================================================

PRIVATE void
monitor_thread(void)
{
    while(top.running)
    {
        sem_wait(&top.sync.mon.sem);
        /* If there is anything that needs monitoring, do it here */
        fprintf(stderr,
                "@@@ mon [%ld]: cb = %d rbi = %d rbo = %d xr = %d\n",
                uni.tick,
                top.jack.blow.cb,
                top.jack.blow.rb.i, top.jack.blow.rb.o, top.jack.blow.xr);
        fflush(stderr);
        memset((char *)&top.jack.blow, 0, sizeof(top.jack.blow));
    }
    pthread_exit(0);
}

//========================================================================

PRIVATE void
process_updates_thread(void)
{
    while(top.running)
    {
        unsigned long NumBytesRead;
        pthread_testcancel();
        while(NULL != fgets(top.parm.buff, 4095, top.parm.fd))
        {
            top.parm.buff[4095] = 0;
            do_update(top.parm.buff, top.verbose ? stderr : 0);
        }
    }

    pthread_exit(0);
}

//========================================================================

BOOLEAN reset_em;

PRIVATE void
gethold(void)
{
    if(ringb_float_write_space(top.jack.ring.o.l) < top.hold.size.frames)
    {
        // pathology
        reset_em = TRUE;
        top.jack.blow.rb.o++;
    } else
    {
        ringb_float_write(top.jack.ring.o.l, top.hold.buf.l,
                          top.hold.size.frames);
        ringb_float_write(top.jack.ring.o.r, top.hold.buf.r,
                          top.hold.size.frames);
#ifdef USE_AUXILIARY
        ringb_float_write(top.jack.auxr.o.l, top.hold.aux.l,
                          top.hold.size.frames);
        ringb_float_write(top.jack.auxr.o.r, top.hold.aux.r,
                          top.hold.size.frames);
#else
        ringb_float_write(top.jack.auxr.o.l, top.hold.buf.l,
                          top.hold.size.frames);
        ringb_float_write(top.jack.auxr.o.r, top.hold.buf.r,
                          top.hold.size.frames);
    }
#endif
        if(ringb_float_read_space(top.jack.ring.i.l) < top.hold.size.frames)
        {
            // pathology
            memset((char *)top.hold.buf.l, 0, top.hold.size.bytes);
            memset((char *)top.hold.buf.r, 0, top.hold.size.bytes);
            memset((char *)top.hold.aux.l, 0, top.hold.size.bytes);
            memset((char *)top.hold.aux.r, 0, top.hold.size.bytes);
            reset_em = TRUE;
            top.jack.blow.rb.i++;
        } else
        {
            ringb_float_read(top.jack.ring.i.l,
                             top.hold.buf.l, top.hold.size.frames);
            ringb_float_read(top.jack.ring.i.r,
                             top.hold.buf.r, top.hold.size.frames);
#ifdef USE_AUXILIARY
            ringb_float_read(top.jack.auxr.i.l,
                             top.hold.aux.l, top.hold.size.frames);
            ringb_float_read(top.jack.auxr.i.r,
                             top.hold.aux.r, top.hold.size.frames);
#else
            ringb_float_read(top.jack.auxr.i.l,
                             top.hold.buf.l, top.hold.size.frames);
            ringb_float_read(top.jack.auxr.i.r,
                             top.hold.buf.r, top.hold.size.frames);
#endif
        }
    }

    PRIVATE BOOLEAN
    canhold(void)
    {
        return (ringb_float_read_space(top.jack.ring.i.l) >=
                    (size_t)top.hold.size.frames);
    }


//------------------------------------------------------------------------

    PRIVATE void
    run_mute(void)
    {
        memset((char *)top.hold.buf.l, 0, top.hold.size.bytes);
        memset((char *)top.hold.buf.r, 0, top.hold.size.bytes);
        memset((char *)top.hold.aux.l, 0, top.hold.size.bytes);
        memset((char *)top.hold.aux.r, 0, top.hold.size.bytes);
        uni.tick++;
    }

    PRIVATE void
    run_pass(void)
    {
        uni.tick++;
    }

    PRIVATE void
    run_play(void)
    {
        process_samples(top.hold.buf.l, top.hold.buf.r,
                        top.hold.aux.l, top.hold.aux.r, top.hold.size.frames);
    }

// NB do not set RUN_SWCH directly via setRunState;
// use setSWCH instead


    PRIVATE void
    run_swch(void)
    {
        int i, n = top.hold.size.frames;
        REAL w;
//	static int count = 0;

        process_samples(top.hold.buf.l, top.hold.buf.r,
                        top.hold.aux.l, top.hold.aux.r,
                        top.hold.size.frames);

        for (i = 0; i < n; i++)
        {
//		count++;
            if(top.swch.env.curr.type == SWCH_FALL)
            {
                top.swch.env.curr.val += top.swch.env.fall.incr;
                w = (REAL)sin(top.swch.env.curr.val * M_PI /  2.0f);
                top.hold.buf.l[i] *= w, top.hold.buf.r[i] *= w;
                top.hold.aux.l[i] *= w, top.hold.aux.r[i] *= w;
//			if (top.swch.env.curr.cnt == 0) fprintf(stderr, "FALL\n"),fflush(stderr);
//			if(top.swch.env.curr.cnt == 0) top.hold.buf.l[i] = top.hold.buf.r[i] = -1.0;
                if(++top.swch.env.curr.cnt >= top.swch.env.fall.size)
                {
                    //top.hold.buf.l[i] = top.hold.buf.r[i] = -1.0;
                    top.swch.env.curr.type = SWCH_STDY;
                    top.swch.env.curr.cnt = 0;
                    top.swch.env.curr.val = 0.0;
//				fprintf(stderr, "Fall End: %d\n", count);
                }
            } else if(top.swch.env.curr.type == SWCH_STDY)
            {
                top.hold.buf.l[i] = top.hold.buf.r[i] =
                    top.hold.aux.l[i] =  top.hold.aux.r[i] = 0.0;
//			if (top.swch.env.curr.cnt == 0) fprintf(stderr, "STDY\n"),fflush(stderr);
                if(++top.swch.env.curr.cnt >= top.swch.env.stdy.size)
                {
//				top.hold.buf.l[i] = top.hold.buf.r[i] = -1.0;
                    top.swch.env.curr.type = SWCH_RISE;
                    top.swch.env.curr.cnt = 0;
                    top.swch.env.curr.val = 0.0;
//				fprintf(stderr, "Stdy End: %d\n", count);
                }
            } else if(top.swch.env.curr.type == SWCH_RISE)
            {
                top.swch.env.curr.val += top.swch.env.rise.incr;
                w = (REAL)sin(top.swch.env.curr.val * M_PI /  2.0f);
                top.hold.buf.l[i] *= w, top.hold.buf.r[i] *= w;
                top.hold.aux.l[i] *= w, top.hold.aux.r[i] *= w;
//			if (top.swch.env.curr.cnt == 0) fprintf(stderr, "RISE\n"),fflush(stderr);
                if(++top.swch.env.curr.cnt >= top.swch.env.rise.size)
                {
//				reset_meters();
//				reset_spectrum();
//				reset_counters();

                    uni.mode.trx = top.swch.trx.next;
                    top.state = top.swch.run.last;
                    break;
//				fprintf(stderr, "Rise End: %d\n", count);
                }
            }
        }
    }

/*
PRIVATE void
run_swch (void)
{
    if (top.swch.bfct.have == 0)
    {
        // first time
        // apply ramp down
        int i, m = top.swch.fade, n = top.swch.tail;
        for (i = 0; i < m; i++)
        {
            float w = (float) 1.0 - (float) i / (float) m;
            top.hold.buf.l[i] *= w, top.hold.buf.r[i] *= w;
        }
        memset ((char *) (top.hold.buf.l + m), 0, n);
        memset ((char *) (top.hold.buf.r + m), 0, n);
        top.swch.bfct.have++;
    }
    else if (top.swch.bfct.have < top.swch.bfct.want)
    {
        // in medias res
        memset ((char *) top.hold.buf.l, 0, top.hold.size.bytes);
        memset ((char *) top.hold.buf.r, 0, top.hold.size.bytes);
        top.swch.bfct.have++;
    }
    else
    {
        // last time
        // apply ramp up
        int i, m = top.swch.fade, n = top.swch.tail;
        for (i = 0; i < m; i++)
        {
            float w = (float) i / m;
            top.hold.buf.l[i] *= w, top.hold.buf.r[i] *= w;
        }
        uni.mode.trx = top.swch.trx.next;
        top.state = top.swch.run.last;
        top.swch.bfct.want = top.swch.bfct.have = 0;
    }

    process_samples (top.hold.buf.l, top.hold.buf.r,
        top.hold.aux.l, top.hold.aux.r, top.hold.size.frames);

}
*/
//========================================================================

    static void reset_system_audio(size_t nframes)
    {
        size_t reset_size = max(top.jack.reset_size, nframes);
        const float zero = 0.;
        int i;

        reset_em = FALSE;

        ringb_float_reset(top.jack.ring.i.l);
        ringb_float_reset(top.jack.ring.i.r);
        ringb_float_reset(top.jack.auxr.i.l);
        ringb_float_reset(top.jack.auxr.i.r);

        if(top.offset < 0)
        {
            for (i = top.offset; i < 0; i++)
            {
                ringb_float_write(top.jack.ring.i.l, &zero, 1);
                ringb_float_write(top.jack.auxr.i.l, &zero, 1);
            }
        } else
        {
            for (i = 0; i < top.offset; i++)
            {
                ringb_float_write(top.jack.ring.i.r, &zero, 1);
                ringb_float_write(top.jack.auxr.i.r, &zero, 1);
            }
        }

        ringb_float_restart(top.jack.ring.o.r, reset_size);
        ringb_float_restart(top.jack.ring.o.l, reset_size);
        ringb_float_restart(top.jack.auxr.o.r, reset_size);
        ringb_float_restart(top.jack.auxr.o.l, reset_size);
    }

    void
    Audio_Callback(float *input_l, float *input_r, float *output_l,
                   float *output_r, unsigned int nframes)
    {
        BOOLEAN b = reset_em;
        if(top.susp)
        {
            memset(output_l, 0, nframes * sizeof(float));
            memset(output_r, 0, nframes * sizeof(float));
            return;
        }

        if(reset_em)
        {
            reset_system_audio(nframes);
            memset(output_l, 0, nframes * sizeof(float));
            memset(output_r, 0, nframes * sizeof(float));
        }

        if((ringb_float_read_space(top.jack.ring.o.l) >= nframes)
           && (ringb_float_read_space(top.jack.ring.o.r) >= nframes))
        {
            ringb_float_read(top.jack.auxr.o.l, output_l, nframes);
            ringb_float_read(top.jack.auxr.o.r, output_r, nframes);
            ringb_float_read(top.jack.ring.o.l, output_l, nframes);
            ringb_float_read(top.jack.ring.o.r, output_r, nframes);
        } else
        {   // rb pathology
            reset_system_audio(nframes);
            memset(output_l, 0, nframes * sizeof(float));
            memset(output_r, 0, nframes * sizeof(float));
            top.jack.blow.rb.o++;
        }

        // input: copy from port to ring
        if((ringb_float_write_space(top.jack.ring.i.l) >= nframes)
           && (ringb_float_write_space(top.jack.ring.i.r) >= nframes))
        {
            ringb_float_write(top.jack.ring.i.l, (float *)input_l, nframes);
            ringb_float_write(top.jack.ring.i.r, (float *)input_r, nframes);
            ringb_float_write(top.jack.auxr.i.l, (float *)input_l, nframes);
            ringb_float_write(top.jack.auxr.i.r, (float *)input_r, nframes);
        } else
        {   // rb pathology
            reset_system_audio(nframes);
            top.jack.blow.rb.i++;
        }

        // if enough accumulated in ring, fire dsp
        if(ringb_float_read_space(top.jack.ring.i.l) >= top.hold.size.frames) sem_post(&top.sync.buf.sem);

        // check for blowups
        if((top.jack.blow.cb > 0) ||
           (top.jack.blow.rb.i > 0) || (top.jack.blow.rb.o > 0)) sem_post(&top.sync.mon.sem);
    }


//========================================================================


    void
    process_samples_thread(void)
    {
        while(top.running)
        {
            sem_wait(&top.sync.buf.sem);
            do
            {
                gethold();
                sem_wait(&top.sync.upd.sem);
                switch(top.state)
                {
                case RUN_MUTE:
                    run_mute();
                    break;
                case RUN_PASS:
                    run_pass();
                    break;
                case RUN_PLAY:
                    run_play();
                    break;
                case RUN_SWCH:
                    run_swch();
                    break;
                }
                sem_post(&top.sync.upd.sem);
            }while(canhold());
        }
    }


    void
    closeup(void)
    {
        top.running = FALSE;
        top.susp = TRUE;
        usleep(96000);
        ringb_float_free(top.jack.auxr.i.l);
        ringb_float_free(top.jack.auxr.i.r);
        ringb_float_free(top.jack.auxr.o.l);
        ringb_float_free(top.jack.auxr.o.r);

        ringb_float_free(top.jack.ring.o.r);
        ringb_float_free(top.jack.ring.o.l);
        ringb_float_free(top.jack.ring.i.r);
        ringb_float_free(top.jack.ring.i.l);

        if(NULL!= top.parm.fp) fclose(top.parm.fp);
        if(NULL!= top.parm.fd) fclose(top.parm.fd);
        if(NULL!= top.parm.path) unlink(top.parm.path);

        if(uni.meter.flag)
        {
            if(NULL!= top.meas.mtr.fp) fclose(top.meas.mtr.fp);
            if(NULL!= top.meas.mtr.fd) fclose(top.meas.mtr.fd);
            if(NULL!= top.meas.mtr.path) unlink(top.meas.mtr.path);
        }

        if(uni.spec.flag)
        {
            if(NULL!= top.meas.spec.fp) fclose(top.meas.spec.fp);
            if(NULL!= top.meas.spec.fd) fclose(top.meas.spec.fd);
            if(NULL!= top.meas.spec.path) unlink(top.meas.spec.path);
        }
        destroy_workspace();
    }

//........................................................................

/*PRIVATE void
setup_switching (void)
{
  top.swch.fade = (int) (0.2 * uni.buflen + 0.5);
  top.swch.tail = (top.hold.size.frames - top.swch.fade);
}*/

    PRIVATE void
    setup_local_audio(void)
    {
        top.hold.size.frames = uni.buflen;
        top.hold.size.bytes = top.hold.size.frames * sizeof(float);
        top.hold.buf.l =
            (float *)safealloc(top.hold.size.frames, sizeof(float),
                               "main hold buffer left");
        top.hold.buf.r =
            (float *)safealloc(top.hold.size.frames, sizeof(float),
                               "main hold buffer right");
        top.hold.aux.l =
            (float *)safealloc(top.hold.size.frames, sizeof(float),
                               "aux hold buffer left");
        top.hold.aux.r =
            (float *)safealloc(top.hold.size.frames, sizeof(float),
                               "aux hold buffer right");
    }


    PRIVATE sem_t setup_update_sem;

    PRIVATE void
    setup_update_server()
    {
        //create fifo file if it does not exist
        int rslt = mkfifo(top.parm.path, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH); // create fifo SDRcommands+pid
        if(0 != rslt)
        {
            fprintf  (stderr,"Update server pipe setup failed, \"%s\", %s",top.parm.path, strerror(errno));
        }

        sem_post(&setup_update_sem);
        top.parm.fd = fopen(top.parm.path, "r");
        if(NULL == top.parm.fd)
        {
            fprintf  (stderr,"Connecting the server to the Update pipe failed, \"%s\", %s",top.parm.path, strerror(errno));
        }
        pthread_exit(0);
    }


    PRIVATE void
    setup_update_client()
    {
        top.parm.fp = fopen(top.parm.path, "w");
        if(NULL == top.parm.fp)
        {
            fprintf(stderr,"The Update Client Open Failed, \"%s\", %s",top.parm.path, strerror(errno));
        }

        sem_post(&setup_update_sem);
        pthread_exit(0);
    }

    PRIVATE void
    setup_meter_server()
    {
        //create fifo file if it does not exist
        int rslt = mkfifo(top.meas.mtr.path, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        if(0 != rslt)
        {
            fprintf(stderr,"Meter server pipe setup failed, \"%s\", %s",top.meas.mtr.path, strerror(errno));
        }

        sem_post(&setup_update_sem);
        top.meas.mtr.fd = fopen(top.meas.mtr.path, "r");
        if(NULL == top.meas.mtr.fd)
        {
            fprintf(stderr,"Connecting the server to the Meter pipe failed, \"%s\", %s",top.meas.mtr.path, strerror(errno));
        }
        pthread_exit(0);
    }

    PRIVATE void
    setup_meter_client()
    {
        top.meas.mtr.fp = fopen(top.meas.mtr.path, "w");
        if(NULL == top.meas.mtr.fp)
        {
            fprintf(stderr,"The Meter Client Open Failed, \"%s\", %s",top.meas.mtr.path, strerror(errno));
        }

        sem_post(&setup_update_sem);
        pthread_exit(0);
    }

    PRIVATE void
    setup_spec_server()
    {
        //create fifo file if it does not exist
        int rslt = mkfifo(top.meas.spec.path, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        if(0 != rslt)
        {
            fprintf(stderr,"Spectrum server pipe setup failed, \"%s\", %s",top.meas.spec.path, strerror(errno));
        }

        sem_post(&setup_update_sem);
        top.meas.spec.fd = fopen(top.meas.spec.path, "r");
        if(NULL == top.meas.spec.fd)
        {
            fprintf(stderr,"Connecting the server to the Spectrum pipe failed, \"%s\", %s",top.meas.spec.path, strerror(errno));
        }
        pthread_exit(0);
    }

    PRIVATE void
    setup_spec_client()
    {
        top.meas.spec.fp = fopen(top.meas.spec.path, "w");
        if(NULL == top.meas.spec.fp)
        {
            fprintf(stderr,"The The Spectrum Client Open Failed, \"%s\", %s",top.meas.spec.path, strerror(errno));
        }

        sem_post(&setup_update_sem);
        pthread_exit(0);
    }
    PRIVATE pthread_t id1, id2, id3, id4, id5, id6;
    PRIVATE void
    setup_updates(void)
    {

        top.parm.path = loc.path.parm;
        sem_init(&setup_update_sem, 0, 0);


        if(uni.meter.flag)
        {
            top.meas.mtr.path = loc.path.meter;
        }
        if(uni.spec.flag)
        {
            top.meas.spec.path = loc.path.spec;
        }

        uni.update.fp = fopen("command_memory.txt", "w+");
        uni.update.flag = TRUE;


        // Do this STUPID stuff to make use of the Named Pipe Mechanism in Windows
        // For the update server


        pthread_create(&id1, NULL, (void *)setup_update_server, NULL);
        sem_wait(&setup_update_sem);
        usleep(100000);
        pthread_create(&id2, NULL, (void *)setup_update_client, NULL);
        sem_wait(&setup_update_sem);
        if(uni.meter.flag)
        {
            pthread_create(&id3, NULL, (void *)setup_meter_server, NULL);
            sem_wait(&setup_update_sem);
            usleep(100000);
            pthread_create(&id4, NULL, (void *)setup_meter_client, NULL);
            sem_wait(&setup_update_sem);
        }

        if(uni.spec.flag)
        {
            pthread_create(&id5, NULL, (void *)setup_spec_server, NULL);
            sem_wait(&setup_update_sem);
            usleep(100000);
            pthread_create(&id6, NULL, (void *)setup_spec_client, NULL);
            sem_wait(&setup_update_sem);
        }
        sem_destroy(&setup_update_sem);
    }

    PRIVATE void
    setup_system_audio(void)
    {
        sprintf(top.jack.name, "sdr-%lu", top.pid);
        top.jack.size = 2048;

        memset((char *)&top.jack.blow, 0, sizeof(top.jack.blow));
        top.jack.ring.i.l = ringb_float_create(top.jack.size * loc.mult.ring);
        top.jack.ring.i.r = ringb_float_create(top.jack.size * loc.mult.ring);
        top.jack.ring.o.l = ringb_float_create(top.jack.size * loc.mult.ring);
        top.jack.ring.o.r = ringb_float_create(top.jack.size * loc.mult.ring);

        top.jack.auxr.i.l = ringb_float_create(top.jack.size * loc.mult.ring);
        top.jack.auxr.i.r = ringb_float_create(top.jack.size * loc.mult.ring);
        top.jack.auxr.o.l = ringb_float_create(top.jack.size * loc.mult.ring);
        top.jack.auxr.o.r = ringb_float_create(top.jack.size * loc.mult.ring);

        ringb_float_clear(top.jack.ring.o.l, top.jack.size);
        ringb_float_clear(top.jack.ring.o.r, top.jack.size);
    }

    PRIVATE void
    setup_threading(void)
    {
        top.susp = FALSE;
        sem_init(&top.sync.upd.sem, 0, 0);
        pthread_create(&top.thrd.upd.id, NULL, (void *)process_updates_thread,
                       NULL);
        sem_init(&top.sync.buf.sem, 0, 0);
        //pthread_create(&top.thrd.trx.id, NULL, (void *) process_samples_thread, NULL);
        sem_init(&top.sync.mon.sem, 0, 0);
        pthread_create(&top.thrd.mon.id, NULL, (void *)monitor_thread, NULL);
/*  if (uni.meter.flag)
    {
      sem_init (&top.sync.mtr.sem, 0, 0);
      pthread_create (&top.thrd.mtr.id, NULL, (void *) meter_thread, NULL);
    }
  if (uni.spec.flag)
    {
      sem_init (&top.sync.pws.sem, 0, 0);
      //pthread_create(&top.thrd.pws.id, NULL, (void *) spectrum_thread, NULL);
    }
*/
    }

//========================================================================
// hard defaults, then environment

    PRIVATE void
    setup_defaults()
    {
        loc.name[0] = 0;      // no default name for jack client
        sprintf(loc.path.rcfile, "%s%0lu", RCBASE, top.pid);
        sprintf(loc.path.parm, "%s%0lu", PARMPATH, top.pid);
        sprintf(loc.path.meter, "%s%0lu", METERPATH, top.pid);
        sprintf(loc.path.spec, "%s%0lu", SPECPATH, top.pid);
        sprintf(loc.path.wisdom, "%s%0lu", WISDOMPATH, top.pid);
        loc.def.rate = DEFRATE;
        loc.def.size = DEFSIZE;
        loc.def.nrx = MAXRX;
        loc.def.mode = DEFMODE;
        loc.def.spec = DEFSPEC;
        loc.mult.ring = RINGMULT;
        loc.def.comp = DEFCOMP;
    }

//========================================================================
    void
    setup()
    {


        top.pid = getpid(); // GetCurrentThreadId ();
        top.uid = geteuid();
        top.start_tv = now_tv();
        top.running = TRUE;
        top.verbose = FALSE;
        top.state = RUN_PLAY;
        top.offset = 0;
        top.jack.reset_size = 1024;
        reset_em = TRUE;
        setup_defaults();

        uni.meter.flag = TRUE;
        uni.spec.flag = TRUE;
        top.swch.env.fall.size = (int)(loc.def.rate * 0.005);
        top.swch.env.stdy.size = (int)(loc.def.rate * 0.050);
        top.swch.env.rise.size = (int)(loc.def.rate * 0.005);


        top.swch.env.curr.val = 0.0;
        top.swch.env.curr.cnt = 0;
        top.swch.env.rise.incr = 1.0f / (float)top.swch.env.rise.size;
        top.swch.env.fall.incr = 1.0f / (float)top.swch.env.fall.size;
        setup_workspace(loc.def.rate,
                        loc.def.size,
                        loc.def.mode,
                        loc.path.wisdom, loc.def.spec, loc.def.nrx, loc.def.comp);

        setup_updates();

        setup_local_audio();
        setup_system_audio();

        setup_threading();
//  setup_switching ();
        uni.spec.flag = TRUE;
        uni.spec.type = SPEC_POST_FILT;
        uni.spec.scale = SPEC_PWR;
        uni.spec.rxk = 0;
        reset_meters();
        reset_spectrum();
        reset_counters();
    }

    int
    reset_for_buflen(int new_buflen)
    {

        // make sure new size is power of 2
        if(popcnt(new_buflen) != 1) return (-1);
        safefree((char *)top.hold.buf.r);
        safefree((char *)top.hold.buf.l);
        safefree((char *)top.hold.aux.r);
        safefree((char *)top.hold.aux.l);
        uni.buflen = new_buflen;
        top.jack.reset_size = new_buflen;


        destroy_workspace();
        loc.def.size = new_buflen;
        setup_workspace(loc.def.rate,
                        loc.def.size,
                        loc.def.mode,
                        loc.path.wisdom, loc.def.spec, loc.def.nrx, loc.def.size);

        setup_local_audio();
//  setup_switching ();

        reset_meters();
        reset_spectrum();
        reset_counters();

        return (0);
    }

    int
    reset_for_samplerate(REAL new_samplerate)
    {

        // make sure new sample rate works
        destroy_workspace();
        loc.def.rate = uni.samplerate = new_samplerate;
        top.swch.env.fall.size = (int)(loc.def.rate * 0.005);
        top.swch.env.stdy.size = (int)(loc.def.rate * 0.050);
        top.swch.env.rise.size = (int)(loc.def.rate * 0.005);
        top.swch.env.curr.val = 0.0;
        top.swch.env.curr.cnt = 0;
        top.swch.env.fall.incr = 1.0f / (float)top.swch.env.fall.size;
        top.swch.env.rise.incr = 1.0f / (float)top.swch.env.rise.size;
        setup_workspace(loc.def.rate,
                        loc.def.size,
                        loc.def.mode,
                        loc.path.wisdom, loc.def.spec, loc.def.nrx, loc.def.size);
        setup_local_audio();
//  setup_switching ();
        reset_meters();
        reset_spectrum();
        reset_counters();
        return (0);
    }
    