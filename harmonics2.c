// Unit 6.3
// C Programming for Music Technology - NYU
// Written by Oriol Nieto

#define SAMPLE_RATE         44100
#define FRAMES_PER_BUFFER   65536
#define NUM_IN_CHANNELS     1
#define NUM_OUT_CHANNELS    1
#define WINDOW_SIZE         FRAMES_PER_BUFFER/4
#define HOP_SIZE            WINDOW_SIZE/2
#define STEREO              2
#define INCREMENT           0.000001
#define threshINCREMENT     0.0001

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> /* for sleep() */
#include <portaudio.h>
#include <sndfile.h>
#include <math.h>
#include <string.h> /* for memset */
#include <ncurses.h>
#include "fft.h"

typedef struct {
    float sampleRate;
    SNDFILE *infile;
    SF_INFO sfinfo_in;
    float file_buff[STEREO * (FRAMES_PER_BUFFER + HOP_SIZE)];
    float window[WINDOW_SIZE];
    float prev_win[WINDOW_SIZE];
    float curr_win[WINDOW_SIZE];
    float second;
    float third;
    float fifth;
    float threshold;
} paData;


float curr_magnitude[WINDOW_SIZE/2];
float curr_phase[WINDOW_SIZE/2];

float prev_magnitude[WINDOW_SIZE/2];
float prev_phase[WINDOW_SIZE/2];

bool  curr_harmonicsindex[WINDOW_SIZE/4];
bool  prev_harmonicsindex[WINDOW_SIZE/4];

/* 
 *  Description:  Callback for Port Audio
 */
static int paCallback( const void *inputBuffer,
			 void *outputBuffer, unsigned long framesPerBuffer,
			 const PaStreamCallbackTimeInfo* timeInfo,
			 PaStreamCallbackFlags statusFlags, void *userData ) 
{
    int i, j, k, readcount;

    /* Cast void pointers */
    float *out = (float*)outputBuffer;
    paData *data = (paData*)userData;

    /* Read frames of float type into our buffer */
    readcount = sf_readf_float( data->infile, data->file_buff, framesPerBuffer + HOP_SIZE);

    /* If we're at the end of the file, let's start again */
    if ( readcount < framesPerBuffer ) {
        sf_seek( data->infile, 0, SEEK_SET );
        readcount = sf_readf_float( data->infile, data->file_buff + readcount * data->sfinfo_in.channels, framesPerBuffer - readcount + HOP_SIZE );
    }

    /* Rewind the hop size */
    sf_seek( data->infile, -HOP_SIZE, SEEK_CUR );

    /* Separate left channel */
    float left[framesPerBuffer+HOP_SIZE];
    for (i = 0; i < framesPerBuffer + HOP_SIZE; ++i)
    {
        left[i] = data->file_buff[2*i];
    }

    /* STFT */
    for (i = 0; i < framesPerBuffer; i+=HOP_SIZE)
    {
        /* Apply window to current frame */
        for (j = 0; j < WINDOW_SIZE; j++) {
            data->curr_win[j] = left[i+j];
        }
        apply_window(data->curr_win, data->window, WINDOW_SIZE);

        /* FFT */
        rfft( data->curr_win, WINDOW_SIZE/2, FFT_FORWARD );
        complex * curr_cbuf = (complex *)data->curr_win;
        rfft( data->prev_win, WINDOW_SIZE/2, FFT_FORWARD );
        complex * prev_cbuf = (complex *)data->prev_win;

        /* Get Magnitude and Phase (polar coordinates) */
        for (j = 0; j < WINDOW_SIZE/2; ++j)
        {
            curr_magnitude[j] = cmp_abs(curr_cbuf[j]);
            curr_phase[j] = atan2f(curr_cbuf[j].im, curr_cbuf[j].re);

            prev_magnitude[j] = cmp_abs(prev_cbuf[j]);
            prev_phase[j] = atan2f(prev_cbuf[j].im, prev_cbuf[j].re);
        }

        findpeaks(curr_magnitude, curr_harmonicsindex, WINDOW_SIZE, data->threshold);
        findpeaks(prev_magnitude, prev_harmonicsindex, WINDOW_SIZE, data->threshold);
        
        for (j = 1; j < WINDOW_SIZE/4; j++)
        {
            if (curr_harmonicsindex[j] == true)
            {
                for (k = j*2; k < WINDOW_SIZE/4; k+=2*j)
                {
                    curr_magnitude[k] += ((1-k/4096) * data->second)/2;
                    curr_magnitude[WINDOW_SIZE/2-k] += ((1-k/4096) * data->second)/2;
                }
            }
            if (prev_harmonicsindex[j] == true)
            {
                for (k = j*2; k < WINDOW_SIZE/4; k+=2*j)
                {
                    prev_magnitude[k] += ((1-k/4096) * data->second)/2;
                    prev_magnitude[WINDOW_SIZE/2-k] += ((1-k/4096) * data->second)/2;
                }
            }
        }

        for (j = 1; j < WINDOW_SIZE/4; j++)
        {
            if (curr_harmonicsindex[j] == true)
            {
                for (k = j*3; k < WINDOW_SIZE/4; k+=3*j)
                {
                    curr_magnitude[k] += ((1-k/4096) * data->third)/2;
                    curr_magnitude[WINDOW_SIZE/2-k] += ((1-k/4096) * data->third)/2;
                }
            }
            if (prev_harmonicsindex[j] == true)
            {
                for (k = j*3; k < WINDOW_SIZE/4; k+=3*j)
                {
                    prev_magnitude[k] += ((1-k/4096) * data->third)/2;
                    prev_magnitude[WINDOW_SIZE/2-k] += ((1-k/4096) * data->third)/2;
                }
            }
        }

        for (j = 1; j < WINDOW_SIZE/4; j++)
        {
            if (curr_harmonicsindex[j] == true)
            {
                for (k = j*5; k < WINDOW_SIZE/4; k+=5*j)
                {
                    curr_magnitude[k] += ((1-k/4096) * data->fifth)/2;
                    curr_magnitude[WINDOW_SIZE/2-k] += ((1-k/4096) * data->fifth)/2;
                }
            }
            if (prev_harmonicsindex[j] == true)
            {
                for (k = j*5; k < WINDOW_SIZE/4; k+=5*j)
                {
                    prev_magnitude[k] += ((1-k/4096) * data->fifth)/2;
                    prev_magnitude[WINDOW_SIZE/2-k] += ((1-k/4096) * data->fifth)/2;
                }
            }
        }

        /* Back to Cartesian coordinates */
        for (j = 0; j < WINDOW_SIZE/2; j++) {
            curr_cbuf[j].re = curr_magnitude[j] * cosf(curr_phase[j]);
            curr_cbuf[j].im = curr_magnitude[j] * sinf(curr_phase[j]);
            prev_cbuf[j].re = prev_magnitude[j] * cosf(prev_phase[j]);
            prev_cbuf[j].im = prev_magnitude[j] * sinf(prev_phase[j]);
        }

        // /* Back to Time Domain */
        rfft( (float*)curr_cbuf, WINDOW_SIZE/2, FFT_INVERSE );
        rfft( (float*)prev_cbuf, WINDOW_SIZE/2, FFT_INVERSE );

        /* Assign to the output */
        for (j = 0; j < HOP_SIZE; j++) {
            out[i+j] = data->prev_win[j+HOP_SIZE] + data->curr_win[j];
        }

        /* Update previous window */
        for (j = 0; j < WINDOW_SIZE; j++) {
            data->prev_win[j] = data->curr_win[j];
        }
    }

    return paContinue;
}

/* 
 * Description: Main function
 */
int main( int argc, char **argv ) {

    PaStream *stream;
    PaStreamParameters outputParameters;
    PaStreamParameters inputParameters;
    PaError err;
    paData data;

    /* Check arguments */
    if ( argc != 2 ) {
        printf("Usage: %s audio_file\n", argv[0]);
        return EXIT_FAILURE;
    }

    /* Open the audio file */
    if (( data.infile = sf_open( argv[1], SFM_READ, &data.sfinfo_in ) ) == NULL ) {
        printf("Error, couldn't open the file\n");
        return EXIT_FAILURE;
    }

    /* Print info about audio file */
    printf("Audio File:\nFrames: %d\nChannels: %d\nSampleRate: %d\n",
            (int)data.sfinfo_in.frames, (int)data.sfinfo_in.channels, 
            (int)data.sfinfo_in.samplerate);

    /* Init Windows */
    hanning(data.window, WINDOW_SIZE);
    memset(&data.prev_win, 0, WINDOW_SIZE*sizeof(float));

    /* Init lowpass and highpass */
    data.second = 0.000000f;
    data.third = 0.000000f;
    data.fifth = 0.000000f;
    data.threshold = 0.0001f;

    /* Initialize PortAudio */
    Pa_Initialize();

    /* Set output stream parameters */
    outputParameters.device = Pa_GetDefaultOutputDevice();
    outputParameters.channelCount = NUM_OUT_CHANNELS;
    outputParameters.sampleFormat = paFloat32;
    outputParameters.suggestedLatency = 
    Pa_GetDeviceInfo( outputParameters.device )->defaultLowOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;

    /* Set input stream parameters */
    inputParameters.device = Pa_GetDefaultInputDevice();
    inputParameters.channelCount = NUM_IN_CHANNELS;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = 
    Pa_GetDeviceInfo( inputParameters.device )->defaultLowInputLatency;
    inputParameters.hostApiSpecificStreamInfo = NULL;

    /* Open audio stream */
    err = Pa_OpenStream( &stream,
            &inputParameters,
		    &outputParameters,
	        SAMPLE_RATE, FRAMES_PER_BUFFER, paNoFlag, 
		    paCallback, &data );

    if (err != paNoError) {
        printf("PortAudio error: open stream: %s\n", Pa_GetErrorText(err));
    }
  
    /* Start audio stream */
    err = Pa_StartStream( stream );
    if (err != paNoError) {
        printf(  "PortAudio error: start stream: %s\n", Pa_GetErrorText(err));
    }

    /**************************************************************************/
    /* Main loop */
    /**************************************************************************/
    
    /* Initialize interactive character input */
    initscr(); /* Start curses mode */
    cbreak();  /* Line buffering disabled*/
    noecho(); /* Comment this out if you want to show characters when they are typed */
    
    char ch;
    ch = '\0'; /* Init ch to null character */
    
    mvprintw(0, 0, "2nd Order: %1.6f 3rd Order: %1.6f\n5th Order: %1.6f Threshold: %1.6f\n[a/s/z] decreases/increases/resets 2nd order harmonics\n" \
             "[d/f/c] decreases/increases/resets 3rd order harmonics\n" \
             "[g/h/b] decreases/increases/resets 5th order harmonics\n" \
             "[l/;/.] decreases/increases/resets sensitivity threshold\n"
             "[q] to quit\n", data.second, data.third, data.fifth, data.threshold);
    
    while (ch != 'q') {
        ch = getch(); /* If cbreak hadn't been called, you would have to press enter
                       before it gets to the program */
        switch (ch) {
            case 'a':
                data.second -= INCREMENT;
                if (data.second < 0) {
                    data.second = 0;
                }
                break;
            case 's':
                data.second += INCREMENT;
                if (data.second > 1) {
                    data.second = 1;
                }
                break;
            case 'z':
                data.second = 0.000000f;
                break;
            case 'd':
                data.third -= INCREMENT;
                if (data.third < 0) {
                    data.third = 0;
                }
                break;
            case 'f':
                data.third += INCREMENT;
                if (data.third > 1) {
                    data.third = 1;
                } 
                break;
            case 'c':
                data.third = 0.000000f;
                break;
            case 'g':
                data.fifth -= INCREMENT;
                if (data.fifth < 0) {
                    data.fifth = 0;
                }
                break;
            case 'h':
                data.fifth += INCREMENT;
                if (data.fifth > 1) {
                    data.fifth = 1;
                } 
                break;
            case 'b':
                data.fifth = 0.000000f;
                break;
            case 'l':
                data.threshold -= threshINCREMENT;
                if (data.threshold < 0) {
                    data.threshold = 0;
                }
                break;
            case ';':
                data.threshold += threshINCREMENT;
                if (data.threshold > 1) {
                    data.threshold = 1;
                }
                break;
            case '.':
                data.threshold = 0.000100f;
                break;
        }
        /* use ncurses function mvprintw(x, y, printf args..)  to a location on the terminal */
        mvprintw(0, 0, "2nd Order: %1.6f 3rd Order: %1.6f\n5th Order: %1.6f Threshold: %1.6f\n[a/s/z] decreases/increases/resets 2nd order harmonics\n" \
             "[d/f/c] decreases/increases/resets 3rd order harmonics\n" \
             "[g/h/b] decreases/increases/resets 5th order harmonics\n" \
             "[l/;/.] decreases/increases/resets sensitivity threshold\n"
             "[q] to quit\n", data.second, data.third, data.fifth, data.threshold);
        
    }
    /* End curses mode  */
    endwin();

    err = Pa_StopStream( stream );

    /* Stop audio stream */
    if (err != paNoError) {
        printf(  "PortAudio error: stop stream: %s\n", Pa_GetErrorText(err));
    }
    /* Close audio stream */
    err = Pa_CloseStream(stream);
    if (err != paNoError) {
        printf("PortAudio error: close stream: %s\n", Pa_GetErrorText(err));
    }
    /* Terminate audio stream */
    err = Pa_Terminate();
    if (err != paNoError) {
        printf("PortAudio error: terminate: %s\n", Pa_GetErrorText(err));
    }

    return 0;
}