/******************************************/
/*
*/
/******************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <portaudio.h>
#include <stdbool.h>
#include <sndfile.h>
#include <string.h>
#include <ncurses.h>
#include "fft.h"

// OpenGL
#ifdef __MACOSX_CORE__
  #include <GLUT/glut.h>
#else
  #include <GL/gl.h>
  #include <GL/glu.h>
  #include <GL/glut.h>
#endif

// Platform-dependent sleep routines.
#if defined( __WINDOWS_ASIO__ ) || defined( __WINDOWS_DS__ )
  #include <windows.h>
  #define SLEEP( milliseconds ) Sleep( (DWORD) milliseconds ) 
#else // Unix variants
  #include <unistd.h>
  #define SLEEP( milliseconds ) usleep( (unsigned long) (milliseconds * 1000.0) )
#endif


//-----------------------------------------------------------------------------
// global variables and #defines
//-----------------------------------------------------------------------------
#define FORMAT                  paFloat32
#define FRAMES_PER_BUFFER  4096
#define NUM_IN_CHANNELS     1
#define NUM_OUT_CHANNELS    1
#define WINDOW_SIZE         (FRAMES_PER_BUFFER/4)
#define HOP_SIZE            (WINDOW_SIZE/2)
#define INCREMENT           0.000010
#define threshINCREMENT     0.0001
#define SAMPLE                  float
#define SAMPLING_RATE           44100
#define MONO                    1
#define STEREO                  2
#define cmp_abs(x)              ( sqrt( (x).re * (x).re + (x).im * (x).im ) )
#define INIT_WIDTH              800
#define INIT_HEIGHT             600
#define BUFFER_SIZE		FRAMES_PER_BUFFER
#define SCROLL_BUFFER_SIZE (FRAMES_PER_BUFFER * 60)
#define G_SCROLL_WRITER (SCROLL_BUFFER_SIZE - BUFFER_SIZE)

typedef double  MY_TYPE;
typedef char BYTE;   // 8-bit unsigned entity.

// width and height of the window
GLsizei g_width = INIT_WIDTH;
GLsizei g_height = INIT_HEIGHT;
GLsizei g_last_width = INIT_WIDTH;
GLsizei g_last_height = INIT_HEIGHT;

// global audio vars
GLint g_buffer_size = BUFFER_SIZE;
SAMPLE g_buffer[BUFFER_SIZE*2];
SAMPLE pre_g_buffer[BUFFER_SIZE*2];
unsigned int g_channels = MONO;
GLint g_scroll_buffer_size = SCROLL_BUFFER_SIZE;
SAMPLE g_scroll_buffer[SCROLL_BUFFER_SIZE];
int g_scroll_reader = 0;
int g_scroll_writer = G_SCROLL_WRITER;
SAMPLE pre_fft_buffer[WINDOW_SIZE/4];
SAMPLE g_adaptive_curve[WINDOW_SIZE/4];

//define paData struct
typedef struct{
    float sampleRate;
    SNDFILE *infile;
    SF_INFO sfinfo;
    float file_buff[STEREO * (FRAMES_PER_BUFFER + HOP_SIZE)];
    float window[WINDOW_SIZE];
    float prev_win[WINDOW_SIZE];
    float curr_win[WINDOW_SIZE];
    float second;
    float third;
    float fifth;
    float threshold;
} paData;

paData data;

//define curr magnitude bins
float curr_magnitude[WINDOW_SIZE/2];
float curr_phase[WINDOW_SIZE/2];

//define prev magnitude bins
float prev_magnitude[WINDOW_SIZE/2];
float prev_phase[WINDOW_SIZE/2];

//define harmonics indices
bool  curr_harmonicsindex[WINDOW_SIZE/4];
bool  prev_harmonicsindex[WINDOW_SIZE/4];

//define adaptive curves
float curr_adaptivecurve[WINDOW_SIZE/4];
float prev_adaptivecurve[WINDOW_SIZE/4];

//turn phase vocoding on and off
bool toggle = true;

// Threads Management
GLboolean g_ready = false;

// fill mode
GLenum g_fillmode = GL_FILL;

// light 0 position
GLfloat g_light0_pos[4] = { 2.0f, 1.2f, 4.0f, 1.0f };

// light 1 parameters
GLfloat g_light1_ambient[] = { .2f, .2f, .2f, 1.0f };
GLfloat g_light1_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
GLfloat g_light1_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
GLfloat g_light1_pos[4] = { -2.0f, 0.0f, -4.0f, 1.0f };

// fullscreen
GLboolean g_fullscreen = false;

// modelview stuff
GLfloat g_linewidth = 1.0f;

// Port Audio struct
PaStream *g_stream;

//-----------------------------------------------------------------------------
// function prototypes
//-----------------------------------------------------------------------------
void idleFunc( );
void displayFunc( );
void reshapeFunc( int width, int height );
void keyboardFunc( unsigned char, int, int );
void specialKey( int key, int x, int y );
void specialUpKey( int key, int x, int y);
void initialize_graphics( );
void initialize_glut(int argc, char *argv[]);
void initialize_audio(int argc, PaStream **stream, char *argv[], void *userData);
void stop_portAudio(PaStream **stream);

//-----------------------------------------------------------------------------
// name: help()
// desc: ...
//-----------------------------------------------------------------------------
void help()
{

  printf( "----------------------------------------------------\n" );
  printf( "'e' - toggle fullscreen\n" );
  printf( "2nd Order: %1.6f 3rd Order: %1.6f\n5th Order: %1.6f Curve scaling: %1.6f\n[a/s/z] decreases/increases/resets 2nd order harmonics\n" \
             "[d/f/c] decreases/increases/resets 3rd order harmonics\n" \
             "[g/h/b] decreases/increases/resets 5th order harmonics\n" \
             "[l/;] decreases/increases adaptive curve\n"
             "[.] toggles phase vocoding effect\n"
             "[/] reset adaptive curve\n"
             "[q] to quit\n", data.second, data.third, data.fifth, data.threshold);
  printf( "----------------------------------------------------\n" );
  printf( "\n" );
}


//-----------------------------------------------------------------------------
// Name: paCallback( )
// Desc: callback from portAudio
//-----------------------------------------------------------------------------
static int paCallback( const void *inputBuffer,
             void *outputBuffer, unsigned long framesPerBuffer,
             const PaStreamCallbackTimeInfo* timeInfo,
             PaStreamCallbackFlags statusFlags, void *userData ) 
{
  SAMPLE * out = (SAMPLE *)outputBuffer;
  paData *data = (paData*)userData;

  // Zero-out the outputbuffer (silence)
  memset( out, 0.0f, sizeof(SAMPLE)*framesPerBuffer);
  
  int i, j, k, readcount;

  // read data from file
  readcount = sf_readf_float (data->infile, data->file_buff, framesPerBuffer + HOP_SIZE);  
  
  // if end of file reached, rewind
  if (readcount < framesPerBuffer) {
    sf_seek(data->infile, 0, SEEK_SET);
    readcount = sf_readf_float (data->infile, data->file_buff+(readcount*data->sfinfo.channels), framesPerBuffer-readcount + HOP_SIZE);  
  }

  /* Rewind the hop size */
  sf_seek( data->infile, -HOP_SIZE, SEEK_CUR );

  /* Separate left channel */
  float left[framesPerBuffer+HOP_SIZE];
  for (i = 0; i < framesPerBuffer + HOP_SIZE; ++i)
  {
      left[i] = data->file_buff[2*i];
      pre_g_buffer[i] = left[i];
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

      for(j = 0; j < WINDOW_SIZE/4; ++j)
      {
        pre_fft_buffer[j] = curr_magnitude[j];
      }

      //toggle signal processing
      if (toggle == true){

        //call function to generate and update adaptive curve
        adaptivecurve(curr_adaptivecurve, curr_magnitude, WINDOW_SIZE, data->threshold);
        adaptivecurve(prev_adaptivecurve, prev_magnitude, WINDOW_SIZE, data->threshold);

        //call findpeaks function to calculate which frequency bins to modify
        findpeaks(curr_magnitude, curr_adaptivecurve, curr_harmonicsindex, WINDOW_SIZE);
        findpeaks(prev_magnitude, prev_adaptivecurve, prev_harmonicsindex, WINDOW_SIZE);
      
        //2nd order harmonics generation
        harmonics(curr_harmonicsindex, prev_harmonicsindex, curr_magnitude, prev_magnitude, WINDOW_SIZE, data->second, 2);

        //3rd order harmonics generation
        harmonics(curr_harmonicsindex, prev_harmonicsindex, curr_magnitude, prev_magnitude, WINDOW_SIZE, data->third, 3);

        //5th order harmonics generation
        harmonics(curr_harmonicsindex, prev_harmonicsindex, curr_magnitude, prev_magnitude, WINDOW_SIZE, data->fifth, 5);
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
          g_buffer[i+j] = out[i+j];
          //printf("i+j: %d, bufsize: %d\n", i+j, BUFFER_SIZE*2); 
      }

      /* Update previous window */
      for (j = 0; j < WINDOW_SIZE; j++) {
          data->prev_win[j] = data->curr_win[j];
      }
    }
  
  // set flag
  g_ready = true;

  return paContinue;
}

//-----------------------------------------------------------------------------
// Name: initialize_glut( )
// Desc: Initializes Glut with the global vars
//-----------------------------------------------------------------------------
void initialize_glut(int argc, char *argv[]) {
  // initialize GLUT
  glutInit( &argc, argv );
  // double buffer, use rgb color, enable depth buffer
  glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );
  // initialize the window size
  glutInitWindowSize( g_width, g_height );
  // set the window postion
  glutInitWindowPosition( 400, 100 );
  // create the window
  glutCreateWindow( "harmonics");
  // full screen
  if( g_fullscreen )
    glutFullScreen();
  
  // set the idle function - called when idle
  glutIdleFunc( idleFunc );
  // set the display function - called when redrawing
  glutDisplayFunc( displayFunc );
  // set the reshape function - called when client area changes
  glutReshapeFunc( reshapeFunc );
  // set the keyboard function - called on keyboard events
  glutKeyboardFunc( keyboardFunc );
  // set window's to specialKey callback
  glutSpecialFunc( specialKey );
  // set window's to specialUpKey callback (when the key is up is called)
  glutSpecialUpFunc( specialUpKey );
  
  // do our own initialization
  initialize_graphics( );  
}

//-----------------------------------------------------------------------------
// Name: initialize_audio( RtAudio *dac )
// Desc: Initializes PortAudio with the global vars and the stream
//-----------------------------------------------------------------------------
void initialize_audio( int argc, PaStream **stream, char *argv[], void *userData) {
  
    PaStreamParameters outputParameters;
    PaStreamParameters inputParameters;
    PaError err;
    paData *data = (paData *)userData;
    
    /* clear structures - set everything to zero */
    memset(&data->sfinfo, 0, sizeof(data->sfinfo));

    // check for usage
    if ( argc != 2 ) {
        printf("Usage: %s audio_file\n", argv[0]);
        exit(1);
    }
    
    // Open Audio file
    data->infile = sf_open (argv[1], SFM_READ, &data->sfinfo);
    if (data->infile == NULL) {
      printf ("Error: could not open file: %s\n", argv[1]) ;
      puts(sf_strerror (NULL)) ;
      exit(1);
    }
    //printf("No of channels: %d", data->sfinfo.channels);
    /* Init Windows */
    hanning(data->window, WINDOW_SIZE);
    memset(&data->prev_win, 0, WINDOW_SIZE*sizeof(float));

    /* Init harmonics and threshold */
    data->second = 0.000000f;
    data->third = 0.000000f;
    data->fifth = 0.000000f;
    data->threshold = 0.0000f;

    /* Initialize PortAudio */
    Pa_Initialize();

    /*
    inputParameters.device = Pa_GetDefaultInputDevice();
    inputParameters.channelCount = MONO;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = 
    Pa_GetDeviceInfo( inputParameters.device )->defaultLowInputLatency;
    inputParameters.hostApiSpecificStreamInfo = NULL;
    */

    /* Set output stream parameters */
    outputParameters.device = Pa_GetDefaultOutputDevice();
    outputParameters.channelCount = g_channels;
    outputParameters.sampleFormat = paFloat32;
    outputParameters.suggestedLatency = 
    Pa_GetDeviceInfo( outputParameters.device )->defaultLowOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;

    /* Open audio stream */
    err = Pa_OpenStream( &(*stream),
            NULL,
            &outputParameters,
            SAMPLING_RATE, g_buffer_size, paNoFlag, 
            paCallback, data);

    if (err != paNoError) {
        printf("PortAudio error: open stream: %s\n", Pa_GetErrorText(err));
    }
  
    /* Start audio stream */
    err = Pa_StartStream( *stream );
    if (err != paNoError) {
        printf(  "PortAudio error: start stream: %s\n", Pa_GetErrorText(err));
    }
}

void stop_portAudio(PaStream **stream) {
    PaError err;

    /* Stop audio stream */
    err = Pa_StopStream( *stream );
    if (err != paNoError) {
        printf(  "PortAudio error: stop stream: %s\n", Pa_GetErrorText(err));
    }
    /* Close audio stream */
    err = Pa_CloseStream(*stream);
    if (err != paNoError) {
        printf("PortAudio error: close stream: %s\n", Pa_GetErrorText(err));
    }
    /* Terminate audio stream */
    err = Pa_Terminate();
    if (err != paNoError) {
        printf("PortAudio error: terminate: %s\n", Pa_GetErrorText(err));
    }
}

//-----------------------------------------------------------------------------
// Name: main
// Desc: ...
//-----------------------------------------------------------------------------
int main( int argc, char *argv[] )
{

  /* Initialize interactive character input */

  // Initialize Glut
  initialize_glut(argc, argv);
  
  // Initialize PortAudio
  initialize_audio(argc, &g_stream, argv, &data);

  // Print help
  help();

  // Wait until 'q' is pressed to stop the process
  glutMainLoop();

  // This will never get executed
  
  return EXIT_SUCCESS;
}

//-----------------------------------------------------------------------------
// Name: idleFunc( )
// Desc: callback from GLUT
//-----------------------------------------------------------------------------
void idleFunc( )
{
  // render the scene
  glutPostRedisplay( );
}

//-----------------------------------------------------------------------------
// Name: keyboardFunc( )
// Desc: key event
//-----------------------------------------------------------------------------
void keyboardFunc( unsigned char key, int x, int y )
{
  int i;
  //printf("key: %c\n", key);
  switch( key )
  {
      
    // Fullscreen
    case 'e':
      if( !g_fullscreen )
      {
        g_last_width = g_width;
        g_last_height = g_height;
        glutFullScreen();
      }
      else
        glutReshapeWindow( g_last_width, g_last_height );
      
      g_fullscreen = !g_fullscreen;
      break;
      
    case 'q':
      // Close Stream before exiting
      stop_portAudio(&g_stream);
      endwin();
      exit( 0 );
      break;
    case 'a':
      // decrease the 2nd order harmonics and refresh the terminal
      printf("\033[2J");
      printf("\033[%d;%dH", 0, 0);
      data.second -= INCREMENT;
      if (data.second < 0) {
        data.second = 0;
      }
      help();
      break;
    case 's':
      // increase the 2nd order harmonics and refresh the terminal
      printf("\033[2J");
      printf("\033[%d;%dH", 0, 0);
      data.second += INCREMENT;
      if (data.second > 1) {
        data.second = 1;
      }
      help();
      break;
    case 'z':
      // reset the 2nd order harmonics and refresh the terminal
      printf("\033[2J");
      printf("\033[%d;%dH", 0, 0);
      data.second = 0.000000f;
      help();
      break;
    case 'd':
      // decrease the 3rd order harmonics and refresh the terminal
      printf("\033[2J");
      printf("\033[%d;%dH", 0, 0);
      data.third -= INCREMENT;
      if (data.third < 0) {
        data.third = 0;
      }
      help();
      break;
    case 'f':
      // increase the 3rd order harmonics and refresh the terminal
      printf("\033[2J");
      printf("\033[%d;%dH", 0, 0);
      data.third += INCREMENT;
      if (data.third > 1) {
         data.third = 1;
      }
      help();
      break;
    case 'c':
      // reset the 3rd order harmonics and refresh the terminal
      printf("\033[2J");
      printf("\033[%d;%dH", 0, 0);
      data.third = 0.000000f;
      help();
      break;
    case 'g':
      // decrease 5th order harmonics and refresh the terminal
      printf("\033[2J");
      printf("\033[%d;%dH", 0, 0);
      data.fifth -= INCREMENT;
      if (data.fifth < 0) {
        data.fifth = 0;
      }
      help();
      break;
    case 'h':
      // increase 5th order harmonics and refresh the terminal
      printf("\033[2J");
      printf("\033[%d;%dH", 0, 0);
      data.fifth += INCREMENT;
      if (data.fifth > 1) {
        data.fifth = 1;
      }
      help();
      break;
    case 'b':
      // reset the 5th order harmonics and refresh the terminal
      printf("\033[2J");
      printf("\033[%d;%dH", 0, 0);
      data.fifth = 0.000000f;
      help();
      break;
    case 'l':
      // shift the adaptive curve down and refresh the terminal
      printf("\033[2J");
      printf("\033[%d;%dH", 0, 0);
      for(i = 0 ; i < WINDOW_SIZE/4 ; i++)
        {
          curr_adaptivecurve[i] -= threshINCREMENT;
          if (curr_adaptivecurve[i] < 0) {
            curr_adaptivecurve[i] = 0;
          }
          prev_adaptivecurve[i] -= threshINCREMENT;
          if (prev_adaptivecurve[i] < 0) {
            prev_adaptivecurve[i] = 0;
          }
        }
      data.threshold -= threshINCREMENT;
      help();
      break;
    case ';':
      // shift the adaptive curve up and refresh the terminal
      printf("\033[2J");
      printf("\033[%d;%dH", 0, 0);
      for(i = 0 ; i < WINDOW_SIZE/4 ; i++)
        {
          curr_adaptivecurve[i] += threshINCREMENT;
          if (curr_adaptivecurve[i] > 1) {
            curr_adaptivecurve[i] = 1;
          }
          prev_adaptivecurve[i] += threshINCREMENT;
          if (prev_adaptivecurve[i] > 1) {
            prev_adaptivecurve[i] = 1;
          }     
        }
      data.threshold += threshINCREMENT;
      help();
      break;
    case '.':
      // toggle the phase vocoding
      if (toggle == true)
        toggle = false;
      else
        toggle = true;
      break;
    case '/':
      //reset the adaptive curve and refresh the terminal
      printf("\033[2J");
      printf("\033[%d;%dH", 0, 0);
      data.threshold = 0.0f;
      help();
      break;
  }
}

//-----------------------------------------------------------------------------
// Name: specialUpKey( int key, int x, int y)
// Desc: Callback to know when a special key is pressed
//-----------------------------------------------------------------------------
void specialKey(int key, int x, int y) { 
  // Check which (arrow) key is pressed
  switch(key) {
    case GLUT_KEY_LEFT : // Arrow key left is pressed
      break;
    case GLUT_KEY_RIGHT :    // Arrow key right is pressed
      break;
    case GLUT_KEY_UP :        // Arrow key up is pressed
      break;
    case GLUT_KEY_DOWN :    // Arrow key down is pressed
      break;   
  }
}  

//-----------------------------------------------------------------------------
// Name: specialUpKey( int key, int x, int y)
// Desc: Callback to know when a special key is up
//-----------------------------------------------------------------------------
void specialUpKey( int key, int x, int y) {
  // Check which (arrow) key is unpressed
  switch(key) {
    case GLUT_KEY_LEFT : // Arrow key left is unpressed
      break;
    case GLUT_KEY_RIGHT :    // Arrow key right is unpressed
      break;
    case GLUT_KEY_UP :        // Arrow key up is unpressed
      break;
    case GLUT_KEY_DOWN :    // Arrow key down is unpressed
      break;   
  }
}


//-----------------------------------------------------------------------------
// Name: reshapeFunc( )
// Desc: called when window size changes
//-----------------------------------------------------------------------------
void reshapeFunc( int w, int h )
{
  // save the new window size
  g_width = w; g_height = h;
  // map the view port to the client area
  glViewport( 0, 0, w, h );
  // set the matrix mode to project
  glMatrixMode( GL_PROJECTION );
  // load the identity matrix
  glLoadIdentity( );
  // create the viewing frustum
  //gluPerspective( 45.0, (GLfloat) w / (GLfloat) h, .05, 50.0 );
  gluPerspective( 45.0, (GLfloat) w / (GLfloat) h, 1.0, 1000.0 );
  // set the matrix mode to modelview
  glMatrixMode( GL_MODELVIEW );
  // load the identity matrix
  glLoadIdentity( );
  
  // position the view point
  //  void gluLookAt( GLdouble eyeX,
  //                 GLdouble eyeY,
  //                 GLdouble eyeZ,
  //                 GLdouble centerX,
  //                 GLdouble centerY,
  //                 GLdouble centerZ,
  //                 GLdouble upX,
  //                 GLdouble upY,
  //                 GLdouble upZ )
  
  gluLookAt( 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f );
  /*gluLookAt( 0.0f, 3.5f * sin( 0.0f ), 3.5f * cos( 0.0f ), 
            0.0f, 0.0f, 0.0f, 
            0.0f, 1.0f , 0.0f );*/
   
}


//-----------------------------------------------------------------------------
// Name: initialize_graphics( )
// Desc: sets initial OpenGL states and initializes any application data
//-----------------------------------------------------------------------------
void initialize_graphics()
{
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);                 // Black Background
  // set the shading model to 'smooth'
  glShadeModel( GL_SMOOTH );
  // enable depth
  glEnable( GL_DEPTH_TEST );
  // set the front faces of polygons
  glFrontFace( GL_CCW );
  // set fill mode
  glPolygonMode( GL_FRONT_AND_BACK, g_fillmode );
  // enable lighting
  glEnable( GL_LIGHTING );
  // enable lighting for front
  glLightModeli( GL_FRONT_AND_BACK, GL_TRUE );
  // material have diffuse and ambient lighting 
  glColorMaterial( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
  // enable color
  glEnable( GL_COLOR_MATERIAL );
  // normalize (for scaling)
  glEnable( GL_NORMALIZE );
  // line width
  glLineWidth( g_linewidth );
  
  // enable light 0
  glEnable( GL_LIGHT0 );
  
  // setup and enable light 1
  glLightfv( GL_LIGHT1, GL_AMBIENT, g_light1_ambient );
  glLightfv( GL_LIGHT1, GL_DIFFUSE, g_light1_diffuse );
  glLightfv( GL_LIGHT1, GL_SPECULAR, g_light1_specular );
  glEnable( GL_LIGHT1 );
}

//-----------------------------------------------------------------------------
// Name: void drawWindowedTimeDomain(SAMPLE *buffer)
// Desc: Draws the Windowed Time Domain signal in the top of the screen
//-----------------------------------------------------------------------------
void drawWindowedTimeDomain(SAMPLE *buffer) {
  // Initialize initial x
  GLfloat x = -6.8;
  
  // Calculate increment x
  GLfloat xinc = fabs((2.1*x)/g_buffer_size);
  
  glPushMatrix();
  {
    glTranslatef(0,2.5f,0.0f);
    glColor3f(0., .2, 0.0);
    
    glBegin(GL_LINE_STRIP);
    
    // Draw Pre-processing Windowed Time Domain
    for (int i=0; i<g_buffer_size; i++)
    {
      glVertex3f(x, pre_g_buffer[i], 0.0f);
      x += xinc;
    }
    
    glEnd();
    
  }
  glPopMatrix();

  x = -6.8;

  glPushMatrix();
  {
    glTranslatef(0,2.5f,0.0f);
    glColor3f(.2, 0.0, 0.0);
    
    glBegin(GL_LINE_STRIP);
    
    // Draw Post-processing Windowed Time Domain
    for (int i=0; i<g_buffer_size; i++)
    {
      glVertex3f(x, buffer[i], 0.0f);
      x += xinc;
    }
    
    glEnd();
    
  }
  glPopMatrix();

}

void drawScrollingTimeDomain(SAMPLE *buffer) {
    //increment g_scroll_writer
    g_scroll_writer += g_buffer_size;
    g_scroll_writer %= g_scroll_buffer_size;
    
    // Initialize initial x
    GLfloat x = -6.8;
    
    // Calculate increment x
    GLfloat xinc = fabs((2.5*x)/g_scroll_buffer_size);
    
    
    // Scroll Time Domain signal
    glPushMatrix();
    {
        glTranslatef(0.0, -3.0f, 0.0f);
        glColor3f(0.2, 0.2, 1.0);

        glBegin(GL_LINE_STRIP);
        
        for (int i = 0; i < g_scroll_buffer_size; i++) {
            glVertex2f (x, 0.7*g_scroll_buffer[g_scroll_reader]);
            x += xinc;
            // Increment the reader buffer_size
            g_scroll_reader++;
            g_scroll_reader %= g_scroll_buffer_size;
        }
        
        // Increment the scroll reader index
        g_scroll_reader += g_buffer_size;
        g_scroll_reader %= g_scroll_buffer_size;
        
        glEnd();
    }
    glPopMatrix();
}

void drawFrequencyDomain(SAMPLE *buffer)
{
// Initialize initial x
  GLfloat x = -6.8;

  // Calculate increment x
  GLfloat xinc = fabs((2.7*x)/(WINDOW_SIZE/4.));
  
  glPushMatrix();
  {
    glTranslatef(0,5.2f,0.0f);
    glColor3f(1., .2, 0.0);
    
    glBegin(GL_LINE_STRIP);
    
    // Draw Windowed Time Domain
    for (int i=0; i<WINDOW_SIZE/4; i++)
    {
      glVertex3f(x, 3*log10(pre_fft_buffer[i]+0.01), 0.0f);
      // glVertex3f(x, 0.0, 0.0f);
      x += xinc;
    }
    
    glEnd();
    
  }
  glPopMatrix();

  x = -6.8;

  glPushMatrix();
  {
    glTranslatef(0,5.2f,0.0f);
    glColor3f(.2, 0.0, 0.0);
    
    glBegin(GL_LINE_STRIP);
    
    // Draw Windowed Time Domain
    for (int i=0; i<WINDOW_SIZE/4; i++)
    {
      glVertex3f(x, 3*log10(curr_adaptivecurve[i]+0.01), 0.0f);
      x += xinc;
    }
    
    glEnd();
    
  }
  glPopMatrix();

}

//-----------------------------------------------------------------------------
// Name: displayFunc( )
// Desc: callback function invoked to draw the client area
//-----------------------------------------------------------------------------
void displayFunc( )
{
  // local state
  static GLfloat zrot = 0.0f, c = 0.0f;
  
  GLfloat x, xinc;
  
  // local variables
  SAMPLE buffer[g_buffer_size];
  
  // wait for data
//  while( !g_ready ) usleep( 1000 );
  int i;
  // copy currently playing audio into buffer
  for(i = 0; i < g_buffer_size; i++)
    buffer[i] = g_buffer[i];

  // copy buffer to the last part of the g_scroll_buffer
  for (int i = g_scroll_writer, j = 0; i<g_scroll_writer+g_buffer_size; i++, j++) {
    g_scroll_buffer[i] = buffer[j];
  }
  
  // Hand off to audio callback thread
  g_ready = false;
  
  // clear the color and depth buffers
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
  
  // Windowed Time Domain
  drawWindowedTimeDomain(buffer);
  
  // Scrolling Time Domain
  drawScrollingTimeDomain(buffer);

  // draw magnitude and adaptive curve
  drawFrequencyDomain(buffer);

  // flush gl commands
  glFlush( );

  // swap the buffers
  glutSwapBuffers( );
}