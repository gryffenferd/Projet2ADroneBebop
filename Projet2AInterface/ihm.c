#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <SDL/SDL.h>
#include <pthread.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>

#include <libARSAL/ARSAL.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARNetwork/ARNetwork.h>
#include <libARNetworkAL/ARNetworkAL.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARStream/ARStream.h>

#define LABEL 8
#define MUTEX 9
#include "ihm.h"

#include "Pilotage_Drone_CMSJ_RIU.h"

/* Global value */
float value = 0;        // battery level (0 - 100)%
int controler = 0;      // controler choice (1 - 5)
float sx = 0;           // speed X
float sy = 0;           // speed Y
float sz = 0;           // speed Z
float altitud = 0;      // Altitude (meter)
float rollValue = 0;    // roll (-3 - 3)
float pitchValue = 0;   // pitch (-3 - 3)
int state = 0;          // connexion state
int isCalibration = 0;    // calibration

PCMD_t PCMD;            // PCMD struct (int flag,int roll,int pitch,int yaw,int gaz,int takeoff,int landing,int escape)
int keyEvent = 0;       // if KeyEvent

static int ptimer[8];
GMutex* mutex[MUTEX];

void configure(GtkWidget* widget, gpointer data)
{
}

void calibration(GtkWidget* widget, gpointer data)
{
    if(g_mutex_trylock(mutex[CALIBRATION]))
    {
        isCalibration = 1;
        g_mutex_unlock(mutex[CALIBRATION]);
    }
}

/* Retrieve the speed X value (shared data) using a citical section with the SPEED mutex
                        and modification of the widget speed X label                    */
void speedX(gpointer data)
{
    if(g_mutex_trylock(mutex[SPEED]))
    {
        char* speedx[10];
        sprintf(speedx,"%.2g",sx);
        g_mutex_unlock(mutex[SPEED]);
        gtk_label_set_label(data,speedx);
    }
}

/* Retrieve the speed Y value (shared data) using a citical section with the SPEED mutex
                        and modification of the widget speed Y label                    */
void speedY(gpointer data)
{

    if(g_mutex_trylock(mutex[SPEED]))
    {
        char* speedy[10];
        sprintf(speedy,"%.2g",sy);
        g_mutex_unlock(mutex[SPEED]);
        gtk_label_set_label(data,speedy);
    }
}

/* Retrieve the speed Z value (shared data) using a citical section with the SPEED mutex
                        and modification of the widget speed Z label                     */
void speedZ(gpointer data)
{
    if(g_mutex_trylock(mutex[SPEED]))
    {
        char* speedz[10];
        sprintf(speedz,"%.2g",sz);
        g_mutex_unlock(mutex[SPEED]);
        gtk_label_set_label(data,speedz);
    }
}

/* Retrieve the altitude value (shared data) using a citical section with the ALTITUDE mutex
                        and modification of the widget altitude label                       */
void altitude(gpointer data)
{
    if(g_mutex_trylock(mutex[ALTITUDE]))
    {
        char* alt[10];
        sprintf(alt,"%.3g",altitud);
        strcat(alt," m");
        g_mutex_unlock(mutex[ALTITUDE]);
        gtk_label_set_label(data,alt);
    }

}

/* Retrieve the roll value (shared data) using a citical section with the ATTITUDE mutex
                        and modification of the widget roll label                      */
void roll(gpointer data)
{
    if(g_mutex_trylock(mutex[ATTITUDE]))
    {
        char* rollVal[30];
        sprintf(rollVal,"%.3g",(rollValue*60.0));
        strcat(rollVal," °");
        g_mutex_unlock(mutex[ATTITUDE]);
        gtk_label_set_label(data,rollVal);
    }
}

/* Retrieve the pitch value (shared data) using a citical section with the ATTITUDE mutex
                        and modification of the widget pitch label                       */
void pitch(gpointer data)
{
    if(g_mutex_trylock(mutex[ATTITUDE]))
    {
        char* pitchVal[10];
        sprintf(pitchVal,"%.2g",pitchValue*60.0);
        strcat(pitchVal," °");
        g_mutex_unlock(mutex[ATTITUDE]);
        gtk_label_set_label(data,pitchVal);
    }
}

/*  Retrieve the state connexion value (shared data) using a citical section with the STATE mutex
                        and modification of the widget state label                                */
void stateConnexion(gpointer data)
{
        gchar* TextConverti;
        if(state == 0)
        {
            TextConverti= g_locale_to_utf8("<span background=\"red\">Offline</span>", -1, NULL, NULL, NULL);
            gtk_label_set_label(data,TextConverti);
            g_free(TextConverti);
            gtk_label_set_use_markup(GTK_LABEL(data),TRUE);
        }
        else
        {
            TextConverti= g_locale_to_utf8("<span background=\"Green\">Online</span>", -1, NULL, NULL, NULL);
            gtk_label_set_label(data,TextConverti);
            g_free(TextConverti);
            gtk_label_set_use_markup(GTK_LABEL(data),TRUE);
        }
}

/*  Retrieve the battery level (shared data) using a citical section with the BATTERY mutex
    and modification of the widget battery barre.
    The value retrieve by the Bebop Drone is a percentage. The value is divised by 100. */
void progress(gpointer data)
{
    if(g_mutex_trylock(mutex[BATTERY]))
    {
        if(value >= 100)
            value = 100;
        gtk_progress_bar_update(GTK_PROGRESS_BAR(data),value/100);
        g_mutex_unlock(mutex[BATTERY]);
    }
}

/*  Event Key Release
    The PCMD is reset when a key is release */
gboolean on_key_release(GtkWidget *widget, GdkEventKey *event, gpointer user_data)
{
        PCMD.escape = 0;
        PCMD.flag = 0;
        PCMD.gaz = 0;
        PCMD.landing = 0;
        PCMD.pitch = 0;
        PCMD.roll = 0;
        PCMD.takeoff = 0;
        PCMD.yaw = 0;
        keyEvent = 1;
}

/* Event Key Pressed */
gboolean on_key_press (GtkWidget *widget, GdkEventKey *event, gpointer user_data)
{
  switch (event->keyval)
  {
    case GDK_z:
        PCMD.flag = 1;
        PCMD.pitch = 50;
        break;
    case GDK_s:
        PCMD.flag = 1;
        PCMD.pitch = -50;
      break;
    case GDK_d:
        PCMD.flag = 1;
        PCMD.roll = 50;
      break;
    case GDK_q:
        PCMD.flag = 1;
        PCMD.roll = -50;
        break;
    case GDK_e:
        PCMD.yaw = 50;
        break;
    case GDK_a:
        PCMD.yaw = -50;
        break;
    case GDK_k:
        PCMD.takeoff = 1;
        break;
    case GDK_space:
        PCMD.landing = 1;
        break;
    case GDK_Up:
        PCMD.gaz = 100;
        break;
    case GDK_Down:
        PCMD.gaz = -100;
        break;
    default:
        PCMD.escape = 0;
        PCMD.flag = 0;
        PCMD.gaz = 0;
        PCMD.landing = 0;
        PCMD.pitch = 0;
        PCMD.roll = 0;
        PCMD.takeoff = 0;
        PCMD.yaw = 0;
  }

    keyEvent = 1;
  return FALSE;
}

/* GUI */
void mainWindow(int argc, char** argv)
{
    /* Variables */
    GtkWidget* MainWindow;
    GtkWidget* Button[3];
    GtkWidget* Table = NULL;
    GtkWidget* barre = NULL;
    GtkWidget* label[15];
    gchar* TextConverti[LABEL + 1];

    GThread *thread;
    GError *error;

    ihm_t* ihm = malloc(sizeof(ihm_t));

    ihm->argc = argc;
    ihm->argv = argv;
    ihm->Window = NULL;

    /* Drone thread */
    thread = g_thread_create(drone,ihm,FALSE,&error);
    if(!thread)
    {
        g_print("Error: %s\n",error->message);
        return(-1);
    }

    /* Window creation */
    MainWindow = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    /* Window parameter */
    gtk_window_set_title(GTK_WINDOW(MainWindow),"Bebop Drone Controler");   // Titre
    gtk_window_set_default_size(GTK_WINDOW(MainWindow),600,500);            // Taille
    gtk_window_set_position(GTK_WINDOW(MainWindow),GTK_WIN_POS_CENTER);     // Position
    gtk_window_set_icon_from_file(GTK_WINDOW(MainWindow),"drone.png",NULL); // icone du logiciel
    gtk_rc_parse("style.txt");

    /* Button creation */
    Button[0] = gtk_button_new_with_label("Configuration");
    Button[1] = gtk_button_new_with_label("Video");
    Button[2] = gtk_button_new_with_label("Calibration");

    /* Progression Bar Creation */
    barre = gtk_progress_bar_new();
    gtk_progress_bar_update(GTK_PROGRESS_BAR(barre),value);

    /* Table creation */
    Table = gtk_table_new(10,8,TRUE);
    gtk_container_add(GTK_CONTAINER(MainWindow),Table);

    /* Labels personalization */
    TextConverti[0] = g_locale_to_utf8("<span><b>Stat Connexion</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises
    TextConverti[1] = g_locale_to_utf8("<span><b>X Speed :</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises
    TextConverti[2] = g_locale_to_utf8("<span><b>Y Speed :</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises
    TextConverti[3] = g_locale_to_utf8("<span><b>Z Speed :</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises
    TextConverti[4] = g_locale_to_utf8("<span><b>Altitude :</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises
    TextConverti[5] = g_locale_to_utf8("<span><b>Battery level :</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises
    TextConverti[6] = g_locale_to_utf8("<span><b>Roll :</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises
    TextConverti[7] = g_locale_to_utf8("<span><b>Pitch :</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises

    /* Labels creation */
    int i=0;
    for(i=0;i<LABEL;i++)
    {
        label[i] = gtk_label_new(TextConverti[i]);
        g_free(TextConverti[i]);
        gtk_label_set_use_markup(GTK_LABEL(label[i]),TRUE);
    }

    label[8] = gtk_label_new("0");
    label[9] = gtk_label_new("0");
    label[10] = gtk_label_new("0");
    label[11] = gtk_label_new("0");
    label[12] = gtk_label_new("0");
    label[13] = gtk_label_new("0");

    TextConverti[LABEL]= g_locale_to_utf8("<span background=\"red\">Offline</span>", -1, NULL, NULL, NULL);
    label[14] = gtk_label_new(TextConverti[LABEL]);
    g_free(TextConverti[LABEL]);
    gtk_label_set_use_markup(GTK_LABEL(label[14]),TRUE);

    /* Add widget in the table */
    gtk_table_attach(GTK_TABLE(Table),Button[1],2,6,1,8,GTK_EXPAND | GTK_FILL,GTK_EXPAND | GTK_FILL,0,0);
    gtk_table_attach(GTK_TABLE(Table),Button[0],0,1,9,10,GTK_EXPAND | GTK_FILL,GTK_EXPAND | GTK_FILL,0,0);
    gtk_table_attach(GTK_TABLE(Table),Button[2],7,8,9,10,GTK_EXPAND | GTK_FILL,GTK_EXPAND | GTK_FILL,0,0);
    gtk_table_attach(GTK_TABLE(Table),barre,3,5,8,9,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[0],2,4,0,1,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[1],0,1,3,4,GTK_EXPAND| GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[2],0,1,4,5,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[3],0,1,5,6,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[4],6,7,6,7,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[5],2,3,8,9,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[6],6,7,3,4,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[7],6,7,4,5,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[8],1,2,3,4,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[9],1,2,4,5,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[10],1,2,5,6,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[11],7,8,6,7,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[12],7,8,3,4,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[13],7,8,4,5,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[14],4,5,0,1,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);

    /* Event on button click */
    g_signal_connect(G_OBJECT(Button[0]),"clicked",G_CALLBACK(configure),NULL);
    g_signal_connect(G_OBJECT(Button[2]),"clicked",G_CALLBACK(calibration),NULL);

    /* Timers to refresh the layout */
    ptimer[0] = gtk_timeout_add (1000, progress, barre);
    ptimer[1] = gtk_timeout_add (100, speedX, label[8]);
    ptimer[2] = gtk_timeout_add (101, speedY, label[9]);
    ptimer[3] = gtk_timeout_add (102, speedZ, label[10]);
    ptimer[4] = gtk_timeout_add (103, altitude, label[11]);
    ptimer[5] = gtk_timeout_add (104, roll, label[12]);
    ptimer[6] = gtk_timeout_add (105, pitch, label[13]);
    ptimer[7] = gtk_timeout_add (5000,stateConnexion, label[14]);

    /* Display and event loop */
    gtk_widget_show_all(MainWindow);
    g_signal_connect(G_OBJECT(MainWindow), "delete-event", G_CALLBACK(gtk_main_quit), NULL);
    g_signal_connect (G_OBJECT (MainWindow), "key_press_event", G_CALLBACK (on_key_press), NULL);
    g_signal_connect(G_OBJECT (MainWindow),"key_release_event",G_CALLBACK(on_key_release),NULL);
}

/*  The keypad controler choice
    The shared data controler is modified */
void keypadIHM(GtkWidget* widget, void* data)
{
    if(g_mutex_trylock(mutex[CONTROLER]))
    {
        ihm_t* data2 = (ihm_t*)data;
        controler = 1;
        mainWindow(data2->argc,data2->argv);
        gtk_widget_destroy(GTK_WIDGET(data2->Window));
        g_mutex_unlock(mutex[CONTROLER]);
    }
}

/*  The joypad controler choice
    The shared data controler is modified */
void joypadIHM(GtkWidget* widget, void* data)
{
    if(g_mutex_trylock(mutex[CONTROLER]))
    {
        ihm_t* data2 = (ihm_t*)data;
        controler = 2;
        g_mutex_unlock(mutex[CONTROLER]);
        mainWindow(data2->argc,data2->argv);
        gtk_widget_destroy(GTK_WIDGET(data2->Window));
    }
}

/*  The joystick controler choice
    The shared data controler is modified */
void joystickIHM(GtkWidget* widget, void* data)
{
    if(g_mutex_trylock(mutex[CONTROLER]))
    {
        ihm_t* data2 = (ihm_t*)data;
        controler = 3;
        g_mutex_unlock(mutex[CONTROLER]);
        mainWindow(data2->argc,data2->argv);
        gtk_widget_destroy(GTK_WIDGET(data2->Window));
    }
}

/*  The mouse3D controler choice
    The shared data controler is modified */
void mouse3dIHM(GtkWidget* widget, void* data)
{
    if(g_mutex_trylock(mutex[CONTROLER]))
    {
        ihm_t* data2 = (ihm_t*)data;
        controler = 4;
        g_mutex_unlock(mutex[CONTROLER]);
        mainWindow(data2->argc,data2->argv);
        gtk_widget_destroy(GTK_WIDGET(data2->Window));
    }
}

/* GUI : choice of the drone controler */
void choice(int argc,char** argv)
{
    GtkWidget* Window = NULL;
    GtkWidget* button[4];
    GtkWidget* label = NULL;
    GtkWidget* table = NULL;
    gchar* TextConverti;

    ihm_t* data = malloc(sizeof(ihm_t));

    Window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(Window),"Bebop Drone Controler");   // Titre
    gtk_window_set_default_size(GTK_WINDOW(Window),600,500);            // Taille
    gtk_window_set_position(GTK_WINDOW(Window),GTK_WIN_POS_CENTER);     // Position
    gtk_window_set_icon_from_file(GTK_WINDOW(Window),"drone.png",NULL); // icone du logiciel
    gtk_rc_parse("style.txt");
    g_signal_connect(G_OBJECT(Window), "delete-event", G_CALLBACK(gtk_main_quit), NULL);

    data->Window = Window;
    data->argc = argc;
    data->argv = argv;

    button[0] = gtk_button_new_with_label("Keypad");
    button[1] = gtk_button_new_with_label("Joypad");
    button[2] = gtk_button_new_with_label("3D Mouse");
    button[3] = gtk_button_new_with_label("Joystick");

    TextConverti = g_locale_to_utf8("<span><b>Choose a controler</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises
    label = gtk_label_new(TextConverti);
    g_free(TextConverti);
    gtk_label_set_use_markup(GTK_LABEL(label),TRUE);

    table = gtk_table_new(5,5,TRUE);
    gtk_container_add(GTK_CONTAINER(Window),table);

    gtk_table_attach(GTK_TABLE(table),label,1,4,0,1,GTK_EXPAND | GTK_FILL,GTK_EXPAND | GTK_FILL,0,0);
    gtk_table_attach(GTK_TABLE(table),button[0],1,2,1,2,GTK_EXPAND | GTK_FILL,GTK_EXPAND | GTK_FILL,0,0);
    gtk_table_attach(GTK_TABLE(table),button[1],3,4,1,2,GTK_EXPAND | GTK_FILL,GTK_EXPAND | GTK_FILL,0,0);
    gtk_table_attach(GTK_TABLE(table),button[2],1,2,3,4,GTK_EXPAND | GTK_FILL,GTK_EXPAND | GTK_FILL,0,0);
    gtk_table_attach(GTK_TABLE(table),button[3],3,4,3,4,GTK_EXPAND | GTK_FILL,GTK_EXPAND | GTK_FILL,0,0);

    g_signal_connect(G_OBJECT(button[0]),"clicked",G_CALLBACK(keypadIHM),G_OBJECT(data));
    g_signal_connect(G_OBJECT(button[1]),"clicked",G_CALLBACK(joypadIHM),G_OBJECT(data));
    g_signal_connect(G_OBJECT(button[2]),"clicked",G_CALLBACK(mouse3dIHM),G_OBJECT(data));
    g_signal_connect(G_OBJECT(button[3]),"clicked",G_CALLBACK(joystickIHM),G_OBJECT(data));

    gtk_widget_show_all(Window);
}

/* The main program */
int main(int argc,char** argv)
{
    PCMD.escape = 0;
    PCMD.flag = 0;
    PCMD.gaz = 0;
    PCMD.landing = 0;
    PCMD.pitch = 0;
    PCMD.roll = 0;
    PCMD.takeoff = 0;
    PCMD.yaw = 0;

    /* Secure glib */
    if (! g_thread_supported())
        g_thread_init(NULL);

    /* Secure gtk */
    gdk_threads_init();

    int i;
    for(i=0;i<MUTEX;i++)
        mutex[i] = g_mutex_new();

    /* Initialisation of GTK+ */
    gtk_init(&argc, &argv);

    /* Launch the GUI choice */
    choice(argc,argv);

    gtk_main();
}
