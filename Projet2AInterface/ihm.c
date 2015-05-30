#include <gtk/gtk.h>

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

#define LABEL 6
#include "ihm.h"

#include "Pilotage_Drone_CMSJ_RIU.h"


float value = 0;
int controler = 0;
float sx = 50;
float sy = 20;
float sz = 30;
float altitud = 10;

static int ptimer[5];

void configure(GtkWidget* widget, gpointer data)
{
}

void speedX(gpointer data)
{
    char speedx[10];
    sprintf(speedx,"%lf",sx);
    gtk_label_set_label(data,speedx);
}

void speedY(gpointer data)
{
    char* speedy[10];
    sprintf(speedy,"%lf",sy);
    gtk_label_set_label(data,speedy);
}

void speedZ(gpointer data)
{
    char* speedz[10];
    sprintf(speedz,"%lf",sz);
    gtk_label_set_label(data,speedz);
}

void altitude(gpointer data)
{
    char* alt[10];
    sprintf(alt,"%lf",altitud);
    gtk_label_set_label(data,alt);
}

void change(GtkWidget* widget, gpointer* data)
{
    choice();
    gtk_widget_destroy(GTK_WIDGET(data));
}

void progress(gpointer data)
{
    gdk_threads_enter();
    if(value >= 100)
        value = 100;
    gtk_progress_bar_update(GTK_PROGRESS_BAR(data),value/100);
    gdk_threads_leave();
}

void mainWindow()
{
    /* Variables */
    GtkWidget* MainWindow;
    GtkWidget* Button[3];
    GtkWidget* Table = NULL;
    GtkWidget* barre = NULL;
    GtkWidget* label[10];
    gchar* TextConverti[LABEL];

    /* Création de la fenêtre */
    MainWindow = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    g_signal_connect(G_OBJECT(MainWindow), "delete-event", G_CALLBACK(gtk_main_quit), NULL);

    /* Personalisation de la fenêtre */
    gtk_window_set_title(GTK_WINDOW(MainWindow),"Bebop Drone Controler");   // Titre
    gtk_window_set_default_size(GTK_WINDOW(MainWindow),600,500);            // Taille
    gtk_window_set_position(GTK_WINDOW(MainWindow),GTK_WIN_POS_CENTER);     // Position
    gtk_window_set_icon_from_file(GTK_WINDOW(MainWindow),"drone.png",NULL); // icone du logiciel
    gtk_rc_parse("style.txt");

    /* Création des bouton */
    Button[0] = gtk_button_new_with_label("Configuration");
    Button[1] = gtk_button_new_with_label("Change controler");
    Button[2] = gtk_button_new_with_label("Video");

    /* Création de la barre */
    barre = gtk_progress_bar_new();
    gtk_progress_bar_update(GTK_PROGRESS_BAR(barre),value);

    /* Création de la table */
    Table = gtk_table_new(10,8,TRUE);
    gtk_container_add(GTK_CONTAINER(MainWindow),Table);

    /* Personnalisation des labels */
    TextConverti[0] = g_locale_to_utf8("<span><b>Stat Connexion</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises
    TextConverti[1] = g_locale_to_utf8("<span><b>X Speed :</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises
    TextConverti[2] = g_locale_to_utf8("<span><b>Y Speed :</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises
    TextConverti[3] = g_locale_to_utf8("<span><b>Z Speed :</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises
    TextConverti[4] = g_locale_to_utf8("<span><b>Altitude :</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises
    TextConverti[5] = g_locale_to_utf8("<span><b>Battery level :</b></span>", -1, NULL, NULL, NULL);  //Convertion du texte avec les balises


    /* Création des labels */
    int i=0;
    for(i=0;i<LABEL;i++)
    {
        label[i] = gtk_label_new(TextConverti[i]);
        g_free(TextConverti[i]);
        gtk_label_set_use_markup(GTK_LABEL(label[i]),TRUE);
    }

    label[6] = gtk_label_new("0");
    label[7] = gtk_label_new("0");
    label[8] = gtk_label_new("0");
    label[9] = gtk_label_new("0");

    /* Ajout des éléments dans le tableau */
    gtk_table_attach(GTK_TABLE(Table),Button[2],2,6,1,8,GTK_EXPAND | GTK_FILL,GTK_EXPAND | GTK_FILL,0,0);
    gtk_table_attach(GTK_TABLE(Table),barre,3,5,8,9,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[0],1,7,0,1,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[1],0,1,3,4,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[2],0,1,4,5,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[3],0,1,5,6,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[4],6,7,6,7,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[5],2,3,8,9,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[6],1,2,3,4,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[7],1,2,4,5,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[8],1,2,5,6,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),label[9],7,8,6,7,GTK_EXPAND | GTK_FILL,GTK_EXPAND,0,0);
    gtk_table_attach(GTK_TABLE(Table),Button[0],0,1,9,10,GTK_EXPAND | GTK_FILL,GTK_EXPAND | GTK_FILL,0,0);
    gtk_table_attach(GTK_TABLE(Table),Button[1],7,8,9,10,GTK_EXPAND | GTK_FILL,GTK_EXPAND | GTK_FILL,0,0);

    /* Évènement sur click du bouton */
    g_signal_connect(G_OBJECT(Button[0]),"clicked",G_CALLBACK(configure),NULL);    //évènement sur le bouton
    g_signal_connect(G_OBJECT(Button[1]),"clicked",G_CALLBACK(change),G_OBJECT(MainWindow));

    ptimer[0] = gtk_timeout_add (1000, progress, barre);
    ptimer[1] = gtk_timeout_add (100, speedX, label[6]);
    ptimer[2] = gtk_timeout_add (100, speedY, label[7]);
    ptimer[3] = gtk_timeout_add (100, speedZ, label[8]);
    ptimer[4] = gtk_timeout_add (100, altitude, label[9]);

    /* Affichage et boucle évènementielle */
    gtk_widget_show_all(MainWindow);
}

void keypadIHM(GtkWidget* widget, gpointer* data)
{
    controler = 1;
    mainWindow();
    gtk_widget_destroy(GTK_WIDGET(data));
}

void joypadIHM(GtkWidget* widget, gpointer* data)
{
    controler = 2;
    mainWindow();
    gtk_widget_destroy(GTK_WIDGET(data));
}

void mouse3dIHM(GtkWidget* widget, gpointer* data)
{
    controler = 3;
    mainWindow();
    gtk_widget_destroy(GTK_WIDGET(data));
}

void joystickIHM(GtkWidget* widget, gpointer* data)
{
    controler = 4;
    mainWindow();
    gtk_widget_destroy(GTK_WIDGET(data));
}

void choice()
{
    GtkWidget* Window = NULL;
    GtkWidget* button[4];
    GtkWidget* label = NULL;
    GtkWidget* table = NULL;
    gchar* TextConverti;

    Window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(Window),"Bebop Drone Controler");   // Titre
    gtk_window_set_default_size(GTK_WINDOW(Window),600,500);            // Taille
    gtk_window_set_position(GTK_WINDOW(Window),GTK_WIN_POS_CENTER);     // Position
    gtk_window_set_icon_from_file(GTK_WINDOW(Window),"drone.png",NULL); // icone du logiciel
    gtk_rc_parse("style.txt");
    g_signal_connect(G_OBJECT(Window), "delete-event", G_CALLBACK(gtk_main_quit), NULL);

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

    g_signal_connect(G_OBJECT(button[0]),"clicked",G_CALLBACK(keypadIHM),G_OBJECT(Window));
    g_signal_connect(G_OBJECT(button[1]),"clicked",G_CALLBACK(joypadIHM),G_OBJECT(Window));
    g_signal_connect(G_OBJECT(button[2]),"clicked",G_CALLBACK(mouse3dIHM),G_OBJECT(Window));
    g_signal_connect(G_OBJECT(button[3]),"clicked",G_CALLBACK(joystickIHM),G_OBJECT(Window));

    gtk_widget_show_all(Window);
}

void *ihm(int argc, char** argv)
{
    GThread *thread;
    GError *error = NULL;

    /* Secure glib */
    if (! g_thread_supported())
        g_thread_init(NULL);

    /* Secure gtk */
    gdk_threads_init();

    /* Initialisation de GTK+ */
    gtk_init(&argc, &argv);
    choice();

    thread = g_thread_create(control,&ihm,FALSE,&error);
    gtk_main();

}
