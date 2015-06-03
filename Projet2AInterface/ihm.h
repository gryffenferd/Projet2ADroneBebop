#ifndef IHM_H
#define IHM_H

#include <gtk/gtk.h>
#include "Pilotage_Drone_CMSJ_RIU.h"

enum {
    ALTITUDE,
    SPEED,
    ATTITUDE,
    BATTERY,
    STATE,
    CONTROLER,
    EVENT,
    PCMDNUM
};

struct PCMD_t
{
    int flag;
    int roll;
    int pitch;
    int yaw;
    int gaz;
    int takeoff;
    int landing;
    int escape;
};

typedef struct PCMD_t PCMD_t;

void configure(GtkWidget* widget, gpointer data);
void progress(gpointer data);
void mainWindow(int argc,char** argv);
void speedX(gpointer data);
void speedY(gpointer data);
void speedZ(gpointer data);
void altitude(gpointer data);

void keypadIHM(GtkWidget* widget, void* data);
void joypadIHM(GtkWidget* widget, void* data);
void mouse3dIHM(GtkWidget* widget, void* data);
void joystickIHM(GtkWidget* widget, void* data);
void choice(int argc,char** argv);

#endif
