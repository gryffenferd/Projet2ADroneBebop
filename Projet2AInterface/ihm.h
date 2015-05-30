#ifndef IHM_H
#define IHM_H

#include <gtk/gtk.h>

void configure(GtkWidget* widget, gpointer data);
void change(GtkWidget* widget, gpointer* data);
void progress(gpointer data);
void mainWindow();
void speedX(gpointer data);
void speedY(gpointer data);
void speedZ(gpointer data);
void altitude(gpointer data);

void keypadIHM(GtkWidget* widget, gpointer* data);
void joypadIHM(GtkWidget* widget, gpointer* data);
void mouse3dIHM(GtkWidget* widget, gpointer* data);
void joystickIHM(GtkWidget* widget, gpointer* data);
void choice();
void *ihm(int argc, char **argv);

#endif
