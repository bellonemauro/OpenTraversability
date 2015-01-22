/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */

#include "PCL_upd_DEMO.h"
#include <QApplication>
#include <QMainWindow>

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  PCL_upd_DEMO window;
  cout<<"Starting GUI"<<endl;
  window.show ();
  return a.exec ();
}
