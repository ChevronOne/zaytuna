

//  Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
//
//  This file is part of the Zaytuna Simulator Project.
//
//  Distributed under the terms of the GNU General Public License
//  as published by the Free Software Foundation; You should have
//  received a copy of the GNU General Public License.
//  If not, see <http://www.gnu.org/licenses/>.
//
//
//  This software is distributed in the hope that it will be useful, but WITHOUT
//  WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
//  WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND
//  NON-INFRINGEMENT. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR ANYONE
//  DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY,
//  WHETHER IN CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. See the GNU
//  General Public License for more details.

/*
 * Copyright Abbas Mohammed Murrey 2019-20
 *
 * Permission to use, copy, modify, distribute and sell this software
 * for any purpose is hereby granted without fee, provided that the
 * above copyright notice appear in all copies and that both the copyright
 * notice and this permission notice appear in supporting documentation.
 * I make no representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 */





#include "zay_primary_win.hpp"
#include <QApplication>
#include <QFontDatabase>
#include <QScreen>
#include <iostream>

int main(int argc, char *argv[])
{
//    QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL,true);
    QApplication a(argc, argv);

    QString fontPath = "fonts/OpenSans-Light.ttf";
    int fontId = QFontDatabase::addApplicationFont(fontPath);
    if (fontId != -1)
    {
        QFont font("OpenSans-Regular");
        a.setFont(font);
    }else
        std::cerr << "WARNING: fonts did not load!\n";

    if (!QGLFormat::hasOpenGL() || !QGLFramebufferObject::hasOpenGLFramebufferObjects()) {
        QMessageBox::information(nullptr, "OpenGL FrameBuffer Object",
                                 "This system does not support OpenGL/FrameBuffer Object!");
        return -1;
    }

    zaytuna::primary_win w;
    w.setWindowTitle("Zaytuna-Simulator");
    w.show();

    return a.exec();
}



