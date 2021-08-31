

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
 * Copyright Abbas Mohammed Murrey 2019-21
 *
 * Permission to use, copy, modify, distribute and sell this software
 * for any purpose is hereby granted without fee, provided that the
 * above copyright notice appear in all copies and that both the copyright
 * notice and this permission notice appear in supporting documentation.
 * I make no representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 */





#include "zay_app.hpp"


namespace zaytuna {

ZaytunaApp::ZaytunaApp(int &argc, char** argv):QApplication(argc, argv)
{
    ros::init(argc, argv, ZAY_PACKAGE_NAME);
}


void ZaytunaApp::checkForMaster(){

    while(!ros::master::check()){

        std::this_thread::sleep_for(2s);

        ROS_ERROR_STREAM("Zaytuna: connection to master at ["
                         << ros::master::getHost()<<":"<<ros::master::getPort() 
                         << "] yet to be established..");
    }

    ZAY_PACKAGE_PATH = ros::package::getPath(ZAY_PACKAGE_NAME);
    if(ZAY_PACKAGE_PATH.size()==0){
        ROS_FATAL_STREAM("Could not find the package path <"<<ZAY_PACKAGE_NAME<< ">");
        exit(EXIT_FAILURE);
    }

}

bool ZaytunaApp::glSupported(){


    if (!QGLFormat::hasOpenGL() || !QGLFramebufferObject::hasOpenGLFramebufferObjects()) {

        QMessageBox::critical(nullptr, "OpenGL FrameBuffer Object",
                                 "This system does not support OpenGL/FrameBuffer Object!");

        ROS_FATAL_STREAM("This system does not support OpenGL/FrameBuffer Object!");

        return 0;
    }

    return 1;
}



bool ZaytunaApp::ok(){

    if(ok_)
        return ok_;


    if (!glSupported()) 
        return 0;

    
    checkForMaster();

    if(!use_shippedFonts())
        return 0;

    return ok_=1;

}



bool ZaytunaApp::use_shippedFonts(){

    std::string fontPath{ZAY_PACKAGE_PATH+"/fonts/OpenSans-Light.ttf"};
    fontId_ = QFontDatabase::addApplicationFont(fontPath.c_str());

    if (fontId_ != -1)
    {
        QFont font("OpenSans-Regular");
        this->setFont(font);

    }else{

        ROS_WARN_STREAM("WARNING: default fonts did not load!");
        if(!(QMessageBox::warning(nullptr, "warning",
                                "Failed to load default fonts! Do you want to proceed?",
                                QMessageBox::StandardButtons(QMessageBox::Yes | QMessageBox::No),
                                QMessageBox::No)==QMessageBox::Yes))
        {
            return 0;
        }
    }

    return 1;
    
}



int ZaytunaApp::run(){

    if(is_running())
        return -1;

    if(!ok_){

        ROS_FATAL("Zaytuna status check failed or it hasn't been checked for!");
        return -1;
    }

    zay_icon.addFile((ZAY_PACKAGE_PATH+"/resources/"+ZAY_PACKAGE_NAME).c_str());
    if(zay_icon.isNull()){
        ROS_ERROR_STREAM("Failed to load zaytuna icon!");
    }

    zay_simu.reset(new zaytuna::primary_win("Zaytuna", zay_icon, ZAY_PACKAGE_PATH));

    zay_simu->show();

    return this->exec();

}


ZaytunaApp::~ZaytunaApp(){

    if(fontId_!=-1)
        QFontDatabase::removeApplicationFont(fontId_);

    zay_simu.reset(nullptr);   /* this is needed as it's static, where all windows need to be destroyed befor the deletion of QApplication */

}



bool ZaytunaApp::is_running(){

    return (zay_simu != nullptr) && (zay_simu->isVisible());
    
}


primary_winPtr ZaytunaApp::zay_simu{nullptr};
bool ZaytunaApp::ok_{0};
int ZaytunaApp::fontId_{-1};



} // namespace  zaytuna



