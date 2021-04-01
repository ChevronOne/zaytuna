

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





#ifndef ZAY_APP_HPP
#define ZAY_APP_HPP

#include "zay_primary_win.hpp"

namespace zaytuna {

class ZaytunaApp final: public QApplication{

    static primary_winPtr zay_simu;
    static bool ok_;
    static int fontId_;
    std::string ZAY_PACKAGE_PATH;
    QIcon zay_icon;
    

    void checkForMaster();
    bool glSupported();
    bool use_shippedFonts();
    bool is_running();

public:
    ZaytunaApp() = delete;
    ZaytunaApp(int&, char**);
    virtual ~ZaytunaApp();
    bool ok();
    int run();

};




} // namespace zaytuna


#endif  // ZAY_APP_HPP



