

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






#ifndef ZAY_CLOCK_HPP
#define ZAY_CLOCK_HPP

#include "zay_headers.hpp"



//  Windows
#ifdef _WIN32
#include <Windows.h>
class __clock{
public:
    static double get_wall_time(){
        LARGE_INTEGER time,freq;
        if (!QueryPerformanceFrequency(&freq)){
            std::cout << "Error: couldn't get wall time!\n";
            exit(EXIT_FAILURE);
            return 0;
        }
        if (!QueryPerformanceCounter(&time)){
            std::cout << "Error: couldn't get the wall time!\n";
            exit(EXIT_FAILURE);
        }
        return (double)time.QuadPart / freq.QuadPart;
    }
    static double get_cpu_time(){
        FILETIME a,b,c,d;
        if (GetProcessTimes(GetCurrentProcess(),&a,&b,&c,&d) != 0){
            return
                    (double)(d.dwLowDateTime |
                             ((unsigned long long)d.dwHighDateTime << 32)) * 0.0000001;
        }else{
            std::cout << "Error: couldn't get cpu time!\n";
            exit(EXIT_FAILURE);
        }
    }

};

//  Posix/Linux
#else
#include <time.h>
#include <sys/time.h>

//class __clock{
//public:
//    static double get_wall_time(){
//        struct timeval time;
//        if (gettimeofday(&time, nullptr)){
//            std::cout << "Error: couldn't get wall time!\n";
//            exit(EXIT_FAILURE);
//        }
//        return (double)time.tv_sec + (double)time.tv_usec * .000001;
//    }
//    static double get_cpu_time(){
//        return (double)clock() / CLOCKS_PER_SEC;
//    }


//};

#endif

#endif // ZAY_CLOCK_HPP
