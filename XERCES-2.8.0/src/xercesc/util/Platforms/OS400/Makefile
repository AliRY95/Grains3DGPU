#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.
# The ASF licenses this file to You under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with
# the License.  You may obtain a copy of the License at
# 
#      http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
# $Log$
# Revision 1.4  2004/09/08 13:56:41  peiyongz
# Apache License Version 2.0
#
# Revision 1.3  2004/07/09 16:23:59  peiyongz
# Build on HP-Itanium, aCC A.05.52
#
# Revision 1.2  2002/07/26 16:49:28  tng
# [Bug 2681] Can't build with gcc/g++ not named 'gcc'/'g++'.  Patch from Jonathan Lennox.
#
# Revision 1.1.1.1  2002/02/01 22:22:25  peiyongz
# sane_include
#
# Revision 1.3  2001/07/06 17:12:07  tng
# Automatic build of single-threaded library.  By Martin Kalen.
#
# Revision 1.2  2001/06/27 17:09:17  tng
# [Bug 1147] Headers install in wrong directory.  By Murray Cumming.
#
# Revision 1.1  2000/03/01 22:33:41  abagchi
# Initial checkin to create object files directly under obj
#:
#

PLATFORM = LINUX
CC  = gcc
CXX = g++
CXXVER = 
GCC = yes
GXX = yes
CXXFLAGS =   -w -O2 -DNDEBUG -DPROJ_XMLPARSER  -DPROJ_XMLUTIL  -DPROJ_PARSERS  -DPROJ_SAX4C  -DPROJ_SAX2  -DPROJ_DOM -DPROJ_DEPRECATED_DOM -DPROJ_VALIDATORS -DXML_USE_NATIVE_TRANSCODER -DXML_USE_INMEM_MESSAGELOADER -DXML_USE_PTHREADS -DXML_USE_NETACCESSOR_SOCKET  -DXML_BITSTOBUILD_64 
CFLAGS =   -w -O2 -DNDEBUG -DPROJ_XMLPARSER  -DPROJ_XMLUTIL  -DPROJ_PARSERS  -DPROJ_SAX4C  -DPROJ_SAX2  -DPROJ_DOM -DPROJ_DEPRECATED_DOM -DPROJ_VALIDATORS -DXML_USE_NATIVE_TRANSCODER -DXML_USE_INMEM_MESSAGELOADER -DXML_USE_PTHREADS -DXML_USE_NETACCESSOR_SOCKET   -DXML_BITSTOBUILD_64 
PREFIX = /usr/local
PREFIX_INCLUDE = ${PREFIX}/include/xercesc
LDFLAGS =    
LIBS =   -lpthread 
OSVER = 
USELIBWWW = 
MESSAGELOADER = INMEM
TRANSCODER = NATIVE
THREADS = pthread
LIBTYPE = shared
ARCH = x86_64

MODULE = util

include ../../../Makefile.incl

SUBMODULE = Platforms/OS400
CPP_PUBHEADERS = OS400Defs.hpp OS400PlatformUtils.hpp
CPP_OBJECTS = OS400PlatformUtils.$(TO)

include ../../Makefile.util.submodule
