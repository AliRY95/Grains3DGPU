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
#
# $Log$
# Revision 1.5  2004/09/08 13:56:33  peiyongz
# Apache License Version 2.0
#
# Revision 1.4  2004/07/09 16:15:46  peiyongz
# Build on HP-Itanium, aCC A.05.52
#
# Revision 1.3  2002/12/12 19:14:00  peiyongz
# InMem Message file name changed.
#
# Revision 1.2  2002/07/26 16:49:27  tng
# [Bug 2681] Can't build with gcc/g++ not named 'gcc'/'g++'.  Patch from Jonathan Lennox.
#
# Revision 1.1.1.1  2002/02/01 22:22:21  peiyongz
# sane_include
#
# Revision 1.7  2001/07/06 17:11:59  tng
# Automatic build of single-threaded library.  By Martin Kalen.
#
# Revision 1.6  2001/06/27 17:08:41  tng
# [Bug 1147] Headers install in wrong directory.  By Murray Cumming.
#
# Revision 1.5  2000/02/06 07:48:22  rahulj
# Year 2K copyright swat.
#
# Revision 1.4  2000/02/01 23:43:42  abagchi
# AS/400 related change
#
# Revision 1.3  1999/12/14 23:53:28  rahulj
# Removed the offending Ctrl-M's from the commit message
# logs which was giving packaging problems.
#
# PR:
# Obtained from:
# Submitted by:
# Reviewed by:
#
# Revision 1.2  1999/11/23 02:00:22  rahulj
# Code now works under HPUX 11. Tested inmemory message loader.
# Revamped makefiles. Builds with both DCE threads as well as pthread libraries.
#
# Revision 1.1.1.1  1999/11/09 01:07:20  twl
# Initial checkin
#
# Revision 1.3  1999/11/08 20:45:27  rahul
# Swat for adding in Product name and CVS comment log variable.
#
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
SUBMODULE = MsgLoaders/InMemory

include ../../../Makefile.incl

CPP_PUBHEADERS = \
	XercesMessages_en_US.hpp \
	InMemMsgLoader.hpp

CPP_OBJECTS = InMemMsgLoader.$(TO)

include ../../Makefile.util.submodule
