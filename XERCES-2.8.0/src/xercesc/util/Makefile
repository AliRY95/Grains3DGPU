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
# Revision 1.46  2005/04/05 09:21:56  gareth
# Fix for jira 1397. *Much* nicer make output. Thanks to Axel Weib.
#
# Revision 1.45  2004/10/20 15:18:35  knoaman
# Allow option of initializing static data in XMLPlatformUtils::Initialize
#
# Revision 1.44  2004/09/08 13:56:22  peiyongz
# Apache License Version 2.0
#
# Revision 1.43  2004/07/09 16:13:57  peiyongz
# Build on HP-Itanium, aCC A.05.52
#
# Revision 1.42  2004/02/24 22:57:27  peiyongz
# XercesDeprecatedDOMLib
#
# Revision 1.41  2003/12/01 22:16:56  neilg
# change spaces to tabs, as suggested in bug 25084
#
# Revision 1.40  2003/11/25 03:46:44  neilg
# enable 1047 transcoder to build on Unix
#
# Revision 1.39  2003/10/30 21:37:32  knoaman
# Enhanced Entity Resolver Support. Thanks to David Cargill.
#
# Revision 1.38  2003/10/09 13:52:17  neilg
# build SynchronizedStringPool
#
# Revision 1.37  2003/10/01 16:32:39  neilg
# improve handling of out of memory conditions, bug #23415.  Thanks to David Cargill.
#
# Revision 1.36  2003/05/29 11:27:51  gareth
# fix to bug #20325. Removed unused file and updated Projects.
#
# Revision 1.35  2003/04/30 15:54:03  knoaman
# Add XMemory.
#
# Revision 1.34  2003/04/24 02:49:36  peiyongz
# Logical Path Resolution
#
# Revision 1.33  2003/04/21 16:15:10  knoaman
# Remove MemoryManagerImpl.
#
# Revision 1.32  2003/04/21 15:58:07  knoaman
# Add MemoryManagerImpl.
#
# Revision 1.31  2003/04/17 21:58:49  neilg
# Adding a new property,
# http://apache.org/xml/properties/security-manager, with
# appropriate getSecurityManager/setSecurityManager methods on DOM
# and SAX parsers.  Also adding a new SecurityManager class.
#
# The purpose of these modifications is to permit applications a
# means to have the parser reject documents whose processing would
# otherwise consume large amounts of system resources.  Malicious
# use of such documents could be used to launch a denial-of-service
# attack against a system running the parser.  Initially, the
# SecurityManager only knows about attacks that can result from
# exponential entity expansion; this is the only known attack that
# involves processing a single XML document.  Other, simlar attacks
# can be launched if arbitrary schemas may be parsed; there already
# exist means (via use of the EntityResolver interface) by which
# applications can deny processing of untrusted schemas.  In future,
# the SecurityManager will be expanded to take these other exploits
# into account.
#
# Add SecurityManager 
# 
# Revision 1.30  2003/03/18 21:06:38  peiyongz
# Build versioned shared library, libXercesMessages on UNIX
#
# Revision 1.29  2003/03/09 21:49:07  peiyongz
# Makefile:559: *** missing separator.
#
# Revision 1.28  2003/03/09 16:40:47  peiyongz
# PanicHandler
#
# Revision 1.27  2003/02/20 18:06:55  peiyongz
# Bug#7077: build error message shared library for ICUMsgLoader
#
# Revision 1.26  2003/01/07 16:02:33  tng
# need to properly copy and clean the message files in make install and clean.
#
# Revision 1.25  2002/12/20 22:10:20  tng
# XML 1.1
#
# Revision 1.24  2002/12/17 17:17:58  gareth
# added abstract base class BaseRefVectorOf from which both RefVectorOf and RefArrayVectorOf inherit
# the new RefArrayVectorOf has proper destructor for array deletion
#
# Revision 1.23  2002/12/12 16:37:09  peiyongz
# generate message catalog files for all UNIX platform if -miconv specified.
#
# Revision 1.22  2002/12/06 16:52:39  peiyongz
# $XERCESCROOT/msg created as home directory for message files.
#
# Revision 1.21  2002/12/02 21:49:16  peiyongz
# Synchronize default message catalog file name with default locale
#
# Revision 1.20  2002/11/22 14:55:32  tng
# 390: Uniconv390 support.  Patch by Chris Larsson and Stephen Dulin.
#
# Revision 1.19  2002/11/20 18:29:50  tng
# Make install should also copy the netaccessormodule header
#
# Revision 1.18  2002/11/13 12:43:05  tng
# DOM Message: new file XMLDOMMsg.hpp
#
# Revision 1.17  2002/11/07 22:37:14  peiyongz
# build -miconv on hpux and linux
#
# Revision 1.16  2002/11/04 22:28:05  peiyongz
# Locale setting for message loader
#
# Revision 1.15  2002/10/22 17:53:43  peiyongz
# Build resource bundles on Unix
#
# Revision 1.14  2002/09/17 21:24:01  peiyongz
# RPM for Xerces2.1.0, patch from Thomas Woerner (thomas@linux.de)
#
# Revision 1.13  2002/08/19 19:38:17  tng
# [Bug 11771] Linux specific IconvGNU transcoder.  Patch from Vasily Tchekalkin.
#
# Revision 1.12  2002/08/19 18:35:56  tng
# [Bug 6467] Installing Xerces C++ on cygwin environment.  Patch from Derek Harmon.
#
# Revision 1.11  2002/07/26 16:49:27  tng
# [Bug 2681] Can't build with gcc/g++ not named 'gcc'/'g++'.  Patch from Jonathan Lennox.
#
# Revision 1.10  2002/07/18 21:57:27  knoaman
# Makefile update.
#
# Revision 1.9  2002/07/10 12:56:45  tng
# [Bug 9154] Requesting Xerces Version Macro.
#
# Revision 1.8  2002/06/18 19:59:12  knoaman
# Remove DOMInputSourceWrapper.
#
# Revision 1.7  2002/06/12 17:10:16  tng
# Add KeyRefPair (similar to KeyValuePair but using reference instead of value).
#
# Revision 1.6  2002/05/29 21:47:08  knoaman
# DOM L3 LS: DOMInputSource, DOMEntityResolver, DOMImplementationLS and DOMBuilder
#
# Revision 1.5  2002/05/24 16:42:20  knoaman
# Performance fixes: eliminate mulitple calls to addRange and sort.
#
# Revision 1.4  2002/04/16 20:10:48  peiyongz
# Bug#8168: Error when attempting to build NetAccessors
#
# Revision 1.3  2002/02/18 06:36:44  jberry
# Modify Mac OS X "unix" build to use new MacOSURLAccessCF NetAccessor.
# This means that Xerces no longer relies on the Carbon.framework and
# links only to CoreServices.framework, which allows it to be called
# from a remote context where no GUI is available.
#
# Revision 1.2  2002/02/11 18:19:19  tng
# [Bug 2715] Build recursion suppresses make failures.
#
# Revision 1.1.1.1  2002/02/01 22:22:11  peiyongz
# sane_include
#
# Revision 1.52  2001/12/03 14:45:11  tng
# FreeBSD native transcoder (IconvFBSD) added by Max Gotlib.
#
# Revision 1.51  2001/11/19 21:59:00  peiyongz
# Reorganization: Double/Float
#
# Revision 1.50  2001/11/12 20:36:54  peiyongz
# SchemaDateTimeException defined
#
# Revision 1.49  2001/11/07 19:16:03  peiyongz
# DateTime Port
#
# Revision 1.48  2001/10/23 23:09:32  peiyongz
# [Bug#880] patch to PlatformUtils:init()/term() and related. from Mark Weaver
#
# Revision 1.47  2001/10/02 16:07:45  tng
# typo: fix extra spaces after the separator that led to make error
#
# Revision 1.46  2001/09/27 14:54:03  peiyongz
# DTV Reorganization: new class
#
# Revision 1.45  2001/08/17 14:06:59  peiyongz
# Typo: XMLHashCMStateSet.hpp -> HashCMStateSet.hpp
#
# Revision 1.44  2001/08/16 21:54:16  peiyongz
# new class creation
#
# Revision 1.43  2001/08/10 16:42:33  peiyongz
# XMLUri Added
#
# Revision 1.42  2001/07/26 20:40:01  peiyongz
# XMLFloat
#
# Revision 1.41  2001/07/24 20:39:44  peiyongz
# XMLDouble
#
# Revision 1.40  2001/07/06 17:11:58  tng
# Automatic build of single-threaded library.  By Martin Kalen.
#
# Revision 1.39  2001/06/27 17:08:30  tng
# [Bug 1147] Headers install in wrong directory.  By Murray Cumming.
#
# Revision 1.38  2001/06/11 15:15:45  knoaman
# StringTokenizer renamed to XMLStringTokenizer.
#
# Revision 1.37  2001/06/06 13:13:58  tng
# Fix regression where TRANSCODERMODULE is not assigned if TRANSCODE == NATIVE.
#
# Revision 1.36  2001/06/05 19:35:51  jberry
# Add support for Mac OS X command line configuration and build.
#
# Revision 1.35  2001/05/16 15:25:40  tng
# Schema: Add Base64 and HexBin.  By Pei Yong Zhang.
#
# Revision 1.34  2001/05/11 13:26:27  tng
# Copyright update.
#
# Revision 1.33  2001/05/10 20:51:18  tng
# Schema: Add DecimalDatatypeValidator and XMLBigDecimal, XMLBigInteger.  By Pei Yong Zhang.
#
# Revision 1.32  2001/04/19 17:47:56  knoaman
# A string tokenizer utility.
#
# Revision 1.31  2001/03/21 21:56:11  tng
# Schema: Add Schema Grammar, Schema Validator, and split the DTDValidator into DTDValidator, DTDScanner, and DTDGrammar.
#
# Revision 1.30  2001/03/02 20:06:19  knoaman
# Schema: Regular expression project and configuration file changes
#
# Revision 1.29  2001/03/02 14:39:13  tng
# Enabling libWWW NetAccessor support under UNIX. Tested with latest tarball of libWWW
# (w3c-libwww-5.3.2) under RedHat Linux 6.1.  Added by Martin Kalen.
#
# There is one MAJOR problem with the use of libwww and the patches
# below, which someone with knowledge of libwww filters etc. might want
# to look into. Default behavior for content-type text/xml is to consume
# all xml data before it reaches the simple HTML presenter. Hence, only
# files with content-type text/html will actually reach the xerces-c
# library. If you have a *.xml file on the webbserver, processing of the
# file will throw an exception stating "The main XML document cannot be
# empty" (correct in a xerces point of view since if you enable debug
# build you will see that libwww "eats" all text/xml).
#
# See "Diffs for enabling libWWW NetAccessor support under UNIX" posted in March 1, 2001
# in the xerces-c-dev mailing list for further information.
#
# Revision 1.28  2001/02/27 18:23:59  tng
# Schema: Add utility RefHash2KeysTableOf.
#
# Revision 1.27  2001/02/26 19:44:23  tng
# Schema: add utility class QName, by Pei Yong Zhang.
#
# Revision 1.26  2001/02/15 18:41:41  tng
# Schema: Exception Handling added by Pei Yong Zhang.  Add NumberFormatException, ParseException,
# InvalidDatatypeFacetException, and InvalidDatatypeValueException.  The associated
# Makefile.in and configure.in are updated.
#
# Revision 1.25  2000/07/25 22:28:15  aruna1
# Char definitions in XMLUni moved to XMLUniDefs
#
# Revision 1.24  2000/06/27 22:11:41  jpolast
# added HashBase, HashXMLCh, HashPtr
#
# Revision 1.23  2000/05/02 19:22:55  aruna1
# Unix related changes in.
#
# Revision 1.22  2000/03/20 23:48:50  rahulj
# Added Socket based NetAccessor. This will enable one to
# use HTTP URL's for system id's. Default build options do
# not use this NetAccessor. Specify the '-n socket' option
# to 'runConfigure' to configure Xerces-C to use this new
# feature. The code works under Solaris 2.6, Linux, AIX
# and HPUX 11 with aCC.
# Todo's: enable proper error handling.
#
# Revision 1.21  2000/03/20 19:17:00  abagchi
# Added XML256TableTranscoder
#
# Revision 1.20  2000/03/02 20:33:44  abagchi
# Changed XML4CDefs to XercesDefs
#
# Revision 1.19  2000/03/01 22:30:03  abagchi
# Object files are now created directly under obj directory
#
# Revision 1.18  2000/02/22 01:06:28  aruna1
# Added XMLChTranscoder
#
# Revision 1.17  2000/02/18 20:17:15  abagchi
# Added XMLWin1252Transcoder
#
# Revision 1.16  2000/02/17 00:58:07  abagchi
# Replaced IBM1047 with IBM1140
#
# Revision 1.15  2000/02/08 02:28:34  abagchi
# Added IBM 1047 Transcoder
#
# Revision 1.14  2000/02/08 02:03:44  aruna1
# Added changes for IBM1047 transcoder
#
# Revision 1.13  2000/02/06 07:48:02  rahulj
# Year 2K copyright swat.
#
# Revision 1.12  2000/02/04 23:48:45  abagchi
# Made sure all targets are appended with :: not single :
#
# Revision 1.11  2000/02/01 23:43:25  abagchi
# AS/400 related change
#
# Revision 1.10  2000/01/19 17:37:24  abagchi
# Removed the streaming classes
#
# Revision 1.9  2000/01/15 01:26:17  rahulj
# Added support for HTTP to the parser using libWWW 5.2.8.
# Renamed URL.[ch]pp to XMLURL.[ch]pp and like wise for the class name.
# Only tested under NT 4.0 SP 5.
# Removed URL.hpp from files where it was not used.
#
# Revision 1.8  2000/01/12 22:54:22  abagchi
# Added additional transcoding object files and headers
#
# Revision 1.7  2000/01/12 20:10:12  aruna1
# Modified for new file
#
# Revision 1.5  2000/01/05 22:06:11  aruna1
# MsgCatalog support for AIX introduced
#
# Revision 1.4  1999/12/23 01:43:17  aruna1
# MsgCatalog support added for solaris
#
# Revision 1.3  1999/12/14 23:53:24  rahulj
# Removed the offending Ctrl-M's from the commit message
# logs which was giving packaging problems.
#
# PR:
# Obtained from:
# Submitted by:
# Reviewed by:
#
# Revision 1.2  1999/11/23 02:00:05  rahulj
# Code now works under HPUX 11. Tested inmemory message loader.
# Revamped makefiles. Builds with both DCE threads as well as pthread libraries.
#
# Revision 1.1.1.1  1999/11/09 01:04:40  twl
# Initial checkin
#
# Revision 1.4  1999/11/08 20:45:09  rahul
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
NETACCESSOR = Socket
THREADS = pthread
LIBTYPE = shared
ARCH = x86_64

MODULE = util

ifeq (${TRANSCODER},NATIVE)
	ifeq (${PLATFORM},MACOSX)
	    TRANSCODERMODULE=MacOSUnicodeConverter
	else
	    TRANSCODERMODULE=Iconv
	endif
else
	ifeq (${TRANSCODER},ICU)
	   TRANSCODERMODULE=ICU
	else
	   ifeq (${TRANSCODER},Iconv400)
		TRANSCODERMODULE=Iconv400
	   else
		ifeq (${TRANSCODER},Iconv390)
		   TRANSCODERMODULE=Iconv390
		else
			ifeq (${TRANSCODER},IconvFBSD)
			TRANSCODERMODULE=IconvFBSD
			else
	                   ifeq (${TRANSCODER},Cygwin)
	                       TRANSCODERMODULE=Cygwin
	                   else
  				    ifeq (${TRANSCODER},IconvGNU)
				    TRANSCODERMODULE=IconvGNU
				else
	      		        ifeq (${TRANSCODER},Uniconv390)
	    			    TRANSCODERMODULE=Uniconv390
	    		        else
	    		            ifeq (${TRANSCODER},Win32)
	    		            TRANSCODERMODULE=Win32
	    		            else
 	    		            TRANSCODERMODULE=Iconv
	    		            endif
	      		        endif
  			        endif
  			    endif
			endif
		endif
	   endif
	endif
endif

ifeq (${MESSAGELOADER},ICU)
   MSG_DIR=${XERCESCROOT}/msg
   MESSAGELOADERMODULE=ICU
else
  ifeq (${MESSAGELOADER},MSGFILE)
	  MESSAGELOADERMODULE=MsgFile
  else
   ifeq (${MESSAGELOADER},ICONV)
	  MESSAGELOADERMODULE=MsgCatalog
	  MSG_DIR=${XERCESCROOT}/msg
   else
	  MESSAGELOADERMODULE=InMemory
   endif
  endif
endif

NETACCESSORMODULE=FileOnly

ifeq (${NETACCESSOR},NATIVE)
	NETACCESSORMODULE=MacOSURLAccessCF
else
	ifeq (${NETACCESSOR},Socket)
	  NETACCESSORMODULE=Socket
	else
	  ifeq (${NETACCESSOR},LibWWW)
		NETACCESSORMODULE=libWWW
	  else
			ifeq (${NETACCESSOR},WinSock)
				NETACCESSORMODULE=WinSock
			endif
	  endif
	endif
endif

include ../Makefile.incl

UTIL_CPP_PUBHEADERS = \
	ArrayIndexOutOfBoundsException.hpp \
	AutoSense.hpp \
	BaseRefVectorOf.hpp \
	Base64.hpp \
	BinFileInputStream.hpp \
	BinInputStream.hpp \
	BinMemInputStream.hpp \
	BitOps.hpp \
	BitSet.hpp \
	CountedPointer.hpp \
	DefaultPanicHandler.hpp \
	EmptyStackException.hpp \
	EncodingValidator.hpp \
	FlagJanitor.hpp \
	HashBase.hpp \
	HashCMStateSet.hpp \
	HashPtr.hpp \
	HashXMLCh.hpp \
	HexBin.hpp \
	IllegalArgumentException.hpp \
	InvalidCastException.hpp \
	IOException.hpp \
	Janitor.hpp \
	KeyRefPair.hpp \
	KeyValuePair.hpp \
	KVStringPair.hpp \
	Mutexes.hpp \
	NameIdPool.hpp \
	NoSuchElementException.hpp \
	NullPointerException.hpp \
	NumberFormatException.hpp \
	OutOfMemoryException.hpp \
	PanicHandler.hpp \
	ParseException.hpp \
	PlatformUtils.hpp \
	QName.hpp \
	RefArrayOf.hpp \
	RefArrayVectorOf.hpp \
	RefHashTableOf.hpp \
	RefHash2KeysTableOf.hpp \
	RefHash3KeysIdPool.hpp \
	RefStackOf.hpp \
	RefVectorOf.hpp \
	RuntimeException.hpp \
	SchemaDateTimeException.hpp \
	SecurityManager.hpp \
	StringPool.hpp \
	SynchronizedStringPool.hpp \
	TranscodingException.hpp \
	TransENameMap.hpp \
	TransService.hpp \
	UnexpectedEOFException.hpp \
	UnsupportedEncodingException.hpp \
	UTFDataFormatException.hpp \
	ValueArrayOf.hpp \
	ValueHashTableOf.hpp \
	ValueStackOf.hpp \
	ValueVectorOf.hpp \
	XercesDefs.hpp \
	XercesVersion.hpp \
	XMemory.hpp \
	XMLAbstractDoubleFloat.hpp \
	XMLASCIITranscoder.hpp \
	XMLBigDecimal.hpp \
	XMLBigInteger.hpp \
	XMLChar.hpp \
	XMLChTranscoder.hpp \
	XMLDateTime.hpp \
	XMLDeleterFor.hpp \
	XMLDOMMsg.hpp \
	XMLDouble.hpp \
	XMLEBCDICTranscoder.hpp \
	XMLEntityResolver.hpp \
	XMLEnumerator.hpp \
	XMLException.hpp \
	XMLExceptMsgs.hpp \
	XMLFloat.hpp \
	XMLIBM1047Transcoder.hpp \
	XMLIBM1140Transcoder.hpp \
	XMLInitializer.hpp \
	XMLInteger.hpp \
	XMLMsgLoader.hpp \
	XMLHolder.hpp \
	XMLNetAccessor.hpp \
	XMLNumber.hpp \
	XMLRegisterCleanup.hpp \
	XMLResourceIdentifier.hpp \
	XMLString.hpp \
	XMLStringTokenizer.hpp \
	XMLUCS4Transcoder.hpp \
	XMLUni.hpp \
	XMLUniDefs.hpp \
	XMLUri.hpp \
	XMLURL.hpp \
	XMLUTF16Transcoder.hpp \
	XMLUTF8Transcoder.hpp \
	XMLWin1252Transcoder.hpp \
	XML256TableTranscoder.hpp \
	XML88591Transcoder.hpp

UTIL_CPP_PRIVHEADERS =

C_FILES = \
	BaseRefVectorOf.c \
	CountedPointer.c \
	FlagJanitor.c \
	Janitor.c \
	KeyRefPair.c \
	KeyValuePair.c \
	NameIdPool.c \
	RefArrayOf.c \
	RefArrayVectorOf.c \
	RefHashTableOf.c \
	RefHash2KeysTableOf.c \
	RefHash3KeysIdPool.c \
	RefStackOf.c \
	RefVectorOf.c \
	TransENameMap.c \
	ValueArrayOf.c \
	ValueHashTableOf.c \
	ValueStackOf.c \
	ValueVectorOf.c \
	XMLDeleterFor.c \
	XMLHolder.c \
	LogicalPath.c

UTIL_CPP_OBJECTS = \
	Base64.$(TO) \
	BinFileInputStream.$(TO) \
	BinInputStream.$(TO) \
	BinMemInputStream.$(TO) \
	BitSet.$(TO) \
	DefaultPanicHandler.$(TO) \
	EncodingValidator.$(TO) \
	HashPtr.$(TO) \
	HashXMLCh.$(TO) \
	HeaderDummy.$(TO) \
	HexBin.$(TO) \
	KVStringPair.$(TO) \
	Mutexes.$(TO) \
	PanicHandler.$(TO) \
	PlatformUtils.$(TO) \
	QName.$(TO) \
	StringPool.$(TO) \
	SynchronizedStringPool.$(TO) \
	TransService.$(TO) \
	XMemory.$(TO) \
	XMLAbstractDoubleFloat.$(TO) \
	XMLASCIITranscoder.$(TO) \
	XMLBigDecimal.$(TO) \
	XMLBigInteger.$(TO) \
	XMLChar.$(TO) \
	XMLChTranscoder.$(TO) \
	XMLDateTime.$(TO) \
	XMLDouble.$(TO) \
	XMLEBCDICTranscoder.$(TO) \
	XMLException.$(TO) \
	XMLFloat.$(TO) \
	XMLIBM1047Transcoder.$(TO) \
	XMLIBM1140Transcoder.$(TO) \
	XMLInitializer.$(TO) \
	XMLMsgLoader.$(TO) \
	XMLNumber.$(TO) \
	XMLRegisterCleanup.$(TO) \
	XMLString.$(TO) \
	XMLStringTokenizer.$(TO) \
	XMLUCSTranscoder.$(TO) \
	XMLUni.$(TO) \
	XMLUri.$(TO) \
	XMLURL.$(TO) \
	XMLUTF16Transcoder.$(TO) \
	XMLUTF8Transcoder.$(TO) \
	XMLWin1252Transcoder.$(TO) \
	XML256TableTranscoder.$(TO) \
	XML88591Transcoder.$(TO)


all:: includes $(UTIL_CPP_OBJECTS) platforms transcoders \
	  messageloaders compilers netaccessors regx

includes:: pubheaders $(C_FILES)
	${MAKE} -C Platforms $@
	${MAKE} -C Transcoders/$(TRANSCODERMODULE) $@
ifeq (${TRANSCODERMODULE},Uniconv390)
	${MAKE} -C Transcoders/ICU $@
endif
	${MAKE} -C MsgLoaders/$(MESSAGELOADERMODULE) $@
ifneq (${NETACCESSORMODULE},FileOnly)
	${MAKE} -C NetAccessors/$(NETACCESSORMODULE) $@
endif
	${MAKE} -C Compilers $@
	${MAKE} -C regx $@


platforms::
	${MAKE} -C Platforms


transcoders::
ifeq (${TRANSCODERMODULE},Uniconv390)
	${MAKE} -C Transcoders/ICU
endif
	${MAKE} -C Transcoders/$(TRANSCODERMODULE)


messageloaders::
ifeq (${MESSAGELOADER},ICONV)
	mkdir -p ${MSG_DIR}
	gencat ${MSG_DIR}/XercesMessages_en_US.cat MsgLoaders/${MESSAGELOADERMODULE}/XercesMessages_en_US.Msg
endif
	${MAKE} -C MsgLoaders/$(MESSAGELOADERMODULE)
ifeq (${MESSAGELOADER},ICU)
	mkdir -p ${MSG_DIR}
	${MAKE} -C MsgLoaders/$(MESSAGELOADERMODULE)/resources
	$(CP) MsgLoaders/$(MESSAGELOADERMODULE)/resources/*.res ${MSG_DIR}	

endif

compilers::
	${MAKE} -C Compilers


netaccessors::
ifneq (${NETACCESSORMODULE},FileOnly)
	${MAKE} -C NetAccessors/$(NETACCESSORMODULE)
endif

regx::
	${MAKE} -C regx


pubheaders::
	$Qmkdir -p $(XML_INC_DIR)/$(MODULE)
	@echo "  (CP)  $(XML_INC_DIR)/$(MODULE)"
	$Q$(CP) $(UTIL_CPP_PUBHEADERS) $(C_FILES) $(XML_INC_DIR)/$(MODULE)


# this may generate unnecessary dependencies, but it makes life easier
depend:: includes
	@echo "  (DEP)"
	$Q$(MAKE_DEPEND) $(XML_INCL)  *.cpp > $(DEPFILE)


clean::
	@echo "Making clean in $(MODULE) ..."
	$(RM2) $(addprefix $(XML_OBJ_DIR)/,$(UTIL_CPP_OBJECTS))
	${MAKE} -C Platforms $@
	${MAKE} -C Transcoders/$(TRANSCODERMODULE) $@
ifeq (${TRANSCODERMODULE},Uniconv390)
	${MAKE} -C Transcoders/ICU $@
endif
	${MAKE} -C MsgLoaders/$(MESSAGELOADERMODULE) $@
ifeq (${MESSAGELOADER},ICU)
	${MAKE} -C MsgLoaders/$(MESSAGELOADERMODULE)/resources $@
	$(RM2) ${MSG_DIR}/*.res
endif
ifeq (${MESSAGELOADER},ICONV)
	$(RM2) ${MSG_DIR}/XercesMessages_en_US.cat
endif
ifneq (${NETACCESSORMODULE},FileOnly)
	${MAKE} -C NetAccessors/$(NETACCESSORMODULE) $@
endif
	${MAKE} -C Compilers $@
	${MAKE} -C regx $@



distclean:: clean
	$(RM) Makefile $(DEPFILE)
	@echo "Removing all $(MODULE) header files ..."
	$(RM2) $(addprefix $(XML_INC_DIR)/$(MODULE)/,$(UTIL_CPP_PUBHEADERS))
	$(RM2) $(addprefix $(XML_INC_DIR)/$(MODULE)/,$(C_FILES))
	${MAKE} -C Platforms $@
	${MAKE} -C Transcoders/$(TRANSCODERMODULE) $@
ifeq (${TRANSCODERMODULE},Uniconv390)
	${MAKE} -C Transcoders/ICU $@
endif
	${MAKE} -C MsgLoaders/$(MESSAGELOADERMODULE) $@
ifeq (${MESSAGELOADER},ICU)
	${MAKE} -C MsgLoaders/$(MESSAGELOADERMODULE)/resources $@
	$(RM2) ${MSG_DIR}/*.res
endif
ifeq (${MESSAGELOADER},ICONV)
	$(RM2) ${MSG_DIR}/XercesMessages_en_US.cat
endif
ifneq (${NETACCESSORMODULE},FileOnly)
	${MAKE} -C NetAccessors/$(NETACCESSORMODULE) $@
endif
	${MAKE} -C Compilers $@
	${MAKE} -C regx $@


install::
	-mkdir -p $(DESTDIR)$(PREFIX_INCLUDE)/$(MODULE)
	$(CP) $(UTIL_CPP_PUBHEADERS) $(C_FILES) $(DESTDIR)$(PREFIX_INCLUDE)/$(MODULE)
	${MAKE} -C Platforms $@
	${MAKE} -C Transcoders/$(TRANSCODERMODULE) $@
ifeq (${TRANSCODERMODULE},Uniconv390)
	${MAKE} -C Transcoders/ICU $@
endif
	${MAKE} -C MsgLoaders/$(MESSAGELOADERMODULE) $@
ifeq (${MESSAGELOADER},ICU)
	-mkdir -p ${DESTDIR}${PREFIX}/msg
	$(CP) ${MSG_DIR}/*.res ${DESTDIR}$(PREFIX)/msg
endif
ifeq (${MESSAGELOADER},ICONV)
	-mkdir -p ${DESTDIR}${PREFIX}/msg
	$(CP) ${MSG_DIR}/XercesMessages_en_US.cat ${DESTDIR}$(PREFIX)/msg
endif
ifneq (${NETACCESSORMODULE},FileOnly)
	${MAKE} -C NetAccessors/$(NETACCESSORMODULE) $@
endif
	${MAKE} -C Compilers $@
	${MAKE} -C regx $@
