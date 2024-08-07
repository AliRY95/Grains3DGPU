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
# $Id: Makefile.in 570129 2007-08-27 14:10:51Z amassari $
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

MODULE = dom

include ../Makefile.incl

DOM_CPP_PUBHEADERS = \
	DOM.hpp \
	DOMAttr.hpp \
	DOMBuilder.hpp \
	DOMCDATASection.hpp \
	DOMCharacterData.hpp \
	DOMComment.hpp \
	DOMConfiguration.hpp \
	DOMDocument.hpp \
	DOMDocumentRange.hpp \
	DOMDocumentTraversal.hpp \
	DOMDocumentFragment.hpp \
	DOMDocumentType.hpp \
	DOMElement.hpp \
	DOMEntity.hpp \
	DOMEntityReference.hpp \
	DOMEntityResolver.hpp \
	DOMError.hpp \
	DOMErrorHandler.hpp \
	DOMException.hpp \
	DOMImplementation.hpp \
	DOMImplementationLS.hpp \
	DOMImplementationRegistry.hpp \
	DOMImplementationSource.hpp \
	DOMInputSource.hpp \
	DOMLocator.hpp \
	DOMNamedNodeMap.hpp \
	DOMNode.hpp \
	DOMNodeFilter.hpp \
	DOMNodeIterator.hpp \
	DOMNodeList.hpp \
	DOMNotation.hpp \
	DOMProcessingInstruction.hpp \
	DOMPSVITypeInfo.hpp \
	DOMRange.hpp \
	DOMRangeException.hpp \
	DOMText.hpp \
	DOMTreeWalker.hpp \
	DOMTypeInfo.hpp \
	DOMUserDataHandler.hpp \
	DOMWriter.hpp \
	DOMWriterFilter.hpp \
	StDOMNode.hpp \
	DOMXPathNamespace.hpp \
	DOMXPathNSResolver.hpp \
	DOMXPathResult.hpp \
	DOMXPathEvaluator.hpp \
	DOMXPathException.hpp \
	DOMXPathExpression.hpp 

DOM_CPP_PRIVHEADERS =

DOM_C_FILES =

DOM_CPP_OBJECTS = \
	DOMException.$(TO) \
	DOMRangeException.$(TO) \
	DOMXPathException.$(TO) 

all::	includes $(DOM_CPP_OBJECTS) impl deprecated

includes:: pubheaders $(DOM_C_FILES)
	${MAKE} -C impl $@
	${MAKE} -C deprecated $@

impl::
	${MAKE} -C impl

deprecated::
	${MAKE} -C deprecated

pubheaders::
	$Qmkdir -p $(XML_INC_DIR)/$(MODULE)
	@echo "  (CP)  $(XML_INC_DIR)/$(MODULE)"
	$Q$(CP) $(DOM_CPP_PUBHEADERS) $(DOM_C_FILES) $(XML_INC_DIR)/$(MODULE)

# this may generate unnecessary dependencies, but it makes life easier
depend:: includes
	@echo "  (DEP)"
	$Q$(MAKE_DEPEND) $(XML_INCL)  *.cpp > $(DEPFILE)

clean::
	@echo "Making clean in $(MODULE) ..."
	$(RM2) $(addprefix $(XML_OBJ_DIR)/,$(DOM_CPP_OBJECTS))
	${MAKE} -C impl $@
	${MAKE} -C deprecated $@

distclean::	clean
	$(RM) Makefile $(DEPFILE)
	@echo "Removing all $(MODULE) header files ..."
	$(RM2) $(addprefix $(XML_INC_DIR)/$(MODULE)/,$(DOM_CPP_PUBHEADERS))
	${MAKE} -C impl $@
	${MAKE} -C deprecated $@

install::
	-mkdir -p $(DESTDIR)$(PREFIX_INCLUDE)/$(MODULE)
	$(CP) $(DOM_CPP_PUBHEADERS) $(DOM_C_FILES) $(DESTDIR)$(PREFIX_INCLUDE)/$(MODULE)
	${MAKE} -C impl $@
	${MAKE} -C deprecated $@
