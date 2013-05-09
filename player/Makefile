# Makefile for Keepaway Players

CC    	= g++
COMPILE = $(CC) -I . $(INCLUDES) ${FLAGS} ${DEFINES}
YACC = bison

SRC_DIR = .
DST_DIR = .
TARGET  = keepaway_player

# ==== INCLUDES =================================================

SOCCER_INCLUDES	= 
INCLUDES	= $(SOCCER_INCLUDES)

# ==== FLAGS ====================================================

# Substitute operating system that is used:
# SunOS_5 linux
OSFLAGS      	= ${OSFLAGS_linux}
CFLAGS_LIBS     = ${CLIBS_linux}
#OSFLAGS        = $(OSFLAGS_SunOS_5)
#CFLAGS_LIBS    = ${CLIBS_SunOS_5}

CFLAGS_DEBUG	= -g
CFLAGS_OPT     	= #-O3 #-static
CFLAGS_WARN     = -Wall

OSFLAGS_SunOS_5 = -DSolaris 
OSFLAGS_linux   = -Dlinux -rdynamic

CLIBS_SunOS_5   = 
CLIBS_linux     =
DEPENDFLAGS     = -I . 
FLAGS           = ${OSFLAGS} ${CFLAGS_OPT} ${CFLAGS_DEBUG} ${CFLAGS_WARN}

# ==== LIBS ====================================================

CLIBS_SunOS_5   = -lsocket -lnsl -lpthread
CLIBS_linux     = -lpthread -ldl

# ==== DEFINES =================================================
#DEFINES	        = -DEC_LOG_ENABLED=1

# ==== SOURCES =================================================

SRCS		=	\
		Connection.cc \
		SenseHandler.cc \
		ActHandler.cc \
		SoccerTypes.cc \
		Objects.cc \
		WorldModel.cc \
		WorldModelHighLevel.cc \
		WorldModelPredict.cc \
		WorldModelUpdate.cc \
		WorldModelKeepaway.cc \
		WorldModelSayMsgTypes.cc \
		ServerSettings.cc \
		PlayerSettings.cc \
		GenericValues.cc \
		Formations.cc \
		Geometry.cc \
		Parse.cc \
		Logger.cc \
		LoggerDraw.cc \

SRCS_PLAYER 	= ${SRCS} \
		BasicPlayer.cc \
		KeepawayPlayer.cc \
		HandCodedAgent.cc \
		tiles2.cc \
		LinearSarsaAgent.cc \
		main.cc \
        LSPIAgent.cc

OBJS_PLAYER	= $(SRCS_PLAYER:.cc=.o)

# ==== RULES ==================================================

all::	$(TARGET) demo_extension.so

.cc.o:
	$(COMPILE) -c -o $@ $(@F:%.o=%.cc)

# TODO Windows and Mac versions of this.
# TODO Convert the whole build to CMake?
demo_extension.so: DemoExtension.cc
	$(COMPILE) -fPIC -shared -o demo_extension.so DemoExtension.cc

keepaway_player: $(OBJS_PLAYER)  
	$(COMPILE) -o  $@ $(OBJS_PLAYER) ${CFLAGS_LIBS}

update:
	cvs update

commit:
	cvs commit

clean:
	rm -f $(TARGET) $(OBJS_PLAYER) *core *~ a.out

depend:
	gccmakedep $(DEPENDFLAGS) -s "# DO NOT DELETE THIS LINE" -- $(INCLUDES) -- $(SRCS_PLAYER)

# DO NOT DELETE THIS LINE
Connection.o: Connection.cc /usr/include/errno.h /usr/include/features.h \
 /usr/include/bits/predefs.h /usr/include/sys/cdefs.h \
 /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h \
 /usr/include/gnu/stubs-64.h /usr/include/bits/errno.h \
 /usr/include/linux/errno.h /usr/include/asm/errno.h \
 /usr/include/asm-generic/errno.h /usr/include/asm-generic/errno-base.h \
 /usr/include/string.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/xlocale.h /usr/include/sys/types.h \
 /usr/include/bits/types.h /usr/include/bits/typesizes.h \
 /usr/include/time.h /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/sys/select.h \
 /usr/include/bits/select.h /usr/include/bits/sigset.h \
 /usr/include/bits/time.h /usr/include/sys/sysmacros.h \
 /usr/include/bits/pthreadtypes.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/sys/socket.h /usr/include/sys/uio.h /usr/include/bits/uio.h \
 /usr/include/bits/socket.h /usr/include/bits/sockaddr.h \
 /usr/include/asm/socket.h /usr/include/asm-generic/socket.h \
 /usr/include/asm/sockios.h /usr/include/asm-generic/sockios.h \
 Connection.h /usr/include/stdio.h /usr/include/libio.h \
 /usr/include/_G_config.h /usr/include/wchar.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef /usr/include/bits/wchar.h \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/bits/sched.h \
 /usr/include/signal.h /usr/include/bits/setjmp.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc /usr/include/netdb.h \
 /usr/include/netinet/in.h /usr/include/stdint.h /usr/include/bits/in.h \
 /usr/include/rpc/netdb.h /usr/include/bits/siginfo.h \
 /usr/include/bits/netdb.h /usr/include/arpa/inet.h Logger.h \
 /usr/include/c++/4.4/fstream /usr/include/c++/4.4/bits/codecvt.h \
 /usr/include/c++/4.4/cstdio \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h
SenseHandler.o: SenseHandler.cc SenseHandler.h Connection.h \
 /usr/include/stdio.h /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/bits/types.h /usr/include/bits/typesizes.h \
 /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef /usr/include/bits/wchar.h \
 /usr/include/xlocale.h /usr/include/c++/4.4/exception \
 /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc /usr/include/netdb.h \
 /usr/include/netinet/in.h /usr/include/stdint.h \
 /usr/include/sys/socket.h /usr/include/sys/uio.h \
 /usr/include/sys/types.h /usr/include/sys/select.h \
 /usr/include/bits/select.h /usr/include/sys/sysmacros.h \
 /usr/include/bits/uio.h /usr/include/bits/socket.h \
 /usr/include/bits/sockaddr.h /usr/include/asm/socket.h \
 /usr/include/asm-generic/socket.h /usr/include/asm/sockios.h \
 /usr/include/asm-generic/sockios.h /usr/include/bits/in.h \
 /usr/include/rpc/netdb.h /usr/include/bits/siginfo.h \
 /usr/include/bits/netdb.h /usr/include/arpa/inet.h WorldModel.h \
 Objects.h SoccerTypes.h Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 PlayerSettings.h Logger.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h \
 LoggerDraw.h Formations.h /usr/include/c++/4.4/list \
 /usr/include/c++/4.4/bits/stl_list.h /usr/include/c++/4.4/bits/list.tcc \
 /usr/include/string.h ActHandler.h Parse.h WorldModelSayMsgFactory.h \
 SayMsgFactory.h SayMsgTypes.h SayMsgIDs.h SayMsgDataTypes.h \
 /usr/include/assert.h WorldModelSayMsgTypes.h /usr/include/bits/signum.h \
 /usr/include/bits/sigaction.h /usr/include/bits/sigcontext.h \
 /usr/include/bits/sigstack.h /usr/include/sys/ucontext.h \
 /usr/include/bits/sigthread.h
ActHandler.o: ActHandler.cc ActHandler.h Connection.h \
 /usr/include/stdio.h /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/bits/types.h /usr/include/bits/typesizes.h \
 /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef /usr/include/bits/wchar.h \
 /usr/include/xlocale.h /usr/include/c++/4.4/exception \
 /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc /usr/include/netdb.h \
 /usr/include/netinet/in.h /usr/include/stdint.h \
 /usr/include/sys/socket.h /usr/include/sys/uio.h \
 /usr/include/sys/types.h /usr/include/sys/select.h \
 /usr/include/bits/select.h /usr/include/sys/sysmacros.h \
 /usr/include/bits/uio.h /usr/include/bits/socket.h \
 /usr/include/bits/sockaddr.h /usr/include/asm/socket.h \
 /usr/include/asm-generic/socket.h /usr/include/asm/sockios.h \
 /usr/include/asm-generic/sockios.h /usr/include/bits/in.h \
 /usr/include/rpc/netdb.h /usr/include/bits/siginfo.h \
 /usr/include/bits/netdb.h /usr/include/arpa/inet.h WorldModel.h \
 Objects.h SoccerTypes.h Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 PlayerSettings.h Logger.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h \
 LoggerDraw.h Formations.h /usr/include/c++/4.4/list \
 /usr/include/c++/4.4/bits/stl_list.h /usr/include/c++/4.4/bits/list.tcc \
 /usr/include/string.h /usr/include/poll.h /usr/include/sys/poll.h \
 /usr/include/bits/poll.h /usr/include/c++/4.4/cstring \
 /usr/include/bits/signum.h /usr/include/bits/sigaction.h \
 /usr/include/bits/sigcontext.h /usr/include/bits/sigstack.h \
 /usr/include/sys/ucontext.h /usr/include/bits/sigthread.h
SoccerTypes.o: SoccerTypes.cc /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/xlocale.h \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/endian.h \
 /usr/include/bits/endian.h /usr/include/bits/byteswap.h \
 /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc /usr/include/libio.h \
 /usr/include/_G_config.h /usr/include/bits/stdio_lim.h \
 /usr/include/bits/sys_errlist.h /usr/include/string.h SoccerTypes.h \
 Geometry.h /usr/include/math.h /usr/include/bits/huge_val.h \
 /usr/include/bits/huge_valf.h /usr/include/bits/huge_vall.h \
 /usr/include/bits/inf.h /usr/include/bits/nan.h \
 /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h \
 ServerSettings.h GenericValues.h Parse.h
Objects.o: Objects.cc Objects.h SoccerTypes.h \
 /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/xlocale.h \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/endian.h \
 /usr/include/bits/endian.h /usr/include/bits/byteswap.h \
 /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 /usr/include/stdlib.h /usr/include/bits/waitflags.h \
 /usr/include/bits/waitstatus.h /usr/include/sys/types.h \
 /usr/include/sys/select.h /usr/include/bits/select.h \
 /usr/include/sys/sysmacros.h /usr/include/alloca.h /usr/include/string.h
WorldModel.o: WorldModel.cc /usr/include/stdio.h /usr/include/features.h \
 /usr/include/bits/predefs.h /usr/include/sys/cdefs.h \
 /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h \
 /usr/include/gnu/stubs-64.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/bits/types.h /usr/include/bits/typesizes.h \
 /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/errno.h /usr/include/bits/errno.h \
 /usr/include/linux/errno.h /usr/include/asm/errno.h \
 /usr/include/asm-generic/errno.h /usr/include/asm-generic/errno-base.h \
 /usr/include/strings.h /usr/include/xlocale.h /usr/include/pthread.h \
 /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/string.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h /usr/include/c++/4.4/map \
 /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/cstddef /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/stl_map.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/stl_multimap.h WorldModel.h Objects.h \
 SoccerTypes.h /usr/include/c++/4.4/iostream /usr/include/c++/4.4/ostream \
 /usr/include/c++/4.4/ios /usr/include/c++/4.4/iosfwd \
 /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/bits/wchar.h /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/unistd.h /usr/include/bits/posix_opt.h \
 /usr/include/bits/environments.h /usr/include/bits/confname.h \
 /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc Geometry.h ServerSettings.h \
 GenericValues.h PlayerSettings.h Logger.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h \
 /usr/include/sys/select.h /usr/include/bits/select.h LoggerDraw.h \
 Formations.h /usr/include/c++/4.4/list \
 /usr/include/c++/4.4/bits/stl_list.h /usr/include/c++/4.4/bits/list.tcc
WorldModelHighLevel.o: WorldModelHighLevel.cc \
 /usr/include/c++/4.4/cstring \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/string.h /usr/include/xlocale.h /usr/include/c++/4.4/list \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/stl_list.h \
 /usr/include/c++/4.4/initializer_list /usr/include/c++/4.4/bits/list.tcc \
 /usr/include/stdio.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/libio.h \
 /usr/include/_G_config.h /usr/include/wchar.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 WorldModel.h Objects.h SoccerTypes.h /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/bits/wchar.h /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 PlayerSettings.h Logger.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h \
 /usr/include/sys/select.h /usr/include/bits/select.h LoggerDraw.h \
 Formations.h
WorldModelPredict.o: WorldModelPredict.cc /usr/include/stdio.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/bits/types.h /usr/include/bits/typesizes.h \
 /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 WorldModel.h Objects.h SoccerTypes.h /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef /usr/include/bits/wchar.h \
 /usr/include/xlocale.h /usr/include/c++/4.4/exception \
 /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 PlayerSettings.h Logger.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h \
 /usr/include/sys/select.h /usr/include/bits/select.h LoggerDraw.h \
 Formations.h /usr/include/c++/4.4/list \
 /usr/include/c++/4.4/bits/stl_list.h /usr/include/c++/4.4/bits/list.tcc \
 /usr/include/string.h
WorldModelUpdate.o: WorldModelUpdate.cc WorldModel.h Objects.h \
 SoccerTypes.h /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/xlocale.h \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/endian.h \
 /usr/include/bits/endian.h /usr/include/bits/byteswap.h \
 /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 PlayerSettings.h Logger.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/libio.h /usr/include/_G_config.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h \
 /usr/include/sys/select.h /usr/include/bits/select.h LoggerDraw.h \
 Formations.h /usr/include/c++/4.4/list \
 /usr/include/c++/4.4/bits/stl_list.h /usr/include/c++/4.4/bits/list.tcc \
 /usr/include/string.h Parse.h SayMsgDecoder.h \
 /usr/include/c++/4.4/vector /usr/include/c++/4.4/bits/stl_construct.h \
 /usr/include/c++/4.4/bits/stl_uninitialized.h \
 /usr/include/c++/4.4/bits/stl_vector.h \
 /usr/include/c++/4.4/bits/stl_bvector.h \
 /usr/include/c++/4.4/bits/vector.tcc SayMsgFactory.h SayMsgTypes.h \
 SayMsgIDs.h SayMsgDataTypes.h /usr/include/assert.h \
 WorldModelSayMsgFactory.h WorldModelSayMsgTypes.h \
 /usr/include/c++/4.4/cstdlib /usr/include/stdlib.h \
 /usr/include/bits/waitflags.h /usr/include/bits/waitstatus.h \
 /usr/include/sys/types.h /usr/include/sys/sysmacros.h \
 /usr/include/alloca.h /usr/include/c++/4.4/cstring \
 /usr/include/sys/times.h /usr/include/sys/resource.h \
 /usr/include/bits/resource.h
WorldModelKeepaway.o: WorldModelKeepaway.cc /usr/include/assert.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h WorldModel.h \
 Objects.h SoccerTypes.h /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/xlocale.h \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/endian.h \
 /usr/include/bits/endian.h /usr/include/bits/byteswap.h \
 /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 PlayerSettings.h Logger.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/libio.h /usr/include/_G_config.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h \
 /usr/include/sys/select.h /usr/include/bits/select.h LoggerDraw.h \
 Formations.h /usr/include/c++/4.4/list \
 /usr/include/c++/4.4/bits/stl_list.h /usr/include/c++/4.4/bits/list.tcc \
 /usr/include/string.h LSPIAgent.h SMDPAgent.h Eigen/Dense Eigen/Core \
 Eigen/src/Core/util/DisableStupidWarnings.h Eigen/src/Core/util/Macros.h \
 /usr/include/c++/4.4/complex /usr/include/c++/4.4/cmath \
 /usr/include/c++/4.4/bits/cmath.tcc /usr/include/c++/4.4/sstream \
 /usr/include/c++/4.4/bits/sstream.tcc Eigen/src/Core/util/MKL_support.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/emmintrin.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/xmmintrin.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/mmintrin.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/mm_malloc.h \
 /usr/include/stdlib.h /usr/include/bits/waitflags.h \
 /usr/include/bits/waitstatus.h /usr/include/sys/types.h \
 /usr/include/sys/sysmacros.h /usr/include/alloca.h \
 /usr/include/c++/4.4/cerrno /usr/include/errno.h \
 /usr/include/bits/errno.h /usr/include/linux/errno.h \
 /usr/include/asm/errno.h /usr/include/asm-generic/errno.h \
 /usr/include/asm-generic/errno-base.h /usr/include/c++/4.4/cstdlib \
 /usr/include/c++/4.4/cassert /usr/include/c++/4.4/functional \
 /usr/include/c++/4.4/cstring /usr/include/c++/4.4/limits \
 /usr/include/c++/4.4/climits \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include-fixed/limits.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include-fixed/syslimits.h \
 /usr/include/limits.h /usr/include/bits/posix1_lim.h \
 /usr/include/bits/local_lim.h /usr/include/linux/limits.h \
 /usr/include/bits/posix2_lim.h /usr/include/bits/xopen_lim.h \
 /usr/include/c++/4.4/algorithm /usr/include/c++/4.4/bits/stl_algo.h \
 /usr/include/c++/4.4/bits/algorithmfwd.h \
 /usr/include/c++/4.4/bits/stl_heap.h \
 /usr/include/c++/4.4/bits/stl_tempbuf.h \
 /usr/include/c++/4.4/bits/stl_construct.h \
 /usr/include/c++/4.4/bits/stl_uninitialized.h \
 Eigen/src/Core/util/Constants.h \
 Eigen/src/Core/util/ForwardDeclarations.h Eigen/src/Core/util/Meta.h \
 Eigen/src/Core/util/XprHelper.h Eigen/src/Core/util/StaticAssert.h \
 Eigen/src/Core/util/Memory.h Eigen/src/Core/NumTraits.h \
 Eigen/src/Core/MathFunctions.h Eigen/src/Core/GenericPacketMath.h \
 Eigen/src/Core/arch/SSE/PacketMath.h \
 Eigen/src/Core/arch/SSE/MathFunctions.h \
 Eigen/src/Core/arch/SSE/Complex.h Eigen/src/Core/arch/Default/Settings.h \
 Eigen/src/Core/Functors.h Eigen/src/Core/DenseCoeffsBase.h \
 Eigen/src/Core/DenseBase.h Eigen/src/Core/../plugins/BlockMethods.h \
 Eigen/src/Core/MatrixBase.h \
 Eigen/src/Core/../plugins/CommonCwiseUnaryOps.h \
 Eigen/src/Core/../plugins/CommonCwiseBinaryOps.h \
 Eigen/src/Core/../plugins/MatrixCwiseUnaryOps.h \
 Eigen/src/Core/../plugins/MatrixCwiseBinaryOps.h \
 Eigen/src/Core/EigenBase.h Eigen/src/Core/Assign.h \
 Eigen/src/Core/util/BlasUtil.h Eigen/src/Core/DenseStorage.h \
 Eigen/src/Core/NestByValue.h Eigen/src/Core/ForceAlignedAccess.h \
 Eigen/src/Core/ReturnByValue.h Eigen/src/Core/NoAlias.h \
 Eigen/src/Core/PlainObjectBase.h Eigen/src/Core/Matrix.h \
 Eigen/src/Core/Array.h Eigen/src/Core/CwiseBinaryOp.h \
 Eigen/src/Core/CwiseUnaryOp.h Eigen/src/Core/CwiseNullaryOp.h \
 Eigen/src/Core/CwiseUnaryView.h Eigen/src/Core/SelfCwiseBinaryOp.h \
 Eigen/src/Core/Dot.h Eigen/src/Core/StableNorm.h \
 Eigen/src/Core/MapBase.h Eigen/src/Core/Stride.h Eigen/src/Core/Map.h \
 Eigen/src/Core/Block.h Eigen/src/Core/VectorBlock.h \
 Eigen/src/Core/Transpose.h Eigen/src/Core/DiagonalMatrix.h \
 Eigen/src/Core/Diagonal.h Eigen/src/Core/DiagonalProduct.h \
 Eigen/src/Core/PermutationMatrix.h Eigen/src/Core/Transpositions.h \
 Eigen/src/Core/Redux.h Eigen/src/Core/Visitor.h Eigen/src/Core/Fuzzy.h \
 Eigen/src/Core/IO.h Eigen/src/Core/Swap.h \
 Eigen/src/Core/CommaInitializer.h Eigen/src/Core/Flagged.h \
 Eigen/src/Core/ProductBase.h Eigen/src/Core/GeneralProduct.h \
 Eigen/src/Core/TriangularMatrix.h Eigen/src/Core/SelfAdjointView.h \
 Eigen/src/Core/products/GeneralBlockPanelKernel.h \
 Eigen/src/Core/products/Parallelizer.h \
 Eigen/src/Core/products/CoeffBasedProduct.h \
 Eigen/src/Core/products/GeneralMatrixVector.h \
 Eigen/src/Core/products/GeneralMatrixMatrix.h \
 Eigen/src/Core/SolveTriangular.h \
 Eigen/src/Core/products/GeneralMatrixMatrixTriangular.h \
 Eigen/src/Core/products/SelfadjointMatrixVector.h \
 Eigen/src/Core/products/SelfadjointMatrixMatrix.h \
 Eigen/src/Core/products/SelfadjointProduct.h \
 Eigen/src/Core/products/SelfadjointRank2Update.h \
 Eigen/src/Core/products/TriangularMatrixVector.h \
 Eigen/src/Core/products/TriangularMatrixMatrix.h \
 Eigen/src/Core/products/TriangularSolverMatrix.h \
 Eigen/src/Core/products/TriangularSolverVector.h \
 Eigen/src/Core/BandMatrix.h Eigen/src/Core/BooleanRedux.h \
 Eigen/src/Core/Select.h Eigen/src/Core/VectorwiseOp.h \
 Eigen/src/Core/Random.h Eigen/src/Core/Replicate.h \
 Eigen/src/Core/Reverse.h Eigen/src/Core/ArrayBase.h \
 Eigen/src/Core/../plugins/ArrayCwiseUnaryOps.h \
 Eigen/src/Core/../plugins/ArrayCwiseBinaryOps.h \
 Eigen/src/Core/ArrayWrapper.h Eigen/src/Core/GlobalFunctions.h \
 Eigen/src/Core/util/ReenableStupidWarnings.h Eigen/LU \
 Eigen/src/misc/Solve.h Eigen/src/misc/Kernel.h Eigen/src/misc/Image.h \
 Eigen/src/LU/FullPivLU.h Eigen/src/LU/PartialPivLU.h \
 Eigen/src/LU/Determinant.h Eigen/src/LU/Inverse.h \
 Eigen/src/LU/arch/Inverse_SSE.h Eigen/Cholesky Eigen/src/Cholesky/LLT.h \
 Eigen/src/Cholesky/LDLT.h Eigen/QR Eigen/Jacobi \
 Eigen/src/Jacobi/Jacobi.h Eigen/Householder \
 Eigen/src/Householder/Householder.h \
 Eigen/src/Householder/HouseholderSequence.h \
 Eigen/src/Householder/BlockHouseholder.h Eigen/src/QR/HouseholderQR.h \
 Eigen/src/QR/FullPivHouseholderQR.h Eigen/src/QR/ColPivHouseholderQR.h \
 Eigen/SVD Eigen/src/SVD/JacobiSVD.h \
 Eigen/src/SVD/UpperBidiagonalization.h Eigen/Geometry \
 Eigen/src/Geometry/OrthoMethods.h Eigen/src/Geometry/EulerAngles.h \
 Eigen/src/Geometry/Homogeneous.h Eigen/src/Geometry/RotationBase.h \
 Eigen/src/Geometry/Rotation2D.h Eigen/src/Geometry/Quaternion.h \
 Eigen/src/Geometry/AngleAxis.h Eigen/src/Geometry/Transform.h \
 Eigen/src/Geometry/Translation.h Eigen/src/Geometry/Scaling.h \
 Eigen/src/Geometry/Hyperplane.h Eigen/src/Geometry/ParametrizedLine.h \
 Eigen/src/Geometry/AlignedBox.h Eigen/src/Geometry/Umeyama.h \
 Eigen/src/Geometry/arch/Geometry_SSE.h Eigen/Eigenvalues \
 Eigen/src/Eigenvalues/Tridiagonalization.h \
 Eigen/src/Eigenvalues/RealSchur.h \
 Eigen/src/Eigenvalues/./HessenbergDecomposition.h \
 Eigen/src/Eigenvalues/EigenSolver.h Eigen/src/Eigenvalues/./RealSchur.h \
 Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h \
 Eigen/src/Eigenvalues/./Tridiagonalization.h \
 Eigen/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h \
 Eigen/src/Eigenvalues/HessenbergDecomposition.h \
 Eigen/src/Eigenvalues/ComplexSchur.h \
 Eigen/src/Eigenvalues/ComplexEigenSolver.h \
 Eigen/src/Eigenvalues/./ComplexSchur.h \
 Eigen/src/Eigenvalues/MatrixBaseEigenvalues.h \
 /usr/include/c++/4.4/vector /usr/include/c++/4.4/bits/stl_vector.h \
 /usr/include/c++/4.4/bits/stl_bvector.h \
 /usr/include/c++/4.4/bits/vector.tcc
WorldModelSayMsgTypes.o: WorldModelSayMsgTypes.cc WorldModelSayMsgTypes.h \
 WorldModel.h Objects.h SoccerTypes.h /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/xlocale.h \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/endian.h \
 /usr/include/bits/endian.h /usr/include/bits/byteswap.h \
 /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 PlayerSettings.h Logger.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/libio.h /usr/include/_G_config.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h \
 /usr/include/sys/select.h /usr/include/bits/select.h LoggerDraw.h \
 Formations.h /usr/include/c++/4.4/list \
 /usr/include/c++/4.4/bits/stl_list.h /usr/include/c++/4.4/bits/list.tcc \
 /usr/include/string.h SayMsgTypes.h SayMsgIDs.h SayMsgDataTypes.h \
 /usr/include/assert.h
ServerSettings.o: ServerSettings.cc ServerSettings.h Geometry.h \
 /usr/include/math.h /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/postypes.h \
 /usr/include/c++/4.4/cwchar /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/xlocale.h \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/iosfwd \
 /usr/include/c++/4.4/cctype /usr/include/ctype.h \
 /usr/include/bits/types.h /usr/include/bits/typesizes.h \
 /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/bits/locale_classes.h \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc GenericValues.h \
 /usr/include/libio.h /usr/include/_G_config.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/string.h
PlayerSettings.o: PlayerSettings.cc PlayerSettings.h GenericValues.h \
 /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/xlocale.h \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/endian.h \
 /usr/include/bits/endian.h /usr/include/bits/byteswap.h \
 /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc
GenericValues.o: GenericValues.cc GenericValues.h \
 /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/xlocale.h \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/endian.h \
 /usr/include/bits/endian.h /usr/include/bits/byteswap.h \
 /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc Parse.h /usr/include/libio.h \
 /usr/include/_G_config.h /usr/include/bits/stdio_lim.h \
 /usr/include/bits/sys_errlist.h /usr/include/stdlib.h \
 /usr/include/bits/waitflags.h /usr/include/bits/waitstatus.h \
 /usr/include/sys/types.h /usr/include/sys/select.h \
 /usr/include/bits/select.h /usr/include/sys/sysmacros.h \
 /usr/include/alloca.h /usr/include/string.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc
Formations.o: Formations.cc Formations.h SoccerTypes.h \
 /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/xlocale.h \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/endian.h \
 /usr/include/bits/endian.h /usr/include/bits/byteswap.h \
 /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h Parse.h \
 /usr/include/c++/4.4/fstream /usr/include/c++/4.4/bits/codecvt.h \
 /usr/include/c++/4.4/cstdio /usr/include/libio.h \
 /usr/include/_G_config.h /usr/include/bits/stdio_lim.h \
 /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc
Geometry.o: Geometry.cc Geometry.h /usr/include/math.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/postypes.h \
 /usr/include/c++/4.4/cwchar /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/xlocale.h \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/iosfwd \
 /usr/include/c++/4.4/cctype /usr/include/ctype.h \
 /usr/include/bits/types.h /usr/include/bits/typesizes.h \
 /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/bits/locale_classes.h \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc /usr/include/libio.h \
 /usr/include/_G_config.h /usr/include/bits/stdio_lim.h \
 /usr/include/bits/sys_errlist.h
Parse.o: Parse.cc Parse.h /usr/include/ctype.h /usr/include/features.h \
 /usr/include/bits/predefs.h /usr/include/sys/cdefs.h \
 /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h \
 /usr/include/gnu/stubs-64.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/endian.h \
 /usr/include/bits/endian.h /usr/include/bits/byteswap.h \
 /usr/include/xlocale.h /usr/include/math.h /usr/include/bits/huge_val.h \
 /usr/include/bits/huge_valf.h /usr/include/bits/huge_vall.h \
 /usr/include/bits/inf.h /usr/include/bits/nan.h \
 /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h \
 /usr/include/string.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h
Logger.o: Logger.cc Logger.h /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/xlocale.h \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/endian.h \
 /usr/include/bits/endian.h /usr/include/bits/byteswap.h \
 /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/libio.h /usr/include/_G_config.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h \
 /usr/include/sys/select.h /usr/include/bits/select.h \
 /usr/include/c++/4.4/cstring /usr/include/string.h
LoggerDraw.o: LoggerDraw.cc LoggerDraw.h SoccerTypes.h \
 /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/xlocale.h \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/endian.h \
 /usr/include/bits/endian.h /usr/include/bits/byteswap.h \
 /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 /usr/include/c++/4.4/fstream /usr/include/c++/4.4/bits/codecvt.h \
 /usr/include/c++/4.4/cstdio /usr/include/libio.h \
 /usr/include/_G_config.h /usr/include/bits/stdio_lim.h \
 /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/cstring \
 /usr/include/string.h
BasicPlayer.o: BasicPlayer.cc BasicPlayer.h ActHandler.h Connection.h \
 /usr/include/stdio.h /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/bits/types.h /usr/include/bits/typesizes.h \
 /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef /usr/include/bits/wchar.h \
 /usr/include/xlocale.h /usr/include/c++/4.4/exception \
 /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc /usr/include/netdb.h \
 /usr/include/netinet/in.h /usr/include/stdint.h \
 /usr/include/sys/socket.h /usr/include/sys/uio.h \
 /usr/include/sys/types.h /usr/include/sys/select.h \
 /usr/include/bits/select.h /usr/include/sys/sysmacros.h \
 /usr/include/bits/uio.h /usr/include/bits/socket.h \
 /usr/include/bits/sockaddr.h /usr/include/asm/socket.h \
 /usr/include/asm-generic/socket.h /usr/include/asm/sockios.h \
 /usr/include/asm-generic/sockios.h /usr/include/bits/in.h \
 /usr/include/rpc/netdb.h /usr/include/bits/siginfo.h \
 /usr/include/bits/netdb.h /usr/include/arpa/inet.h WorldModel.h \
 Objects.h SoccerTypes.h Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 PlayerSettings.h Logger.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h \
 LoggerDraw.h Formations.h /usr/include/c++/4.4/list \
 /usr/include/c++/4.4/bits/stl_list.h /usr/include/c++/4.4/bits/list.tcc \
 /usr/include/string.h Parse.h
KeepawayPlayer.o: KeepawayPlayer.cc KeepawayPlayer.h BasicPlayer.h \
 ActHandler.h Connection.h /usr/include/stdio.h /usr/include/features.h \
 /usr/include/bits/predefs.h /usr/include/sys/cdefs.h \
 /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h \
 /usr/include/gnu/stubs-64.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/bits/types.h /usr/include/bits/typesizes.h \
 /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef /usr/include/bits/wchar.h \
 /usr/include/xlocale.h /usr/include/c++/4.4/exception \
 /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc /usr/include/netdb.h \
 /usr/include/netinet/in.h /usr/include/stdint.h \
 /usr/include/sys/socket.h /usr/include/sys/uio.h \
 /usr/include/sys/types.h /usr/include/sys/select.h \
 /usr/include/bits/select.h /usr/include/sys/sysmacros.h \
 /usr/include/bits/uio.h /usr/include/bits/socket.h \
 /usr/include/bits/sockaddr.h /usr/include/asm/socket.h \
 /usr/include/asm-generic/socket.h /usr/include/asm/sockios.h \
 /usr/include/asm-generic/sockios.h /usr/include/bits/in.h \
 /usr/include/rpc/netdb.h /usr/include/bits/siginfo.h \
 /usr/include/bits/netdb.h /usr/include/arpa/inet.h WorldModel.h \
 Objects.h SoccerTypes.h Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 PlayerSettings.h Logger.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h \
 LoggerDraw.h Formations.h /usr/include/c++/4.4/list \
 /usr/include/c++/4.4/bits/stl_list.h /usr/include/c++/4.4/bits/list.tcc \
 /usr/include/string.h SMDPAgent.h LSPIAgent.h Eigen/Dense Eigen/Core \
 Eigen/src/Core/util/DisableStupidWarnings.h Eigen/src/Core/util/Macros.h \
 /usr/include/c++/4.4/complex /usr/include/c++/4.4/cmath \
 /usr/include/c++/4.4/bits/cmath.tcc /usr/include/c++/4.4/sstream \
 /usr/include/c++/4.4/bits/sstream.tcc Eigen/src/Core/util/MKL_support.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/emmintrin.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/xmmintrin.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/mmintrin.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/mm_malloc.h \
 /usr/include/stdlib.h /usr/include/bits/waitflags.h \
 /usr/include/bits/waitstatus.h /usr/include/alloca.h \
 /usr/include/c++/4.4/cerrno /usr/include/errno.h \
 /usr/include/bits/errno.h /usr/include/linux/errno.h \
 /usr/include/asm/errno.h /usr/include/asm-generic/errno.h \
 /usr/include/asm-generic/errno-base.h /usr/include/c++/4.4/cstdlib \
 /usr/include/c++/4.4/cassert /usr/include/assert.h \
 /usr/include/c++/4.4/functional /usr/include/c++/4.4/cstring \
 /usr/include/c++/4.4/limits /usr/include/c++/4.4/climits \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include-fixed/limits.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include-fixed/syslimits.h \
 /usr/include/limits.h /usr/include/bits/posix1_lim.h \
 /usr/include/bits/local_lim.h /usr/include/linux/limits.h \
 /usr/include/bits/posix2_lim.h /usr/include/bits/xopen_lim.h \
 /usr/include/c++/4.4/algorithm /usr/include/c++/4.4/bits/stl_algo.h \
 /usr/include/c++/4.4/bits/algorithmfwd.h \
 /usr/include/c++/4.4/bits/stl_heap.h \
 /usr/include/c++/4.4/bits/stl_tempbuf.h \
 /usr/include/c++/4.4/bits/stl_construct.h \
 /usr/include/c++/4.4/bits/stl_uninitialized.h \
 Eigen/src/Core/util/Constants.h \
 Eigen/src/Core/util/ForwardDeclarations.h Eigen/src/Core/util/Meta.h \
 Eigen/src/Core/util/XprHelper.h Eigen/src/Core/util/StaticAssert.h \
 Eigen/src/Core/util/Memory.h Eigen/src/Core/NumTraits.h \
 Eigen/src/Core/MathFunctions.h Eigen/src/Core/GenericPacketMath.h \
 Eigen/src/Core/arch/SSE/PacketMath.h \
 Eigen/src/Core/arch/SSE/MathFunctions.h \
 Eigen/src/Core/arch/SSE/Complex.h Eigen/src/Core/arch/Default/Settings.h \
 Eigen/src/Core/Functors.h Eigen/src/Core/DenseCoeffsBase.h \
 Eigen/src/Core/DenseBase.h Eigen/src/Core/../plugins/BlockMethods.h \
 Eigen/src/Core/MatrixBase.h \
 Eigen/src/Core/../plugins/CommonCwiseUnaryOps.h \
 Eigen/src/Core/../plugins/CommonCwiseBinaryOps.h \
 Eigen/src/Core/../plugins/MatrixCwiseUnaryOps.h \
 Eigen/src/Core/../plugins/MatrixCwiseBinaryOps.h \
 Eigen/src/Core/EigenBase.h Eigen/src/Core/Assign.h \
 Eigen/src/Core/util/BlasUtil.h Eigen/src/Core/DenseStorage.h \
 Eigen/src/Core/NestByValue.h Eigen/src/Core/ForceAlignedAccess.h \
 Eigen/src/Core/ReturnByValue.h Eigen/src/Core/NoAlias.h \
 Eigen/src/Core/PlainObjectBase.h Eigen/src/Core/Matrix.h \
 Eigen/src/Core/Array.h Eigen/src/Core/CwiseBinaryOp.h \
 Eigen/src/Core/CwiseUnaryOp.h Eigen/src/Core/CwiseNullaryOp.h \
 Eigen/src/Core/CwiseUnaryView.h Eigen/src/Core/SelfCwiseBinaryOp.h \
 Eigen/src/Core/Dot.h Eigen/src/Core/StableNorm.h \
 Eigen/src/Core/MapBase.h Eigen/src/Core/Stride.h Eigen/src/Core/Map.h \
 Eigen/src/Core/Block.h Eigen/src/Core/VectorBlock.h \
 Eigen/src/Core/Transpose.h Eigen/src/Core/DiagonalMatrix.h \
 Eigen/src/Core/Diagonal.h Eigen/src/Core/DiagonalProduct.h \
 Eigen/src/Core/PermutationMatrix.h Eigen/src/Core/Transpositions.h \
 Eigen/src/Core/Redux.h Eigen/src/Core/Visitor.h Eigen/src/Core/Fuzzy.h \
 Eigen/src/Core/IO.h Eigen/src/Core/Swap.h \
 Eigen/src/Core/CommaInitializer.h Eigen/src/Core/Flagged.h \
 Eigen/src/Core/ProductBase.h Eigen/src/Core/GeneralProduct.h \
 Eigen/src/Core/TriangularMatrix.h Eigen/src/Core/SelfAdjointView.h \
 Eigen/src/Core/products/GeneralBlockPanelKernel.h \
 Eigen/src/Core/products/Parallelizer.h \
 Eigen/src/Core/products/CoeffBasedProduct.h \
 Eigen/src/Core/products/GeneralMatrixVector.h \
 Eigen/src/Core/products/GeneralMatrixMatrix.h \
 Eigen/src/Core/SolveTriangular.h \
 Eigen/src/Core/products/GeneralMatrixMatrixTriangular.h \
 Eigen/src/Core/products/SelfadjointMatrixVector.h \
 Eigen/src/Core/products/SelfadjointMatrixMatrix.h \
 Eigen/src/Core/products/SelfadjointProduct.h \
 Eigen/src/Core/products/SelfadjointRank2Update.h \
 Eigen/src/Core/products/TriangularMatrixVector.h \
 Eigen/src/Core/products/TriangularMatrixMatrix.h \
 Eigen/src/Core/products/TriangularSolverMatrix.h \
 Eigen/src/Core/products/TriangularSolverVector.h \
 Eigen/src/Core/BandMatrix.h Eigen/src/Core/BooleanRedux.h \
 Eigen/src/Core/Select.h Eigen/src/Core/VectorwiseOp.h \
 Eigen/src/Core/Random.h Eigen/src/Core/Replicate.h \
 Eigen/src/Core/Reverse.h Eigen/src/Core/ArrayBase.h \
 Eigen/src/Core/../plugins/ArrayCwiseUnaryOps.h \
 Eigen/src/Core/../plugins/ArrayCwiseBinaryOps.h \
 Eigen/src/Core/ArrayWrapper.h Eigen/src/Core/GlobalFunctions.h \
 Eigen/src/Core/util/ReenableStupidWarnings.h Eigen/LU \
 Eigen/src/misc/Solve.h Eigen/src/misc/Kernel.h Eigen/src/misc/Image.h \
 Eigen/src/LU/FullPivLU.h Eigen/src/LU/PartialPivLU.h \
 Eigen/src/LU/Determinant.h Eigen/src/LU/Inverse.h \
 Eigen/src/LU/arch/Inverse_SSE.h Eigen/Cholesky Eigen/src/Cholesky/LLT.h \
 Eigen/src/Cholesky/LDLT.h Eigen/QR Eigen/Jacobi \
 Eigen/src/Jacobi/Jacobi.h Eigen/Householder \
 Eigen/src/Householder/Householder.h \
 Eigen/src/Householder/HouseholderSequence.h \
 Eigen/src/Householder/BlockHouseholder.h Eigen/src/QR/HouseholderQR.h \
 Eigen/src/QR/FullPivHouseholderQR.h Eigen/src/QR/ColPivHouseholderQR.h \
 Eigen/SVD Eigen/src/SVD/JacobiSVD.h \
 Eigen/src/SVD/UpperBidiagonalization.h Eigen/Geometry \
 Eigen/src/Geometry/OrthoMethods.h Eigen/src/Geometry/EulerAngles.h \
 Eigen/src/Geometry/Homogeneous.h Eigen/src/Geometry/RotationBase.h \
 Eigen/src/Geometry/Rotation2D.h Eigen/src/Geometry/Quaternion.h \
 Eigen/src/Geometry/AngleAxis.h Eigen/src/Geometry/Transform.h \
 Eigen/src/Geometry/Translation.h Eigen/src/Geometry/Scaling.h \
 Eigen/src/Geometry/Hyperplane.h Eigen/src/Geometry/ParametrizedLine.h \
 Eigen/src/Geometry/AlignedBox.h Eigen/src/Geometry/Umeyama.h \
 Eigen/src/Geometry/arch/Geometry_SSE.h Eigen/Eigenvalues \
 Eigen/src/Eigenvalues/Tridiagonalization.h \
 Eigen/src/Eigenvalues/RealSchur.h \
 Eigen/src/Eigenvalues/./HessenbergDecomposition.h \
 Eigen/src/Eigenvalues/EigenSolver.h Eigen/src/Eigenvalues/./RealSchur.h \
 Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h \
 Eigen/src/Eigenvalues/./Tridiagonalization.h \
 Eigen/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h \
 Eigen/src/Eigenvalues/HessenbergDecomposition.h \
 Eigen/src/Eigenvalues/ComplexSchur.h \
 Eigen/src/Eigenvalues/ComplexEigenSolver.h \
 Eigen/src/Eigenvalues/./ComplexSchur.h \
 Eigen/src/Eigenvalues/MatrixBaseEigenvalues.h \
 /usr/include/c++/4.4/vector /usr/include/c++/4.4/bits/stl_vector.h \
 /usr/include/c++/4.4/bits/stl_bvector.h \
 /usr/include/c++/4.4/bits/vector.tcc Parse.h SayMsgEncoder.h \
 SayMsgTypes.h SayMsgIDs.h SayMsgDataTypes.h SayMsgFactory.h \
 WorldModelSayMsgFactory.h WorldModelSayMsgTypes.h SayMsgDecoder.h
HandCodedAgent.o: HandCodedAgent.cc /usr/include/string.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/xlocale.h /usr/include/stdlib.h \
 /usr/include/bits/waitflags.h /usr/include/bits/waitstatus.h \
 /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/sys/types.h \
 /usr/include/bits/types.h /usr/include/bits/typesizes.h \
 /usr/include/time.h /usr/include/sys/select.h /usr/include/bits/select.h \
 /usr/include/bits/sigset.h /usr/include/bits/time.h \
 /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h \
 /usr/include/alloca.h HandCodedAgent.h SMDPAgent.h WorldModel.h \
 Objects.h SoccerTypes.h /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/c++/4.4/exception \
 /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/bits/sched.h \
 /usr/include/signal.h /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 PlayerSettings.h Logger.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/libio.h /usr/include/_G_config.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h \
 LoggerDraw.h Formations.h /usr/include/c++/4.4/list \
 /usr/include/c++/4.4/bits/stl_list.h /usr/include/c++/4.4/bits/list.tcc \
 /usr/include/c++/4.4/cstdlib
tiles2.o: tiles2.cc /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/xlocale.h \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/endian.h \
 /usr/include/bits/endian.h /usr/include/bits/byteswap.h \
 /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc tiles2.h /usr/include/libio.h \
 /usr/include/_G_config.h /usr/include/bits/stdio_lim.h \
 /usr/include/bits/sys_errlist.h /usr/include/stdlib.h \
 /usr/include/bits/waitflags.h /usr/include/bits/waitstatus.h \
 /usr/include/sys/types.h /usr/include/sys/select.h \
 /usr/include/bits/select.h /usr/include/sys/sysmacros.h \
 /usr/include/alloca.h /usr/include/fcntl.h /usr/include/bits/fcntl.h \
 /usr/include/bits/uio.h /usr/include/sys/stat.h /usr/include/bits/stat.h \
 /usr/include/math.h /usr/include/bits/huge_val.h \
 /usr/include/bits/huge_valf.h /usr/include/bits/huge_vall.h \
 /usr/include/bits/inf.h /usr/include/bits/nan.h \
 /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
LinearSarsaAgent.o: LinearSarsaAgent.cc /usr/include/stdlib.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/bits/waitflags.h /usr/include/bits/waitstatus.h \
 /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/xlocale.h \
 /usr/include/sys/types.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/time.h \
 /usr/include/sys/select.h /usr/include/bits/select.h \
 /usr/include/bits/sigset.h /usr/include/bits/time.h \
 /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h \
 /usr/include/alloca.h /usr/include/string.h /usr/include/sys/stat.h \
 /usr/include/bits/stat.h /usr/include/fcntl.h /usr/include/bits/fcntl.h \
 /usr/include/bits/uio.h LinearSarsaAgent.h SMDPAgent.h tiles2.h \
 /usr/include/stdio.h /usr/include/libio.h /usr/include/_G_config.h \
 /usr/include/wchar.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/unistd.h /usr/include/bits/posix_opt.h \
 /usr/include/bits/environments.h /usr/include/bits/confname.h \
 /usr/include/getopt.h LoggerDraw.h SoccerTypes.h \
 /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef /usr/include/bits/wchar.h \
 /usr/include/c++/4.4/exception /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/bits/sched.h \
 /usr/include/signal.h /usr/include/bits/setjmp.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 /usr/include/c++/4.4/fstream /usr/include/c++/4.4/bits/codecvt.h \
 /usr/include/c++/4.4/cstdio \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc
main.o: main.cc SenseHandler.h Connection.h /usr/include/stdio.h \
 /usr/include/features.h /usr/include/bits/predefs.h \
 /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h \
 /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/bits/types.h /usr/include/bits/typesizes.h \
 /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/iostream \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/ostream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef /usr/include/bits/wchar.h \
 /usr/include/xlocale.h /usr/include/c++/4.4/exception \
 /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/time.h \
 /usr/include/bits/sched.h /usr/include/bits/time.h /usr/include/signal.h \
 /usr/include/bits/sigset.h /usr/include/bits/pthreadtypes.h \
 /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc \
 /usr/include/c++/4.4/bits/ostream.tcc /usr/include/c++/4.4/istream \
 /usr/include/c++/4.4/bits/istream.tcc /usr/include/netdb.h \
 /usr/include/netinet/in.h /usr/include/stdint.h \
 /usr/include/sys/socket.h /usr/include/sys/uio.h \
 /usr/include/sys/types.h /usr/include/sys/select.h \
 /usr/include/bits/select.h /usr/include/sys/sysmacros.h \
 /usr/include/bits/uio.h /usr/include/bits/socket.h \
 /usr/include/bits/sockaddr.h /usr/include/asm/socket.h \
 /usr/include/asm-generic/socket.h /usr/include/asm/sockios.h \
 /usr/include/asm-generic/sockios.h /usr/include/bits/in.h \
 /usr/include/rpc/netdb.h /usr/include/bits/siginfo.h \
 /usr/include/bits/netdb.h /usr/include/arpa/inet.h WorldModel.h \
 Objects.h SoccerTypes.h Geometry.h /usr/include/math.h \
 /usr/include/bits/huge_val.h /usr/include/bits/huge_valf.h \
 /usr/include/bits/huge_vall.h /usr/include/bits/inf.h \
 /usr/include/bits/nan.h /usr/include/bits/mathdef.h \
 /usr/include/bits/mathcalls.h ServerSettings.h GenericValues.h \
 PlayerSettings.h Logger.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/iomanip \
 /usr/include/c++/4.4/set /usr/include/c++/4.4/bits/stl_tree.h \
 /usr/include/c++/4.4/bits/stl_set.h \
 /usr/include/c++/4.4/bits/stl_multiset.h /usr/include/sys/time.h \
 LoggerDraw.h Formations.h /usr/include/c++/4.4/list \
 /usr/include/c++/4.4/bits/stl_list.h /usr/include/c++/4.4/bits/list.tcc \
 /usr/include/string.h ActHandler.h KeepawayPlayer.h BasicPlayer.h \
 SMDPAgent.h LSPIAgent.h Eigen/Dense Eigen/Core \
 Eigen/src/Core/util/DisableStupidWarnings.h Eigen/src/Core/util/Macros.h \
 /usr/include/c++/4.4/complex /usr/include/c++/4.4/cmath \
 /usr/include/c++/4.4/bits/cmath.tcc /usr/include/c++/4.4/sstream \
 /usr/include/c++/4.4/bits/sstream.tcc Eigen/src/Core/util/MKL_support.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/emmintrin.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/xmmintrin.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/mmintrin.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/mm_malloc.h \
 /usr/include/stdlib.h /usr/include/bits/waitflags.h \
 /usr/include/bits/waitstatus.h /usr/include/alloca.h \
 /usr/include/c++/4.4/cerrno /usr/include/errno.h \
 /usr/include/bits/errno.h /usr/include/linux/errno.h \
 /usr/include/asm/errno.h /usr/include/asm-generic/errno.h \
 /usr/include/asm-generic/errno-base.h /usr/include/c++/4.4/cstdlib \
 /usr/include/c++/4.4/cassert /usr/include/assert.h \
 /usr/include/c++/4.4/functional /usr/include/c++/4.4/cstring \
 /usr/include/c++/4.4/limits /usr/include/c++/4.4/climits \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include-fixed/limits.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include-fixed/syslimits.h \
 /usr/include/limits.h /usr/include/bits/posix1_lim.h \
 /usr/include/bits/local_lim.h /usr/include/linux/limits.h \
 /usr/include/bits/posix2_lim.h /usr/include/bits/xopen_lim.h \
 /usr/include/c++/4.4/algorithm /usr/include/c++/4.4/bits/stl_algo.h \
 /usr/include/c++/4.4/bits/algorithmfwd.h \
 /usr/include/c++/4.4/bits/stl_heap.h \
 /usr/include/c++/4.4/bits/stl_tempbuf.h \
 /usr/include/c++/4.4/bits/stl_construct.h \
 /usr/include/c++/4.4/bits/stl_uninitialized.h \
 Eigen/src/Core/util/Constants.h \
 Eigen/src/Core/util/ForwardDeclarations.h Eigen/src/Core/util/Meta.h \
 Eigen/src/Core/util/XprHelper.h Eigen/src/Core/util/StaticAssert.h \
 Eigen/src/Core/util/Memory.h Eigen/src/Core/NumTraits.h \
 Eigen/src/Core/MathFunctions.h Eigen/src/Core/GenericPacketMath.h \
 Eigen/src/Core/arch/SSE/PacketMath.h \
 Eigen/src/Core/arch/SSE/MathFunctions.h \
 Eigen/src/Core/arch/SSE/Complex.h Eigen/src/Core/arch/Default/Settings.h \
 Eigen/src/Core/Functors.h Eigen/src/Core/DenseCoeffsBase.h \
 Eigen/src/Core/DenseBase.h Eigen/src/Core/../plugins/BlockMethods.h \
 Eigen/src/Core/MatrixBase.h \
 Eigen/src/Core/../plugins/CommonCwiseUnaryOps.h \
 Eigen/src/Core/../plugins/CommonCwiseBinaryOps.h \
 Eigen/src/Core/../plugins/MatrixCwiseUnaryOps.h \
 Eigen/src/Core/../plugins/MatrixCwiseBinaryOps.h \
 Eigen/src/Core/EigenBase.h Eigen/src/Core/Assign.h \
 Eigen/src/Core/util/BlasUtil.h Eigen/src/Core/DenseStorage.h \
 Eigen/src/Core/NestByValue.h Eigen/src/Core/ForceAlignedAccess.h \
 Eigen/src/Core/ReturnByValue.h Eigen/src/Core/NoAlias.h \
 Eigen/src/Core/PlainObjectBase.h Eigen/src/Core/Matrix.h \
 Eigen/src/Core/Array.h Eigen/src/Core/CwiseBinaryOp.h \
 Eigen/src/Core/CwiseUnaryOp.h Eigen/src/Core/CwiseNullaryOp.h \
 Eigen/src/Core/CwiseUnaryView.h Eigen/src/Core/SelfCwiseBinaryOp.h \
 Eigen/src/Core/Dot.h Eigen/src/Core/StableNorm.h \
 Eigen/src/Core/MapBase.h Eigen/src/Core/Stride.h Eigen/src/Core/Map.h \
 Eigen/src/Core/Block.h Eigen/src/Core/VectorBlock.h \
 Eigen/src/Core/Transpose.h Eigen/src/Core/DiagonalMatrix.h \
 Eigen/src/Core/Diagonal.h Eigen/src/Core/DiagonalProduct.h \
 Eigen/src/Core/PermutationMatrix.h Eigen/src/Core/Transpositions.h \
 Eigen/src/Core/Redux.h Eigen/src/Core/Visitor.h Eigen/src/Core/Fuzzy.h \
 Eigen/src/Core/IO.h Eigen/src/Core/Swap.h \
 Eigen/src/Core/CommaInitializer.h Eigen/src/Core/Flagged.h \
 Eigen/src/Core/ProductBase.h Eigen/src/Core/GeneralProduct.h \
 Eigen/src/Core/TriangularMatrix.h Eigen/src/Core/SelfAdjointView.h \
 Eigen/src/Core/products/GeneralBlockPanelKernel.h \
 Eigen/src/Core/products/Parallelizer.h \
 Eigen/src/Core/products/CoeffBasedProduct.h \
 Eigen/src/Core/products/GeneralMatrixVector.h \
 Eigen/src/Core/products/GeneralMatrixMatrix.h \
 Eigen/src/Core/SolveTriangular.h \
 Eigen/src/Core/products/GeneralMatrixMatrixTriangular.h \
 Eigen/src/Core/products/SelfadjointMatrixVector.h \
 Eigen/src/Core/products/SelfadjointMatrixMatrix.h \
 Eigen/src/Core/products/SelfadjointProduct.h \
 Eigen/src/Core/products/SelfadjointRank2Update.h \
 Eigen/src/Core/products/TriangularMatrixVector.h \
 Eigen/src/Core/products/TriangularMatrixMatrix.h \
 Eigen/src/Core/products/TriangularSolverMatrix.h \
 Eigen/src/Core/products/TriangularSolverVector.h \
 Eigen/src/Core/BandMatrix.h Eigen/src/Core/BooleanRedux.h \
 Eigen/src/Core/Select.h Eigen/src/Core/VectorwiseOp.h \
 Eigen/src/Core/Random.h Eigen/src/Core/Replicate.h \
 Eigen/src/Core/Reverse.h Eigen/src/Core/ArrayBase.h \
 Eigen/src/Core/../plugins/ArrayCwiseUnaryOps.h \
 Eigen/src/Core/../plugins/ArrayCwiseBinaryOps.h \
 Eigen/src/Core/ArrayWrapper.h Eigen/src/Core/GlobalFunctions.h \
 Eigen/src/Core/util/ReenableStupidWarnings.h Eigen/LU \
 Eigen/src/misc/Solve.h Eigen/src/misc/Kernel.h Eigen/src/misc/Image.h \
 Eigen/src/LU/FullPivLU.h Eigen/src/LU/PartialPivLU.h \
 Eigen/src/LU/Determinant.h Eigen/src/LU/Inverse.h \
 Eigen/src/LU/arch/Inverse_SSE.h Eigen/Cholesky Eigen/src/Cholesky/LLT.h \
 Eigen/src/Cholesky/LDLT.h Eigen/QR Eigen/Jacobi \
 Eigen/src/Jacobi/Jacobi.h Eigen/Householder \
 Eigen/src/Householder/Householder.h \
 Eigen/src/Householder/HouseholderSequence.h \
 Eigen/src/Householder/BlockHouseholder.h Eigen/src/QR/HouseholderQR.h \
 Eigen/src/QR/FullPivHouseholderQR.h Eigen/src/QR/ColPivHouseholderQR.h \
 Eigen/SVD Eigen/src/SVD/JacobiSVD.h \
 Eigen/src/SVD/UpperBidiagonalization.h Eigen/Geometry \
 Eigen/src/Geometry/OrthoMethods.h Eigen/src/Geometry/EulerAngles.h \
 Eigen/src/Geometry/Homogeneous.h Eigen/src/Geometry/RotationBase.h \
 Eigen/src/Geometry/Rotation2D.h Eigen/src/Geometry/Quaternion.h \
 Eigen/src/Geometry/AngleAxis.h Eigen/src/Geometry/Transform.h \
 Eigen/src/Geometry/Translation.h Eigen/src/Geometry/Scaling.h \
 Eigen/src/Geometry/Hyperplane.h Eigen/src/Geometry/ParametrizedLine.h \
 Eigen/src/Geometry/AlignedBox.h Eigen/src/Geometry/Umeyama.h \
 Eigen/src/Geometry/arch/Geometry_SSE.h Eigen/Eigenvalues \
 Eigen/src/Eigenvalues/Tridiagonalization.h \
 Eigen/src/Eigenvalues/RealSchur.h \
 Eigen/src/Eigenvalues/./HessenbergDecomposition.h \
 Eigen/src/Eigenvalues/EigenSolver.h Eigen/src/Eigenvalues/./RealSchur.h \
 Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h \
 Eigen/src/Eigenvalues/./Tridiagonalization.h \
 Eigen/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h \
 Eigen/src/Eigenvalues/HessenbergDecomposition.h \
 Eigen/src/Eigenvalues/ComplexSchur.h \
 Eigen/src/Eigenvalues/ComplexEigenSolver.h \
 Eigen/src/Eigenvalues/./ComplexSchur.h \
 Eigen/src/Eigenvalues/MatrixBaseEigenvalues.h \
 /usr/include/c++/4.4/vector /usr/include/c++/4.4/bits/stl_vector.h \
 /usr/include/c++/4.4/bits/stl_bvector.h \
 /usr/include/c++/4.4/bits/vector.tcc HandCodedAgent.h LinearSarsaAgent.h \
 tiles2.h /usr/include/fcntl.h /usr/include/bits/fcntl.h \
 /usr/include/sys/stat.h /usr/include/bits/stat.h Parse.h \
 /usr/include/dlfcn.h /usr/include/bits/dlfcn.h
LSPIAgent.o: LSPIAgent.cc /usr/include/stdlib.h /usr/include/features.h \
 /usr/include/bits/predefs.h /usr/include/sys/cdefs.h \
 /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h \
 /usr/include/gnu/stubs-64.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stddef.h \
 /usr/include/bits/waitflags.h /usr/include/bits/waitstatus.h \
 /usr/include/endian.h /usr/include/bits/endian.h \
 /usr/include/bits/byteswap.h /usr/include/xlocale.h \
 /usr/include/sys/types.h /usr/include/bits/types.h \
 /usr/include/bits/typesizes.h /usr/include/time.h \
 /usr/include/sys/select.h /usr/include/bits/select.h \
 /usr/include/bits/sigset.h /usr/include/bits/time.h \
 /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h \
 /usr/include/alloca.h /usr/include/string.h /usr/include/sys/stat.h \
 /usr/include/bits/stat.h /usr/include/fcntl.h /usr/include/bits/fcntl.h \
 /usr/include/bits/uio.h LSPIAgent.h SMDPAgent.h Eigen/Dense Eigen/Core \
 Eigen/src/Core/util/DisableStupidWarnings.h Eigen/src/Core/util/Macros.h \
 /usr/include/c++/4.4/complex \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++config.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/os_defines.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/cpu_defines.h \
 /usr/include/c++/4.4/bits/cpp_type_traits.h \
 /usr/include/c++/4.4/ext/type_traits.h /usr/include/c++/4.4/cmath \
 /usr/include/math.h /usr/include/bits/huge_val.h \
 /usr/include/bits/huge_valf.h /usr/include/bits/huge_vall.h \
 /usr/include/bits/inf.h /usr/include/bits/nan.h \
 /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h \
 /usr/include/c++/4.4/bits/cmath.tcc /usr/include/c++/4.4/sstream \
 /usr/include/c++/4.4/istream /usr/include/c++/4.4/ios \
 /usr/include/c++/4.4/iosfwd /usr/include/c++/4.4/bits/stringfwd.h \
 /usr/include/c++/4.4/bits/postypes.h /usr/include/c++/4.4/cwchar \
 /usr/include/c++/4.4/cstddef /usr/include/wchar.h /usr/include/stdio.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/stdarg.h \
 /usr/include/bits/wchar.h /usr/include/c++/4.4/exception \
 /usr/include/c++/4.4/bits/char_traits.h \
 /usr/include/c++/4.4/bits/stl_algobase.h \
 /usr/include/c++/4.4/bits/functexcept.h \
 /usr/include/c++/4.4/exception_defines.h \
 /usr/include/c++/4.4/ext/numeric_traits.h \
 /usr/include/c++/4.4/bits/stl_pair.h /usr/include/c++/4.4/bits/move.h \
 /usr/include/c++/4.4/bits/concept_check.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_types.h \
 /usr/include/c++/4.4/bits/stl_iterator_base_funcs.h \
 /usr/include/c++/4.4/bits/stl_iterator.h \
 /usr/include/c++/4.4/debug/debug.h /usr/include/c++/4.4/bits/localefwd.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++locale.h \
 /usr/include/c++/4.4/clocale /usr/include/locale.h \
 /usr/include/bits/locale.h /usr/include/c++/4.4/cctype \
 /usr/include/ctype.h /usr/include/c++/4.4/bits/ios_base.h \
 /usr/include/c++/4.4/ext/atomicity.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/gthr-default.h \
 /usr/include/pthread.h /usr/include/sched.h /usr/include/bits/sched.h \
 /usr/include/signal.h /usr/include/bits/setjmp.h /usr/include/unistd.h \
 /usr/include/bits/posix_opt.h /usr/include/bits/environments.h \
 /usr/include/bits/confname.h /usr/include/getopt.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/atomic_word.h \
 /usr/include/c++/4.4/bits/locale_classes.h /usr/include/c++/4.4/string \
 /usr/include/c++/4.4/bits/allocator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++allocator.h \
 /usr/include/c++/4.4/ext/new_allocator.h /usr/include/c++/4.4/new \
 /usr/include/c++/4.4/bits/ostream_insert.h \
 /usr/include/c++/4.4/cxxabi-forced.h \
 /usr/include/c++/4.4/bits/stl_function.h \
 /usr/include/c++/4.4/backward/binders.h \
 /usr/include/c++/4.4/bits/basic_string.h \
 /usr/include/c++/4.4/initializer_list \
 /usr/include/c++/4.4/bits/basic_string.tcc \
 /usr/include/c++/4.4/bits/locale_classes.tcc \
 /usr/include/c++/4.4/streambuf /usr/include/c++/4.4/bits/streambuf.tcc \
 /usr/include/c++/4.4/bits/basic_ios.h \
 /usr/include/c++/4.4/bits/locale_facets.h /usr/include/c++/4.4/cwctype \
 /usr/include/wctype.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_base.h \
 /usr/include/c++/4.4/bits/streambuf_iterator.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/ctype_inline.h \
 /usr/include/c++/4.4/bits/locale_facets.tcc \
 /usr/include/c++/4.4/bits/basic_ios.tcc /usr/include/c++/4.4/ostream \
 /usr/include/c++/4.4/bits/ostream.tcc \
 /usr/include/c++/4.4/bits/istream.tcc \
 /usr/include/c++/4.4/bits/sstream.tcc Eigen/src/Core/util/MKL_support.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/emmintrin.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/xmmintrin.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/mmintrin.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include/mm_malloc.h \
 /usr/include/c++/4.4/cerrno /usr/include/errno.h \
 /usr/include/bits/errno.h /usr/include/linux/errno.h \
 /usr/include/asm/errno.h /usr/include/asm-generic/errno.h \
 /usr/include/asm-generic/errno-base.h /usr/include/c++/4.4/cstdlib \
 /usr/include/c++/4.4/cassert /usr/include/assert.h \
 /usr/include/c++/4.4/functional /usr/include/c++/4.4/cstring \
 /usr/include/c++/4.4/limits /usr/include/c++/4.4/climits \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include-fixed/limits.h \
 /usr/lib/gcc/x86_64-linux-gnu/4.4.3/include-fixed/syslimits.h \
 /usr/include/limits.h /usr/include/bits/posix1_lim.h \
 /usr/include/bits/local_lim.h /usr/include/linux/limits.h \
 /usr/include/bits/posix2_lim.h /usr/include/bits/xopen_lim.h \
 /usr/include/bits/stdio_lim.h /usr/include/c++/4.4/algorithm \
 /usr/include/c++/4.4/bits/stl_algo.h \
 /usr/include/c++/4.4/bits/algorithmfwd.h \
 /usr/include/c++/4.4/bits/stl_heap.h \
 /usr/include/c++/4.4/bits/stl_tempbuf.h \
 /usr/include/c++/4.4/bits/stl_construct.h \
 /usr/include/c++/4.4/bits/stl_uninitialized.h \
 Eigen/src/Core/util/Constants.h \
 Eigen/src/Core/util/ForwardDeclarations.h Eigen/src/Core/util/Meta.h \
 Eigen/src/Core/util/XprHelper.h Eigen/src/Core/util/StaticAssert.h \
 Eigen/src/Core/util/Memory.h Eigen/src/Core/NumTraits.h \
 Eigen/src/Core/MathFunctions.h Eigen/src/Core/GenericPacketMath.h \
 Eigen/src/Core/arch/SSE/PacketMath.h \
 Eigen/src/Core/arch/SSE/MathFunctions.h \
 Eigen/src/Core/arch/SSE/Complex.h Eigen/src/Core/arch/Default/Settings.h \
 Eigen/src/Core/Functors.h Eigen/src/Core/DenseCoeffsBase.h \
 Eigen/src/Core/DenseBase.h Eigen/src/Core/../plugins/BlockMethods.h \
 Eigen/src/Core/MatrixBase.h \
 Eigen/src/Core/../plugins/CommonCwiseUnaryOps.h \
 Eigen/src/Core/../plugins/CommonCwiseBinaryOps.h \
 Eigen/src/Core/../plugins/MatrixCwiseUnaryOps.h \
 Eigen/src/Core/../plugins/MatrixCwiseBinaryOps.h \
 Eigen/src/Core/EigenBase.h Eigen/src/Core/Assign.h \
 Eigen/src/Core/util/BlasUtil.h Eigen/src/Core/DenseStorage.h \
 Eigen/src/Core/NestByValue.h Eigen/src/Core/ForceAlignedAccess.h \
 Eigen/src/Core/ReturnByValue.h Eigen/src/Core/NoAlias.h \
 Eigen/src/Core/PlainObjectBase.h Eigen/src/Core/Matrix.h \
 Eigen/src/Core/Array.h Eigen/src/Core/CwiseBinaryOp.h \
 Eigen/src/Core/CwiseUnaryOp.h Eigen/src/Core/CwiseNullaryOp.h \
 Eigen/src/Core/CwiseUnaryView.h Eigen/src/Core/SelfCwiseBinaryOp.h \
 Eigen/src/Core/Dot.h Eigen/src/Core/StableNorm.h \
 Eigen/src/Core/MapBase.h Eigen/src/Core/Stride.h Eigen/src/Core/Map.h \
 Eigen/src/Core/Block.h Eigen/src/Core/VectorBlock.h \
 Eigen/src/Core/Transpose.h Eigen/src/Core/DiagonalMatrix.h \
 Eigen/src/Core/Diagonal.h Eigen/src/Core/DiagonalProduct.h \
 Eigen/src/Core/PermutationMatrix.h Eigen/src/Core/Transpositions.h \
 Eigen/src/Core/Redux.h Eigen/src/Core/Visitor.h Eigen/src/Core/Fuzzy.h \
 Eigen/src/Core/IO.h Eigen/src/Core/Swap.h \
 Eigen/src/Core/CommaInitializer.h Eigen/src/Core/Flagged.h \
 Eigen/src/Core/ProductBase.h Eigen/src/Core/GeneralProduct.h \
 Eigen/src/Core/TriangularMatrix.h Eigen/src/Core/SelfAdjointView.h \
 Eigen/src/Core/products/GeneralBlockPanelKernel.h \
 Eigen/src/Core/products/Parallelizer.h \
 Eigen/src/Core/products/CoeffBasedProduct.h \
 Eigen/src/Core/products/GeneralMatrixVector.h \
 Eigen/src/Core/products/GeneralMatrixMatrix.h \
 Eigen/src/Core/SolveTriangular.h \
 Eigen/src/Core/products/GeneralMatrixMatrixTriangular.h \
 Eigen/src/Core/products/SelfadjointMatrixVector.h \
 Eigen/src/Core/products/SelfadjointMatrixMatrix.h \
 Eigen/src/Core/products/SelfadjointProduct.h \
 Eigen/src/Core/products/SelfadjointRank2Update.h \
 Eigen/src/Core/products/TriangularMatrixVector.h \
 Eigen/src/Core/products/TriangularMatrixMatrix.h \
 Eigen/src/Core/products/TriangularSolverMatrix.h \
 Eigen/src/Core/products/TriangularSolverVector.h \
 Eigen/src/Core/BandMatrix.h Eigen/src/Core/BooleanRedux.h \
 Eigen/src/Core/Select.h Eigen/src/Core/VectorwiseOp.h \
 Eigen/src/Core/Random.h Eigen/src/Core/Replicate.h \
 Eigen/src/Core/Reverse.h Eigen/src/Core/ArrayBase.h \
 Eigen/src/Core/../plugins/ArrayCwiseUnaryOps.h \
 Eigen/src/Core/../plugins/ArrayCwiseBinaryOps.h \
 Eigen/src/Core/ArrayWrapper.h Eigen/src/Core/GlobalFunctions.h \
 Eigen/src/Core/util/ReenableStupidWarnings.h Eigen/LU \
 Eigen/src/misc/Solve.h Eigen/src/misc/Kernel.h Eigen/src/misc/Image.h \
 Eigen/src/LU/FullPivLU.h Eigen/src/LU/PartialPivLU.h \
 Eigen/src/LU/Determinant.h Eigen/src/LU/Inverse.h \
 Eigen/src/LU/arch/Inverse_SSE.h Eigen/Cholesky Eigen/src/Cholesky/LLT.h \
 Eigen/src/Cholesky/LDLT.h Eigen/QR Eigen/Jacobi \
 Eigen/src/Jacobi/Jacobi.h Eigen/Householder \
 Eigen/src/Householder/Householder.h \
 Eigen/src/Householder/HouseholderSequence.h \
 Eigen/src/Householder/BlockHouseholder.h Eigen/src/QR/HouseholderQR.h \
 Eigen/src/QR/FullPivHouseholderQR.h Eigen/src/QR/ColPivHouseholderQR.h \
 Eigen/SVD Eigen/src/SVD/JacobiSVD.h \
 Eigen/src/SVD/UpperBidiagonalization.h Eigen/Geometry \
 Eigen/src/Geometry/OrthoMethods.h Eigen/src/Geometry/EulerAngles.h \
 Eigen/src/Geometry/Homogeneous.h Eigen/src/Geometry/RotationBase.h \
 Eigen/src/Geometry/Rotation2D.h Eigen/src/Geometry/Quaternion.h \
 Eigen/src/Geometry/AngleAxis.h Eigen/src/Geometry/Transform.h \
 Eigen/src/Geometry/Translation.h Eigen/src/Geometry/Scaling.h \
 Eigen/src/Geometry/Hyperplane.h Eigen/src/Geometry/ParametrizedLine.h \
 Eigen/src/Geometry/AlignedBox.h Eigen/src/Geometry/Umeyama.h \
 Eigen/src/Geometry/arch/Geometry_SSE.h Eigen/Eigenvalues \
 Eigen/src/Eigenvalues/Tridiagonalization.h \
 Eigen/src/Eigenvalues/RealSchur.h \
 Eigen/src/Eigenvalues/./HessenbergDecomposition.h \
 Eigen/src/Eigenvalues/EigenSolver.h Eigen/src/Eigenvalues/./RealSchur.h \
 Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h \
 Eigen/src/Eigenvalues/./Tridiagonalization.h \
 Eigen/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h \
 Eigen/src/Eigenvalues/HessenbergDecomposition.h \
 Eigen/src/Eigenvalues/ComplexSchur.h \
 Eigen/src/Eigenvalues/ComplexEigenSolver.h \
 Eigen/src/Eigenvalues/./ComplexSchur.h \
 Eigen/src/Eigenvalues/MatrixBaseEigenvalues.h \
 /usr/include/c++/4.4/vector /usr/include/c++/4.4/bits/stl_vector.h \
 /usr/include/c++/4.4/bits/stl_bvector.h \
 /usr/include/c++/4.4/bits/vector.tcc LoggerDraw.h SoccerTypes.h \
 /usr/include/c++/4.4/iostream Geometry.h ServerSettings.h \
 GenericValues.h /usr/include/c++/4.4/fstream \
 /usr/include/c++/4.4/bits/codecvt.h /usr/include/c++/4.4/cstdio \
 /usr/include/libio.h /usr/include/_G_config.h \
 /usr/include/bits/sys_errlist.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/basic_file.h \
 /usr/include/c++/4.4/x86_64-linux-gnu/bits/c++io.h \
 /usr/include/c++/4.4/bits/fstream.tcc /usr/include/c++/4.4/ctime
