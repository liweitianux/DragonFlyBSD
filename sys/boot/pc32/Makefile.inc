# Common defines for all of /sys/boot/i386/

.if !target(__pc32__) && !defined(__no_pc32_defines)
__pc32__:

LOADER_ADDRESS?= 0x200000
WORLD_CCOPTLEVEL=	1 # Force -O1, -O2 implodes
CFLAGS+=	-D_STANDALONE
CFLAGS+=	-ffreestanding -fno-omit-frame-pointer
CFLAGS+=	-mno-mmx -mno-3dnow -mno-sse -mno-sse2 -mno-sse3 -msoft-float

# override CPU type, so that gcc doesn't insert mmx/sse instructions
FORCE_CPUTYPE=	i386

WARNS?=	2

# if a gcc compiler is being used, identify GCC-only flags,
# otherwise check for clang and set value of BOOT_CCASM_FLAGS if found
.if ${CCVER:Mgcc*}
CFLAGS+=		-mpreferred-stack-boundary=2
BOOT2_GCC_FLAGS=	-fno-guess-branch-probability \
			-fno-unit-at-a-time
.elif ${CCVER:Mclang*}
BOOT_CCASM_FLAGS+=	-no-integrated-as
.endif

.if defined(REALLY_X86_64)
.include "../pc64/Makefile.inc"
.endif
.endif
