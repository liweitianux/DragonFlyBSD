.include "../../Makefile.inc1"
.include "../Makefile.block2"

.PATH: ${SRCDIR}/binutils

PROG=	elfedit
SRCS=   elfedit.c version.c elfcomm.c
MFILE=	elfedit${MANPAGEVER}.1
MAN=	${MFILE}
LDADD=	${LDADD_IBERTY}

.if defined(IS_PRIMARY)
MLINKS+=	${MFILE} elfedit.1
.endif

.include <bsd.prog.mk>
