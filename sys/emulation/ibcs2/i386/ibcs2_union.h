/*
 * Union of syscall args for messaging.
 *
 * DO NOT EDIT-- this file is automatically generated.
 * $DragonFly: src/sys/emulation/ibcs2/i386/Attic/ibcs2_union.h,v 1.4 2003/08/12 02:36:13 dillon Exp $
 * created from DragonFly: src/sys/emulation/ibcs2/i386/syscalls.master,v 1.3 2003/08/07 21:17:17 dillon Exp 
 */

union sysunion {
#ifdef _KERNEL /* header only applies in kernel */
	struct	lwkt_msg lmsg;
	union	sysmsg sysmsg;
#endif
	struct	ibcs2_read_args ibcs2_read;
	struct	ibcs2_open_args ibcs2_open;
	struct	ibcs2_wait_args ibcs2_wait;
	struct	ibcs2_creat_args ibcs2_creat;
	struct	ibcs2_unlink_args ibcs2_unlink;
	struct	ibcs2_execv_args ibcs2_execv;
	struct	ibcs2_chdir_args ibcs2_chdir;
	struct	ibcs2_time_args ibcs2_time;
	struct	ibcs2_mknod_args ibcs2_mknod;
	struct	ibcs2_chmod_args ibcs2_chmod;
	struct	ibcs2_chown_args ibcs2_chown;
	struct	ibcs2_stat_args ibcs2_stat;
	struct	ibcs2_lseek_args ibcs2_lseek;
	struct	ibcs2_mount_args ibcs2_mount;
	struct	ibcs2_umount_args ibcs2_umount;
	struct	ibcs2_setuid_args ibcs2_setuid;
	struct	ibcs2_stime_args ibcs2_stime;
	struct	ibcs2_alarm_args ibcs2_alarm;
	struct	ibcs2_fstat_args ibcs2_fstat;
	struct	ibcs2_pause_args ibcs2_pause;
	struct	ibcs2_utime_args ibcs2_utime;
	struct	ibcs2_stty_args ibcs2_stty;
	struct	ibcs2_gtty_args ibcs2_gtty;
	struct	ibcs2_access_args ibcs2_access;
	struct	ibcs2_nice_args ibcs2_nice;
	struct	ibcs2_statfs_args ibcs2_statfs;
	struct	ibcs2_kill_args ibcs2_kill;
	struct	ibcs2_fstatfs_args ibcs2_fstatfs;
	struct	ibcs2_pgrpsys_args ibcs2_pgrpsys;
	struct	ibcs2_xenix_args ibcs2_xenix;
	struct	ibcs2_times_args ibcs2_times;
	struct	ibcs2_plock_args ibcs2_plock;
	struct	ibcs2_setgid_args ibcs2_setgid;
	struct	ibcs2_sigsys_args ibcs2_sigsys;
	struct	ibcs2_msgsys_args ibcs2_msgsys;
	struct	ibcs2_sysi86_args ibcs2_sysi86;
	struct	ibcs2_shmsys_args ibcs2_shmsys;
	struct	ibcs2_semsys_args ibcs2_semsys;
	struct	ibcs2_ioctl_args ibcs2_ioctl;
	struct	ibcs2_uadmin_args ibcs2_uadmin;
	struct	ibcs2_utssys_args ibcs2_utssys;
	struct	ibcs2_execve_args ibcs2_execve;
	struct	ibcs2_fcntl_args ibcs2_fcntl;
	struct	ibcs2_ulimit_args ibcs2_ulimit;
	struct	ibcs2_rmdir_args ibcs2_rmdir;
	struct	ibcs2_mkdir_args ibcs2_mkdir;
	struct	ibcs2_getdents_args ibcs2_getdents;
	struct	ibcs2_sysfs_args ibcs2_sysfs;
	struct	ibcs2_getmsg_args ibcs2_getmsg;
	struct	ibcs2_putmsg_args ibcs2_putmsg;
	struct	ibcs2_poll_args ibcs2_poll;
	struct	ibcs2_secure_args ibcs2_secure;
	struct	ibcs2_symlink_args ibcs2_symlink;
	struct	ibcs2_lstat_args ibcs2_lstat;
	struct	ibcs2_readlink_args ibcs2_readlink;
	struct	ibcs2_isc_args ibcs2_isc;
};
