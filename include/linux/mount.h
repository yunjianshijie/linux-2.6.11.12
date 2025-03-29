/*
 *
 * Definitions for mount interface. This describes the in the kernel build 
 * linkedlist with mounted filesystems.
 *
 * Author:  Marco van Wieringen <mvw@planets.elm.net>
 *
 * Version: $Id: mount.h,v 2.0 1996/11/17 16:48:14 mvw Exp mvw $
 *
 */
#ifndef _LINUX_MOUNT_H
#define _LINUX_MOUNT_H
#ifdef __KERNEL__

#include <linux/list.h>
#include <linux/spinlock.h>
#include <asm/atomic.h>

/**
 * mnt_no suid 挂载文件系统时不允许设置用户ID（SUID）位 (suid是要访问这个文件的用户是否具有和文件属主相同的权限) 1
 *
#define MNT_NOSUID 1 // 
/** 
 *  mnt_no dev没有设备文件 2 
 */
#define MNT_NODEV	2 //
/** 
 *  mnt_no exec 没进程可执行文件 4
 */
#define MNT_NOEXEC	4 //

struct vfsmount {
    struct list_head mnt_hash; /* hash list */ // 散列表，把vfsmount结构体挂载到散列表中
	
	struct vfsmount *mnt_parent;	/* fs we are mounted on */ // 指向父文件系统的vfsmount结构体
	
	struct dentry *mnt_mountpoint;	/* dentry of mountpoint */ // 挂载点的（目录项） 指针
	
	struct dentry *mnt_root;	/* root of the mounted tree */ // 挂载点的根目录项
	
	struct super_block *mnt_sb;	/* pointer to superblock */  // 指向超级块的指针 // 
	
	struct list_head mnt_mounts;	/* list of children, anchored here */ // 指向子文件系统的vfsmount结构体链表
	
	struct list_head mnt_child;	/* and going through their mnt_child */ // 指向父文件系统的vfsmount结构体链表
	
	atomic_t mnt_count; 
	
	int mnt_flags; 
	
	int mnt_expiry_mark;		/* true if marked for expiry */
	
	char *mnt_devname;		/* Name of device e.g. /dev/dsk/hda1 */
	
	struct list_head mnt_list;

	struct list_head mnt_fslink;	/* link in fs-specific expiry list */
	
	struct namespace *mnt_namespace; /* containing namespace */
};

static inline struct vfsmount *mntget(struct vfsmount *mnt)
{
	if (mnt)
		atomic_inc(&mnt->mnt_count);
	return mnt;
}

extern void __mntput(struct vfsmount *mnt);

static inline void _mntput(struct vfsmount *mnt)
{
	if (mnt) {
		if (atomic_dec_and_test(&mnt->mnt_count))
			__mntput(mnt);
	}
}

static inline void mntput(struct vfsmount *mnt)
{
	if (mnt) {
		mnt->mnt_expiry_mark = 0;
		_mntput(mnt);
	}
}

extern void free_vfsmnt(struct vfsmount *mnt);
extern struct vfsmount *alloc_vfsmnt(const char *name);
extern struct vfsmount *do_kern_mount(const char *fstype, int flags,
				      const char *name, void *data);

struct nameidata;

extern int do_add_mount(struct vfsmount *newmnt, struct nameidata *nd,
			int mnt_flags, struct list_head *fslist);

extern void mark_mounts_for_expiry(struct list_head *mounts);

/**
 * �����Ѿ���װ�ļ�ϵͳ��������
 */
extern spinlock_t vfsmount_lock;

#endif
#endif /* _LINUX_MOUNT_H */
