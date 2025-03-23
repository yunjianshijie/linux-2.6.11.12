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
 * ���Ѿ���װ�ļ�ϵͳ�н�ֹsetuid��setgid��־��
 */
#define MNT_NOSUID	1
/**
 * ���Ѿ���װ�ļ�ϵͳ�н�ֹ�����豸�ļ�
 */
#define MNT_NODEV	2
/**
 * ���Ѿ���װ�ļ�ϵͳ�в���������ִ�С�
 */
#define MNT_NOEXEC	4


struct vfsmount
{
	struct list_head mnt_hash; /* hash list */
	
	struct vfsmount *mnt_parent;	/* fs we are mounted on */
	/**
	 * ��װ��Ŀ¼�ڵ㡣
	 */
	struct dentry *mnt_mountpoint;	/* dentry of mountpoint */
	/**
	 * ָ������ļ�ϵͳ��Ŀ¼��dentry��
	 */
	struct dentry *mnt_root;	/* root of the mounted tree */
	/**
	 * ���ļ�ϵͳ�ĳ��������
	 */
	struct super_block *mnt_sb;	/* pointer to superblock */
	/**
	 * ���������ļ�ϵͳ������������ͷ
	 */
	struct list_head mnt_mounts;	/* list of children, anchored here */
	/**
	 * �Ѱ�װ�ļ�ϵͳ����ͷ��ͨ�����ֶν�����븸�ļ�ϵͳ��mnt_mounts�����С�
	 */
	struct list_head mnt_child;	/* and going through their mnt_child */
	/**
	 * ���ü���������ֹ�ļ�ϵͳ��ж�ء�
	 */
	atomic_t mnt_count;
	/**
	 * mount��־
	 */
	int mnt_flags;
	/**
	 * ����ļ�ϵͳ���Ϊ���ڣ������������־��
	 */
	int mnt_expiry_mark;		/* true if marked for expiry */
	/**
	 * �豸�ļ�����
	 */
	char *mnt_devname;		/* Name of device e.g. /dev/dsk/hda1 */
	/**
	 * �Ѱ�װ�ļ�ϵͳ��������namespace����ָ��?
	 * ͨ�����ֶν�����뵽namespace��list�����С�
	 */
	struct list_head mnt_list;
	/**
	 * �ļ�ϵͳ��������ָ�롣
	 */
	struct list_head mnt_fslink;	/* link in fs-specific expiry list */
	/**
	 * ���������ռ�ָ��
	 */
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
