/*
 *  linux/fs/namei.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 */

/*
 * Some corrections by tytso.
 */

/* [Feb 1997 T. Schoebel-Theuer] Complete rewrite of the pathname
 * lookup logic.
 */
/* [Feb-Apr 2000, AV] Rewrite to the new namespace architecture.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/quotaops.h>
#include <linux/pagemap.h>
#include <linux/dnotify.h>
#include <linux/smp_lock.h>
#include <linux/personality.h>
#include <linux/security.h>
#include <linux/syscalls.h>
#include <linux/mount.h>
#include <linux/audit.h>
#include <asm/namei.h>
#include <asm/uaccess.h>

#define ACC_MODE(x) ("\000\004\002\006"[(x)&O_ACCMODE])

/* [Feb-1997 T. Schoebel-Theuer]
 * Fundamental changes in the pathname lookup mechanisms (namei)
 * were necessary because of omirr.  The reason is that omirr needs
 * to know the _real_ pathname, not the user-supplied one, in case
 * of symlinks (and also when transname replacements occur).
 *
 * The new code replaces the old recursive symlink resolution with
 * an iterative one (in case of non-nested symlink chains).  It does
 * this with calls to <fs>_follow_link().
 * As a side effect, dir_namei(), _namei() and follow_link() are now 
 * replaced with a single function lookup_dentry() that can handle all 
 * the special cases of the former code.
 *
 * With the new dcache, the pathname is stored at each inode, at least as
 * long as the refcount of the inode is positive.  As a side effect, the
 * size of the dcache depends on the inode cache and thus is dynamic.
 *
 * [29-Apr-1998 C. Scott Ananian] Updated above description of symlink
 * resolution to correspond with current state of the code.
 *
 * Note that the symlink resolution is not *completely* iterative.
 * There is still a significant amount of tail- and mid- recursion in
 * the algorithm.  Also, note that <fs>_readlink() is not used in
 * lookup_dentry(): lookup_dentry() on the result of <fs>_readlink()
 * may return different results than <fs>_follow_link().  Many virtual
 * filesystems (including /proc) exhibit this behavior.
 */

/* [24-Feb-97 T. Schoebel-Theuer] Side effects caused by new implementation:
 * New symlink semantics: when open() is called with flags O_CREAT | O_EXCL
 * and the name already exists in form of a symlink, try to create the new
 * name indicated by the symlink. The old code always complained that the
 * name already exists, due to not following the symlink even if its target
 * is nonexistent.  The new semantics affects also mknod() and link() when
 * the name is a symlink pointing to a non-existant name.
 *
 * I don't know which semantics is the right one, since I have no access
 * to standards. But I found by trial that HP-UX 9.0 has the full "new"
 * semantics implemented, while SunOS 4.1.1 and Solaris (SunOS 5.4) have the
 * "old" one. Personally, I think the new semantics is much more logical.
 * Note that "ln old new" where "new" is a symlink pointing to a non-existing
 * file does succeed in both HP-UX and SunOs, but not in Solaris
 * and in the old Linux semantics.
 */

/* [16-Dec-97 Kevin Buhr] For security reasons, we change some symlink
 * semantics.  See the comments in "open_namei" and "do_link" below.
 *
 * [10-Sep-98 Alan Modra] Another symlink change.
 */

/* [Feb-Apr 2000 AV] Complete rewrite. Rules for symlinks:
 *	inside the path - always follow.
 *	in the last component in creation/removal/renaming - never follow.
 *	if LOOKUP_FOLLOW passed - follow.
 *	if the pathname has trailing slashes - follow.
 *	otherwise - don't follow.
 * (applied in that order).
 *
 * [Jun 2000 AV] Inconsistent behaviour of open() in case if flags==O_CREAT
 * restored for 2.4. This is the last surviving part of old 4.2BSD bug.
 * During the 2.4 we need to fix the userland stuff depending on it -
 * hopefully we will be able to get rid of that wart in 2.5. So far only
 * XEmacs seems to be relying on it...
 */
/*
 * [Sep 2001 AV] Single-semaphore locking scheme (kudos to David Holland)
 * implemented.  Let's see if raised priority of ->s_vfs_rename_sem gives
 * any extra contention...
 */

/* In order to reduce some races, while at the same time doing additional
 * checking and hopefully speeding things up, we copy filenames to the
 * kernel data space before using them..
 *
 * POSIX.1 2.4: an empty pathname is invalid (ENOENT).
 * PATH_MAX includes the nul terminator --RR.
 */
static inline int do_getname(const char __user *filename, char *page)
{
	int retval;
	unsigned long len = PATH_MAX;

	if (!segment_eq(get_fs(), KERNEL_DS)) {
		if ((unsigned long) filename >= TASK_SIZE)
			return -EFAULT;
		if (TASK_SIZE - (unsigned long) filename < PATH_MAX)
			len = TASK_SIZE - (unsigned long) filename;
	}

	retval = strncpy_from_user(page, filename, len);
	if (retval > 0) {
		if (retval < len)
			return 0;
		return -ENAMETOOLONG;
	} else if (!retval)
		retval = -ENOENT;
	return retval;
}

char * getname(const char __user * filename)
{
	char *tmp, *result;

	result = ERR_PTR(-ENOMEM);
	tmp = __getname();
	if (tmp)  {
		int retval = do_getname(filename, tmp);

		result = tmp;
		if (retval < 0) {
			__putname(tmp);
			result = ERR_PTR(retval);
		}
	}
	if (unlikely(current->audit_context) && !IS_ERR(result) && result)
		audit_getname(result);
	return result;
}

/**
 * generic_permission  -  check for access rights on a Posix-like filesystem
 * @inode:	inode to check access rights for
 * @mask:	right to check for (%MAY_READ, %MAY_WRITE, %MAY_EXEC)
 * @check_acl:	optional callback to check for Posix ACLs
 *
 * Used to check for read/write/execute permissions on a file.
 * We use "fsuid" for this, letting us set arbitrary permissions
 * for filesystem access without changing the "normal" uids which
 * are used for other things..
 */
int generic_permission(struct inode *inode, int mask,
		int (*check_acl)(struct inode *inode, int mask))
{
	umode_t			mode = inode->i_mode;

	if (current->fsuid == inode->i_uid)
		mode >>= 6;
	else {
		if (IS_POSIXACL(inode) && (mode & S_IRWXG) && check_acl) {
			int error = check_acl(inode, mask);
			if (error == -EACCES)
				goto check_capabilities;
			else if (error != -EAGAIN)
				return error;
		}

		if (in_group_p(inode->i_gid))
			mode >>= 3;
	}

	/*
	 * If the DACs are ok we don't need any capability check.
	 */
	if (((mode & mask & (MAY_READ|MAY_WRITE|MAY_EXEC)) == mask))
		return 0;

 check_capabilities:
	/*
	 * Read/write DACs are always overridable.
	 * Executable DACs are overridable if at least one exec bit is set.
	 */
	if (!(mask & MAY_EXEC) ||
	    (inode->i_mode & S_IXUGO) || S_ISDIR(inode->i_mode))
		if (capable(CAP_DAC_OVERRIDE))
			return 0;

	/*
	 * Searching includes executable on directories, else just read.
	 */
	if (mask == MAY_READ || (S_ISDIR(inode->i_mode) && !(mask & MAY_WRITE)))
		if (capable(CAP_DAC_READ_SEARCH))
			return 0;

	return -EACCES;
}

int permission(struct inode *inode, int mask, struct nameidata *nd)
{
	int retval, submask;

	if (mask & MAY_WRITE) {
		umode_t mode = inode->i_mode;

		/*
		 * Nobody gets write access to a read-only fs.
		 */
		if (IS_RDONLY(inode) &&
		    (S_ISREG(mode) || S_ISDIR(mode) || S_ISLNK(mode)))
			return -EROFS;

		/*
		 * Nobody gets write access to an immutable file.
		 */
		if (IS_IMMUTABLE(inode))
			return -EACCES;
	}


	/* Ordinary permission routines do not understand MAY_APPEND. */
	submask = mask & ~MAY_APPEND;
	if (inode->i_op && inode->i_op->permission)
		retval = inode->i_op->permission(inode, submask, nd);
	else
		retval = generic_permission(inode, submask, NULL);
	if (retval)
		return retval;

	return security_inode_permission(inode, mask, nd);
}

/*
 * get_write_access() gets write permission for a file.
 * put_write_access() releases this write permission.
 * This is used for regular files.
 * We cannot support write (and maybe mmap read-write shared) accesses and
 * MAP_DENYWRITE mmappings simultaneously. The i_writecount field of an inode
 * can have the following values:
 * 0: no writers, no VM_DENYWRITE mappings
 * < 0: (-i_writecount) vm_area_structs with VM_DENYWRITE set exist
 * > 0: (i_writecount) users are writing to the file.
 *
 * Normally we operate on that counter with atomic_{inc,dec} and it's safe
 * except for the cases where we don't hold i_writecount yet. Then we need to
 * use {get,deny}_write_access() - these functions check the sign and refuse
 * to do the change if sign is wrong. Exclusion between them is provided by
 * the inode->i_lock spinlock.
 */

int get_write_access(struct inode * inode)
{
	spin_lock(&inode->i_lock);
	if (atomic_read(&inode->i_writecount) < 0) {
		spin_unlock(&inode->i_lock);
		return -ETXTBSY;
	}
	atomic_inc(&inode->i_writecount);
	spin_unlock(&inode->i_lock);

	return 0;
}

int deny_write_access(struct file * file)
{
	struct inode *inode = file->f_dentry->d_inode;

	spin_lock(&inode->i_lock);
	/**
	 * deny_write_accessͨ�������������i_writecount�ֶΣ�
	 * ȷ����ִ���ļ�û�б�д�롣
	 */
	if (atomic_read(&inode->i_writecount) > 0) {
		spin_unlock(&inode->i_lock);
		return -ETXTBSY;
	}
	/**
	 * ����-1���������ֶ��Խ�ֹ��һ����д����
	 */
	atomic_dec(&inode->i_writecount);
	spin_unlock(&inode->i_lock);

	return 0;
}

void path_release(struct nameidata *nd)
{
	dput(nd->dentry);
	mntput(nd->mnt);
}

/*
 * umount() mustn't call path_release()/mntput() as that would clear
 * mnt_expiry_mark
 */
void path_release_on_umount(struct nameidata *nd)
{
	dput(nd->dentry);
	_mntput(nd->mnt);
}

/*
 * Internal lookup() using the new generic dcache.
 * SMP-safe
 */
static struct dentry * cached_lookup(struct dentry * parent, struct qstr * name, struct nameidata *nd)
{
	struct dentry * dentry = __d_lookup(parent, name);

	/* lockess __d_lookup may fail due to concurrent d_move() 
	 * in some unrelated directory, so try with d_lookup
	 */
	if (!dentry)
		dentry = d_lookup(parent, name);

	if (dentry && dentry->d_op && dentry->d_op->d_revalidate) {
		if (!dentry->d_op->d_revalidate(dentry, nd) && !d_invalidate(dentry)) {
			dput(dentry);
			dentry = NULL;
		}
	}
	return dentry;
}

/*
 * Short-cut version of permission(), for calling by
 * path_walk(), when dcache lock is held.  Combines parts
 * of permission() and generic_permission(), and tests ONLY for
 * MAY_EXEC permission.
 *
 * If appropriate, check DAC only.  If not appropriate, or
 * short-cut DAC fails, then call permission() to do more
 * complete permission check.
 */
/**
 * ��������������i_node�ֶεķ���ģʽ�����н��̵���Ȩ��
 */
static inline int exec_permission_lite(struct inode *inode,
				       struct nameidata *nd)
{
	umode_t	mode = inode->i_mode;

	if (inode->i_op && inode->i_op->permission)
		return -EAGAIN;

	if (current->fsuid == inode->i_uid)
		mode >>= 6;
	else if (in_group_p(inode->i_gid))
		mode >>= 3;

	if (mode & MAY_EXEC)
		goto ok;

	if ((inode->i_mode & S_IXUGO) && capable(CAP_DAC_OVERRIDE))
		goto ok;

	if (S_ISDIR(inode->i_mode) && capable(CAP_DAC_OVERRIDE))
		goto ok;

	if (S_ISDIR(inode->i_mode) && capable(CAP_DAC_READ_SEARCH))
		goto ok;

	return -EACCES;
ok:
	return security_inode_permission(inode, MAY_EXEC, nd);
}

/*
 * This is called when everything else fails, and we actually have
 * to go to the low-level filesystem to find out what we should do..
 *
 * We get the directory semaphore, and after getting that we also
 * make sure that nobody added the entry to the dcache in the meantime..
 * SMP-safe
 */
/**
 * �Ӵ����ж�ȡĿ¼��������ض����Ƶ�Ŀ¼��
 */
static struct dentry * real_lookup(struct dentry * parent, struct qstr * name, struct nameidata *nd)
{
	struct dentry * result;
	struct inode *dir = parent->d_inode;

	/* ��ȡĿ¼����� */
	down(&dir->i_sem);
	/*
	 * First re-do the cached lookup just in case it was created
	 * while we waited for the directory semaphore..
	 *
	 * FIXME! This could use version numbering or similar to
	 * avoid unnecessary cache lookups.
	 *
	 * The "dcache_lock" is purely to protect the RCU list walker
	 * from concurrent renames at this point (we mustn't get false
	 * negatives from the RCU list walk here, unlike the optimistic
	 * fast walk).
	 *
	 * so doing d_lookup() (with seqlock), instead of lockfree __d_lookup
	 */
	/* �ڻ�ȡ���Ĺ����У�Ŀ¼������Ѿ������������У��ٴδ�Ŀ¼��������� */
	result = d_lookup(parent, name);
	if (!result) {/* Ŀ¼�������Ȼ������ */
		/* ���䲢��ʼ��Ŀ¼��� */
		struct dentry * dentry = d_alloc(parent, name);
		result = ERR_PTR(-ENOMEM);
		if (dentry) {
			/* �Ӵ����ж�ȡĿ¼��� */
			result = dir->i_op->lookup(dir, dentry, nd);
			if (result)
				dput(dentry);
			else
				result = dentry;
		}
		up(&dir->i_sem);/* �ͷ��������� */
		return result;
	}

	/*
	 * Uhhuh! Nasty case: the cache was re-populated while
	 * we waited on the semaphore. Need to revalidate.
	 */
	/* �������Ѿ�����Ŀ¼���ˣ��ͷ��� */
	up(&dir->i_sem);
	/* �ļ�ϵͳҪ���Ŀ¼����У�� */
	if (result->d_op && result->d_op->d_revalidate) {
		/* ���У�鲻�ɹ���˵��Ŀ¼���Ѿ�ʧЧ������ʧ�� */
		if (!result->d_op->d_revalidate(result, nd) && !d_invalidate(result)) {
			dput(result);
			result = ERR_PTR(-ENOENT);
		}
	}
	return result;
}

static int __emul_lookup_dentry(const char *, struct nameidata *);

/* SMP-safe */
static inline int
walk_init_root(const char *name, struct nameidata *nd)
{
	read_lock(&current->fs->lock);
	if (current->fs->altroot && !(nd->flags & LOOKUP_NOALT)) {
		nd->mnt = mntget(current->fs->altrootmnt);
		nd->dentry = dget(current->fs->altroot);
		read_unlock(&current->fs->lock);
		if (__emul_lookup_dentry(name,nd))
			return 0;
		read_lock(&current->fs->lock);
	}
	nd->mnt = mntget(current->fs->rootmnt);
	nd->dentry = dget(current->fs->root);
	read_unlock(&current->fs->lock);
	return 1;
}

static inline int __vfs_follow_link(struct nameidata *nd, const char *link)
{
	int res = 0;
	char *name;
	if (IS_ERR(link))
		goto fail;

	/**
	 * �����������Ƿ���'/'��ͷ
	 */
	if (*link == '/') {
		/**
		 * ����'/'��ͷ���Ѿ��ҵ�һ������·���ˣ����û�б�Ҫ����ǰһ��·�����κ���Ϣ��
		 * path_release�ͻ��ͷ�ǰһ�����Ҳ�������Ķ���nameidata���ݽṹ�е�·����Ϣ�������
		 * һ�д�ͷ��ʼ��
		 */
		path_release(nd);
		/**
		 * ����nameidata��dentry��mnt�ֶΣ���ʹ����ָ��ǰ���̵ĸ�Ŀ¼
		 * walk_init_root����������顣
		 */
		if (!walk_init_root(link, nd))
			/* weird __emul_prefix() stuff did it */
			goto out;
	}
	/**
	 * link_path_walk������������·����,������res
	 */
	res = link_path_walk(link, nd);
out:
	if (nd->depth || res || nd->last_type!=LAST_NORM)
		return res;
	/*
	 * If it is an iterative symlinks resolution in open_namei() we
	 * have to copy the last component. And all that crap because of
	 * bloody create() on broken symlinks. Furrfu...
	 */
	name = __getname();
	if (unlikely(!name)) {
		path_release(nd);
		return -ENOMEM;
	}
	strcpy(name, nd->last.name);
	nd->last.name = name;
	return 0;
fail:
	path_release(nd);
	return PTR_ERR(link);
}

/**
 * �������Ӳ���
 */
static inline int __do_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	int error;

	/**
	 * �����������ķ���ʱ�䡣
	 */
	touch_atime(nd->mnt, dentry);
	nd_set_link(nd, NULL);
	/**
	 * �����ļ�ϵͳ��صĺ���ʵ�ֵ�follow_link������
	 * ����ȡ����ڷ���������������е�·�������������·����������nd->save_names���������
	 */
	error = dentry->d_inode->i_op->follow_link(dentry, nd);
	if (!error) {
		/* nd_get_link���follow_link��ȡ���ַ��� */
		char *s = nd_get_link(nd);
		if (s)
		 	/**
		 	 * ����������ӵĵ�һ���ַ���"/"�������ڴ��в��ñ���ǰһ��·������Ϣ���������ָ��ǰ���̵ĸ�Ŀ¼��
		 	 * �������link_path_walk����·������
		 	 */
			error = __vfs_follow_link(nd, s);
		/**
		 * �����put_link��������ִ�������ͷ���follow_link�����������ʱ���ݽṹ��
		 */
		if (dentry->d_inode->i_op->put_link)
			dentry->d_inode->i_op->put_link(dentry, nd);
	}

	return error;
}

/*
 * This limits recursive symlink follows to 8, while
 * limiting consecutive symlinks to 40.
 *
 * Without that kind of total limit, nasty chains of consecutive
 * symlinks can cause almost arbitrarily long lookups. 
 */
/**
 * ��link_path_walk���á�link_path_walk���ڲ��ҷ�������
 * dentry�Ƿ������ӵĵ�ַ��nd��nameidata�ṹ��ַַ��
 */
static inline int do_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	int err = -ELOOP;
	/**
	 * ���ڷ������ӿ���Ƕ�ף�������ж�һ�£������ڴ��γɵݹ���á���ѭ����
	 */
	if (current->link_count >= MAX_NESTED_LINKS)
		goto loop;
	if (current->total_link_count >= 40)
		goto loop;
	BUG_ON(nd->depth >= MAX_NESTED_LINKS);
	/**
	 * �ж�һ����ռ
	 */
	cond_resched();
	err = security_inode_follow_link(dentry, nd);
	if (err)
		goto loop;
	/**
	 * ��������μ�ǰ���ע�͡�
	 */
	current->link_count++;
	current->total_link_count++;
	nd->depth++;
	/**
	 * __do_follow_link��һ���ݹ���ã�
	 */
	err = __do_follow_link(dentry, nd);
	/**
	 * link_count��__do_follow_linkǶ�״�������������������
	 */
	current->link_count--;
	/**
	 * ���β��ҵĵݹ������
	 */
	nd->depth--;
	return err;
loop:
	path_release(nd);
	return err;
}

int follow_up(struct vfsmount **mnt, struct dentry **dentry)
{
	struct vfsmount *parent;
	struct dentry *mountpoint;
	spin_lock(&vfsmount_lock);
	parent=(*mnt)->mnt_parent;
	if (parent == *mnt) {
		spin_unlock(&vfsmount_lock);
		return 0;
	}
	mntget(parent);
	mountpoint=dget((*mnt)->mnt_mountpoint);
	spin_unlock(&vfsmount_lock);
	dput(*dentry);
	*dentry = mountpoint;
	mntput(*mnt);
	*mnt = parent;
	return 1;
}

/* no need for dcache_lock, as serialization is taken care in
 * namespace.c
 */
/* �ҵ����ص� */
static int follow_mount(struct vfsmount **mnt, struct dentry **dentry)
{
	int res = 0;
	/* ֱ����ǰ·��������һ�����ص�����˳� */
	while (d_mountpoint(*dentry)) {
		/* �ڹ�ϣ���в��ҵ�ǰ·�����ļ�ϵͳװ��ʵ�� */
		struct vfsmount *mounted = lookup_mnt(*mnt, *dentry);
		if (!mounted)/* δ�����ļ�ϵͳ���˳� */
			break;
		/* ���ļ�ϵͳ�ĸ�Ŀ¼��Ϊ��ǰ·������������ */
		mntput(*mnt);
		*mnt = mounted;
		dput(*dentry);
		*dentry = dget(mounted->mnt_root);
		res = 1;
	}
	return res;
}

/* no need for dcache_lock, as serialization is taken care in
 * namespace.c
 */
static inline int __follow_down(struct vfsmount **mnt, struct dentry **dentry)
{
	struct vfsmount *mounted;

	mounted = lookup_mnt(*mnt, *dentry);
	if (mounted) {
		mntput(*mnt);
		*mnt = mounted;
		dput(*dentry);
		*dentry = dget(mounted->mnt_root);
		return 1;
	}
	return 0;
}

int follow_down(struct vfsmount **mnt, struct dentry **dentry)
{
	return __follow_down(mnt,dentry);
}
 
/* ������·��ʱ������".."���ص��ϼ�·�� */
static inline void follow_dotdot(struct vfsmount **mnt, struct dentry **dentry)
{
	while(1) {
		struct vfsmount *parent;
		struct dentry *old = *dentry;

                read_lock(&current->fs->lock);
		if (*dentry == current->fs->root &&/* �Ѿ����ﵱǰ���̵ĸ�Ŀ¼���˳� */
		    *mnt == current->fs->rootmnt) {
                        read_unlock(&current->fs->lock);
			break;
		}
                read_unlock(&current->fs->lock);
		spin_lock(&dcache_lock);
		/* ��ǰĿ¼����������Ŀ¼�ĸ�Ŀ¼ */
		if (*dentry != (*mnt)->mnt_root) {
			/* ���ص���һ��Ŀ¼ */
			*dentry = dget((*dentry)->d_parent);
			spin_unlock(&dcache_lock);
			dput(old);
			break;
		}
		spin_unlock(&dcache_lock);
		/* ���е����˵�����������ļ�ϵͳ�ĸ�Ŀ¼ */
		spin_lock(&vfsmount_lock);
		/* ת�������ļ�ϵͳ���ϼ�Ŀ¼ */
		parent = (*mnt)->mnt_parent;
		if (parent == *mnt) {/* �ϼ�����Ŀ¼�Ѿ��Ǹ�Ŀ¼�� */
			spin_unlock(&vfsmount_lock);
			break;
		}
		mntget(parent);
		/* Ŀ¼���Ϊ��ǰ�ļ�ϵͳ�Ĺ��ص� */
		*dentry = dget((*mnt)->mnt_mountpoint);
		spin_unlock(&vfsmount_lock);
		dput(old);
		mntput(*mnt);
		*mnt = parent;
	}
	/* ����ҵ���Ŀ¼������һ�����ص㣬����follow_mount���ٵ����һ�ι��ص�Ŀ¼�� */
	follow_mount(mnt, dentry);
}

/* �ں��б�ʾ��·�� */
struct path {
	/* �ļ�ϵͳ��װ�ض��� */
	struct vfsmount *mnt;
	/* Ŀ¼���б� */
	struct dentry *dentry;
};

/*
 *  It's more convoluted than I'd like it to be, but... it's still fairly
 *  small and for now I'd prefer to have fast path as straight as possible.
 *  It _is_ time-critical.
 */
/**
 * �õ�������ĸ�Ŀ¼���ļ�����ص�Ŀ¼�����.
 */
static int do_lookup(struct nameidata *nd, struct qstr *name,
		     struct path *path)
{
	struct vfsmount *mnt = nd->mnt;
	/**
	 * __d_lookup��Ŀ¼����ٻ���������������Ŀ¼�����.
	 */
	struct dentry *dentry = __d_lookup(nd->dentry, name);

	/* ��Ŀ¼�����û���ҵ�������Ҫ�Ӵ����϶�ȡĿ¼������ */
	if (!dentry)
		goto need_lookup;
	/* ��Ŀ¼�����������Ŀ¼������ļ�ϵͳҪ��Ի����е�Ŀ¼�����У�� */
	if (dentry->d_op && dentry->d_op->d_revalidate)
		goto need_revalidate;
done:
	path->mnt = mnt;
	path->dentry = dentry;
	return 0;

need_lookup:
	/**
	 * û����Ŀ¼����ٻ������ҵ�Ŀ¼�����,�͵���real_lookup
	 * real_lookupִ�������ڵ��lookup�����Ӵ��̶�ȡĿ¼,����һ���µ�Ŀ¼����󲢰������뵽Ŀ¼����ٻ�����.
	 * Ȼ�󴴽�һ���µ������ڵ�,���������뵽�����ڵ���ٻ�����.
	 */
	dentry = real_lookup(nd->dentry, name, nd);
	if (IS_ERR(dentry))
		goto fail;
	goto done;

need_revalidate:
	/* У��ɹ� */
	if (dentry->d_op->d_revalidate(dentry, nd))
		goto done;
	if (d_invalidate(dentry))
		goto done;
	dput(dentry);
	goto need_lookup;

fail:
	return PTR_ERR(dentry);
}

/*
 * Name resolution.
 *
 * This is the basic name resolution function, turning a pathname
 * into the final dentry.
 *
 * We expect 'base' to be positive and a directory.
 */
/**
 * ������·������ʵ�֡�
 */
int fastcall link_path_walk(const char * name, struct nameidata *nd)
{
	struct path next;
	struct inode *inode;
	int err;
	/* ���ұ�־ */
	unsigned int lookup_flags = nd->flags;

	/* ���/������һ��/ */
	while (*name=='/')
		name++;
	if (!*name)/* ���Ҹ�Ŀ¼���߿�Ŀ¼������Ѿ�������nd���ˣ�ֱ�ӷ��� */
		goto return_reval;

	/* ÿ����inode���ڵ�Ŀ¼�½��в��� */
	inode = nd->dentry->d_inode;
	/**
	 * �������Ӳ��ҡ�
	 */
	if (nd->depth)
		lookup_flags = LOOKUP_FOLLOW;

	/* At this point we know we have a real path component. */
	/**
	 * ѭ������ÿһ��·��������
	 */
	for(;;) {
		unsigned long hash;
		struct qstr this;
		unsigned int c;

		/**
		 * ֻ��Ŀ¼�п�ִ��Ȩ�ޣ����ܱ�������
		 */
		err = exec_permission_lite(inode, nd);
		if (err == -EAGAIN) { 
			err = permission(inode, MAY_EXEC, nd);
		}
 		if (err)/* Ȩ�޴��� */
			break;

		this.name = name;
		c = *(const unsigned char *)name;

		/**
		 * ����ɢ��ֵ
		 */
		hash = init_name_hash();
		do {
			name++;
			hash = partial_name_hash(c, hash);
			c = *(const unsigned char *)name;
		} while (c && (c != '/'));
		this.len = name - (const char *) this.name;
		this.hash = end_name_hash(hash);

		/* remove trailing slashes? */
		/**
		 * ���һ������������û����"/"��������һ���ļ���
		 */
		if (!c)
			goto last_component;
		/**
		 * ���������"/"�������˳���
		 */
		while (*++name == '/');
		if (!*name)
			goto last_with_slashes;

		/*
		 * "." and ".." are special - ".." especially so because it has
		 * to be able to know about the current root directory and
		 * parent relationships.
		 */
		if (this.name[0] == '.') switch (this.len) {/* ������һ��"." */
			default:/* ����"."����ʼ�����÷����� */
				break;
			case 2:	
				if (this.name[1] != '.')
					break;
				follow_dotdot(&nd->mnt, &nd->dentry);
				inode = nd->dentry->d_inode;/* dentry������һ��Ŀ¼��ת���ϼ�Ŀ¼�������� */
				/* fallthrough */
			case 1:/* ��ǰĿ¼������������һ�������� */
				continue;
		}
		/*
		 * See if the low-level filesystem might want
		 * to use its own hash..
		 */
		/**
		 * ����ļ�ϵͳ����Ŀ¼���棬�������Զ����ɢ�к��������������ɢ��ֵ��
		 */
		if (nd->dentry->d_op && nd->dentry->d_op->d_hash) {
			err = nd->dentry->d_op->d_hash(nd->dentry, &this);
			if (err < 0)
				break;
		}
		nd->flags |= LOOKUP_CONTINUE;
		/* This does the actual lookups.. */
		/**
		 * �ڵ�ǰĿ¼��������һ��������
		 */
		err = do_lookup(nd, &this, &next);
		if (err)
			break;
		/* Check mountpoints.. */
		/**
		 * ���ս����ķ����Ƿ��Ӧһ���ļ�ϵͳ�İ�װ�㡣����ǣ������next��mnt��dentry
		 */
		follow_mount(&next.mnt, &next.dentry);

		err = -ENOENT;
		inode = next.dentry->d_inode;
		if (!inode)/* Ŀ¼������ */
			goto out_dput;
		err = -ENOTDIR; 
		if (!inode->i_op)
			goto out_dput;

		if (inode->i_op->follow_link) {/* �ս����ķ�����һ���������� */
			mntget(next.mnt);
			/**
			 * �����������ӡ�
			 */
			err = do_follow_link(next.dentry, nd);
			dput(next.dentry);
			mntput(next.mnt);
			if (err)
				goto return_err;
			err = -ENOENT;
			inode = nd->dentry->d_inode;
			if (!inode)
				break;
			err = -ENOTDIR; 
			if (!inode->i_op)
				break;
		} else {
			dput(nd->dentry);
			nd->mnt = next.mnt;
			nd->dentry = next.dentry;
		}
		err = -ENOTDIR; 
		/**
		 * �ս����ķ����Ƿ�ָ��һ��Ŀ¼�����û�У��򷵻ش�����Ϊ�м�Ŀ¼����������һ��Ŀ¼��
		 */
		if (!inode->i_op->lookup)
			break;
		continue;
		/* here ends the main loop */

last_with_slashes:
		/**
		 * �ļ������һ��������"/"������LOOKUP_FOLLOW��LOOKUP_DIRECTORY��־������ĺ�����ǿ�ƽ����һ��������ΪĿ¼��
		 */
		lookup_flags |= LOOKUP_FOLLOW | LOOKUP_DIRECTORY;
last_component:
		/**
		 * ���һ�����������LOOKUP_CONTINUE
		 */
		nd->flags &= ~LOOKUP_CONTINUE;
		if (lookup_flags & LOOKUP_PARENT)/* ���Ҹ�·���� */
			goto lookup_parent;
		if (this.name[0] == '.') switch (this.len) {
			default:
				break;
			case 2:	
				if (this.name[1] != '.')/* ����".."��Ӧ�����������ļ��� */
					break;
				/* ".."�����Իص���Ŀ¼,�����ǰ���ļ�ϵͳ��Ŀ¼���Ҹ�Ŀ¼û�а�����Ŀ¼���򷵻ظ�Ŀ¼�����������ƶ� */
				follow_dotdot(&nd->mnt, &nd->dentry);
				inode = nd->dentry->d_inode;
				/* fallthrough */
			case 1:/* ���һ���ַ���"."���޴���ֱ�ӷ��ء����ؽ����dentry��mnt�ֶ�ָ��·�����е����ڶ���������Ӧ�Ķ��� */
				goto return_reval;
		}
		/**
		 * ���һ���������ļ���Ŀ¼��������ͼ���ļ�ϵͳ�Ĺ�ϣ�������¼����ϣֵ��
		 */
		if (nd->dentry->d_op && nd->dentry->d_op->d_hash) {
			err = nd->dentry->d_op->d_hash(nd->dentry, &this);
			if (err < 0)
				break;
		}
		/**
		 * ��Ŀ¼���ٻ�����ߴ����������ļ���
		 */
		err = do_lookup(nd, &this, &next);
		if (err)
			break;
		/**
		 * ������һ�������Ƿ�Ϊĳһ���ļ�ϵͳ�İ�װ�㡣
		 * ����ǣ����next��������Ϊ���ϲ��ļ�ϵͳ�ĸ�Ŀ¼��Ӧ��Ŀ¼�������Ѿ���װ�ļ�ϵͳ����ĵ�ַ��
		 */
		follow_mount(&next.mnt, &next.dentry);
		inode = next.dentry->d_inode;
		if ((lookup_flags & LOOKUP_FOLLOW)/* ��Ҫ������һ�������ķ������� */
		    && inode && inode->i_op && inode->i_op->follow_link) {/* ���Զ����follow_link���� */
			mntget(next.mnt);
			err = do_follow_link(next.dentry, nd);
			dput(next.dentry);
			mntput(next.mnt);
			if (err)
				goto return_err;
			inode = nd->dentry->d_inode;
		} else {/* ����Ҫ�������Ӳ��� */
			/**
			 * �ͷ�dentry�����ã���ΪҪ�������ҵķ�������Ϊ���ؽ����dentry
			 */
			dput(nd->dentry);
			nd->mnt = next.mnt;
			nd->dentry = next.dentry;
		}
		/**
		 * ������һ��������Ӧ��d_inode�Ƿ�Ϊ�գ�����ǣ���˵����Ӧ��Ŀ¼�����ڣ����ش���
		 */
		err = -ENOENT;
		if (!inode)
			break;
		/**
		 * LOOKUP_DIRECTORY��־Ҫ�����һ������������Ŀ¼
		 */
		if (lookup_flags & LOOKUP_DIRECTORY) {
			err = -ENOTDIR; 
			/**
			 * û��lookup˵���ⲻ��һ��Ŀ¼������ENOTDIR����
			 */
			if (!inode->i_op || !inode->i_op->lookup)
				break;
		}
		goto return_base;
lookup_parent:
		/**
		 * ��last����Ϊ���һ������
		 */
		nd->last = this;
		/**
		 * ������һ����������"."����".."����ô���һ������Ĭ�Ͼ���LAST_NORM
		 */
		nd->last_type = LAST_NORM;
		if (this.name[0] != '.')
			goto return_base;
		/**
		 * ��"."��".."�������⴦��
		 */
		if (this.len == 1)
			nd->last_type = LAST_DOT;
		else if (this.len == 2 && this.name[1] == '.')
			nd->last_type = LAST_DOTDOT;
		else
			goto return_base;/* ����û�ж����һ���������н��ͣ���˷��ص�dentry���Ǹ�Ŀ¼ */
return_reval:
		/*
		 * We bypassed the ordinary revalidation routines.
		 * We may need to check the cached dentry for staleness.
		 */
		if (nd->dentry && nd->dentry->d_sb &&
		    (nd->dentry->d_sb->s_type->fs_flags & FS_REVAL_DOT)) {
			err = -ESTALE;
			/* Note: we do not d_invalidate() */
			if (!nd->dentry->d_op->d_revalidate(nd->dentry, nd))
				break;
		}
return_base:
		return 0;
out_dput:
		dput(next.dentry);
		break;
	}
	path_release(nd);
return_err:
	return err;
}

int fastcall path_walk(const char * name, struct nameidata *nd)
{
	current->total_link_count = 0;
	return link_path_walk(name, nd);
}

/* SMP-safe */
/* returns 1 if everything is done */
static int __emul_lookup_dentry(const char *name, struct nameidata *nd)
{
	if (path_walk(name, nd))
		return 0;		/* something went wrong... */

	if (!nd->dentry->d_inode || S_ISDIR(nd->dentry->d_inode->i_mode)) {
		struct dentry *old_dentry = nd->dentry;
		struct vfsmount *old_mnt = nd->mnt;
		struct qstr last = nd->last;
		int last_type = nd->last_type;
		/*
		 * NAME was not found in alternate root or it's a directory.  Try to find
		 * it in the normal root:
		 */
		nd->last_type = LAST_ROOT;
		read_lock(&current->fs->lock);
		nd->mnt = mntget(current->fs->rootmnt);
		nd->dentry = dget(current->fs->root);
		read_unlock(&current->fs->lock);
		if (path_walk(name, nd) == 0) {
			if (nd->dentry->d_inode) {
				dput(old_dentry);
				mntput(old_mnt);
				return 1;
			}
			path_release(nd);
		}
		nd->dentry = old_dentry;
		nd->mnt = old_mnt;
		nd->last = last;
		nd->last_type = last_type;
	}
	return 1;
}

void set_fs_altroot(void)
{
	char *emul = __emul_prefix();
	struct nameidata nd;
	struct vfsmount *mnt = NULL, *oldmnt;
	struct dentry *dentry = NULL, *olddentry;
	int err;

	if (!emul)
		goto set_it;
	err = path_lookup(emul, LOOKUP_FOLLOW|LOOKUP_DIRECTORY|LOOKUP_NOALT, &nd);
	if (!err) {
		mnt = nd.mnt;
		dentry = nd.dentry;
	}
set_it:
	write_lock(&current->fs->lock);
	oldmnt = current->fs->altrootmnt;
	olddentry = current->fs->altroot;
	current->fs->altrootmnt = mnt;
	current->fs->altroot = dentry;
	write_unlock(&current->fs->lock);
	if (olddentry) {
		dput(olddentry);
		mntput(oldmnt);
	}
}

/**
 * ����·����
 *		name:Ҫ���ҵ��ļ�·����
 *		flags:��ʾ�������ʲ��ҵ��ļ���־����LOOKUP_FOLLOW
 *		nd:��Ų��ҵĽ����
 */
int fastcall path_lookup(const char *name, unsigned int flags, struct nameidata *nd)
{
	int retval;

	/**
	 * �ڽ�������ó�ֵ��
	 */
	nd->last_type = LAST_ROOT; /* if there are only slashes... */
	nd->flags = flags;
	nd->depth = 0;

	/**
	 * ��ȡfs��lock��������
	 */
	read_lock(&current->fs->lock);
	if (*name=='/') {/* �Ӹ�Ŀ¼��ʼ���� */
		if (current->fs->altroot && !(nd->flags & LOOKUP_NOALT)) {
			/**
			 * �Ӹ�Ŀ¼��ʼ���ҡ�
			 */
			nd->mnt = mntget(current->fs->altrootmnt);
			nd->dentry = dget(current->fs->altroot);
			read_unlock(&current->fs->lock);
			if (__emul_lookup_dentry(name,nd))
				return 0;
			read_lock(&current->fs->lock);
		}
		nd->mnt = mntget(current->fs->rootmnt);
		nd->dentry = dget(current->fs->root);
	} else {
		/**
		 * �ӵ�ǰĿ¼��ʼ���ҡ�
		 */
		nd->mnt = mntget(current->fs->pwdmnt);
		nd->dentry = dget(current->fs->pwd);
	}
	/**
	 * ��ʱ�Ѿ�����˳�ʼĿ¼������ã������ͷ��������ˡ�
	 */
	read_unlock(&current->fs->lock);
	/**
	 * ��ǰ���̵ķ������Ӳ��Ҽ������г�ʼ����
	 */
	current->total_link_count = 0;
	/**
	 * link_path_walkִ��������·�����ҡ�
	 */
	retval = link_path_walk(name, nd);
	if (unlikely(current->audit_context
		     && nd && nd->dentry && nd->dentry->d_inode))
		audit_inode(name,
			    nd->dentry->d_inode->i_ino,
			    nd->dentry->d_inode->i_rdev);
	return retval;
}

/*
 * Restricted form of lookup. Doesn't follow links, single-component only,
 * needs parent already locked. Doesn't follow mounts.
 * SMP-safe.
 */
static struct dentry * __lookup_hash(struct qstr *name, struct dentry * base, struct nameidata *nd)
{
	struct dentry * dentry;
	struct inode *inode;
	int err;

	inode = base->d_inode;
	err = permission(inode, MAY_EXEC, nd);
	dentry = ERR_PTR(err);
	if (err)
		goto out;

	/*
	 * See if the low-level filesystem might want
	 * to use its own hash..
	 */
	if (base->d_op && base->d_op->d_hash) {
		err = base->d_op->d_hash(base, name);
		dentry = ERR_PTR(err);
		if (err < 0)
			goto out;
	}

	dentry = cached_lookup(base, name, nd);
	if (!dentry) {
		struct dentry *new = d_alloc(base, name);
		dentry = ERR_PTR(-ENOMEM);
		if (!new)
			goto out;
		dentry = inode->i_op->lookup(inode, new, nd);
		if (!dentry)
			dentry = new;
		else
			dput(new);
	}
out:
	return dentry;
}

struct dentry * lookup_hash(struct qstr *name, struct dentry * base)
{
	return __lookup_hash(name, base, NULL);
}

/* SMP-safe */
struct dentry * lookup_one_len(const char * name, struct dentry * base, int len)
{
	unsigned long hash;
	struct qstr this;
	unsigned int c;

	this.name = name;
	this.len = len;
	if (!len)
		goto access;

	hash = init_name_hash();
	while (len--) {
		c = *(const unsigned char *)name++;
		if (c == '/' || c == '\0')
			goto access;
		hash = partial_name_hash(c, hash);
	}
	this.hash = end_name_hash(hash);

	return lookup_hash(&this, base);
access:
	return ERR_PTR(-EACCES);
}

/*
 *	namei()
 *
 * is used by most simple commands to get the inode of a specified name.
 * Open, link etc use their own routines, but this is enough for things
 * like 'chmod' etc.
 *
 * namei exists in two versions: namei/lnamei. The only difference is
 * that namei follows links, while lnamei does not.
 * SMP-safe
 */
int fastcall __user_walk(const char __user *name, unsigned flags, struct nameidata *nd)
{
	char *tmp = getname(name);
	int err = PTR_ERR(tmp);

	if (!IS_ERR(tmp)) {
		err = path_lookup(tmp, flags, nd);
		putname(tmp);
	}
	return err;
}

/*
 * It's inline, so penalty for filesystems that don't use sticky bit is
 * minimal.
 */
static inline int check_sticky(struct inode *dir, struct inode *inode)
{
	if (!(dir->i_mode & S_ISVTX))
		return 0;
	if (inode->i_uid == current->fsuid)
		return 0;
	if (dir->i_uid == current->fsuid)
		return 0;
	return !capable(CAP_FOWNER);
}

/*
 *	Check whether we can remove a link victim from directory dir, check
 *  whether the type of victim is right.
 *  1. We can't do it if dir is read-only (done in permission())
 *  2. We should have write and exec permissions on dir
 *  3. We can't remove anything from append-only dir
 *  4. We can't do anything with immutable dir (done in permission())
 *  5. If the sticky bit on dir is set we should either
 *	a. be owner of dir, or
 *	b. be owner of victim, or
 *	c. have CAP_FOWNER capability
 *  6. If the victim is append-only or immutable we can't do antyhing with
 *     links pointing to it.
 *  7. If we were asked to remove a directory and victim isn't one - ENOTDIR.
 *  8. If we were asked to remove a non-directory and victim isn't one - EISDIR.
 *  9. We can't remove a root or mountpoint.
 * 10. We don't allow removal of NFS sillyrenamed files; it's handled by
 *     nfs_async_unlink().
 */
static inline int may_delete(struct inode *dir,struct dentry *victim,int isdir)
{
	int error;

	if (!victim->d_inode)
		return -ENOENT;

	BUG_ON(victim->d_parent->d_inode != dir);

	error = permission(dir,MAY_WRITE | MAY_EXEC, NULL);
	if (error)
		return error;
	if (IS_APPEND(dir))
		return -EPERM;
	if (check_sticky(dir, victim->d_inode)||IS_APPEND(victim->d_inode)||
	    IS_IMMUTABLE(victim->d_inode))
		return -EPERM;
	if (isdir) {
		if (!S_ISDIR(victim->d_inode->i_mode))
			return -ENOTDIR;
		if (IS_ROOT(victim))
			return -EBUSY;
	} else if (S_ISDIR(victim->d_inode->i_mode))
		return -EISDIR;
	if (IS_DEADDIR(dir))
		return -ENOENT;
	if (victim->d_flags & DCACHE_NFSFS_RENAMED)
		return -EBUSY;
	return 0;
}

/*	Check whether we can create an object with dentry child in directory
 *  dir.
 *  1. We can't do it if child already exists (open has special treatment for
 *     this case, but since we are inlined it's OK)
 *  2. We can't do it if dir is read-only (done in permission())
 *  3. We should have write and exec permissions on dir
 *  4. We can't do it if dir is immutable (done in permission())
 */
static inline int may_create(struct inode *dir, struct dentry *child,
			     struct nameidata *nd)
{
	if (child->d_inode)
		return -EEXIST;
	if (IS_DEADDIR(dir))
		return -ENOENT;
	return permission(dir,MAY_WRITE | MAY_EXEC, nd);
}

/* 
 * Special case: O_CREAT|O_EXCL implies O_NOFOLLOW for security
 * reasons.
 *
 * O_DIRECTORY translates into forcing a directory lookup.
 */
static inline int lookup_flags(unsigned int f)
{
	unsigned long retval = LOOKUP_FOLLOW;

	if (f & O_NOFOLLOW)
		retval &= ~LOOKUP_FOLLOW;
	
	if ((f & (O_CREAT|O_EXCL)) == (O_CREAT|O_EXCL))
		retval &= ~LOOKUP_FOLLOW;
	
	if (f & O_DIRECTORY)
		retval |= LOOKUP_DIRECTORY;

	return retval;
}

/*
 * p1 and p2 should be directories on the same fs.
 */
struct dentry *lock_rename(struct dentry *p1, struct dentry *p2)
{
	struct dentry *p;

	if (p1 == p2) {
		down(&p1->d_inode->i_sem);
		return NULL;
	}

	down(&p1->d_inode->i_sb->s_vfs_rename_sem);

	for (p = p1; p->d_parent != p; p = p->d_parent) {
		if (p->d_parent == p2) {
			down(&p2->d_inode->i_sem);
			down(&p1->d_inode->i_sem);
			return p;
		}
	}

	for (p = p2; p->d_parent != p; p = p->d_parent) {
		if (p->d_parent == p1) {
			down(&p1->d_inode->i_sem);
			down(&p2->d_inode->i_sem);
			return p;
		}
	}

	down(&p1->d_inode->i_sem);
	down(&p2->d_inode->i_sem);
	return NULL;
}

void unlock_rename(struct dentry *p1, struct dentry *p2)
{
	up(&p1->d_inode->i_sem);
	if (p1 != p2) {
		up(&p2->d_inode->i_sem);
		up(&p1->d_inode->i_sb->s_vfs_rename_sem);
	}
}

int vfs_create(struct inode *dir, struct dentry *dentry, int mode,
		struct nameidata *nd)
{
	int error = may_create(dir, dentry, nd);

	if (error)
		return error;

	if (!dir->i_op || !dir->i_op->create)
		return -EACCES;	/* shouldn't it be ENOSYS? */
	mode &= S_IALLUGO;
	mode |= S_IFREG;
	error = security_inode_create(dir, dentry, mode);
	if (error)
		return error;
	DQUOT_INIT(dir);
	error = dir->i_op->create(dir, dentry, mode, nd);
	if (!error) {
		inode_dir_notify(dir, DN_CREATE);
		security_inode_post_create(dir, dentry, mode);
	}
	return error;
}

int may_open(struct nameidata *nd, int acc_mode, int flag)
{
	struct dentry *dentry = nd->dentry;
	struct inode *inode = dentry->d_inode;
	int error;

	if (!inode)
		return -ENOENT;

	if (S_ISLNK(inode->i_mode))
		return -ELOOP;
	
	if (S_ISDIR(inode->i_mode) && (flag & FMODE_WRITE))
		return -EISDIR;

	error = permission(inode, acc_mode, nd);
	if (error)
		return error;

	/*
	 * FIFO's, sockets and device files are special: they don't
	 * actually live on the filesystem itself, and as such you
	 * can write to them even if the filesystem is read-only.
	 */
	if (S_ISFIFO(inode->i_mode) || S_ISSOCK(inode->i_mode)) {
	    	flag &= ~O_TRUNC;
	} else if (S_ISBLK(inode->i_mode) || S_ISCHR(inode->i_mode)) {
		if (nd->mnt->mnt_flags & MNT_NODEV)
			return -EACCES;

		flag &= ~O_TRUNC;
	} else if (IS_RDONLY(inode) && (flag & FMODE_WRITE))
		return -EROFS;
	/*
	 * An append-only file must be opened in append mode for writing.
	 */
	if (IS_APPEND(inode)) {
		if  ((flag & FMODE_WRITE) && !(flag & O_APPEND))
			return -EPERM;
		if (flag & O_TRUNC)
			return -EPERM;
	}

	/* O_NOATIME can only be set by the owner or superuser */
	if (flag & O_NOATIME)
		if (current->fsuid != inode->i_uid && !capable(CAP_FOWNER))
			return -EPERM;

	/*
	 * Ensure there are no outstanding leases on the file.
	 */
	error = break_lease(inode, flag);
	if (error)
		return error;

	if (flag & O_TRUNC) {
		error = get_write_access(inode);
		if (error)
			return error;

		/*
		 * Refuse to truncate files with mandatory locks held on them.
		 */
		error = locks_verify_locked(inode);
		if (!error) {
			DQUOT_INIT(inode);
			
			error = do_truncate(dentry, 0);
		}
		put_write_access(inode);
		if (error)
			return error;
	} else
		if (flag & FMODE_WRITE)
			DQUOT_INIT(inode);

	return 0;
}

/*
 *	open_namei()
 *
 * namei for open - this is in fact almost the whole open-routine.
 *
 * Note that the low bits of "flag" aren't the same as in the open
 * system call - they are 00 - no permissions needed
 *			  01 - read permission needed
 *			  10 - write permission needed
 *			  11 - read/write permissions needed
 * which is a lot more logical, and also allows the "no perm" needed
 * for symlinks (where the permissions are checked later).
 * SMP-safe
 */
int open_namei(const char * pathname, int flag, int mode, struct nameidata *nd)
{
	int acc_mode, error = 0;
	struct dentry *dentry;
	struct dentry *dir;
	int count = 0;

	acc_mode = ACC_MODE(flag);

	/* Allow the LSM permission hook to distinguish append 
	   access from general write access. */
	if (flag & O_APPEND)
		acc_mode |= MAY_APPEND;

	/* Fill in the open() intent data */
	nd->intent.open.flags = flag;
	nd->intent.open.create_mode = mode;

	/*
	 * The simplest case - just a plain lookup.
	 */
	if (!(flag & O_CREAT)) {/* ����������ļ�������LOOKUP_OPEN��־�����ļ� */
		error = path_lookup(pathname, lookup_flags(flag)|LOOKUP_OPEN, nd);
		if (error)
			return error;
		goto ok;
	}

	/*
	 * Create - we need to know the parent.
	 */
	/**
	 * ��O_CREAT��־������LOOKUP_CREATE��־�����ļ���
	 */
	error = path_lookup(pathname, LOOKUP_PARENT|LOOKUP_OPEN|LOOKUP_CREATE, nd);
	if (error)
		return error;

	/*
	 * We have the parent and last component. First of all, check
	 * that we are not asked to creat(2) an obvious directory - that
	 * will not do.
	 */
	error = -EISDIR;
	/**
	 * ���һ��������Ŀ¼��ʧ��
	 */
	if (nd->last_type != LAST_NORM || nd->last.name[nd->last.len])
		goto exit;

	dir = nd->dentry;
	nd->flags &= ~LOOKUP_PARENT;
	down(&dir->d_inode->i_sem);
	dentry = __lookup_hash(&nd->last, nd->dentry, nd);

do_last:
	error = PTR_ERR(dentry);
	if (IS_ERR(dentry)) {
		up(&dir->d_inode->i_sem);
		goto exit;
	}

	/* Negative dentry, just create the file */
	if (!dentry->d_inode) {/* �ļ��������� */
		if (!IS_POSIXACL(dir->d_inode))
			mode &= ~current->fs->umask;
		/**
		 * ����vfs_create�������ļ�
		 */
		error = vfs_create(dir->d_inode, dentry, mode, nd);
		up(&dir->d_inode->i_sem);
		dput(nd->dentry);
		nd->dentry = dentry;
		if (error)
			goto exit;
		/* Don't check for write permission, don't truncate */
		acc_mode = 0;
		flag &= ~O_TRUNC;
		goto ok;
	}

	/*
	 * It already exists.
	 */
	up(&dir->d_inode->i_sem);

	error = -EEXIST;
	if (flag & O_EXCL)/* �ļ��Ѿ����ڣ����ָ����O_EXCL��־������Ҫ���ش��� */
		goto exit_dput;

	if (d_mountpoint(dentry)) {
		error = -ELOOP;
		if (flag & O_NOFOLLOW)
			goto exit_dput;
		while (__follow_down(&nd->mnt,&dentry) && d_mountpoint(dentry));
	}
	error = -ENOENT;
	if (!dentry->d_inode)
		goto exit_dput;
	if (dentry->d_inode->i_op && dentry->d_inode->i_op->follow_link)
		goto do_link;

	dput(nd->dentry);
	nd->dentry = dentry;
	error = -EISDIR;
	if (dentry->d_inode && S_ISDIR(dentry->d_inode->i_mode))
		goto exit;
ok:
	error = may_open(nd, acc_mode, flag);
	if (error)
		goto exit;
	return 0;

exit_dput:
	dput(dentry);
exit:
	path_release(nd);
	return error;

do_link:
	error = -ELOOP;
	if (flag & O_NOFOLLOW)
		goto exit_dput;
	/*
	 * This is subtle. Instead of calling do_follow_link() we do the
	 * thing by hands. The reason is that this way we have zero link_count
	 * and path_walk() (called from ->follow_link) honoring LOOKUP_PARENT.
	 * After that we have the parent and last component, i.e.
	 * we are in the same situation as after the first path_walk().
	 * Well, almost - if the last component is normal we get its copy
	 * stored in nd->last.name and we will have to putname() it when we
	 * are done. Procfs-like symlinks just set LAST_BIND.
	 */
	nd->flags |= LOOKUP_PARENT;
	error = security_inode_follow_link(dentry, nd);
	if (error)
		goto exit_dput;
	error = __do_follow_link(dentry, nd);
	dput(dentry);
	if (error)
		return error;
	nd->flags &= ~LOOKUP_PARENT;
	if (nd->last_type == LAST_BIND) {
		dentry = nd->dentry;
		goto ok;
	}
	error = -EISDIR;
	if (nd->last_type != LAST_NORM)
		goto exit;
	if (nd->last.name[nd->last.len]) {
		putname(nd->last.name);
		goto exit;
	}
	error = -ELOOP;
	if (count++==32) {
		putname(nd->last.name);
		goto exit;
	}
	dir = nd->dentry;
	down(&dir->d_inode->i_sem);
	dentry = __lookup_hash(&nd->last, nd->dentry, nd);
	putname(nd->last.name);
	goto do_last;
}

/**
 * lookup_create - lookup a dentry, creating it if it doesn't exist
 * @nd: nameidata info
 * @is_dir: directory flag
 *
 * Simple function to lookup and return a dentry and create it
 * if it doesn't exist.  Is SMP-safe.
 */
struct dentry *lookup_create(struct nameidata *nd, int is_dir)
{
	struct dentry *dentry;

	down(&nd->dentry->d_inode->i_sem);
	dentry = ERR_PTR(-EEXIST);
	if (nd->last_type != LAST_NORM)
		goto fail;
	nd->flags &= ~LOOKUP_PARENT;
	dentry = lookup_hash(&nd->last, nd->dentry);
	if (IS_ERR(dentry))
		goto fail;
	if (!is_dir && nd->last.name[nd->last.len] && !dentry->d_inode)
		goto enoent;
	return dentry;
enoent:
	dput(dentry);
	dentry = ERR_PTR(-ENOENT);
fail:
	return dentry;
}

int vfs_mknod(struct inode *dir, struct dentry *dentry, int mode, dev_t dev)
{
	int error = may_create(dir, dentry, NULL);

	if (error)
		return error;

	if ((S_ISCHR(mode) || S_ISBLK(mode)) && !capable(CAP_MKNOD))
		return -EPERM;

	if (!dir->i_op || !dir->i_op->mknod)
		return -EPERM;

	error = security_inode_mknod(dir, dentry, mode, dev);
	if (error)
		return error;

	DQUOT_INIT(dir);
	error = dir->i_op->mknod(dir, dentry, mode, dev);
	if (!error) {
		inode_dir_notify(dir, DN_CREATE);
		security_inode_post_mknod(dir, dentry, mode, dev);
	}
	return error;
}

/** 
 * �����豸�ļ����豸�ļ�ͨ��λ��/devĿ¼��
 */
asmlinkage long sys_mknod(const char __user * filename, int mode, unsigned dev)
{
	int error = 0;
	char * tmp;
	struct dentry * dentry;
	struct nameidata nd;

	if (S_ISDIR(mode))
		return -EPERM;
	tmp = getname(filename);
	if (IS_ERR(tmp))
		return PTR_ERR(tmp);

	error = path_lookup(tmp, LOOKUP_PARENT, &nd);
	if (error)
		goto out;
	dentry = lookup_create(&nd, 0);
	error = PTR_ERR(dentry);

	if (!IS_POSIXACL(nd.dentry->d_inode))
		mode &= ~current->fs->umask;
	if (!IS_ERR(dentry)) {
		switch (mode & S_IFMT) {
		case 0: case S_IFREG:
			error = vfs_create(nd.dentry->d_inode,dentry,mode,&nd);
			break;
		case S_IFCHR: case S_IFBLK:
			error = vfs_mknod(nd.dentry->d_inode,dentry,mode,
					new_decode_dev(dev));
			break;
		case S_IFIFO: case S_IFSOCK:
			error = vfs_mknod(nd.dentry->d_inode,dentry,mode,0);
			break;
		case S_IFDIR:
			error = -EPERM;
			break;
		default:
			error = -EINVAL;
		}
		dput(dentry);
	}
	up(&nd.dentry->d_inode->i_sem);
	path_release(&nd);
out:
	putname(tmp);

	return error;
}

int vfs_mkdir(struct inode *dir, struct dentry *dentry, int mode)
{
	int error = may_create(dir, dentry, NULL);

	if (error)
		return error;

	if (!dir->i_op || !dir->i_op->mkdir)
		return -EPERM;

	mode &= (S_IRWXUGO|S_ISVTX);
	error = security_inode_mkdir(dir, dentry, mode);
	if (error)
		return error;

	DQUOT_INIT(dir);
	error = dir->i_op->mkdir(dir, dentry, mode);
	if (!error) {
		inode_dir_notify(dir, DN_CREATE);
		security_inode_post_mkdir(dir,dentry, mode);
	}
	return error;
}

asmlinkage long sys_mkdir(const char __user * pathname, int mode)
{
	int error = 0;
	char * tmp;

	tmp = getname(pathname);
	error = PTR_ERR(tmp);
	if (!IS_ERR(tmp)) {
		struct dentry *dentry;
		struct nameidata nd;

		error = path_lookup(tmp, LOOKUP_PARENT, &nd);
		if (error)
			goto out;
		dentry = lookup_create(&nd, 1);
		error = PTR_ERR(dentry);
		if (!IS_ERR(dentry)) {
			if (!IS_POSIXACL(nd.dentry->d_inode))
				mode &= ~current->fs->umask;
			error = vfs_mkdir(nd.dentry->d_inode, dentry, mode);
			dput(dentry);
		}
		up(&nd.dentry->d_inode->i_sem);
		path_release(&nd);
out:
		putname(tmp);
	}

	return error;
}

/*
 * We try to drop the dentry early: we should have
 * a usage count of 2 if we're the only user of this
 * dentry, and if that is true (possibly after pruning
 * the dcache), then we drop the dentry now.
 *
 * A low-level filesystem can, if it choses, legally
 * do a
 *
 *	if (!d_unhashed(dentry))
 *		return -EBUSY;
 *
 * if it cannot handle the case of removing a directory
 * that is still in use by something else..
 */
void dentry_unhash(struct dentry *dentry)
{
	dget(dentry);
	spin_lock(&dcache_lock);
	switch (atomic_read(&dentry->d_count)) {
	default:
		spin_unlock(&dcache_lock);
		shrink_dcache_parent(dentry);
		spin_lock(&dcache_lock);
		if (atomic_read(&dentry->d_count) != 2)
			break;
	case 2:
		__d_drop(dentry);
	}
	spin_unlock(&dcache_lock);
}

int vfs_rmdir(struct inode *dir, struct dentry *dentry)
{
	int error = may_delete(dir, dentry, 1);

	if (error)
		return error;

	if (!dir->i_op || !dir->i_op->rmdir)
		return -EPERM;

	DQUOT_INIT(dir);

	down(&dentry->d_inode->i_sem);
	dentry_unhash(dentry);
	if (d_mountpoint(dentry))
		error = -EBUSY;
	else {
		error = security_inode_rmdir(dir, dentry);
		if (!error) {
			error = dir->i_op->rmdir(dir, dentry);
			if (!error)
				dentry->d_inode->i_flags |= S_DEAD;
		}
	}
	up(&dentry->d_inode->i_sem);
	if (!error) {
		inode_dir_notify(dir, DN_DELETE);
		d_delete(dentry);
	}
	dput(dentry);

	return error;
}

asmlinkage long sys_rmdir(const char __user * pathname)
{
	int error = 0;
	char * name;
	struct dentry *dentry;
	struct nameidata nd;

	name = getname(pathname);
	if(IS_ERR(name))
		return PTR_ERR(name);

	error = path_lookup(name, LOOKUP_PARENT, &nd);
	if (error)
		goto exit;

	switch(nd.last_type) {
		case LAST_DOTDOT:
			error = -ENOTEMPTY;
			goto exit1;
		case LAST_DOT:
			error = -EINVAL;
			goto exit1;
		case LAST_ROOT:
			error = -EBUSY;
			goto exit1;
	}
	down(&nd.dentry->d_inode->i_sem);
	dentry = lookup_hash(&nd.last, nd.dentry);
	error = PTR_ERR(dentry);
	if (!IS_ERR(dentry)) {
		error = vfs_rmdir(nd.dentry->d_inode, dentry);
		dput(dentry);
	}
	up(&nd.dentry->d_inode->i_sem);
exit1:
	path_release(&nd);
exit:
	putname(name);
	return error;
}

int vfs_unlink(struct inode *dir, struct dentry *dentry)
{
	int error = may_delete(dir, dentry, 0);

	if (error)
		return error;

	if (!dir->i_op || !dir->i_op->unlink)
		return -EPERM;

	DQUOT_INIT(dir);

	down(&dentry->d_inode->i_sem);
	if (d_mountpoint(dentry))
		error = -EBUSY;
	else {
		error = security_inode_unlink(dir, dentry);
		if (!error)
			error = dir->i_op->unlink(dir, dentry);
	}
	up(&dentry->d_inode->i_sem);

	/* We don't d_delete() NFS sillyrenamed files--they still exist. */
	if (!error && !(dentry->d_flags & DCACHE_NFSFS_RENAMED)) {
		d_delete(dentry);
		inode_dir_notify(dir, DN_DELETE);
	}
	return error;
}

/*
 * Make sure that the actual truncation of the file will occur outside its
 * directory's i_sem.  Truncate can take a long time if there is a lot of
 * writeout happening, and we don't want to prevent access to the directory
 * while waiting on the I/O.
 * 
 * 确保文件的实际截断将在其目录的 i_sem 外部进行。由于如果有大量写入操作，
 * 截断可能会耗费很长时间，我们不想在等待 I/O 时阻止对目录的访问。
 */
asmlinkage long sys_unlink(const char __user * pathname)
{
	int error = 0;
	char * name;
	struct dentry *dentry;
	struct nameidata nd;
	struct inode *inode = NULL;

	name = getname(pathname);
	if(IS_ERR(name))
		return PTR_ERR(name);

	error = path_lookup(name, LOOKUP_PARENT, &nd);
	if (error)
		goto exit;
	error = -EISDIR;
	if (nd.last_type != LAST_NORM)
		goto exit1;
	down(&nd.dentry->d_inode->i_sem);
	dentry = lookup_hash(&nd.last, nd.dentry);
	error = PTR_ERR(dentry);
	if (!IS_ERR(dentry)) {
		/* Why not before? Because we want correct error value */
		if (nd.last.name[nd.last.len])
			goto slashes;
		inode = dentry->d_inode;
		if (inode)
			atomic_inc(&inode->i_count);
		error = vfs_unlink(nd.dentry->d_inode, dentry);
	exit2:
		dput(dentry);
	}
	up(&nd.dentry->d_inode->i_sem);
	if (inode)
		iput(inode);	/* truncate the inode here */
exit1:
	path_release(&nd);
exit:
	putname(name);
	return error;

slashes:
	error = !dentry->d_inode ? -ENOENT :
		S_ISDIR(dentry->d_inode->i_mode) ? -EISDIR : -ENOTDIR;
	goto exit2;
}

int vfs_symlink(struct inode *dir, struct dentry *dentry, const char *oldname, int mode)
{
	int error = may_create(dir, dentry, NULL);

	if (error)
		return error;

	if (!dir->i_op || !dir->i_op->symlink)
		return -EPERM;

	error = security_inode_symlink(dir, dentry, oldname);
	if (error)
		return error;

	DQUOT_INIT(dir);
	error = dir->i_op->symlink(dir, dentry, oldname);
	if (!error) {
		inode_dir_notify(dir, DN_CREATE);
		security_inode_post_symlink(dir, dentry, oldname);
	}
	return error;
}

asmlinkage long sys_symlink(const char __user * oldname, const char __user * newname)
{
	int error = 0;
	char * from;
	char * to;

	from = getname(oldname);
	if(IS_ERR(from))
		return PTR_ERR(from);
	to = getname(newname);
	error = PTR_ERR(to);
	if (!IS_ERR(to)) {
		struct dentry *dentry;
		struct nameidata nd;

		error = path_lookup(to, LOOKUP_PARENT, &nd);
		if (error)
			goto out;
		dentry = lookup_create(&nd, 0);
		error = PTR_ERR(dentry);
		if (!IS_ERR(dentry)) {
			error = vfs_symlink(nd.dentry->d_inode, dentry, from, S_IALLUGO);
			dput(dentry);
		}
		up(&nd.dentry->d_inode->i_sem);
		path_release(&nd);
out:
		putname(to);
	}
	putname(from);
	return error;
}

int vfs_link(struct dentry *old_dentry, struct inode *dir, struct dentry *new_dentry)
{
	struct inode *inode = old_dentry->d_inode;
	int error;

	if (!inode)
		return -ENOENT;

	error = may_create(dir, new_dentry, NULL);
	if (error)
		return error;

	if (dir->i_sb != inode->i_sb)
		return -EXDEV;

	/*
	 * A link to an append-only or immutable file cannot be created.
	 */
	if (IS_APPEND(inode) || IS_IMMUTABLE(inode))
		return -EPERM;
	if (!dir->i_op || !dir->i_op->link)
		return -EPERM;
	if (S_ISDIR(old_dentry->d_inode->i_mode))
		return -EPERM;

	error = security_inode_link(old_dentry, dir, new_dentry);
	if (error)
		return error;

	down(&old_dentry->d_inode->i_sem);
	DQUOT_INIT(dir);
	error = dir->i_op->link(old_dentry, dir, new_dentry);
	up(&old_dentry->d_inode->i_sem);
	if (!error) {
		inode_dir_notify(dir, DN_CREATE);
		security_inode_post_link(old_dentry, dir, new_dentry);
	}
	return error;
}

/*
 * Hardlinks are often used in delicate situations.  We avoid
 * security-related surprises by not following symlinks on the
 * newname.  --KAB
 *
 * We don't follow them on the oldname either to be compatible
 * with linux 2.0, and to avoid hard-linking to directories
 * and other special files.  --ADM
 */
asmlinkage long sys_link(const char __user * oldname, const char __user * newname)
{
	struct dentry *new_dentry;
	struct nameidata nd, old_nd;
	int error;
	char * to;

	to = getname(newname);
	if (IS_ERR(to))
		return PTR_ERR(to);

	error = __user_walk(oldname, 0, &old_nd);
	if (error)
		goto exit;
	error = path_lookup(to, LOOKUP_PARENT, &nd);
	if (error)
		goto out;
	error = -EXDEV;
	if (old_nd.mnt != nd.mnt)
		goto out_release;
	new_dentry = lookup_create(&nd, 0);
	error = PTR_ERR(new_dentry);
	if (!IS_ERR(new_dentry)) {
		error = vfs_link(old_nd.dentry, nd.dentry->d_inode, new_dentry);
		dput(new_dentry);
	}
	up(&nd.dentry->d_inode->i_sem);
out_release:
	path_release(&nd);
out:
	path_release(&old_nd);
exit:
	putname(to);

	return error;
}

/*
 * The worst of all namespace operations - renaming directory. "Perverted"
 * doesn't even start to describe it. Somebody in UCB had a heck of a trip...
 * Problems:
 *	a) we can get into loop creation. Check is done in is_subdir().
 *	b) race potential - two innocent renames can create a loop together.
 *	   That's where 4.4 screws up. Current fix: serialization on
 *	   sb->s_vfs_rename_sem. We might be more accurate, but that's another
 *	   story.
 *	c) we have to lock _three_ objects - parents and victim (if it exists).
 *	   And that - after we got ->i_sem on parents (until then we don't know
 *	   whether the target exists).  Solution: try to be smart with locking
 *	   order for inodes.  We rely on the fact that tree topology may change
 *	   only under ->s_vfs_rename_sem _and_ that parent of the object we
 *	   move will be locked.  Thus we can rank directories by the tree
 *	   (ancestors first) and rank all non-directories after them.
 *	   That works since everybody except rename does "lock parent, lookup,
 *	   lock child" and rename is under ->s_vfs_rename_sem.
 *	   HOWEVER, it relies on the assumption that any object with ->lookup()
 *	   has no more than 1 dentry.  If "hybrid" objects will ever appear,
 *	   we'd better make sure that there's no link(2) for them.
 *	d) some filesystems don't support opened-but-unlinked directories,
 *	   either because of layout or because they are not ready to deal with
 *	   all cases correctly. The latter will be fixed (taking this sort of
 *	   stuff into VFS), but the former is not going away. Solution: the same
 *	   trick as in rmdir().
 *	e) conversion from fhandle to dentry may come in the wrong moment - when
 *	   we are removing the target. Solution: we will have to grab ->i_sem
 *	   in the fhandle_to_dentry code. [FIXME - current nfsfh.c relies on
 *	   ->i_sem on parents, which works but leads to some truely excessive
 *	   locking].
 */
int vfs_rename_dir(struct inode *old_dir, struct dentry *old_dentry,
	       struct inode *new_dir, struct dentry *new_dentry)
{
	int error = 0;
	struct inode *target;

	/*
	 * If we are going to change the parent - check write permissions,
	 * we'll need to flip '..'.
	 */
	if (new_dir != old_dir) {
		error = permission(old_dentry->d_inode, MAY_WRITE, NULL);
		if (error)
			return error;
	}

	error = security_inode_rename(old_dir, old_dentry, new_dir, new_dentry);
	if (error)
		return error;

	target = new_dentry->d_inode;
	if (target) {
		down(&target->i_sem);
		dentry_unhash(new_dentry);
	}
	if (d_mountpoint(old_dentry)||d_mountpoint(new_dentry))
		error = -EBUSY;
	else 
		error = old_dir->i_op->rename(old_dir, old_dentry, new_dir, new_dentry);
	if (target) {
		if (!error)
			target->i_flags |= S_DEAD;
		up(&target->i_sem);
		if (d_unhashed(new_dentry))
			d_rehash(new_dentry);
		dput(new_dentry);
	}
	if (!error) {
		d_move(old_dentry,new_dentry);
		security_inode_post_rename(old_dir, old_dentry,
					   new_dir, new_dentry);
	}
	return error;
}

int vfs_rename_other(struct inode *old_dir, struct dentry *old_dentry,
	       struct inode *new_dir, struct dentry *new_dentry)
{
	struct inode *target;
	int error;

	error = security_inode_rename(old_dir, old_dentry, new_dir, new_dentry);
	if (error)
		return error;

	dget(new_dentry);
	target = new_dentry->d_inode;
	if (target)
		down(&target->i_sem);
	if (d_mountpoint(old_dentry)||d_mountpoint(new_dentry))
		error = -EBUSY;
	else
		error = old_dir->i_op->rename(old_dir, old_dentry, new_dir, new_dentry);
	if (!error) {
		/* The following d_move() should become unconditional */
		if (!(old_dir->i_sb->s_type->fs_flags & FS_ODD_RENAME))
			d_move(old_dentry, new_dentry);
		security_inode_post_rename(old_dir, old_dentry, new_dir, new_dentry);
	}
	if (target)
		up(&target->i_sem);
	dput(new_dentry);
	return error;
}

int vfs_rename(struct inode *old_dir, struct dentry *old_dentry,
	       struct inode *new_dir, struct dentry *new_dentry)
{
	int error;
	int is_dir = S_ISDIR(old_dentry->d_inode->i_mode);

	if (old_dentry->d_inode == new_dentry->d_inode)
 		return 0;
 
	error = may_delete(old_dir, old_dentry, is_dir);
	if (error)
		return error;

	if (!new_dentry->d_inode)
		error = may_create(new_dir, new_dentry, NULL);
	else
		error = may_delete(new_dir, new_dentry, is_dir);
	if (error)
		return error;

	if (!old_dir->i_op || !old_dir->i_op->rename)
		return -EPERM;

	DQUOT_INIT(old_dir);
	DQUOT_INIT(new_dir);

	if (is_dir)
		error = vfs_rename_dir(old_dir,old_dentry,new_dir,new_dentry);
	else
		error = vfs_rename_other(old_dir,old_dentry,new_dir,new_dentry);
	if (!error) {
		if (old_dir == new_dir)
			inode_dir_notify(old_dir, DN_RENAME);
		else {
			inode_dir_notify(old_dir, DN_DELETE);
			inode_dir_notify(new_dir, DN_CREATE);
		}
	}
	return error;
}

static inline int do_rename(const char * oldname, const char * newname)
{
	int error = 0;
	struct dentry * old_dir, * new_dir;
	struct dentry * old_dentry, *new_dentry;
	struct dentry * trap;
	struct nameidata oldnd, newnd;

	error = path_lookup(oldname, LOOKUP_PARENT, &oldnd);
	if (error)
		goto exit;

	error = path_lookup(newname, LOOKUP_PARENT, &newnd);
	if (error)
		goto exit1;

	error = -EXDEV;
	if (oldnd.mnt != newnd.mnt)
		goto exit2;

	old_dir = oldnd.dentry;
	error = -EBUSY;
	if (oldnd.last_type != LAST_NORM)
		goto exit2;

	new_dir = newnd.dentry;
	if (newnd.last_type != LAST_NORM)
		goto exit2;

	trap = lock_rename(new_dir, old_dir);

	old_dentry = lookup_hash(&oldnd.last, old_dir);
	error = PTR_ERR(old_dentry);
	if (IS_ERR(old_dentry))
		goto exit3;
	/* source must exist */
	error = -ENOENT;
	if (!old_dentry->d_inode)
		goto exit4;
	/* unless the source is a directory trailing slashes give -ENOTDIR */
	if (!S_ISDIR(old_dentry->d_inode->i_mode)) {
		error = -ENOTDIR;
		if (oldnd.last.name[oldnd.last.len])
			goto exit4;
		if (newnd.last.name[newnd.last.len])
			goto exit4;
	}
	/* source should not be ancestor of target */
	error = -EINVAL;
	if (old_dentry == trap)
		goto exit4;
	new_dentry = lookup_hash(&newnd.last, new_dir);
	error = PTR_ERR(new_dentry);
	if (IS_ERR(new_dentry))
		goto exit4;
	/* target should not be an ancestor of source */
	error = -ENOTEMPTY;
	if (new_dentry == trap)
		goto exit5;

	error = vfs_rename(old_dir->d_inode, old_dentry,
				   new_dir->d_inode, new_dentry);
exit5:
	dput(new_dentry);
exit4:
	dput(old_dentry);
exit3:
	unlock_rename(new_dir, old_dir);
exit2:
	path_release(&newnd);
exit1:
	path_release(&oldnd);
exit:
	return error;
}

asmlinkage long sys_rename(const char __user * oldname, const char __user * newname)
{
	int error;
	char * from;
	char * to;

	from = getname(oldname);
	if(IS_ERR(from))
		return PTR_ERR(from);
	to = getname(newname);
	error = PTR_ERR(to);
	if (!IS_ERR(to)) {
		error = do_rename(from,to);
		putname(to);
	}
	putname(from);
	return error;
}

int vfs_readlink(struct dentry *dentry, char __user *buffer, int buflen, const char *link)
{
	int len;

	len = PTR_ERR(link);
	if (IS_ERR(link))
		goto out;

	len = strlen(link);
	if (len > (unsigned) buflen)
		len = buflen;
	if (copy_to_user(buffer, link, len))
		len = -EFAULT;
out:
	return len;
}

/*
 * A helper for ->readlink().  This should be used *ONLY* for symlinks that
 * have ->follow_link() touching nd only in nd_set_link().  Using (or not
 * using) it for any given inode is up to filesystem.
 */
int generic_readlink(struct dentry *dentry, char __user *buffer, int buflen)
{
	struct nameidata nd;
	int res;
	nd.depth = 0;
	res = dentry->d_inode->i_op->follow_link(dentry, &nd);
	if (!res) {
		res = vfs_readlink(dentry, buffer, buflen, nd_get_link(&nd));
		if (dentry->d_inode->i_op->put_link)
			dentry->d_inode->i_op->put_link(dentry, &nd);
	}
	return res;
}

int vfs_follow_link(struct nameidata *nd, const char *link)
{
	return __vfs_follow_link(nd, link);
}

/* get the link contents into pagecache */
static char *page_getlink(struct dentry * dentry, struct page **ppage)
{
	struct page * page;
	struct address_space *mapping = dentry->d_inode->i_mapping;
	page = read_cache_page(mapping, 0, (filler_t *)mapping->a_ops->readpage,
				NULL);
	if (IS_ERR(page))
		goto sync_fail;
	wait_on_page_locked(page);
	if (!PageUptodate(page))
		goto async_fail;
	*ppage = page;
	return kmap(page);

async_fail:
	page_cache_release(page);
	return ERR_PTR(-EIO);

sync_fail:
	return (char*)page;
}

int page_readlink(struct dentry *dentry, char __user *buffer, int buflen)
{
	struct page *page = NULL;
	char *s = page_getlink(dentry, &page);
	int res = vfs_readlink(dentry,buffer,buflen,s);
	if (page) {
		kunmap(page);
		page_cache_release(page);
	}
	return res;
}

int page_follow_link_light(struct dentry *dentry, struct nameidata *nd)
{
	struct page *page;
	nd_set_link(nd, page_getlink(dentry, &page));
	return 0;
}

void page_put_link(struct dentry *dentry, struct nameidata *nd)
{
	if (!IS_ERR(nd_get_link(nd))) {
		struct page *page;
		page = find_get_page(dentry->d_inode->i_mapping, 0);
		if (!page)
			BUG();
		kunmap(page);
		page_cache_release(page);
		page_cache_release(page);
	}
}

int page_symlink(struct inode *inode, const char *symname, int len)
{
	struct address_space *mapping = inode->i_mapping;
	struct page *page = grab_cache_page(mapping, 0);
	int err = -ENOMEM;
	char *kaddr;

	if (!page)
		goto fail;
	err = mapping->a_ops->prepare_write(NULL, page, 0, len-1);
	if (err)
		goto fail_map;
	kaddr = kmap_atomic(page, KM_USER0);
	memcpy(kaddr, symname, len-1);
	kunmap_atomic(kaddr, KM_USER0);
	mapping->a_ops->commit_write(NULL, page, 0, len-1);
	/*
	 * Notice that we are _not_ going to block here - end of page is
	 * unmapped, so this will only try to map the rest of page, see
	 * that it is unmapped (typically even will not look into inode -
	 * ->i_size will be enough for everything) and zero it out.
	 * OTOH it's obviously correct and should make the page up-to-date.
	 */
	if (!PageUptodate(page)) {
		err = mapping->a_ops->readpage(NULL, page);
		wait_on_page_locked(page);
	} else {
		unlock_page(page);
	}
	page_cache_release(page);
	if (err < 0)
		goto fail;
	mark_inode_dirty(inode);
	return 0;
fail_map:
	unlock_page(page);
	page_cache_release(page);
fail:
	return err;
}

struct inode_operations page_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= page_follow_link_light,
	.put_link	= page_put_link,
};

EXPORT_SYMBOL(__user_walk);
EXPORT_SYMBOL(follow_down);
EXPORT_SYMBOL(follow_up);
EXPORT_SYMBOL(get_write_access); /* binfmt_aout */
EXPORT_SYMBOL(getname);
EXPORT_SYMBOL(lock_rename);
EXPORT_SYMBOL(lookup_hash);
EXPORT_SYMBOL(lookup_one_len);
EXPORT_SYMBOL(page_follow_link_light);
EXPORT_SYMBOL(page_put_link);
EXPORT_SYMBOL(page_readlink);
EXPORT_SYMBOL(page_symlink);
EXPORT_SYMBOL(page_symlink_inode_operations);
EXPORT_SYMBOL(path_lookup);
EXPORT_SYMBOL(path_release);
EXPORT_SYMBOL(path_walk);
EXPORT_SYMBOL(permission);
EXPORT_SYMBOL(unlock_rename);
EXPORT_SYMBOL(vfs_create);
EXPORT_SYMBOL(vfs_follow_link);
EXPORT_SYMBOL(vfs_link);
EXPORT_SYMBOL(vfs_mkdir);
EXPORT_SYMBOL(vfs_mknod);
EXPORT_SYMBOL(generic_permission);
EXPORT_SYMBOL(vfs_readlink);
EXPORT_SYMBOL(vfs_rename);
EXPORT_SYMBOL(vfs_rmdir);
EXPORT_SYMBOL(vfs_symlink);
EXPORT_SYMBOL(vfs_unlink);
EXPORT_SYMBOL(dentry_unhash);
EXPORT_SYMBOL(generic_readlink);
